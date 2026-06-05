import { createRobutek } from "./libs/robutek.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
import * as wifi from "wifi";
import { UdpSocket } from "udp";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { Servo } from "./libs/servo.js";
import { driveStraight, rotateAngle, driveArc } from "./libs/drive.js";

const robutek = createRobutek("V2");

// Konfigurace pinů a směrování
const START_DRIVE_BUTTON_PIN = 0;   // Pravé tlačítko (IO00) pro Ruční Režim
const START_BUTTON_PIN = 2;         // Levé tlačítko (IO02) pro Autonomní Režim
const EMERGENCY_BUTTON_PIN = 17;    // Nouzové stop tlačítko
const I2C2_SDA = 15;
const I2C2_SCL = 16;
const SERVO_PIN = 21;
const LED_PIN = 36;
const LED_COUNT = 8;
const CURVE_SIGN = -1; // Směr zatáčení
const ANGLE_CENTER = 90;

const leds = new SmartLed(LED_PIN, LED_COUNT, LED_WS2812B);
const servo = new Servo(SERVO_PIN, 1, 4);

// LED barvy
const OFF = 0x000000;
const RED = 0x200000;
const GREEN = 0x002000;
const BLUE = 0x000020;
const YELLOW = 0x202000;
const WHITE = 0x101010;
const PURPLE = 0x200020;

function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

async function stopRobot(): Promise<void> {
  try {
    await robutek.stop(true);
  } catch (e) {}
  try {
    robutek.setSpeed(0);
  } catch (e) {}
}

function setServoAngle(angle: number): void {
  servo.write(Math.round((angle / 180) * 1023));
}

function setAllLeds(color: number): void {
  leds.clear();
  for (let i = 0; i < LED_COUNT; i++) {
    leds.set(i, color);
  }
  leds.show();
}

// Nouzový stop mechanismus
let emergencyLatched = false;

async function emergencyStop(): Promise<void> {
  if (emergencyLatched) return;
  emergencyLatched = true;
  console.log("!!! NOUZOVÉ STOP TLAČÍTKO STISKNUTO - OKAMŽITÉ ZASTAVENÍ POHONU !!!");
  
  // Zastavíme veškerý pohyb (brzda)
  try {
    await robutek.stop(true);
  } catch (e) {}
  try {
    robutek.setSpeed(0);
  } catch (e) {}

  // Červené varovné LED
  leds.clear();
  for (let i = 0; i < LED_COUNT; i++) {
    leds.set(i, RED);
  }
  leds.show();
  
  // Ukončení běhu programu
  try {
    exit(1);
  } catch (e) {}
  throw new Error("Emergency stop triggered on IO17");
}

// MPU6050 Ovladač pro Akcelerometr a Gyroskop
class MPU6050 {
  private i2c: any;
  public ad: number;

  constructor(i2c: any) {
    this.i2c = i2c;
    this.ad = 0x68;
  }

  probe(): boolean {
    for (const addr of [0x68, 0x69]) {
      try {
        this.i2c.writeTo(addr, 0x75);
        const id = this.i2c.readFrom(addr, 1)[0];
        console.log("MPU6050: Nalezen na adrese 0x" + addr.toString(16) + " (ID: 0x" + id.toString(16) + ")");
        this.ad = addr;
        return true;
      } catch (e) {
        // Ignorovat neúspěšnou adresu
      }
    }
    return false;
  }

  init() {
    this.i2c.writeTo(this.ad, [0x6B, 0x00]);
    this.i2c.writeTo(this.ad, [0x1A, 0x04]);
    this.i2c.writeTo(this.ad, [0x1B, 0x00]);
  }

  read() {
    const data = this.i2c.writeRead(this.ad, 0x3B, 14);
    
    const toInt16 = (high: number, low: number) => {
      let val = (high << 8) | low;
      if (val & 0x8000) val -= 0x10000;
      return val;
    };

    return {
      accel: {
        x: toInt16(data[0], data[1]),
        y: toInt16(data[2], data[3]),
        z: toInt16(data[4], data[5]),
      },
      gyro: {
        x: toInt16(data[8], data[9]),
        y: toInt16(data[10], data[11]),
        z: toInt16(data[12], data[13]),
      }
    };
  }
}

let gyro: MPU6050 | null = null;
let gyroZOffset = 0;
const angleState = { angleZ: 0 };
let lastTime = 0;
let intervalId: number | null = null;

let laptopIp = "";
let laptopPort = 0;
let driveEnabled = false; // Příznak pro aktivaci ručního pohonu
let selectedMode: "manual" | "auto" | null = null; // Zvolený režim
let ledState = false; // Ručně ovládaný stav LED pásku

// ==========================================
// 🎮 RUČNÍ REŽIM: WebSocket / UDP OVLÁDÁNÍ
// ==========================================
let lastPacketTime = 0;
let udpSocket: UdpSocket | null = null;
let lastPrintTime = 0;

async function startUdpServer() {
  console.log("Spouštím UDP server na portu 4444...");
  
  udpSocket = new UdpSocket({
    port: 4444,
    onReadable: (avail) => {
      lastPacketTime = Date.now();
      if (!udpSocket) return;
      while (avail > 0) {
        const dgram = udpSocket.read();
        if (!dgram) break;
        
        laptopIp = dgram.address;
        laptopPort = dgram.port;
        
        try {
          const arr = new Uint8Array(dgram);
          let str = "";
          for (let i = 0; i < arr.length; i++) {
            str += String.fromCharCode(arr[i]);
          }
          const msg = JSON.parse(str);
          
          if (msg.type === "led") {
            ledState = !!msg.state;
          } else {
            const speed = Number(msg.speed);
            const steer = Number(msg.steer);
            
            // Výpis stavu ovladače do terminálu (omezen na každých 300 ms)
            const now = Date.now();
            if (now - lastPrintTime > 300) {
              const speedPct = Math.round((speed / 700) * 100);
              const steerPct = Math.round(steer * 100);
              
              let dirStr = "Stojí";
              if (speed > 5) {
                const rtVal = (speed / 700).toFixed(2);
                dirStr = `Vpřed ${speedPct}% (RT=${rtVal})`;
              } else if (speed < -5) {
                const ltVal = (Math.abs(speed) / 400).toFixed(2);
                dirStr = `Vzad ${Math.abs(speedPct)}% (LT=${ltVal})`;
              }
              
              let steerStr = "Rovně";
              if (steer > 0.05) {
                steerStr = `Vpravo ${steerPct}% (Stick=${steer.toFixed(2)})`;
              } else if (steer < -0.05) {
                steerStr = `Vlevo ${Math.abs(steerPct)}% (Stick=${steer.toFixed(2)})`;
              }
              
              console.log(`Ovladač -> Rychlost: ${dirStr} | Zatáčení: ${steerStr}`);
              lastPrintTime = now;
            }
            
            // Nastavení motorů
            if (driveEnabled) {
              robutek.setSpeed(speed);
              robutek.move(CURVE_SIGN * steer);
            } else {
              robutek.setSpeed(0);
              robutek.stop(true);
            }
          }
        } catch (e) {
          // Ignorovat chyby
        }
        avail--;
      }
    }
  });
}

// ==========================================
// 🤖 AUTONOMNÍ REŽIM: LIDAR & GYRO
// ==========================================
let lidar: VL53L0X | null = null;       // Přední Lidar (na I2C2)
let leftLidar: VL53L0X | null = null;   // Levý Lidar (na I2C1)

let latestFrontDistance = 9999;
let latestLeftDistance = 9999;
let sensorUpdateCount = 0;

// Konstanty pro autonomní režim
const SPEED_NORMAL = 700; // Maximální rychlost na rovinkách v mm/s
const SPEED_SLOW = 350;   // Snížená rychlost při přiblížení k zatáčce
const SPEED_TURN = 200;   // Rychlost pro pomalé otáčení a dorovnání
const SPEED_ARC = 300;    // Rychlost projíždění oblouků (zatáček) v mm/s
const RAMP_AUTO = 3000;

const FRONT_SLOW_DIST = 850;   // Vzdálenost k překážce v mm pro zpomalování
const FRONT_DANGER_DIST = 450; // Vzdálenost pro zahájení pravého oblouku
const LEFT_WALL_THRESHOLD = 300; // Práh konce levé stěny pro zatáčení vlevo

const RADIUS_LEFT = 105;   // Poloměr levé zatáčky v mm
const RADIUS_RIGHT = 110;  // Poloměr pravé zatáčky v mm

let lastLeftArcTime = 0;
let lastRightArcTime = 0;

async function getDistance(sensor: VL53L0X | null): Promise<number> {
  if (sensor == null) return 9999;
  try {
    const m = await sensor.read();
    if (m.distance < 30 || m.distance > 2000) {
      return 9999;
    }
    return m.distance;
  } catch (e) {
    return 9999;
  }
}

function updateSensorLeds(front: number, left: number): void {
  if (emergencyLatched) return;
  leds.clear();
  // LED 0: Přední dálkoměr
  if (front <= FRONT_DANGER_DIST) {
    leds.set(0, 0x300000); // Červená (zeď blízko)
  } else {
    leds.set(0, 0x003000); // Zelená (volno)
  }
  // LED 1: Levý dálkoměr
  if (left <= LEFT_WALL_THRESHOLD) {
    leds.set(1, 0x300000); // Červená (stěna blízko)
  } else {
    leds.set(1, 0x003000); // Zelená (volno/roh)
  }
  leds.show();
}

async function startLidarLoop(): Promise<void> {
  console.log("=== START SENSOR LOOP (Autonomní LiDARy) ===");
  while (!emergencyLatched) {
    try {
      const left = await getDistance(leftLidar);
      const front = await getDistance(lidar);
      latestLeftDistance = left;
      latestFrontDistance = front;
      sensorUpdateCount++;
      updateSensorLeds(front, left);
    } catch (e) {
      // ignore
    }
    await sleep(5);
  }
}

async function waitForFreshSensors(): Promise<void> {
  const startCount = sensorUpdateCount;
  while (sensorUpdateCount < startCount + 2 && !emergencyLatched) {
    await sleep(5);
  }
}

async function jedem(): Promise<void> {
  console.log("=== START POHYBU JEDEM (Wall Follower) ===");
  while (!emergencyLatched) {
    const left = latestLeftDistance;
    const front = latestFrontDistance;

    console.log(`[jedem] Leve: ${left.toFixed(0)} mm | Predni: ${front.toFixed(0)} mm`);

    const canTurnLeft = (left > LEFT_WALL_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);

    if (canTurnLeft) {
      console.log(`-> Vlevo volno: zatáčím 180° vlevo (R=${RADIUS_LEFT} mm)`);
      lastLeftArcTime = Date.now();
      await driveArc(
        robutek,
        angleState,
        RADIUS_LEFT,
        180,
        SPEED_ARC,
        EMERGENCY_BUTTON_PIN,
        leds,
        emergencyStop,
        () => emergencyLatched
      );
      await waitForFreshSensors();
    } else if (front > FRONT_DANGER_DIST) {
      const currentSpeed = (front < FRONT_SLOW_DIST) ? SPEED_SLOW : SPEED_NORMAL;
      console.log(`-> Vepředu volno: jedu rovně rychlostí ${currentSpeed} mm/s`);
      
      await driveStraight(
        robutek,
        gyro,
        gyroZOffset,
        5000,
        currentSpeed,
        EMERGENCY_BUTTON_PIN,
        angleState,
        leds,
        emergencyStop,
        () => emergencyLatched,
        async () => {
          const currLeft = latestLeftDistance;
          const currFront = latestFrontDistance;
          const stopForLeft = (currLeft > LEFT_WALL_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);
          const stopForFront = (currFront <= FRONT_DANGER_DIST);
          const speedShouldChange = (currentSpeed === SPEED_NORMAL && currFront < FRONT_SLOW_DIST) ||
                                    (currentSpeed === SPEED_SLOW && currFront >= FRONT_SLOW_DIST);
          return (stopForLeft || stopForFront || speedShouldChange);
        }
      );
    } else {
      if (Date.now() - lastRightArcTime > 5000) {
        console.log(`-> Zablokováno: zatáčím 90° vpravo (R=${RADIUS_RIGHT} mm)`);
        lastRightArcTime = Date.now();
        await driveArc(
          robutek,
          angleState,
          RADIUS_RIGHT,
          -90,
          SPEED_ARC,
          EMERGENCY_BUTTON_PIN,
          leds,
          emergencyStop,
          () => emergencyLatched
        );
        await waitForFreshSensors();
      } else {
        console.log("-> Pravé zatáčení blokováno 5s limitem. Zastavuji...");
        await stopRobot();
        await sleep(50);
      }
    }
    await sleep(5);
  }
}

async function runSequence(): Promise<void> {
  setServoAngle(ANGLE_CENTER);
  await sleep(100);
  await jedem();
}

// ==========================================
// 🏁 INICIALIZACE A HLAVNÍ METODA MAIN
// ==========================================
async function main() {
  // Inicializace LED
  leds.clear();
  for (let i = 0; i < LED_COUNT; i++) {
    leds.set(i, WHITE);
  }
  leds.show();
  
  await stopRobot();
  setServoAngle(ANGLE_CENTER);

  // Konfigurace nouzového tlačítka (IO17 a GND)
  try {
    gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.on("falling", EMERGENCY_BUTTON_PIN, () => {
      emergencyStop();
    });
    console.log("HW interrupt pro nouzové tlačítko (IO17) aktivován.");
  } catch (e) {
    console.log("Chyba nastavení HW interruptu: " + e);
  }

  setInterval(async () => {
    if (gpio.read(EMERGENCY_BUTTON_PIN) === 0) {
      await emergencyStop();
    }
  }, 5);

  // Konfigurace tlačítek režimů
  try {
    gpio.pinMode(START_DRIVE_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
  } catch (e) {
    console.log("Chyba nastavení tlačítek režimů: " + e);
  }

  // Inicializace I2C2 pro gyroskop/akcelerometr
  try {
    if (I2C2) {
      I2C2.setup({
        sda: I2C2_SDA,
        scl: I2C2_SCL,
        bitrate: 400000,
      });
      console.log("I2C2 sběrnice nastavena.");

      gyro = new MPU6050(I2C2);
      if (gyro.probe()) {
        gyro.init();
        console.log("MPU6050 gyroskop úspěšně inicializován.");
      } else {
        gyro = null;
        console.log("MPU6050 gyroskop nebyl nalezen.");
      }
    } else {
      console.log("I2C2 sběrnice není dostupná.");
    }
  } catch (e) {
    gyro = null;
    console.log("Chyba při inicializaci MPU6050: " + e);
  }

  // --- REŽIM VOLBY (Boot) ---
  console.log("=====================================================");
  console.log("=== VOLBA REŽIMU ROBOTA:");
  console.log("=== Stiskněte IO00 (Pravé tlačítko) pro RUČNÍ OVLÁDÁNÍ");
  console.log("=== Stiskněte IO02 (Levé tlačítko) pro AUTONOMNÍ JÍZDU");
  console.log("=====================================================");

  // Vizuální indikace: levé 4 LED modré (IO0), pravé 4 LED žluté (IO2)
  leds.clear();
  for (let i = 0; i < 4; i++) {
    leds.set(i, BLUE);
  }
  for (let i = 4; i < 8; i++) {
    leds.set(i, YELLOW);
  }
  leds.show();

  while (selectedMode === null) {
    if (gpio.read(START_DRIVE_BUTTON_PIN) === 0) {
      await sleep(150);
      if (gpio.read(START_DRIVE_BUTTON_PIN) === 0) {
        selectedMode = "manual";
        console.log("=== ZVOLEN REŽIM: MANUÁLNÍ OVLÁDÁNÍ ===");
        break;
      }
    }
    if (gpio.read(START_BUTTON_PIN) === 0) {
      await sleep(150);
      if (gpio.read(START_BUTTON_PIN) === 0) {
        selectedMode = "auto";
        console.log("=== ZVOLEN REŽIM: AUTONOMNÍ JÍZDA ===");
        break;
      }
    }
    await sleep(20);
  }

  if (selectedMode === "manual") {
    // --- OVLÁDÁNÍ PŘES WEBSOCKET / UDP ---
    robutek.setRamp(120);
    await startUdpServer();
    driveEnabled = true;

    // Bezpečnostní pojistka a indikátor signálu
    lastPacketTime = Date.now();
    setInterval(async () => {
      const latency = Date.now() - lastPacketTime;
      leds.clear();
      if (latency > 400) {
        await stopRobot();
        for (let i = 0; i < LED_COUNT; i++) {
          leds.set(i, RED);
        }
      } else {
        const color = ledState ? GREEN : OFF;
        for (let i = 0; i < LED_COUNT; i++) {
          leds.set(i, color);
        }
      }
      leds.show();
    }, 100);

    // Odesílání telemetrie (akcelerometr) přes UDP
    setInterval(() => {
      if (gyro && laptopIp && laptopPort && udpSocket) {
        try {
          const raw = gyro.read();
          const ax = Number((raw.accel.x / 16384.0).toFixed(2));
          const ay = Number((raw.accel.y / 16384.0).toFixed(2));
          const az = Number((raw.accel.z / 16384.0).toFixed(2));
          
          const payload = JSON.stringify({
            type: "telemetry",
            ax: ax,
            ay: ay,
            az: az
          });
          
          const buf = new Uint8Array(payload.length);
          for (let i = 0; i < payload.length; i++) {
            buf[i] = payload.charCodeAt(i);
          }
          udpSocket.write(buf.buffer, laptopIp, laptopPort);
        } catch (e) {}
      }
    }, 50);

    console.log("=== MANUÁLNÍ OVLÁDÁNÍ PŘIPRAVENO ===");
    while (true) {
      const ip = wifi.currentIp();
      if (ip) {
        console.log(`WiFi aktivní. IP adresa robota: ${ip}`);
      } else {
        console.log("Čekám na aktivaci WiFi...");
      }
      await sleep(2000);
    }
  } else {
    // --- AUTONOMNÍ REŽIM ---
    robutek.setRamp(RAMP_AUTO);

    // Inicializace I2C1 sběrnice pro levý Lidar
    try {
      if (I2C1) {
        I2C1.setup({
          sda: 4,
          scl: 5,
          bitrate: 400000,
        });
        console.log("I2C1 sběrnice nastavena.");
      } else {
        console.log("I2C1 sběrnice není dostupná.");
      }
    } catch (e) {
      console.log("Chyba I2C1: " + e);
    }

    // Přední Lidar (na I2C2)
    try {
      if (I2C2) {
        lidar = new VL53L0X(I2C2);
        console.log("Přední Lidar VL53L0X (I2C2) připojen.");
      } else {
        console.log("Chyba Přední Lidar (I2C2): I2C2 není dostupný.");
      }
    } catch (e) {
      console.log("Chyba Přední Lidar (I2C2): " + e);
    }

    // Levý Lidar (na I2C1)
    try {
      if (I2C1) {
        leftLidar = new VL53L0X(I2C1);
        console.log("Levý Lidar VL53L0X (I2C1) připojen.");
      } else {
        console.log("Chyba Levý Lidar (I2C1): I2C1 není dostupný.");
      }
    } catch (e) {
      console.log("Chyba Levý Lidar (I2C1): " + e);
    }

    if (gyro) {
      // Kalibrace gyroskopu
      console.log("KALIBRACE GYROSKOPU - NEHÝBEJTE S ROBOTEM...");
      setAllLeds(PURPLE);
      let sum = 0;
      const CALIBRATION_STEPS = 50;
      for (let i = 0; i < CALIBRATION_STEPS; i++) {
        const data = gyro.read();
        sum += data.gyro.z;
        await sleep(5);
      }
      gyroZOffset = sum / CALIBRATION_STEPS;
      console.log("Kalibrace hotova. Offset Z: " + gyroZOffset.toFixed(2));
      
      // Integrace úhlu
      lastTime = Date.now();
      angleState.angleZ = 0;
      intervalId = setInterval(() => {
        if (gyro) {
          try {
            const data = gyro.read();
            const now = Date.now();
            const dt = (now - lastTime) / 1000.0;
            lastTime = now;
            
            let gz_dps = (data.gyro.z - gyroZOffset) / 131.0;
            if (Math.abs(gz_dps) < 0.3) {
              gz_dps = 0.0;
            }
            if (dt > 0 && dt < 0.2) {
              angleState.angleZ += gz_dps * dt;
            }
          } catch (e) {}
        }
      }, 10);
    }

    // Spustíme čtení Lidarů na pozadí
    startLidarLoop();
    
    // Spustíme testovací a závodní sekvenci autonomní jízdy
    await runSequence();
  }
}

main().catch(async (e) => {
  console.log("HLAVNÍ CHYBA PROGRAMU: " + e);
  await stopRobot();
  setAllLeds(PURPLE);
});
