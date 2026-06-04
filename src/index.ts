import { createRobutek } from "./libs/robutek.js";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
import { driveStraight, rotateAngle, driveArc } from "./libs/drive.js";


// ======================================================
// TEST JÍZDY ROVNĚ 1 METR POMOCÍ GYROSKOPU
// - Robot po stisku tlačítka IO2 uzamkne aktuální směr
// - Jede rovně přesně 1 metr (1000 mm) podle enkodérů kol
// - Udržuje přímý směr regulací podle gyroskopu (MPU6050)
// - Po ujetí 1 metru zabrzdí a rozsvítí fialovou barvu
// ======================================================

const robutek = createRobutek("V2");

// -------------------- PINY --------------------
const START_BUTTON_PIN = 2;
const EMERGENCY_BUTTON_PIN = 17;
const I2C2_SDA = 15;
const I2C2_SCL = 16;
const SERVO_PIN = 21;
const LED_PIN = 36;
const LED_COUNT = 8;

const ANGLE_CENTER = 90;

// LED barvy
const OFF = 0x000000;
const BLUE = 0x000030;
const GREEN = 0x003000;
const RED = 0x300000;
const YELLOW = 0x303000;
const CYAN = 0x003030;
const PURPLE = 0x300030;
const WHITE = 0x202020;

const leds = new SmartLed(LED_PIN, LED_COUNT, LED_WS2812B);
const servo = new Servo(SERVO_PIN, 1, 4);

// -------------------- GYROSKOP (MPU6050) OVLADAČ --------------------
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
        console.log("MPU6050: WHO_AM_I na adrese 0x" + addr.toString(16) + " vrátil 0x" + id.toString(16));
        this.ad = addr;
        return true;
      } catch (e) {
        // Adresa neodpovídá
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
      temp: toInt16(data[6], data[7]) / 340.0 + 36.53,
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
let lidar: VL53L0X | null = null;       // Přední Lidar (na I2C2)
let leftLidar: VL53L0X | null = null;   // Levý Lidar (na I2C1)
let lastTime = 0;
let intervalId: number | null = null;
let emergencyLatched = false;
let lastLeftArcTime = 0;   // Timestamp posledního zatáčení vlevo
let lastRightArcTime = 0;  // Timestamp posledního zatáčení vpravo

// -------------------- PARAMETRY JÍZDY --------------------
const SPEED_NORMAL = 450; // Zrychleno na 450 mm/s pro rychlejší a plynulejší jízdu rovně
const SPEED_TURN = 150;
const RAMP = 3000;

// Pokud robot při zatáčení uhýbá na špatnou stranu, změň na +1.
const CURVE_SIGN = -1;

// -------------------- POMOCNÉ FUNKCE --------------------
function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function isPressed(pin: number): boolean {
  return gpio.read(pin) == 0;
}

function setAllLeds(color: number): void {
  leds.clear();
  for (let i = 0; i < LED_COUNT; i++) {
    leds.set(i, color);
  }
  leds.show();
}

function setServoAngle(angle: number): void {
  servo.write(Math.round((angle / 180) * 1023));
}

async function stopRobot(): Promise<void> {
  try {
    await robutek.stop(true);
  } catch (e) {
    console.log("stop chyba: " + e);
  }
  try {
    robutek.setSpeed(0);
  } catch (e) {
    console.log("setSpeed chyba: " + e);
  }
}

async function emergencyStop(): Promise<void> {
  emergencyLatched = true;
  console.log("!!! NOUZOVÉ STOP TLAČÍTKO STISKNUTO - VYPNUTÍ POHYBU A RESET !!!");
  
  // Zastavíme veškerý pohyb
  try {
    await stopRobot();
  } catch (e) {}

  // Rozsvítíme červeně
  setAllLeds(RED);
  await sleep(500);

  // Ukončíme program s chybovým kódem (pro restart ze strany supervisora)
  exit(1);

  // Jako záloha vyvoláme unhandled exception k vynucení tvrdého restartu firmware
  throw new Error("Emergency restart requested");
}

function applySteering(speed: number, steer: number) {
  robutek.setSpeed(speed);
  robutek.move(CURVE_SIGN * steer);
}

// -------------------- INICIALIZACE HW --------------------
async function initHardware(): Promise<void> {
  gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
  gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);

  setAllLeds(WHITE);
  await stopRobot();
  robutek.setRamp(RAMP);

  setServoAngle(ANGLE_CENTER); // Nastavíme na střed a už s ním neotáčíme!

  console.log("======================================");
  console.log("TEST JÍZDY ROVNĚ 1 METR - INICIALIZACE");
  console.log("IO2 = START, IO17 = NOUZOVE STOP");
  console.log("======================================");

  // Inicializace I2C2 sběrnice
  try {
    I2C2.setup({
      sda: I2C2_SDA,
      scl: I2C2_SCL,
      bitrate: 400000,
    });
    console.log("OK: I2C2 sběrnice nastavena.");
  } catch (e) {
    console.log("CHYBA I2C2: " + e);
  }

  // Přední Lidar (na I2C2)
  try {
    lidar = new VL53L0X(I2C2);
    console.log("OK: Přední Lidar VL53L0X (I2C2) připojen.");
  } catch (e) {
    console.log("CHYBA Přední Lidar (VL53L0X na I2C2): " + e);
  }

  // Levý Lidar (na I2C1)
  try {
    leftLidar = new VL53L0X(I2C1);
    console.log("OK: Levý Lidar VL53L0X (I2C1) připojen.");
  } catch (e) {
    console.log("CHYBA Levý Lidar (VL53L0X na I2C1): " + e);
  }

  // Gyroskop MPU6050 (na I2C2)
  try {
    gyro = new MPU6050(I2C2);
    if (gyro.probe()) {
      gyro.init();
      console.log("OK: Gyroskop MPU6050 inicializován.");
      
      // Kalibrace gyroskopu - 200 měření v klidu
      console.log("KALIBRACE GYROSKOPU - NEHÝBEJTE S ROBOTEM...");
      setAllLeds(PURPLE); // Během kalibrace svítíme fialově
      let sum = 0;
      for (let i = 0; i < 200; i++) {
        const data = gyro.read();
        sum += data.gyro.z;
        await sleep(5);
      }
      gyroZOffset = sum / 200;
      console.log("KALIBRACE HOTOVA. Gyro Z Offset: " + gyroZOffset.toFixed(2));
      
      // Spuštění intervalu pro integraci úhlu
      lastTime = Date.now();
      angleState.angleZ = 0;
      if (intervalId !== null) {
        clearInterval(intervalId);
      }
      intervalId = setInterval(() => {
        if (gyro) {
          try {
            const data = gyro.read();
            const now = Date.now();
            const dt = (now - lastTime) / 1000.0;
            lastTime = now;
            
            // Převod na stupně za sekundu (dps) s odečtením offsetu
            let gz_dps = (data.gyro.z - gyroZOffset) / 131.0;
            
            // Mrtvá zóna (snížená na 0.3 dps pro extrémní přesnost při pomalém otáčení)
            if (Math.abs(gz_dps) < 0.3) {
              gz_dps = 0.0;
            }
            
            if (dt > 0 && dt < 0.2) {
              angleState.angleZ += gz_dps * dt;
            }
          } catch (e) {
            // ignorovat
          }
        }
      }, 10);
      
    } else {
      gyro = null;
      console.log("CHYBA: Gyroskop MPU6050 nebyl nalezen na I2C2.");
    }
  } catch (e) {
    gyro = null;
    console.log("CHYBA Gyroskop: " + e);
  }

  setAllLeds(YELLOW); // Připraven ke startu

  // Samostatný background task pro sledování nouzového tlačítka a okamžitý reset
  setInterval(async () => {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      await emergencyStop();
    }
  }, 10);

  // Hardware interrupt na spádovou hranu (tlačítko stisknuto) pro okamžitou odezvu bez čekání
  try {
    gpio.on("falling", EMERGENCY_BUTTON_PIN, () => {
      emergencyStop();
    });
    console.log("OK: Hardware interrupt pro IO17 aktivován.");
  } catch (e) {
    console.log("CHYBA při registraci HW interruptu: " + e);
  }
}

// -------------------- ČEKÁNÍ NA START --------------------
async function waitForStart(): Promise<void> {
  await stopRobot();
  setAllLeds(BLUE);
  console.log("CEKAM NA START - zmackni IO2");

  while (true) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      await emergencyStop();
    }

    if (emergencyLatched) {
      setAllLeds(RED);
      if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
        await sleep(200);
        if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
          emergencyLatched = false;
          console.log("NOUZOVE STOP RESETOVANO");
          setAllLeds(YELLOW);
          await sleep(300);
          return;
        }
      }
      await sleep(20);
      continue;
    }

    if (isPressed(START_BUTTON_PIN)) {
      await sleep(200);
      if (isPressed(START_BUTTON_PIN)) {
        console.log("=== AUTONOMNI START ===");
        angleState.angleZ = 0;
        lastTime = Date.now();
        setAllLeds(YELLOW);
        await sleep(300);
        return;
      }
    }
    await sleep(20);
  }
}

async function getDistance(sensor: VL53L0X | null): Promise<number> {
  if (sensor == null) return 9999;
  try {
    const m = await sensor.read();
    // Jakákoliv hodnota pod 30 mm (3 cm) je považována za chybu senzoru (error) a bere se jako prázdno (9999)
    if (m.distance < 30 || m.distance > 2000) {
      return 9999;
    }
    return m.distance;
  } catch (e) {
    return 9999;
  }
}

// -------------------- VIZUALIZACE SENZORŮ NA LED PÁSKU --------------------
function updateSensorLeds(front: number, left: number): void {
  if (emergencyLatched) return;

  // LED 0 (první): Přední senzor (hranice 40 cm = 400 mm)
  if (front <= 400) {
    leds.set(0, 0x300000); // Červená (překážka nablízku)
  } else {
    leds.set(0, 0x003000); // Zelená (volno)
  }

  // LED 1 (druhá): Levý/boční senzor (hranice 30 cm = 300 mm)
  if (left <= 300) {
    leds.set(1, 0x300000); // Červená (zeď nablízku)
  } else {
    leds.set(1, 0x003000); // Zelená (volno/roh)
  }

  leds.show();
}

// -------------------- AUTONOMNÍ POHYB JEDEM (Wall Follower) --------------------
async function jedem(): Promise<void> {
  const DIST_THRESHOLD = 300; // Zvýšeno z 200 na 300 mm (30 cm) pro včasnou detekci levého rohu
  console.log("=== START POHYBU JEDEM (Wall Follower) ===");

  while (!emergencyLatched) {
    // Paralelní čtení z obou LiDARů pro maximální rychlost odezvy
    const [left, front] = await Promise.all([
      getDistance(leftLidar),
      getDistance(lidar)
    ]);

    // Aktualizujeme LED stav na základě měření
    updateSensorLeds(front, left);

    console.log(`[jedem] Leve: ${left.toFixed(0)} mm | Predni: ${front.toFixed(0)} mm`);

    // Výpočet podmínek zatáčení
    const canTurnLeft = (left > DIST_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);

    if (canTurnLeft) {
      console.log("-> Vlevo volno: zatáčím plynulým obloukem 180° vlevo (R=120 mm)");
      lastLeftArcTime = Date.now();
      await driveArc(
        robutek,
        angleState,
        120, // poloměr 12 cm
        180, // 180 stupňů vlevo
        216, // rychlost 216 mm/s
        EMERGENCY_BUTTON_PIN,
        leds,
        emergencyStop,
        () => emergencyLatched
      );
    } else if (front > 400) {
      console.log("-> Vepředu volno (vlevo zeď): jedu rovně");
      // Jedeme rovně, dokud se neuvolní levá strana (s ohledem na 5s limit) nebo se nezablokuje předek
      await driveStraight(
        robutek,
        gyro,
        gyroZOffset,
        5000, // Dlouhá jízda, kterou přerušíme senzory
        SPEED_NORMAL,
        EMERGENCY_BUTTON_PIN,
        angleState,
        leds,
        emergencyStop,
        () => emergencyLatched,
        async () => {
          const [currLeft, currFront] = await Promise.all([
            getDistance(leftLidar),
            getDistance(lidar)
          ]);
          updateSensorLeds(currFront, currLeft);
          
          const stopForLeft = (currLeft > DIST_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);
          const stopForFront = (currFront <= 400);
          
          return (stopForLeft || stopForFront);
        }
      );
    } else {
      // Zablokováno vepředu i vlevo: zkusíme zabočit vpravo, pokud od minulého pravého oblouku uběhlo více než 5 sekund
      if (Date.now() - lastRightArcTime > 5000) {
        console.log("-> Zablokováno (vlevo zeď, vepředu zeď): zatáčím plynulým obloukem 90° vpravo (R=120 mm)");
        lastRightArcTime = Date.now();
        await driveArc(
          robutek,
          angleState,
          120,  // poloměr 12 cm
          -90,  // 90 stupňů vpravo
          216,  // rychlost 216 mm/s
          EMERGENCY_BUTTON_PIN,
          leds,
          emergencyStop,
          () => emergencyLatched
        );
      } else {
        console.log("-> Zablokováno, ale pravé zatáčení je blokováno 5s limitem. Čekám...");
        await sleep(50);
      }
    }

    await sleep(20);
  }
}

// -------------------- TESTOVACÍ SEKVENCE POHYBŮ --------------------
async function runSequence(): Promise<void> {
  setServoAngle(ANGLE_CENTER);
  await sleep(100);
  await jedem();
}

// -------------------- MAIN --------------------
async function main(): Promise<void> {
  await initHardware();
  while (true) {
    await waitForStart();
    await runSequence();
  }
}

main().catch(async (e) => {
  console.log("HLAVNÍ CHYBA PROGRAMU: " + e);
  await stopRobot();
  setAllLeds(PURPLE);
});


