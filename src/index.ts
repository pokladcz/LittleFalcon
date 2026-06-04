import { createRobutek } from "./libs/robutek.js";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { ZSCS2016C } from "./libs/zscs2016c.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";

// ======================================================
// TESTOVACÍ PROGRAM PRO SENSORY ROBOTA
// - Čtení hodnot z předního Lidaru (VL53L0X na I2C2) na servu (BEZ OTÁČENÍ SERVA!)
// - Čtení hodnot z levého Lidaru (VL53L0X na I2C1)
// - Čtení dolních infračervených senzorů čáry (LineFL, LineFR, LineBL, LineBR)
// - Vyhledání a integrace gyroskopu (MPU6050) pro výpočet úhlu ve stupních
// - Ovládání LED pásku (na startu čekání, po stisku IO2 blikání/indikace hodnot)
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

// Úhly serva
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

let lidar: VL53L0X | null = null;       // Přední Lidar (na I2C2)
let leftLidar: VL53L0X | null = null;   // Levý Lidar (na I2C1)
let rgb: ZSCS2016C | null = null;

// -------------------- GYROSKOP (MPU6050) OVLADAČ --------------------
class MPU6050 {
  private i2c: any;
  public ad: number;

  constructor(i2c: any) {
    this.i2c = i2c;
    this.ad = 0x68;
  }

  // Zkusí najít gyroskop na adresách 0x68 nebo 0x69
  probe(): boolean {
    for (const addr of [0x68, 0x69]) {
      try {
        // Zápis registru WHO_AM_I (0x75)
        this.i2c.writeTo(addr, 0x75);
        const id = this.i2c.readFrom(addr, 1)[0];
        console.log("MPU6050: WHO_AM_I na adrese 0x" + addr.toString(16) + " vrátil 0x" + id.toString(16));
        this.ad = addr;
        return true;
      } catch (e) {
        // Adresa neodpovídá, zkusit další
      }
    }
    return false;
  }

  init() {
    // Probudit gyroskop: zapsat 0x00 do registru PWR_MGMT_1 (0x6B)
    this.i2c.writeTo(this.ad, [0x6B, 0x00]);
    // Nastavit low-pass filtr (DLPF) na 21 Hz: zapsat 0x04 do registru CONFIG (0x1A)
    this.i2c.writeTo(this.ad, [0x1A, 0x04]);
    // Nastavit rozsah gyroskopu na ±250 °/s: zapsat 0x00 do registru GYRO_CONFIG (0x1B)
    this.i2c.writeTo(this.ad, [0x1B, 0x00]);
  }

  read() {
    // Přečte 14 bajtů dat od registru 0x3B (ACCEL_XOUT_H)
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
let angleZ = 0;
let lastTime = 0;
let intervalId: number | null = null;
let emergencyLatched = false;

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

// -------------------- INICIALIZACE HW --------------------
async function initHardware(): Promise<void> {
  gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
  gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);

  setAllLeds(WHITE);
  servo.write(Math.round((ANGLE_CENTER / 180) * 1023)); // Nastavíme na střed a už s ním neotáčíme!

  console.log("======================================");
  console.log("TEST SENSORŮ - INICIALIZACE");
  console.log("IO2 = START, IO17 = NOUZOVE STOP");
  console.log("I2C2 SDA=" + I2C2_SDA + " SCL=" + I2C2_SCL);
  console.log("======================================");

  // Nastavení I2C2 sběrnice
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

  // Scan I2C2 sběrnice pro diagnostiku
  console.log("Skenuji I2C2 sběrnici...");
  for (let addr = 1; addr < 127; addr++) {
    try {
      I2C2.writeTo(addr, []);
      console.log(` -> Nalezeno I2C zařízení na I2C2 adrese 0x${addr.toString(16)} (${addr})`);
    } catch (e) {
      // Žádné zařízení neodpovědělo
    }
  }

  // Scan I2C1 sběrnice pro diagnostiku
  console.log("Skenuji I2C1 sběrnici...");
  for (let addr = 1; addr < 127; addr++) {
    try {
      I2C1.writeTo(addr, []);
      console.log(` -> Nalezeno I2C zařízení na I2C1 adrese 0x${addr.toString(16)} (${addr})`);
    } catch (e) {
      // Žádné zařízení neodpovědělo
    }
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

  // RGB senzor
  try {
    rgb = new ZSCS2016C(I2C2, false);
    rgb.enable();
    console.log("OK: RGB ZSCS2016C připojen.");
  } catch (e) {
    console.log("CHYBA RGB: " + e);
  }

  // Gyroskop MPU6050 (na I2C2)
  try {
    gyro = new MPU6050(I2C2);
    if (gyro.probe()) {
      gyro.init();
      console.log("OK: Gyroskop MPU6050 inicializován.");
      
      // Kalibrace gyroskopu - 200 přesnějších měření v klidu
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
      angleZ = 0;
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
            let gyroZ_dps = (data.gyro.z - gyroZOffset) / 131.0;
            
            // Prahová mrtvá zóna (deadband) po vzoru FOTONu (cca 0.85 °/s)
            // Tím se zabrání samovolnému načítání úhlu (driftu), když robot stojí
            if (Math.abs(gyroZ_dps) < 0.85) {
              gyroZ_dps = 0.0;
            }
            
            // Integrace úhlu (pokud je časový krok rozumný)
            if (dt > 0 && dt < 0.2) {
              angleZ += gyroZ_dps * dt;
            }
          } catch (e) {
            // ignorovat případné chyby čtení na sběrnici
          }
        }
      }, 10); // Čtení každých 10 ms pro vysokou vzorkovací frekvenci a přesnost
      
    } else {
      gyro = null;
      console.log("CHYBA: Gyroskop MPU6050 nebyl nalezen na I2C2.");
    }
  } catch (e) {
    gyro = null;
    console.log("CHYBA Gyroskop: " + e);
  }

  setAllLeds(YELLOW); // Po úspěšném bootu a kalibraci svítí žlutě a čeká na start
}

// -------------------- ČEKÁNÍ NA START --------------------
async function waitForStart(): Promise<void> {
  console.log("CEKAM NA START - zmackni IO2");

  while (true) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      emergencyLatched = true;
      setAllLeds(RED);
    }

    if (emergencyLatched) {
      if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
        await sleep(200);
        if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
          emergencyLatched = false;
          console.log("NOUZOVE STOP RESETOVANO");
          setAllLeds(YELLOW);
          await sleep(300);
        }
      }
      await sleep(20);
      continue;
    }

    if (isPressed(START_BUTTON_PIN)) {
      await sleep(200);
      if (isPressed(START_BUTTON_PIN)) {
        console.log("=== START TESTU ===");
        angleZ = 0; // Vynulování úhlu při každém novém startu testu!
        lastTime = Date.now();
        setAllLeds(BLUE); // Při startu se rozsvítí modrá!
        await sleep(300);
        return;
      }
    }
    await sleep(20);
  }
}

// -------------------- SMYČKA MĚŘENÍ --------------------
async function runTest(): Promise<void> {
  while (!emergencyLatched) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      emergencyLatched = true;
      setAllLeds(RED);
      console.log("NOUZOVE ZASTAVENI");
      break;
    }

    console.log("----------------------------------------");

    // 1. Měření předního Lidaru (středová vzdálenost, bez otáčení serva)
    let frontDistStr = "N/A";
    let frontDistVal = 0;
    if (lidar != null) {
      try {
        const m = await lidar.read();
        frontDistVal = m.distance;
        frontDistStr = frontDistVal + " mm";
      } catch (e) {
        frontDistStr = "Chyba (" + e + ")";
      }
    }
    console.log("Predni senzor (Lidar): " + frontDistStr);

    // 2. Měření levého Lidaru (na I2C1)
    let leftDistStr = "N/A";
    let leftDistVal = 0;
    if (leftLidar != null) {
      try {
        const m = await leftLidar.read();
        leftDistVal = m.distance;
        leftDistStr = leftDistVal + " mm";
      } catch (e) {
        leftDistStr = "Chyba (" + e + ")";
      }
    }
    console.log("Levy senzor (Lidar): " + leftDistStr);

    // 3. Měření spodních IR senzorů čáry
    const lfl = robutek.readSensor('LineFL');
    const lfr = robutek.readSensor('LineFR');
    const lbl = robutek.readSensor('LineBL');
    const lbr = robutek.readSensor('LineBR');
    console.log(`Senzory čáry: FL=${lfl} | FR=${lfr} | BL=${lbl} | BR=${lbr}`);

    // Doplňkově RGB senzor (pokud je připojen)
    if (rgb != null) {
      try {
        const clear = rgb.readRawClear();
        const raw = rgb.readRawRGB();
        console.log(`RGB Senzor: Clear=${clear} | R=${raw[0]} | G=${raw[1]} | B=${raw[2]}`);
      } catch (e) {
        // ignore
      }
    }

    // 4. Měření Gyroskopu (Pouze úhel ve stupních)
    if (gyro != null) {
      console.log(`Gyroskop (Úhel): ${angleZ.toFixed(1)} °`);
    } else {
      console.log("Gyroskop: Nedostupný");
    }

    // 5. Ovládání a animace LED pásku na základě hodnot
    // LED 0, 1, 2, 3 odpovídají senzorům čáry
    // LED 4, 5 odpovídají přednímu dálkoměru (Predni senzor)
    // LED 6, 7 odpovídají levému dálkoměru (Levy senzor)
    leds.clear();
    
    // Mapování čáry (modrá intenzita, max 60 ze 255 pro rozumný jas)
    leds.set(0, Math.round((lfl / 4095) * 60)); 
    leds.set(1, Math.round((lfr / 4095) * 60));
    leds.set(2, Math.round((lbl / 4095) * 60));
    leds.set(3, Math.round((lbr / 4095) * 60));

    // Mapování předního Lidaru na LED 4, 5
    let frontLidarColor = BLUE;
    if (frontDistVal > 0) {
      if (frontDistVal < 300) {
        frontLidarColor = RED;
      } else if (frontDistVal < 600) {
        frontLidarColor = YELLOW;
      } else {
        frontLidarColor = GREEN;
      }
    }
    leds.set(4, frontLidarColor);
    leds.set(5, frontLidarColor);

    // Mapování levého Lidaru na LED 6, 7
    let leftLidarColor = BLUE;
    if (leftDistVal > 0) {
      if (leftDistVal < 300) {
        leftLidarColor = RED;
      } else if (leftDistVal < 600) {
        leftLidarColor = YELLOW;
      } else {
        leftLidarColor = GREEN;
      }
    }
    leds.set(6, leftLidarColor);
    leds.set(7, leftLidarColor);

    leds.show();

    await sleep(500); // Výpis každou půl sekundu
  }
}

// -------------------- MAIN --------------------
async function main(): Promise<void> {
  await initHardware();
  while (true) {
    await waitForStart();
    await runTest();
  }
}

main().catch(async (e) => {
  console.log("CHYBA V TEST PROGRAMU: " + e);
  setAllLeds(PURPLE);
});
