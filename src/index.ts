import { createRobutek } from "./libs/robutek.js";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { ZSCS2016C } from "./libs/zscs2016c.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";

// ======================================================
// AUTONOMNÍ PROGRAM ROBO CARTS 2026
// - Sledování levé stěny (vnitřní dráha při jízdě proti směru hodinových ručiček)
// - Detekce levých zatáček a zatáčení pomocí gyroskopu (MPU6050)
// - Hlídání překážek / čelní stěny předním dálkoměrem pro vyhýbání se
// - Počítání kol pomocí spodního RGB senzoru a cílové čáry
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
    // Probudit gyroskop: zapsat 0x00 do registru PWR_MGMT_1 (0x6B)
    this.i2c.writeTo(this.ad, [0x6B, 0x00]);
    // Nastavit low-pass filtr (DLPF) na 21 Hz: zapsat 0x04 do registru CONFIG (0x1A)
    this.i2c.writeTo(this.ad, [0x1A, 0x04]);
    // Nastavit rozsah gyroskopu na ±250 °/s: zapsat 0x00 do registru GYRO_CONFIG (0x1B)
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
let angleZ = 0;
let gyroZ_dps = 0;
let lastTime = 0;
let intervalId: number | null = null;
let emergencyLatched = false;
let driveStartTime = 0;


// -------------------- PARAMETRY JÍZDY --------------------
const SPEED_NORMAL = 270;
const SPEED_SLOW = 190;
const SPEED_TURN = 150;
const RAMP = 350;

// Když robot zatáčí opačně, změň na +1.
const CURVE_SIGN = -1;

// -------------------- CÍLOVÁ ČÁRA (RGB) --------------------
const RGB_ENABLE = true;
const BLACK_CLEAR_MAX = 180;
const BLACK_RGB_SUM_MAX = 420;
const LINE_DEBOUNCE_MS = 1200;
let lapCount = 0;
let lastLineTime = 0;

// Zde nastav cílový počet kol pro automatické zastavení (0 = nevypínat sám)
const TARGET_LAPS = 0; 

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
  await stopRobot();
  setAllLeds(RED);
  console.log("NOUZOVE STOP - uvolni IO17 a zmackni IO2 pro novy start");
}

async function sleepCheck(ms: number): Promise<boolean> {
  const end = Date.now() + ms;
  while (Date.now() < end) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      await emergencyStop();
      return false;
    }
    await sleep(2);
  }
  return true;
}

// Pomocník pro řízení s jednotným znaménkem zatáčení
// steer: kladné = doprava, záporné = doleva
function applySteering(speed: number, steer: number) {
  robutek.setSpeed(speed);
  robutek.move(CURVE_SIGN * steer);
}

// -------------------- RGB ČTENÍ ČÁRY --------------------
function readBlackLine(): boolean {
  if (!rgb) return false;

  try {
    const clear = rgb.readRawClear();
    const raw = rgb.readRawRGB();
    const sum = raw[0] + raw[1] + raw[2];

    return clear > 0 &&
           clear < BLACK_CLEAR_MAX &&
           sum > 0 &&
           sum < BLACK_RGB_SUM_MAX;
  } catch (e) {
    return false;
  }
}

async function checkFinishLine(): Promise<boolean> {
  if (!readBlackLine()) {
    return false;
  }

  const now = Date.now();
  if (now - lastLineTime < LINE_DEBOUNCE_MS) {
    return false;
  }

  lastLineTime = now;
  lapCount++;

  console.log("CÍLOVÁ ČÁRA DETEKOVÁNA! Kolo: " + lapCount);
  setAllLeds(CYAN);

  if (TARGET_LAPS > 0 && lapCount >= TARGET_LAPS) {
    console.log("DOSAŽEN CÍLOVÝ POČET KOL (" + TARGET_LAPS + "). Zastavuji robot.");
    await stopRobot();
    setAllLeds(PURPLE);
    return true;
  }

  return false;
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
  console.log("AUTONOMNÍ ROBO CARTS 2026 - INICIALIZACE");
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

  // RGB senzor
  if (RGB_ENABLE) {
    try {
      rgb = new ZSCS2016C(I2C2, false);
      rgb.enable();
      console.log("OK: RGB ZSCS2016C připojen.");
    } catch (e) {
      rgb = null;
      console.log("CHYBA RGB: " + e);
    }
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
            let gz_dps = (data.gyro.z - gyroZOffset) / 131.0;
            
            // Mrtvá zóna proti driftu v klidu (cca 0.85 °/s)
            if (Math.abs(gz_dps) < 0.85) {
              gz_dps = 0.0;
            }
            
            gyroZ_dps = gz_dps; // Uložíme pro D-složku PD regulátoru
            
            if (dt > 0 && dt < 0.2) {
              angleZ += gyroZ_dps * dt;
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

  setAllLeds(YELLOW); // Svítí žlutě, připraven ke startu
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
        lapCount = 0;
        lastLineTime = 0;
        angleZ = 0;
        lastTime = Date.now();
        setAllLeds(YELLOW);
        await sleep(300);
        return;
      }
    }
    await sleep(20);
  }
}

// -------------------- HLAVNÍ AUTONOMNÍ JÍZDA --------------------
async function drive(): Promise<void> {
  if (emergencyLatched) return;

  await stopRobot();
  setServoAngle(ANGLE_CENTER);
  await sleepCheck(200);

  driveStartTime = Date.now();
  setAllLeds(GREEN);

  // 1. FÁZE: Výjezd ze startovního boxu
  // Robot jede 200 ms rovně a udržuje původní směr podle gyroskopu.
  const startTargetAngle = angleZ;
  const departureStart = Date.now();
  console.log("Výjezd ze startovního boxu...");
  while (Date.now() - departureStart < 200) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      await emergencyStop();
      return;
    }
    const gyroError = angleZ - startTargetAngle;
    const steer = gyroError * 0.05; // P-regulace pro držení rovného směru
    applySteering(SPEED_NORMAL, steer);
    await sleep(5);
  }

  // 2. FÁZE: Autonomní smyčka řízení (State Machine)
  let state = "FOLLOW"; // Výchozí stav: Sledování levé stěny
  let leftTurnStartAngle = 0;
  let lastPrint = 0;

  while (!emergencyLatched) {
    if (isPressed(EMERGENCY_BUTTON_PIN)) {
      await emergencyStop();
      break;
    }

    // Přečteme dálkoměry
    let frontDist = 1200;
    if (lidar != null) {
      try {
        const m = await lidar.read();
        frontDist = m.distance;
      } catch (e) {}
    }

    let leftDist = 1200;
    if (leftLidar != null) {
      try {
        const m = await leftLidar.read();
        leftDist = m.distance;
      } catch (e) {}
    }

    // Kontrola cílové čáry
    if (await checkFinishLine()) {
      break;
    }

    // Rozhodování chování (State Machine)
    if (state === "FOLLOW") {
      // 1. Nouzové vyhnutí doprava, pokud je překážka vpředu moc blízko
      if (frontDist < 350) {
        state = "AVOID";
        setAllLeds(RED);
        console.log("STATE CHANGE: AVOID (Překážka vpředu: " + frontDist + " mm)");
      }
      // 2. Zatáčení doleva, pokud zmizí stěna po levé straně
      else if (leftDist > 550) {
        state = "LEFT_TURN";
        leftTurnStartAngle = angleZ;
        setAllLeds(CYAN);
        console.log("STATE CHANGE: LEFT_TURN (Levá stěna zmizela: " + leftDist + " mm)");
      }
      // 3. Sledování levé stěny
      else {
        // PD-regulátor pro udržování stěny
        // Kp reguluje vzdálenost (target = 220 mm), Kd tlumí otáčení pomocí gyroskopu (gyroZ_dps)
        const distError = leftDist - 220;
        const steer = distError * 0.003 - gyroZ_dps * 0.005;

        applySteering(SPEED_NORMAL, steer);
        setAllLeds(GREEN);
      }
    }
    else if (state === "LEFT_TURN") {
      // Zatáčíme plynule vlevo (steer = -0.7)
      applySteering(SPEED_TURN, -0.7);

      const turnAngle = Math.abs(angleZ - leftTurnStartAngle);

      // Zatáčení končí, pokud se otočíme o více než 80° nebo pokud se přiblížíme k levé stěně
      if (turnAngle > 80 || leftDist < 400) {
        state = "FOLLOW";
        setAllLeds(GREEN);
        console.log("STATE CHANGE: FOLLOW (Zatáčka dokončena, úhel: " + turnAngle.toFixed(1) + "°)");
      }
    }
    else if (state === "AVOID") {
      // Zatáčíme ostře doprava (steer = 0.8)
      applySteering(SPEED_TURN, 0.8);

      // Vyhýbání končí, když je před námi volná cesta
      if (frontDist > 550) {
        state = "FOLLOW";
        setAllLeds(GREEN);
        console.log("STATE CHANGE: FOLLOW (Překážka objetá, volno: " + frontDist + " mm)");
      }
    }

    // Diagnostický výpis
    if (Date.now() - lastPrint > 250) {
      lastPrint = Date.now();
      console.log(
        "AUTO_RUN" +
        " | state=" + state +
        " | L_Lidar=" + leftDist + " mm" +
        " | F_Lidar=" + frontDist + " mm" +
        " | Gyro_Angle=" + angleZ.toFixed(1) + " °" +
        " | Gyro_Rate=" + gyroZ_dps.toFixed(1) + " °/s" +
        " | lap=" + lapCount
      );
    }

    await sleep(10); // Smyčka běží na 100 Hz
  }

  await stopRobot();
}

// -------------------- MAIN --------------------
async function main(): Promise<void> {
  await initHardware();
  while (true) {
    await waitForStart();
    await drive();
  }
}

main().catch(async (e) => {
  console.log("HLAVNÍ CHYBA PROGRAMU: " + e);
  await stopRobot();
  setAllLeds(PURPLE);
});
