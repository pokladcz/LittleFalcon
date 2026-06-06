import { createRobutek } from "./libs/robutek.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { Servo } from "./libs/servo.js";
import { driveStraight, driveArc } from "./libs/drive.js";

const robutek = createRobutek("V2");

// Piny a Konstanty
const START_BUTTON_PIN = 2;         // Levé tlačítko (IO02) - Spouští kalibraci
const EMERGENCY_BUTTON_PIN = 17;    // Nouzové stop tlačítko
const I2C2_SDA = 15;
const I2C2_SCL = 16;
const SERVO_PIN = 21;
const LED_PIN = 36;
const LED_COUNT = 8;
const ANGLE_CENTER = 90;

const leds = new SmartLed(LED_PIN, LED_COUNT, LED_WS2812B);
const servo = new Servo(SERVO_PIN, 1, 4);

// Barvy
const RED = 0x200000, GREEN = 0x002000, BLUE = 0x000020, PURPLE = 0x200020;

const sleep = (ms: number) => new Promise<void>(res => setTimeout(() => res(), ms));

async function stopRobot() {
  try { await robutek.stop(true); } catch (e) {}
  try { robutek.setSpeed(0); } catch (e) {}
}

const setServoAngle = (ang: number) => servo.write(Math.round((ang / 180) * 1023));

function setAllLeds(color: number) {
  leds.clear();
  for (let i = 0; i < LED_COUNT; i++) leds.set(i, color);
  leds.show();
}

// Nouzový stop
let emergencyLatched = false;
async function emergencyStop() {
  if (emergencyLatched) return;
  emergencyLatched = true;
  console.log("!!! NOUZOVÝ STOP !!!");
  await stopRobot();
  setAllLeds(RED);
  try { exit(1); } catch (e) {}
  throw new Error("Emergency stop");
}

// MPU6050 Ovladač
class MPU6050 {
  constructor(private i2c: any, public ad = 0x68) {}
  probe() {
    try { this.i2c.writeTo(this.ad, 0x75); return this.i2c.readFrom(this.ad, 1)[0] === 0x68 || true; } catch (e) { return false; }
  }
  init() {
    this.i2c.writeTo(this.ad, [0x6B, 0x00]);
    this.i2c.writeTo(this.ad, [0x1A, 0x04]);
  }
  read() {
    const d = this.i2c.writeRead(this.ad, 0x3B, 14);
    const val = (h: number, l: number) => { let v = (h << 8) | l; return v & 0x8000 ? v - 0x10000 : v; };
    return {
      accel: { x: val(d[0], d[1]), y: val(d[2], d[3]), z: val(d[4], d[5]) },
      gyro: { z: val(d[12], d[13]) }
    };
  }
}

let gyro: MPU6050 | null = null;
let gyroZOffset = 0;
const angleState = { angleZ: 0 };
let lidar: VL53L0X | null = null, leftLidar: VL53L0X | null = null;
let latestFront = 9999, latestLeft = 9999, sensorUpdateCount = 0;

// ==========================================
// 🤖 AUTONOMNÍ REŽIM: LIDAR & GYRO
// ==========================================

const SPEED_NORMAL = 350, SPEED_SLOW = 175, SPEED_ARC = 150;
const FRONT_SLOW_DIST = 500, FRONT_DANGER_DIST = 350, LEFT_WALL_THRESHOLD = 500;
const RADIUS_LEFT = 105, RADIUS_RIGHT = 110;

async function getDistance(sensor: VL53L0X | null) {
  if (!sensor) return 9999;
  try {
    const m = await sensor.read();
    return m.distance < 30 || m.distance > 2000 ? 9999 : m.distance;
  } catch (e) { return 9999; }
}

function filterDistance(raw: number, currentAccepted: number, lowCountState: { count: number }): number {
  if (raw >= currentAccepted - 150) {
    lowCountState.count = 0;
    return raw;
  } else {
    lowCountState.count++;
    if (lowCountState.count >= 2) {
      lowCountState.count = 0;
      return raw;
    }
    return currentAccepted;
  }
}

let showSensorLeds = false;

async function startLidarLoop() {
  console.log("=== LIDAR LOOP START ===");
  const frontState = { count: 0 };
  const leftState = { count: 0 };
  while (!emergencyLatched) {
    try {
      const rawLeft = await getDistance(leftLidar);
      const rawFront = await getDistance(lidar);
      
      latestLeft = filterDistance(rawLeft, latestLeft, leftState);
      latestFront = filterDistance(rawFront, latestFront, frontState);
      
      sensorUpdateCount++;
      
      if (showSensorLeds) {
        leds.set(0, latestFront <= FRONT_DANGER_DIST ? RED : GREEN);
        leds.set(1, latestLeft <= LEFT_WALL_THRESHOLD ? RED : GREEN);
        leds.show();
      }
    } catch (e) {}
    await sleep(10);
  }
}

async function autonomniJizda() {
  console.log("=== AUTONOMNÍ START (Stop-and-Go) ===");
  let lastLogTime = 0;
  
  while (!emergencyLatched) {
    const left = latestLeft;
    const front = latestFront;
    
    // Vypisování hodnot senzorů každých 200 ms pro přehlednost za jízdy
    const now = Date.now();
    if (now - lastLogTime > 200) {
      console.log(`[Jízda] Přední: ${front.toFixed(0)} mm | Levý: ${left.toFixed(0)} mm`);
      lastLogTime = now;
    }

    if (front > 350) {
      // Jízda rovně s regulací odstupu 200 mm od levé stěny
      const spd = front < FRONT_SLOW_DIST ? SPEED_SLOW : SPEED_NORMAL;
      await driveStraight(robutek, gyro, gyroZOffset, 5000, spd, EMERGENCY_BUTTON_PIN, angleState, leds, emergencyStop, () => emergencyLatched, async () => {
        // Zastavíme jízdu rovně, pokud se předek zablokuje pod 350 mm
        return latestFront <= 350;
      }, () => latestLeft, true);
      
    } else {
      // PŘEKÁŽKA PŘED ROBOTEM (front <= 350) -> ZASTAVENÍ A ROZHODOVÁNÍ
      console.log(`[Překážka] Předek zablokován: ${front.toFixed(0)} mm (<= 350 mm). Zastavuji na 1 sekundu...`);
      await stopRobot();
      await sleep(1000); // Zastavení na 1 sekundu
      
      // Načteme aktuální čerstvé hodnoty z čidel po zastavení
      const freshLeft = latestLeft;
      console.log(`[Rozhodování] Levý senzor: ${freshLeft.toFixed(0)} mm`);
      
      if (freshLeft > 500) {
        console.log(`[Zatáčení VLEVO] Levý senzor > 500 mm (${freshLeft.toFixed(0)} mm). Zatáčím vlevo o 90°.`);
        await driveArc(robutek, angleState, RADIUS_LEFT, 90, SPEED_ARC, EMERGENCY_BUTTON_PIN, leds, emergencyStop, () => emergencyLatched);
      } else {
        console.log(`[Zatáčení VPRAVO] Levý senzor <= 500 mm (${freshLeft.toFixed(0)} mm). Zatáčím vpravo o 90°.`);
        await driveArc(robutek, angleState, RADIUS_RIGHT, -90, SPEED_ARC, EMERGENCY_BUTTON_PIN, leds, emergencyStop, () => emergencyLatched);
      }
      
      // Po dokončení zatáčky na chvíli zabrzdíme a vynulujeme senzory pro plynulý start
      await stopRobot();
      latestFront = 9999;
      latestLeft = 200;
      await sleep(50);
    }
    await sleep(10);
  }
}

// ==========================================
// 🏁 MAIN (AUTONOMNÍ)
// ==========================================
async function main() {
  setAllLeds(0); // LED zhasnuté při zapnutí
  await stopRobot();
  setServoAngle(ANGLE_CENTER);

  try {
    gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.on("falling", EMERGENCY_BUTTON_PIN, emergencyStop);
    gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
  } catch (e) {}

  setInterval(() => { if (gpio.read(EMERGENCY_BUTTON_PIN) === 0) { emergencyStop(); } }, 50);

  // Inicializace I2C2 a gyroskopu
  try {
    I2C2.setup({ sda: I2C2_SDA, scl: I2C2_SCL, bitrate: 400000 });
    gyro = new MPU6050(I2C2);
    if (gyro.probe()) gyro.init();
    else gyro = null;
  } catch (e) { gyro = null; }

  // Inicializace Lidarů
  try { lidar = new VL53L0X(I2C2); } catch (e) {}
  try { leftLidar = new VL53L0X(I2C1); } catch (e) {}

  startLidarLoop();
  setServoAngle(ANGLE_CENTER);

  console.log("=== ČEKÁM NA STISK TLAČÍTKA IO02 PRO START KALIBRACE ===");
  let lastLogTime = 0;
  while (true) {
    if (gpio.read(START_BUTTON_PIN) === 0) {
      await sleep(150); // Debounce
      if (gpio.read(START_BUTTON_PIN) === 0) {
        break; // Stisknuto -> spouštíme kalibraci
      }
    }
    
    // Vypisování hodnot předního a bočního (levého) senzoru před kalibrací každých 200 ms
    const now = Date.now();
    if (now - lastLogTime > 200) {
      console.log(`[Před kalibrací] Přední: ${latestFront.toFixed(0)} mm | Levý: ${latestLeft.toFixed(0)} mm`);
      lastLogTime = now;
    }
    await sleep(20);
  }

  // Kalibrace gyroskopu (svítí fialově)
  console.log("KALIBRACE GYRA (NEHÝBEJTE S ROBOTEM)...");
  setAllLeds(PURPLE);
  if (gyro) {
    let sum = 0;
    for (let i = 0; i < 50; i++) { sum += gyro.read().gyro.z; await sleep(5); }
    gyroZOffset = sum / 50;
    
    let lastTime = Date.now();
    setInterval(() => {
      if (gyro) {
        try {
          const d = gyro.read();
          const dt = (Date.now() - lastTime) / 1000.0;
          lastTime = Date.now();
          let gz_dps = (d.gyro.z - gyroZOffset) / 131.0;
          if (Math.abs(gz_dps) < 0.3) gz_dps = 0.0;
          if (dt > 0 && dt < 0.2) angleState.angleZ += gz_dps * dt;
        } catch (e) {}
      }
    }, 10);
  } else {
    console.log("Varování: Gyroskop nenalezen.");
    await sleep(500);
  }

  // Po dokončení kalibrace zhasneme LED
  setAllLeds(0);
  console.log("KALIBRACE DOKONČENA. LED ZHASNUTY.");
  await sleep(200);

  // Čekání na volno před robotem (např. zvednutí startovací brány)
  console.log("=== ČEKÁM NA VOLNO PŘED ROBOTEM PRO START ===");
  while (latestFront <= FRONT_DANGER_DIST && !emergencyLatched) {
    await sleep(20);
  }
  console.log("=== START AUTONOMNÍ JÍZDY! ===");
  showSensorLeds = true; // Zapneme zobrazování senzorů na LED 0 a 1

  robutek.setRamp(3000);

  await autonomniJizda();
}

main().catch(async (e) => {
  console.log("CHYBA: " + e);
  await stopRobot();
  setAllLeds(PURPLE);
});
