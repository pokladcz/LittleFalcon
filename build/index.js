import { createRobutek } from "./libs/robutek.js";
import { I2C2 } from "i2c";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
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
    i2c;
    ad;
    constructor(i2c) {
        this.i2c = i2c;
        this.ad = 0x68;
    }
    probe() {
        for (const addr of [0x68, 0x69]) {
            try {
                this.i2c.writeTo(addr, 0x75);
                const id = this.i2c.readFrom(addr, 1)[0];
                console.log("MPU6050: WHO_AM_I na adrese 0x" + addr.toString(16) + " vrátil 0x" + id.toString(16));
                this.ad = addr;
                return true;
            }
            catch (e) {
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
        const toInt16 = (high, low) => {
            let val = (high << 8) | low;
            if (val & 0x8000)
                val -= 0x10000;
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
let gyro = null;
let gyroZOffset = 0;
let angleZ = 0;
let lastTime = 0;
let intervalId = null;
let emergencyLatched = false;
// -------------------- PARAMETRY JÍZDY --------------------
const SPEED_NORMAL = 270;
const RAMP = 350;
// Pokud robot při zatáčení uhýbá na špatnou stranu, změň na +1.
const CURVE_SIGN = -1;
// -------------------- POMOCNÉ FUNKCE --------------------
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
function isPressed(pin) {
    return gpio.read(pin) == 0;
}
function setAllLeds(color) {
    leds.clear();
    for (let i = 0; i < LED_COUNT; i++) {
        leds.set(i, color);
    }
    leds.show();
}
function setServoAngle(angle) {
    servo.write(Math.round((angle / 180) * 1023));
}
async function stopRobot() {
    try {
        await robutek.stop(true);
    }
    catch (e) {
        console.log("stop chyba: " + e);
    }
    try {
        robutek.setSpeed(0);
    }
    catch (e) {
        console.log("setSpeed chyba: " + e);
    }
}
async function emergencyStop() {
    emergencyLatched = true;
    await stopRobot();
    setAllLeds(RED);
    console.log("NOUZOVE STOP - uvolni IO17 a zmackni IO2 pro novy start");
}
function applySteering(speed, steer) {
    robutek.setSpeed(speed);
    robutek.move(CURVE_SIGN * steer);
}
// -------------------- INICIALIZACE HW --------------------
async function initHardware() {
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
    }
    catch (e) {
        console.log("CHYBA I2C2: " + e);
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
                        // Mrtvá zóna
                        if (Math.abs(gz_dps) < 0.85) {
                            gz_dps = 0.0;
                        }
                        if (dt > 0 && dt < 0.2) {
                            angleZ += gz_dps * dt;
                        }
                    }
                    catch (e) {
                        // ignorovat
                    }
                }
            }, 10);
        }
        else {
            gyro = null;
            console.log("CHYBA: Gyroskop MPU6050 nebyl nalezen na I2C2.");
        }
    }
    catch (e) {
        gyro = null;
        console.log("CHYBA Gyroskop: " + e);
    }
    setAllLeds(YELLOW); // Připraven ke startu
}
// -------------------- ČEKÁNÍ NA START --------------------
async function waitForStart() {
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
// -------------------- JÍZDA ROVNĚ 1 METR --------------------
async function driveStraight() {
    if (emergencyLatched)
        return;
    await stopRobot();
    setServoAngle(ANGLE_CENTER);
    await sleep(100);
    // Načteme počáteční pozice enkodérů
    const startLeft = robutek.leftMotor.getPosition();
    const startRight = robutek.rightMotor.getPosition();
    // Uzamkneme cílový směr (0 stupňů)
    const targetAngle = angleZ;
    console.log(`Start jízdy rovně na 1 metr. Cílový úhel: ${targetAngle.toFixed(1)} °`);
    setAllLeds(GREEN);
    while (!emergencyLatched) {
        if (isPressed(EMERGENCY_BUTTON_PIN)) {
            await emergencyStop();
            break;
        }
        // Spočítáme ujetou vzdálenost (průměr obou kol)
        const currentLeft = robutek.leftMotor.getPosition();
        const currentRight = robutek.rightMotor.getPosition();
        const distLeft = currentLeft - startLeft;
        const distRight = currentRight - startRight;
        const distTraveled = (distLeft + distRight) / 2; // v mm
        console.log(`Ujeto: ${distTraveled.toFixed(0)} mm | Úhel: ${angleZ.toFixed(1)} °`);
        // Pokud ujedeme 1000 mm (1 metr), zastavíme
        if (distTraveled >= 1000) {
            console.log("Cílová vzdálenost 1m dosažena. Zastavuji robot.");
            break;
        }
        // Regulace směru podle gyroskopu (P-regulátor)
        const error = angleZ - targetAngle;
        const steer = error * 0.05; // zisk regulátoru (lze upravit, např. 0.03 - 0.08)
        applySteering(SPEED_NORMAL, steer);
        await sleep(10);
    }
    await stopRobot();
    setAllLeds(PURPLE); // Hotovo
    await sleep(1000);
}
// -------------------- MAIN --------------------
async function main() {
    await initHardware();
    while (true) {
        await waitForStart();
        await driveStraight();
    }
}
main().catch(async (e) => {
    console.log("HLAVNÍ CHYBA PROGRAMU: " + e);
    await stopRobot();
    setAllLeds(PURPLE);
});
