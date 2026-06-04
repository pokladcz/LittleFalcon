import { createRobutek } from "./libs/robutek.js";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
const robutek = createRobutek("V2");
// =======================================================
// ROBOCARTS 2026 - ADVANCED AUTONOMNI PROGRAM
// =======================================================
// Strategie:
// - zavod se jede proti smeru hodinovych rucicek,
// - robot drzi LEVOU stenu / levy okraj trate,
// - predni lidar na servu hlida zatacky a prekazky,
// - cerna cilova cara se pocita senzory cary,
// - IO2 = start / reset po nouzovem stopu,
// - IO17 = nouzove stop, po stisku zustane program zastaveny.
//
// Zapojeni podle tve varianty:
// - levy VL53L0X: hlavni I2C konektor robutka SDA/SCL
// - predni VL53L0X na servu: I2C2, IO15 = SDA, IO16 = SCL
// - servo s prednim lidarem: IO21
// - RGB pasek: IO36
// - start: IO2
// - nouzove stop: IO17
//
// DULEZITE LADENI:
// 1) Kdyz robot zataci na spatnou stranu, zmen DRIVE_CURVE_SIGN z 1 na -1.
// 2) Kdyz se lepi na levou stenu, zvetsi TARGET_LEFT_MM nebo zmens LEFT_KP.
// 3) Kdyz je moc pomaly, zvetsi SPEED_NORMAL postupne po 20.
// =======================================================
// PINY
// =======================================================
const START_BUTTON_PIN = 2;
const EMERGENCY_BUTTON_PIN = 17;
const FRONT_I2C_SDA = 15;
const FRONT_I2C_SCL = 16;
const SERVO_PIN = 21;
const LED_PIN = 36;
const LED_COUNT = 8;
// =======================================================
// RYCHLOSTI A RIZENI
// =======================================================
const SPEED_NORMAL = 300;
const SPEED_CURVE = 240;
const SPEED_DANGER = 170;
const SPEED_LOST = 200;
const RAMP = 700;
// Pokud se robot po zapnuti senzoru zase bude tocit opacne, zmen na -1.
const DRIVE_CURVE_SIGN = 1;
// Max zatoceni v normalni jizde. Drzime male, aby robot neudelal piruetu.
const MAX_CURVE_NORMAL = 0.24;
const MAX_CURVE_DANGER = 0.42;
// =======================================================
// LEVY LIDAR - JIZDA PODLE LEVE STENY
// =======================================================
const TARGET_LEFT_MM = 155;
const LEFT_MIN_MM = 45;
const LEFT_MAX_MM = 430;
const LEFT_DEADBAND_MM = 18;
// PD regulator. Kp resi vzdalenost, Kd tlumi kmitani.
const LEFT_KP = 0.0018;
const LEFT_KD = 0.0010;
const LEFT_MAX_CORRECTION = 0.20;
// Pri ztrate leve steny robot nebude panikarit, jen mirne hleda doleva.
const LOST_LEFT_SEARCH_CURVE = -0.10;
// =======================================================
// PREDNI LIDAR NA SERVU
// =======================================================
const SERVO_REVERSE = false;
const SERVO_LEFT_ANGLE = 55;
const SERVO_CENTER_ANGLE = 90;
const SERVO_RIGHT_ANGLE = 125;
const SERVO_SETTLE_MS = 22;
// Predni lidar je na drzaku 84 mm pred stredem robota.
// Proto brzdi driv, aby robot nenarazil cumakem.
const LIDAR_FRONT_OFFSET_MM = 84;
const FRONT_WARN_MM = 330;
const FRONT_DANGER_MM = 190;
const FRONT_PANIC_MM = 115;
const FRONT_VALID_MIN_MM = 25;
const FRONT_VALID_MAX_MM = 1000;
// =======================================================
// CILOVA CARA / KOLA
// =======================================================
// 0 = nepocitat kola a jet porad, dokud nezmacknes stop.
// Dej sem pocet kol, ktery rekne rozhodci, napr. 3.
const TARGET_LAPS = 0;
// Cerna cara byva na senzorech mensi hodnota nez bila.
// Kdyz program pocita spatne, vypis v monitoru ti ukaze LineFL/FR/BL/BR.
const BLACK_LINE_THRESHOLD = 650;
const LINE_ARM_AFTER_START_MS = 3000;
const LINE_DEBOUNCE_MS = 1400;
// =======================================================
// VYPIS
// =======================================================
const PRINT_MS = 250;
// =======================================================
// LED BARVY
// =======================================================
const OFF = 0x000000;
const BLUE = 0x000030;
const GREEN = 0x003000;
const RED = 0x300000;
const YELLOW = 0x303000;
const PURPLE = 0x300030;
const WHITE = 0x202020;
const leds = new SmartLed(LED_PIN, LED_COUNT, LED_WS2812B);
const servo = new Servo(SERVO_PIN, 1, 4);
let leftSensor = null;
let frontSensor = null;
let emergencyLatched = false;
let lastLeft = -1;
let prevLeftError = 0;
let lastFront = -1;
let frontLeft = -1;
let frontCenter = -1;
let frontRight = -1;
let servoAngle = SERVO_CENTER_ANGLE;
let lineFL = 1023;
let lineFR = 1023;
let laps = 0;
let lastLineMs = 0;
let driveStartMs = 0;
function clamp(x, min, max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}
function abs(x) {
    return x < 0 ? -x : x;
}
function isPressed(pin) {
    return gpio.read(pin) == 0;
}
function isValidLeftDistance(x) {
    return x >= LEFT_MIN_MM && x <= LEFT_MAX_MM;
}
function isValidFrontDistance(x) {
    return x >= FRONT_VALID_MIN_MM && x <= FRONT_VALID_MAX_MM;
}
function setAllLeds(color) {
    for (let i = 0; i < LED_COUNT; i++)
        leds.set(i, color);
    leds.show();
}
function showLaps() {
    leds.clear();
    const count = TARGET_LAPS > 0 ? clamp(laps, 0, LED_COUNT) : 1;
    for (let i = 0; i < count; i++)
        leds.set(i, GREEN);
    leds.show();
}
function servoValue(angle) {
    angle = clamp(angle, 0, 180);
    if (SERVO_REVERSE)
        angle = 180 - angle;
    return Math.round((angle / 180) * 1023);
}
function setServoAngle(angle) {
    servoAngle = angle;
    servo.write(servoValue(angle));
}
async function stopRobot() {
    try {
        await robutek.stop(true);
    }
    catch (e) {
        console.log("STOP chyba: " + e);
    }
    try {
        robutek.setSpeed(0);
    }
    catch (e) {
        console.log("setSpeed stop chyba: " + e);
    }
    setServoAngle(SERVO_CENTER_ANGLE);
}
async function emergencyStop() {
    emergencyLatched = true;
    await stopRobot();
    setAllLeds(RED);
    console.log("NOUZOVE STOP - pohony vypnute. Uvolni IO17 a zmackni IO2 pro novy start.");
}
async function sleepCheck(ms) {
    const end = Date.now() + ms;
    while (Date.now() < end) {
        if (isPressed(EMERGENCY_BUTTON_PIN)) {
            await emergencyStop();
            return false;
        }
        await sleep(5);
    }
    return true;
}
async function readDistance(sensor) {
    if (sensor == null)
        return -1;
    if (isPressed(EMERGENCY_BUTTON_PIN))
        return -1;
    try {
        const m = await sensor.read();
        return m.distance;
    }
    catch (e) {
        return -1;
    }
}
function readLineSensors() {
    try {
        lineFL = robutek.readSensor("LineFL");
    }
    catch (e) {
        lineFL = 1023;
    }
    try {
        lineFR = robutek.readSensor("LineFR");
    }
    catch (e) {
        lineFR = 1023;
    }
}
function seesFinishLine() {
    // Pouzivame dokumentovane predni senzory LineFL a LineFR.
    // Cilova cara je napric trati, takze ji pri prejezdu vetsinou uvidi oba.
    return lineFL < BLACK_LINE_THRESHOLD && lineFR < BLACK_LINE_THRESHOLD;
}
function checkLapCounter() {
    if (TARGET_LAPS <= 0)
        return false;
    const now = Date.now();
    if (now - driveStartMs < LINE_ARM_AFTER_START_MS)
        return false;
    if (now - lastLineMs < LINE_DEBOUNCE_MS)
        return false;
    if (!seesFinishLine())
        return false;
    laps++;
    lastLineMs = now;
    showLaps();
    console.log("CILOVA CARA - kolo " + laps + " / " + TARGET_LAPS);
    return laps >= TARGET_LAPS;
}
async function initHardware() {
    gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.on("falling", EMERGENCY_BUTTON_PIN, () => {
        emergencyLatched = true;
    });
    setAllLeds(WHITE);
    await stopRobot();
    robutek.setRamp(RAMP);
    robutek.setSpeed(0);
    setServoAngle(SERVO_CENTER_ANGLE);
    console.log("======================================");
    console.log("ROBOCARTS 2026 - AUTONOMNI LEVA STENA");
    console.log("IO2 = start / reset, IO17 = nouzove stop");
    console.log("TARGET_LAPS=" + TARGET_LAPS + " (0 znamena bez automatickeho konce)");
    console.log("DRIVE_CURVE_SIGN=" + DRIVE_CURVE_SIGN);
    console.log("======================================");
    try {
        I2C1.setup({ sda: robutek.Pins.SDA, scl: robutek.Pins.SCL, bitrate: 400000 });
        leftSensor = new VL53L0X(I2C1);
        console.log("OK levy lidar na hlavnim I2C");
    }
    catch (e) {
        leftSensor = null;
        console.log("ERR levy lidar: " + e);
    }
    try {
        I2C2.setup({ sda: FRONT_I2C_SDA, scl: FRONT_I2C_SCL, bitrate: 400000 });
        frontSensor = new VL53L0X(I2C2);
        console.log("OK predni lidar na I2C2 IO15/IO16");
    }
    catch (e) {
        frontSensor = null;
        console.log("ERR predni lidar: " + e);
    }
    readLineSensors();
    console.log("LineFL=" + lineFL + " LineFR=" + lineFR);
    setAllLeds(BLUE);
}
async function waitForStart() {
    await stopRobot();
    while (true) {
        if (emergencyLatched) {
            setAllLeds(RED);
            if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
                await sleep(120);
                if (!isPressed(EMERGENCY_BUTTON_PIN) && isPressed(START_BUTTON_PIN)) {
                    emergencyLatched = false;
                    console.log("NOUZOVE STOP RESETOVANO");
                    setAllLeds(YELLOW);
                    await sleep(400);
                    return;
                }
            }
            await sleep(20);
            continue;
        }
        setAllLeds(BLUE);
        if (isPressed(EMERGENCY_BUTTON_PIN)) {
            await emergencyStop();
            continue;
        }
        if (isPressed(START_BUTTON_PIN)) {
            await sleep(120);
            if (isPressed(START_BUTTON_PIN)) {
                console.log("START - cekam 0.5s a jedu");
                setAllLeds(YELLOW);
                await sleep(500);
                return;
            }
        }
        await sleep(20);
    }
}
async function scanFront() {
    setServoAngle(SERVO_LEFT_ANGLE);
    if (!(await sleepCheck(SERVO_SETTLE_MS)))
        return false;
    frontLeft = await readDistance(frontSensor);
    setServoAngle(SERVO_CENTER_ANGLE);
    if (!(await sleepCheck(SERVO_SETTLE_MS)))
        return false;
    frontCenter = await readDistance(frontSensor);
    setServoAngle(SERVO_RIGHT_ANGLE);
    if (!(await sleepCheck(SERVO_SETTLE_MS)))
        return false;
    frontRight = await readDistance(frontSensor);
    setServoAngle(SERVO_CENTER_ANGLE);
    lastFront = frontCenter;
    return true;
}
function calcLeftWallCurve() {
    if (!isValidLeftDistance(lastLeft)) {
        prevLeftError = 0;
        return { curve: LOST_LEFT_SEARCH_CURVE, mode: "LEFT_LOST_SEARCH" };
    }
    const error = lastLeft - TARGET_LEFT_MM;
    if (abs(error) <= LEFT_DEADBAND_MM) {
        prevLeftError = error;
        return { curve: 0, mode: "LEFT_OK" };
    }
    const derivative = error - prevLeftError;
    prevLeftError = error;
    // error < 0: jsme moc blizko leve steny => zatoc doprava (+)
    // error > 0: jsme moc daleko od leve steny => zatoc doleva (-)
    let curve = -(error * LEFT_KP) - (derivative * LEFT_KD);
    curve = clamp(curve, -LEFT_MAX_CORRECTION, LEFT_MAX_CORRECTION);
    return {
        curve: curve,
        mode: error < 0 ? "LEFT_TOO_CLOSE_RIGHT" : "LEFT_TOO_FAR_LEFT"
    };
}
function nearestFront() {
    let n = 9999;
    if (isValidFrontDistance(frontLeft) && frontLeft < n)
        n = frontLeft;
    if (isValidFrontDistance(frontCenter) && frontCenter < n)
        n = frontCenter;
    if (isValidFrontDistance(frontRight) && frontRight < n)
        n = frontRight;
    return n == 9999 ? -1 : n;
}
function calcDrive() {
    const left = calcLeftWallCurve();
    let curve = left.curve;
    let speed = SPEED_NORMAL;
    let mode = left.mode;
    const nf = nearestFront();
    // Upozorneni: prictu posun cidla dopredu. Kdyz lidar ukazuje 190 mm,
    // predni hrana robota muze byt realne o 84 mm bliz ke zdi.
    const danger = FRONT_DANGER_MM + LIDAR_FRONT_OFFSET_MM;
    const panic = FRONT_PANIC_MM + LIDAR_FRONT_OFFSET_MM;
    if (isValidFrontDistance(nf) && nf < FRONT_WARN_MM) {
        speed = SPEED_CURVE;
        // Kdyz je vepredu zed, jedeme proti smeru hodinovych rucicek:
        // preferujeme levou zatacku, ale kdyz je vlevo moc blizko, uhneme doprava.
        const leftBlocked = isValidFrontDistance(frontLeft) && frontLeft < danger;
        const centerBlocked = isValidFrontDistance(frontCenter) && frontCenter < danger;
        const rightBlocked = isValidFrontDistance(frontRight) && frontRight < danger;
        if (centerBlocked || nf < danger) {
            speed = SPEED_DANGER;
            if (!leftBlocked && (rightBlocked || frontLeft > frontRight)) {
                curve += -0.28; // vlevo
                mode = "FRONT_TURN_LEFT";
            }
            else {
                curve += 0.24; // doprava, kdyz je leva strana blokovana
                mode = "FRONT_AVOID_RIGHT";
            }
        }
        else {
            // Jemne vyhybani podle toho, na ktere strane je prekazka.
            if (frontLeft > 0 && frontLeft < FRONT_WARN_MM)
                curve += 0.10;
            if (frontRight > 0 && frontRight < FRONT_WARN_MM)
                curve += -0.10;
            mode = "FRONT_WARN";
        }
    }
    if (isValidFrontDistance(nf) && nf < panic) {
        speed = SPEED_DANGER;
        // Nicit se o zed nema cenu: kratka silnejsi zatacka misto toceni na miste.
        curve = isValidLeftDistance(lastLeft) && lastLeft < TARGET_LEFT_MM ? 0.38 : -0.38;
        mode = "FRONT_PANIC";
    }
    if (mode == "LEFT_LOST_SEARCH") {
        speed = SPEED_LOST;
    }
    const maxCurve = mode == "FRONT_PANIC" ? MAX_CURVE_DANGER : MAX_CURVE_NORMAL;
    curve = clamp(curve, -maxCurve, maxCurve);
    return { curve: curve, speed: speed, mode: mode };
}
async function driveRace() {
    let lastPrint = 0;
    laps = 0;
    lastLineMs = 0;
    driveStartMs = Date.now();
    prevLeftError = 0;
    lastLeft = -1;
    frontLeft = -1;
    frontCenter = -1;
    frontRight = -1;
    setAllLeds(GREEN);
    robutek.setRamp(RAMP);
    robutek.setSpeed(SPEED_NORMAL);
    robutek.move(0);
    console.log("JEDU - leva stena + predni scan + cilova cara");
    while (!emergencyLatched) {
        if (isPressed(EMERGENCY_BUTTON_PIN)) {
            await emergencyStop();
            break;
        }
        lastLeft = await readDistance(leftSensor);
        if (!(await scanFront()))
            break;
        readLineSensors();
        if (checkLapCounter()) {
            console.log("HOTOVO - dosazen pocet kol, zastavuji");
            break;
        }
        const d = calcDrive();
        robutek.setSpeed(d.speed);
        robutek.move(DRIVE_CURVE_SIGN * d.curve);
        if (Date.now() - lastPrint > PRINT_MS) {
            lastPrint = Date.now();
            console.log("RUN mode=" + d.mode +
                " speed=" + d.speed +
                " curve=" + d.curve.toFixed(3) +
                " L=" + lastLeft +
                " F[L/C/R]=" + frontLeft + "/" + frontCenter + "/" + frontRight +
                " line=" + lineFL + "/" + lineFR +
                " laps=" + laps);
        }
        if (!(await sleepCheck(8)))
            break;
    }
    await stopRobot();
}
async function main() {
    await initHardware();
    while (true) {
        await waitForStart();
        await driveRace();
    }
}
main().catch(async (e) => {
    console.log("CHYBA PROGRAMU: " + e);
    await stopRobot();
    setAllLeds(PURPLE);
});
