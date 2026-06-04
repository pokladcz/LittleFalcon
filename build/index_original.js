import { createRobutek } from "./libs/robutek.js";
import { I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { ZSCS2016C } from "./libs/zscs2016c.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
// ======================================================
// ROBOCARTS - NOVY CISTY PROGRAM
// Pouze:
// 1x VL53L0X na servu
// spodní RGB senzor na černou cílovou čáru
// START tlačítko IO2
// NOUZOVE tlačítko IO17
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
// POZOR:
// GPIO36 na ESP32 často neumí výstup.
// Pokud LED nesvítí, dej LED pásek na jiný výstupní pin.
// -------------------- SERVO --------------------
// Když se servo dívá opačně, změň na true.
const SERVO_REVERSE = false;
// Úhly možná bude potřeba doladit podle reálného držáku.
const ANGLE_LEFT = 45;
const ANGLE_CENTER = 90;
const ANGLE_RIGHT = 135;
// -------------------- JIZDA --------------------
// Když robot při move(0) nejede rovně:
// zatáčí doleva  -> dej třeba +0.08
// zatáčí doprava -> dej třeba -0.08
const MOTOR_TRIM = 0.00;
// Když robot zatáčí opačně, změň na -1.
const CURVE_SIGN = -1;
// SILNEJSI ZATACENI:
// snizena rychlost v zatacce, mensi RAMP a vyssi zisk korekce.
// Kdyby robot zatacel na spatnou stranu, zmen pouze CURVE_SIGN na opacne znamenko.
const SPEED_START = 260;
const SPEED_NORMAL = 270;
const SPEED_SLOW = 190;
const SPEED_TURN = 150;
const RAMP = 350;
// Po startu robot hned jede podle laseru na servu.
// 0 = zadna rovna faze bez rozhodovani.
const START_STRAIGHT_MS = 0;
// -------------------- LIDAR --------------------
const SAMPLES_PER_SIDE = 5;
const SAMPLE_DELAY_MS = 25;
const SERVO_SETTLE_MS = 160;
const DIST_MIN_VALID = 30;
const DIST_MAX_VALID = 1200;
const DIST_NO_READ = 1200;
// Odpočet chyby měření.
const DIST_ERROR_SUBTRACT = 30;
// -------------------- ROZHODOVANI --------------------
const FRONT_DANGER = 320;
const FRONT_TURN = 650;
const FRONT_SLOW = 850;
const SIDE_TOO_CLOSE = 300;
const SIDE_NEAR = 620;
// -------------------- POMEROVE RIZENI PODLE LIDARU --------------------
// Zataceni se pocita SYMETRICKY podle pomeru vzdalenosti vlevo/vpravo:
// ratio = (prava - leva) / (prava + leva)
// ratio > 0  => vice mista vpravo  => robot zataci doprava
// ratio < 0  => vice mista vlevo   => robot zataci doleva
// ratio = 0  => strany jsou stejne => robot jede rovne
//
// Kdyz robot zataci malo, zvys RATIO_CURVE_GAIN nebo MAX_NORMAL_CURVE.
// Kdyz kmitá ze strany na stranu, sniz RATIO_CURVE_GAIN.
const RATIO_DEADBAND = 0.08;
const RATIO_CURVE_GAIN = 1.70;
const RATIO_REPEL_GAIN = 1.25;
const FRONT_RATIO_GAIN = 0.55;
const MAX_NORMAL_CURVE = 0.85;
const MIN_TURN_CURVE = 0.45;
const MAX_TURN_CURVE = 1.00;
const HARD_TURN_CURVE = 1.00;
// Kompenzace mechanickeho posunu lidaru/serva.
// Nech 0/0 pro uplne stejne chovani doleva i doprava.
// Kdyz robot i v rovne chodbe porad tahne doleva, dej treba RIGHT_DISTANCE_OFFSET_MM = 20.
// Kdyz robot porad tahne doprava, dej treba LEFT_DISTANCE_OFFSET_MM = 20.
const LEFT_DISTANCE_OFFSET_MM = 0;
const RIGHT_DISTANCE_OFFSET_MM = 0;
// Pri uplne stejnem prostoru vlevo/vpravo program NESMI preferovat jednu stranu.
// Kdyz je pred robotem prekazka a pomer je temer nulovy, smer se bude stridat.
let lastForcedTurnDir = 1;
// -------------------- RGB CILOVA CARA --------------------
const RGB_ENABLE = true;
// false = běžná adresa, true = druhá varianta adresy
const RGB_ADDR_BIT = false;
// Tohle musíš doladit podle výpisu v monitoru.
// Na černé bude clear a RGB součet výrazně menší než na bílé.
const BLACK_CLEAR_MAX = 180;
const BLACK_RGB_SUM_MAX = 420;
const LINE_DEBOUNCE_MS = 1200;
// 0 = robot se po čáře sám nezastaví
const TARGET_LAPS = 0;
// -------------------- LED BARVY --------------------
const OFF = 0x000000;
const BLUE = 0x000030;
const GREEN = 0x003000;
const RED = 0x300000;
const YELLOW = 0x303000;
const CYAN = 0x003030;
const PURPLE = 0x300030;
const WHITE = 0x202020;
// -------------------- PROMENNE --------------------
const leds = new SmartLed(LED_PIN, LED_COUNT, LED_WS2812B);
const servo = new Servo(SERVO_PIN, 1, 4);
let lidar = null;
let rgb = null;
let emergencyLatched = false;
let lapCount = 0;
let lastLineTime = 0;
let lastLeft = DIST_NO_READ;
let lastFront = DIST_NO_READ;
let lastRight = DIST_NO_READ;
let lastLeftRaw = "";
let lastFrontRaw = "";
let lastRightRaw = "";
let lastRgbClear = -1;
let lastRgbSum = -1;
let driveStartTime = 0;
// -------------------- POMOCNE FUNKCE --------------------
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
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
function safeDistance(d) {
    if (!validDistance(d)) {
        return DIST_NO_READ;
    }
    return clamp(d, DIST_MIN_VALID, DIST_NO_READ);
}
function correctedLeftDistance(left) {
    return safeDistance(left) + LEFT_DISTANCE_OFFSET_MM;
}
function correctedRightDistance(right) {
    return safeDistance(right) + RIGHT_DISTANCE_OFFSET_MM;
}
function sideRatio(left, right) {
    const l = correctedLeftDistance(left);
    const r = correctedRightDistance(right);
    const sum = l + r;
    if (sum <= 0) {
        return 0;
    }
    // -1 az +1. Kladne = vpravo je vice mista.
    return clamp((r - l) / sum, -1, 1);
}
function ratioToCurve(ratio, gain, maxCurve) {
    if (abs(ratio) < RATIO_DEADBAND) {
        return MOTOR_TRIM;
    }
    const c = ratio * gain;
    return MOTOR_TRIM + clamp(c, -maxCurve, maxCurve);
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
function servoValue(angle) {
    angle = clamp(angle, 0, 180);
    if (SERVO_REVERSE) {
        angle = 180 - angle;
    }
    return Math.round((angle / 180) * 1023);
}
function setServoAngle(angle) {
    servo.write(servoValue(angle));
}
function validDistance(d) {
    return d >= DIST_MIN_VALID && d <= DIST_MAX_VALID;
}
function samplesToText(values) {
    let s = "[";
    for (let i = 0; i < values.length; i++) {
        if (i > 0)
            s += ",";
        s += values[i];
    }
    s += "]";
    return s;
}
function sortNumbers(values) {
    const a = [];
    for (let i = 0; i < values.length; i++) {
        a.push(values[i]);
    }
    for (let i = 0; i < a.length - 1; i++) {
        for (let j = i + 1; j < a.length; j++) {
            if (a[j] < a[i]) {
                const t = a[i];
                a[i] = a[j];
                a[j] = t;
            }
        }
    }
    return a;
}
function averageGoodSamples(values) {
    const good = [];
    for (let i = 0; i < values.length; i++) {
        const d = values[i];
        if (validDistance(d)) {
            good.push(d);
        }
    }
    if (good.length == 0) {
        return DIST_NO_READ;
    }
    const sorted = sortNumbers(good);
    // Pri 5 merenich odrizneme nejmensi a nejvetsi hodnotu.
    // Tim se rozhodovani neridi jednou spatnou spickou z lidaru.
    let start = 0;
    let end = sorted.length;
    if (sorted.length >= 5) {
        start = 1;
        end = sorted.length - 1;
    }
    let sum = 0;
    let count = 0;
    for (let i = start; i < end; i++) {
        sum += sorted[i];
        count++;
    }
    if (count == 0) {
        return DIST_NO_READ;
    }
    let avg = Math.round(sum / count);
    avg -= DIST_ERROR_SUBTRACT;
    return clamp(avg, DIST_MIN_VALID, DIST_NO_READ);
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
    setServoAngle(ANGLE_CENTER);
}
async function emergencyStop() {
    emergencyLatched = true;
    await stopRobot();
    setAllLeds(RED);
    console.log("NOUZOVE STOP - uvolni IO17 a zmackni IO2 pro novy start");
}
async function sleepCheck(ms) {
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
// -------------------- LIDAR MERENI --------------------
async function readLidarRaw() {
    if (lidar == null)
        return -1;
    try {
        const m = await lidar.read();
        return m.distance;
    }
    catch (e) {
        return -1;
    }
}
async function measureAngle(angle) {
    setServoAngle(angle);
    if (!(await sleepCheck(SERVO_SETTLE_MS))) {
        return { dist: DIST_NO_READ, raw: "STOP" };
    }
    const values = [];
    for (let i = 0; i < SAMPLES_PER_SIDE; i++) {
        const d = await readLidarRaw();
        values.push(d);
        if (i < SAMPLES_PER_SIDE - 1) {
            if (!(await sleepCheck(SAMPLE_DELAY_MS))) {
                return { dist: DIST_NO_READ, raw: "STOP" };
            }
        }
    }
    return {
        dist: averageGoodSamples(values),
        raw: samplesToText(values),
    };
}
async function scanAll() {
    const front1 = await measureAngle(ANGLE_CENTER);
    if (front1.raw == "STOP")
        return false;
    const left = await measureAngle(ANGLE_LEFT);
    if (left.raw == "STOP")
        return false;
    const front2 = await measureAngle(ANGLE_CENTER);
    if (front2.raw == "STOP")
        return false;
    const right = await measureAngle(ANGLE_RIGHT);
    if (right.raw == "STOP")
        return false;
    lastLeft = left.dist;
    lastFront = front1.dist < front2.dist ? front1.dist : front2.dist;
    lastRight = right.dist;
    lastLeftRaw = left.raw;
    lastFrontRaw = front1.raw + "/" + front2.raw;
    lastRightRaw = right.raw;
    return true;
}
// -------------------- RGB --------------------
function readBlackLine() {
    if (!RGB_ENABLE || rgb == null) {
        return false;
    }
    try {
        const clear = rgb.readRawClear();
        const raw = rgb.readRawRGB();
        const sum = raw[0] + raw[1] + raw[2];
        lastRgbClear = clear;
        lastRgbSum = sum;
        return clear > 0 &&
            clear < BLACK_CLEAR_MAX &&
            sum > 0 &&
            sum < BLACK_RGB_SUM_MAX;
    }
    catch (e) {
        lastRgbClear = -1;
        lastRgbSum = -1;
        return false;
    }
}
async function checkFinishLine() {
    if (!readBlackLine()) {
        return false;
    }
    const now = Date.now();
    if (now - lastLineTime < LINE_DEBOUNCE_MS) {
        return false;
    }
    lastLineTime = now;
    lapCount++;
    console.log("CERNA CILOVA CARA: kolo=" + lapCount +
        " clear=" + lastRgbClear +
        " sum=" + lastRgbSum);
    setAllLeds(CYAN);
    if (TARGET_LAPS > 0 && lapCount >= TARGET_LAPS) {
        console.log("HOTOVO - dosazen pocet kol: " + TARGET_LAPS);
        await stopRobot();
        setAllLeds(PURPLE);
        return true;
    }
    return false;
}
// -------------------- ROZHODOVANI JIZDY --------------------
function chooseTurnDirection() {
    // -1 = vlevo, +1 = vpravo
    // Rozhodnuti je symetricky podle pomeru stran.
    const ratio = sideRatio(lastLeft, lastRight);
    if (abs(ratio) > RATIO_DEADBAND) {
        return ratio > 0 ? 1 : -1;
    }
    // Kdyz jsou obe strany prakticky stejne, nesmi byt pevna preference.
    // Proto pri nucene zatacce smer stridame.
    lastForcedTurnDir = -lastForcedTurnDir;
    return lastForcedTurnDir;
}
function calculateCurve() {
    let speed = SPEED_NORMAL;
    let curve = MOTOR_TRIM;
    let mode = "ROVNE";
    const ratio = sideRatio(lastLeft, lastRight);
    // Pred robotem je hodne blizko zed.
    // Zataceni je podle pomeru stran, ale ma minimalni silu, aby se robot opravdu odlepil.
    if (lastFront < FRONT_DANGER) {
        const dir = chooseTurnDirection();
        speed = SPEED_TURN;
        curve = dir * HARD_TURN_CURVE;
        mode = dir < 0 ? "NEBEZPECI_POMER_VLEVO" : "NEBEZPECI_POMER_VPRAVO";
        return { speed, curve, mode };
    }
    // Pred robotem se blizi zatacka.
    // Cim vetsi pomerovy rozdil mezi levou a pravou stranou, tim silneji zatoci.
    if (lastFront < FRONT_TURN) {
        const dir = chooseTurnDirection();
        const frontUrgency = clamp((FRONT_TURN - lastFront) / FRONT_TURN, 0, 1);
        const ratioPower = clamp(abs(ratio) * FRONT_RATIO_GAIN + frontUrgency * 0.55, 0, 1);
        speed = SPEED_TURN;
        curve = dir * clamp(MIN_TURN_CURVE + ratioPower, MIN_TURN_CURVE, MAX_TURN_CURVE);
        mode = dir < 0 ? "ZATACKA_POMER_VLEVO" : "ZATACKA_POMER_VPRAVO";
        return { speed, curve, mode };
    }
    // Pred robotem je mene mista, zpomal.
    if (lastFront < FRONT_SLOW) {
        speed = SPEED_SLOW;
    }
    else {
        speed = SPEED_NORMAL;
    }
    // Kdyz je jedna strana moc blizko, pouzij silnejsi pomerove odtlaceni.
    // Blizko vlevo => ratio bude kladne => zatoc doprava.
    // Blizko vpravo => ratio bude zaporne => zatoc doleva.
    if (lastLeft < SIDE_TOO_CLOSE || lastRight < SIDE_TOO_CLOSE) {
        curve = ratioToCurve(ratio, RATIO_CURVE_GAIN + RATIO_REPEL_GAIN, MAX_NORMAL_CURVE);
        mode = curve > MOTOR_TRIM ? "ODTLACENI_POMER_DOPRAVA" : "ODTLACENI_POMER_DOLEVA";
        return { speed, curve, mode };
    }
    // Normalni centrovani mezi stenami podle pomeru vzdalenosti.
    if (lastLeft < SIDE_NEAR || lastRight < SIDE_NEAR) {
        curve = ratioToCurve(ratio, RATIO_CURVE_GAIN, MAX_NORMAL_CURVE);
        if (abs(curve - MOTOR_TRIM) > 0.001) {
            mode = curve > MOTOR_TRIM ? "CENTRUJI_POMER_DOPRAVA" : "CENTRUJI_POMER_DOLEVA";
            return { speed, curve, mode };
        }
    }
    // Jinak rovne. Pri vyrovnanem pomeru nedavame zadnou stranovou preferenci.
    curve = MOTOR_TRIM;
    mode = "ROVNE_POMER_VYROVNANY_BEZ_PREFERENCE";
    return { speed, curve, mode };
}
// -------------------- INIT --------------------
async function initHardware() {
    gpio.pinMode(START_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    gpio.pinMode(EMERGENCY_BUTTON_PIN, gpio.PinMode.INPUT_PULLUP);
    setAllLeds(WHITE);
    await stopRobot();
    robutek.setRamp(RAMP);
    robutek.setSpeed(0);
    setServoAngle(ANGLE_CENTER);
    console.log("======================================");
    console.log("ROBOCARTS NOVY CISTY PROGRAM");
    console.log("IO2 = START");
    console.log("IO17 = NOUZOVE STOP");
    console.log("I2C2 SDA=" + I2C2_SDA + " SCL=" + I2C2_SCL);
    console.log("Servo pin=" + SERVO_PIN);
    console.log("======================================");
    try {
        I2C2.setup({
            sda: I2C2_SDA,
            scl: I2C2_SCL,
            bitrate: 400000,
        });
        console.log("OK I2C2");
    }
    catch (e) {
        console.log("CHYBA I2C2: " + e);
    }
    try {
        lidar = new VL53L0X(I2C2);
        console.log("OK VL53L0X");
    }
    catch (e) {
        lidar = null;
        console.log("CHYBA VL53L0X: " + e);
    }
    if (RGB_ENABLE) {
        try {
            rgb = new ZSCS2016C(I2C2, RGB_ADDR_BIT);
            rgb.enable();
            console.log("OK RGB ZSCS2016C");
        }
        catch (e) {
            rgb = null;
            console.log("CHYBA RGB: " + e);
            console.log("Kdyz je RGB zapojeny, zkus zmenit RGB_ADDR_BIT na true.");
        }
    }
    setAllLeds(BLUE);
}
// -------------------- CEKANI NA START --------------------
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
                console.log("START");
                lapCount = 0;
                lastLineTime = 0;
                setAllLeds(YELLOW);
                await sleep(300);
                return;
            }
        }
        await sleep(20);
    }
}
// -------------------- HLAVNI JIZDA --------------------
async function drive() {
    let lastPrint = 0;
    await stopRobot();
    console.log("LASER OD STARTU - kazdy smer meri 5x, odstrani spicky a podle prumeru rozhodne");
    setServoAngle(ANGLE_CENTER);
    await sleepCheck(300);
    driveStartTime = Date.now();
    setAllLeds(GREEN);
    while (!emergencyLatched) {
        if (isPressed(EMERGENCY_BUTTON_PIN)) {
            await emergencyStop();
            break;
        }
        // HNED OD STARTU: sken vlevo / stred / vpravo.
        // Kazdy smer se meri 5x a rozhoduje se az z ocisteneho prumeru.
        if (!(await scanAll())) {
            break;
        }
        const d = calculateCurve();
        const outputCurve = CURVE_SIGN * d.curve;
        robutek.setSpeed(d.speed);
        robutek.move(outputCurve);
        if (await checkFinishLine()) {
            break;
        }
        if (Date.now() - lastPrint > 200) {
            lastPrint = Date.now();
            console.log("LASER_RUN" +
                " | mode=" + d.mode +
                " | speed=" + d.speed +
                " | curve=" + d.curve.toFixed(3) +
                " | out=" + outputCurve.toFixed(3) +
                " | ratio=" + sideRatio(lastLeft, lastRight).toFixed(3) +
                " | L=" + lastLeft + " corr=" + correctedLeftDistance(lastLeft) + " raw" + lastLeftRaw +
                " | F=" + lastFront + " raw" + lastFrontRaw +
                " | R=" + lastRight + " corr=" + correctedRightDistance(lastRight) + " raw" + lastRightRaw +
                " | RGB clear=" + lastRgbClear +
                " sum=" + lastRgbSum +
                " | lap=" + lapCount);
        }
        await sleepCheck(5);
    }
    await stopRobot();
}
// -------------------- MAIN --------------------
async function main() {
    await initHardware();
    while (true) {
        await waitForStart();
        await drive();
    }
}
main().catch(async (e) => {
    console.log("HLAVNI CHYBA PROGRAMU: " + e);
    await stopRobot();
    setAllLeds(PURPLE);
});
