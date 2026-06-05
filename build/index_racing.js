import { createRobutek } from "./libs/robutek.js";
import { I2C1, I2C2 } from "i2c";
import { VL53L0X } from "./libs/VL53L0X.js";
import { Servo } from "./libs/servo.js";
import { SmartLed, LED_WS2812B } from "smartled";
import * as gpio from "gpio";
import { driveStraight, driveArc } from "./libs/drive.js";
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
const angleState = { angleZ: 0 };
let lidar = null; // Přední Lidar (na I2C2)
let leftLidar = null; // Levý Lidar (na I2C1)
let lastTime = 0;
let intervalId = null;
let emergencyLatched = false;
let lastLeftArcTime = 0; // Timestamp posledního zatáčení vlevo
let lastRightArcTime = 0; // Timestamp posledního zatáčení vpravo
let latestFrontDistance = 9999; // Sdílená hodnota předního senzoru
let latestLeftDistance = 9999; // Sdílená hodnota levého senzoru
let sensorUpdateCount = 0; // Čítač aktualizací senzorů na pozadí
// -------------------- PARAMETRY JÍZDY --------------------
const SPEED_NORMAL = 700; // Maximální rychlost na rovinkách v mm/s (zvýšeno z 550)
const SPEED_SLOW = 350; // Snížená rychlost při přiblížení k zatáčce v mm/s
const SPEED_TURN = 200; // Rychlost pro pomalé otáčení a dorovnání
const SPEED_ARC = 300; // Rychlost projíždění oblouků (zatáček) v mm/s
const RAMP = 3000;
const FRONT_SLOW_DIST = 850; // Vzdálenost k překážce v mm, při které začíná zpomalovat
const FRONT_DANGER_DIST = 450; // Vzdálenost k překážce v mm pro zahájení pravého oblouku (sníženo na 450 pro pozdnější zatáčení)
const LEFT_WALL_THRESHOLD = 300; // Práh konce levé stěny v mm pro zatáčení vlevo
const RADIUS_LEFT = 105; // Poloměr levé zatáčky v mm
const RADIUS_RIGHT = 110; // Poloměr pravé zatáčky v mm
// Pokud robot při zatáčení uhýbá na špatnou stranu, změň na +1.
const CURVE_SIGN = -1;
// -------------------- POMOCNÉ FUNKCE --------------------
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
    console.log("!!! NOUZOVÉ STOP TLAČÍTKO STISKNUTO - VYPNUTÍ POHYBU A RESET !!!");
    // Zastavíme veškerý pohyb
    try {
        await stopRobot();
    }
    catch (e) { }
    // Rozsvítíme červeně
    setAllLeds(RED);
    await sleep(500);
    // Ukončíme program s chybovým kódem (pro restart ze strany supervisora)
    exit(1);
    // Jako záloha vyvoláme unhandled exception k vynucení tvrdého restartu firmware
    throw new Error("Emergency restart requested");
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
    // Přední Lidar (na I2C2)
    try {
        lidar = new VL53L0X(I2C2);
        console.log("OK: Přední Lidar VL53L0X (I2C2) připojen.");
    }
    catch (e) {
        console.log("CHYBA Přední Lidar (VL53L0X na I2C2): " + e);
    }
    // Levý Lidar (na I2C1)
    try {
        leftLidar = new VL53L0X(I2C1);
        console.log("OK: Levý Lidar VL53L0X (I2C1) připojen.");
    }
    catch (e) {
        console.log("CHYBA Levý Lidar (VL53L0X na I2C1): " + e);
    }
    // Gyroskop MPU6050 (na I2C2)
    try {
        gyro = new MPU6050(I2C2);
        if (gyro.probe()) {
            gyro.init();
            console.log("OK: Gyroskop MPU6050 inicializován.");
            // Kalibrace gyroskopu - 50 měření v klidu
            console.log("KALIBRACE GYROSKOPU - NEHÝBEJTE S ROBOTEM...");
            setAllLeds(PURPLE); // Během kalibrace svítíme fialově
            let sum = 0;
            const CALIBRATION_STEPS = 50;
            for (let i = 0; i < CALIBRATION_STEPS; i++) {
                const data = gyro.read();
                sum += data.gyro.z;
                await sleep(5);
            }
            gyroZOffset = sum / CALIBRATION_STEPS;
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
    }
    catch (e) {
        console.log("CHYBA při registraci HW interruptu: " + e);
    }
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
async function getDistance(sensor) {
    if (sensor == null)
        return 9999;
    try {
        const m = await sensor.read();
        // Jakákoliv hodnota pod 30 mm (3 cm) je považována za chybu senzoru (error) a bere se jako prázdno (9999)
        if (m.distance < 30 || m.distance > 2000) {
            return 9999;
        }
        return m.distance;
    }
    catch (e) {
        return 9999;
    }
}
// -------------------- VIZUALIZACE SENZORŮ NA LED PÁSKU --------------------
function updateSensorLeds(front, left) {
    if (emergencyLatched)
        return;
    // LED 0 (první): Přední senzor
    if (front <= FRONT_DANGER_DIST) {
        leds.set(0, 0x300000); // Červená (překážka nablízku)
    }
    else {
        leds.set(0, 0x003000); // Zelená (volno)
    }
    // LED 1 (druhá): Levý/boční senzor
    if (left <= LEFT_WALL_THRESHOLD) {
        leds.set(1, 0x300000); // Červená (zeď nablízku)
    }
    else {
        leds.set(1, 0x003000); // Zelená (volno/roh)
    }
    leds.show();
}
// -------------------- KONTINUÁLNÍ ČTENÍ SENZORŮ NA POZADÍ --------------------
async function startLidarLoop() {
    console.log("=== START SENSOR LOOP (Kontinuální čtení na pozadí) ===");
    while (!emergencyLatched) {
        try {
            // Čteme z obou I2C sběrnic sekvenčně pro zamezení konfliktů v I2C ovladači
            const left = await getDistance(leftLidar);
            const front = await getDistance(lidar);
            latestLeftDistance = left;
            latestFrontDistance = front;
            // Inkrementujeme počítadlo aktualizací senzorů
            sensorUpdateCount++;
            // Aktualizujeme LED vizualizaci
            updateSensorLeds(front, left);
        }
        catch (e) {
            // Ignorujeme případné chyby pro zachování chodu smyčky
        }
        // Krátká prodleva pro uvolnění event loopu
        await sleep(5);
    }
}
// Počká na dokončení nového měřicího cyklu (alspoň 2 měření, aby levý i přední dálkoměr byly čerstvé)
async function waitForFreshSensors() {
    const startCount = sensorUpdateCount;
    while (sensorUpdateCount < startCount + 2 && !emergencyLatched) {
        await sleep(5);
    }
}
// -------------------- AUTONOMNÍ POHYB JEDEM (Wall Follower) --------------------
async function jedem() {
    console.log("=== START POHYBU JEDEM (Wall Follower) ===");
    while (!emergencyLatched) {
        // Okamžité synchronní čtení z pozadí (nulová prodleva)
        const left = latestLeftDistance;
        const front = latestFrontDistance;
        console.log(`[jedem] Leve: ${left.toFixed(0)} mm | Predni: ${front.toFixed(0)} mm`);
        // Výpočet podmínek zatáčení
        const canTurnLeft = (left > LEFT_WALL_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);
        if (canTurnLeft) {
            console.log(`-> Vlevo volno: zatáčím plynulým obloukem 180° vlevo (R=${RADIUS_LEFT} mm)`);
            lastLeftArcTime = Date.now();
            await driveArc(robutek, angleState, RADIUS_LEFT, // poloměr 12 cm -> 10.5 cm
            180, // 180 stupňů vlevo
            SPEED_ARC, // rychlost v mm/s
            EMERGENCY_BUTTON_PIN, leds, emergencyStop, () => emergencyLatched);
            await waitForFreshSensors(); // Počkáme na čerstvá data ze senzorů v nové pozici
        }
        else if (front > FRONT_DANGER_DIST) {
            // Pokud se blíží zatáčka (předek < FRONT_SLOW_DIST), zpomalíme, jinak jedeme plnou rychlostí
            const currentSpeed = (front < FRONT_SLOW_DIST) ? SPEED_SLOW : SPEED_NORMAL;
            console.log(`-> Vepředu volno (vlevo zeď): jedu rovně rychlostí ${currentSpeed} mm/s`);
            // Jedeme rovně, dokud se neuvolní levá strana (s ohledem na 5s limit) nebo se nezablokuje předek
            await driveStraight(robutek, gyro, gyroZOffset, 5000, // Dlouhá jízda, kterou přerušíme senzory
            currentSpeed, EMERGENCY_BUTTON_PIN, angleState, leds, emergencyStop, () => emergencyLatched, async () => {
                // Synchronní čtení z pozadí v predicate smyčce (vysoká rychlost)
                const currLeft = latestLeftDistance;
                const currFront = latestFrontDistance;
                const stopForLeft = (currLeft > LEFT_WALL_THRESHOLD) && (Date.now() - lastLeftArcTime > 5000);
                const stopForFront = (currFront <= FRONT_DANGER_DIST);
                // Zastavíme a re-evaluujeme rychlost, pokud se změnila podmínka pro zpomalení před zdí
                const speedShouldChange = (currentSpeed === SPEED_NORMAL && currFront < FRONT_SLOW_DIST) ||
                    (currentSpeed === SPEED_SLOW && currFront >= FRONT_SLOW_DIST);
                return (stopForLeft || stopForFront || speedShouldChange);
            });
            // BEZ PRODLEVY: Po jízdě rovně nečekáme a ihned vyhodnocujeme zatáčení v dalším kroku smyčky!
        }
        else {
            // Zablokováno vepředu i vlevo: zkusíme zabočit vpravo, pokud od minulého pravého oblouku uběhlo více než 5 sekund
            if (Date.now() - lastRightArcTime > 5000) {
                console.log(`-> Zablokováno (vlevo zeď, vepředu zeď): zatáčím plynulým obloukem 90° vpravo (R=${RADIUS_RIGHT} mm)`);
                lastRightArcTime = Date.now();
                await driveArc(robutek, angleState, RADIUS_RIGHT, // poloměr 12 cm -> 11 cm
                -90, // 90 stupňů vpravo
                SPEED_ARC, // rychlost v mm/s
                EMERGENCY_BUTTON_PIN, leds, emergencyStop, () => emergencyLatched);
                await waitForFreshSensors(); // Počkáme na čerstvá data ze senzorů v nové pozici
            }
            else {
                console.log("-> Zablokováno, ale pravé zatáčení je blokováno 5s limitem. Zastavuji a čekám...");
                await stopRobot(); // Zastavíme kola, aby robot při čekání nejel dál dopředu
                await sleep(50);
            }
        }
        await sleep(5);
    }
}
// -------------------- TESTOVACÍ SEKVENCE POHYBŮ --------------------
async function runSequence() {
    setServoAngle(ANGLE_CENTER);
    await sleep(100);
    await jedem();
}
// -------------------- MAIN --------------------
async function main() {
    await initHardware();
    startLidarLoop(); // Spustíme čtení LiDARů na pozadí
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
