import * as gpio from "gpio";
// Pomocné funkce pro vnitřní logiku pohybu
function isPressed(pin) {
    return gpio.read(pin) == 0;
}
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}
/**
 * Jízda rovně na určenou vzdálenost se stabilizací pomocí gyroskopu (PID regulace)
 * @param robutek Instancovaný objekt robutka
 * @param gyro Instancovaný objekt gyroskopu
 * @param gyroZOffset Zkalibrovaný offset gyroskopu
 * @param distanceMm Požadovaná vzdálenost k ujetí v milimetrech
 * @param speed Rychlost pohybu (např. SPEED_NORMAL)
 * @param emergencyPin Pin nouzového tlačítka (např. IO17)
 * @param angleState Objekt obsahující aktuální integrovaný úhel angleZ (předávaný referencí)
 * @param leds Instancovaný LED pásek pro vizuální indikaci
 * @param emergencyStopCallback Funkce pro vyvolání nouzového zastavení
 * @param isEmergencyLatched Funkce vracející stav nouzového zastavení
 */
export async function driveStraight(robutek, gyro, gyroZOffset, distanceMm, speed, emergencyPin, angleState, leds, emergencyStopCallback, isEmergencyLatched, shouldStopPredicate) {
    if (isEmergencyLatched())
        return;
    // Zastavíme předchozí pohyb a uvolníme motory pro nový start (předchází chybě Motor is already moving)
    try {
        await robutek.stop();
        await sleep(50); // Krátká pauza na zprocesování stopu v systému (zabraňuje race condition)
    }
    catch (e) { }
    const GREEN = 0x003000;
    const PURPLE = 0x300030;
    const setAllLeds = (color) => {
        // Nastavujeme pouze LED 2 až 7 (LED 0 a 1 jsou vyhrazeny pro sensory)
        for (let i = 2; i < 8; i++) {
            leds.set(i, color);
        }
        leds.show();
    };
    // Načteme počáteční pozice enkodérů
    const startLeft = robutek.leftMotor.getPosition();
    const startRight = robutek.rightMotor.getPosition();
    // Resetujeme úhel na začátku jízdy
    angleState.angleZ = 0;
    const targetAngle = 0;
    // Inicializace PID proměnných
    let integral = 0;
    let lastError = 0;
    let lastTimeMs = Date.now();
    console.log(`Start jízdy rovně na ${distanceMm} mm. Rychlost: ${speed} mm/s. Cílový úhel: ${targetAngle.toFixed(1)} °`);
    setAllLeds(GREEN);
    while (!isEmergencyLatched()) {
        if (isPressed(emergencyPin)) {
            await emergencyStopCallback();
            break;
        }
        if (shouldStopPredicate && await shouldStopPredicate()) {
            console.log("Jízda rovně přerušena externí podmínkou (senzor).");
            break;
        }
        // Spočítáme ujetou vzdálenost (průměr obou kol)
        const currentLeft = robutek.leftMotor.getPosition();
        const currentRight = robutek.rightMotor.getPosition();
        const distLeft = currentLeft - startLeft;
        const distRight = currentRight - startRight;
        const distTraveled = (distLeft + distRight) / 2; // v mm
        console.log(`Ujeto: ${distTraveled.toFixed(0)} mm / ${distanceMm} mm | Úhel: ${angleState.angleZ.toFixed(1)} °`);
        // Pokud ujedeme požadovanou vzdálenost, zastavíme
        if (distTraveled >= distanceMm) {
            console.log(`Cílová vzdálenost ${distanceMm} mm dosažena. Zastavuji robot.`);
            break;
        }
        // Výpočet dt
        const now = Date.now();
        const dt = (now - lastTimeMs) / 1000.0;
        lastTimeMs = now;
        if (dt > 0 && dt < 0.2) {
            const error = angleState.angleZ - targetAngle; // Kladná = vychýlení doleva, záporná = vychýlení doprava
            // Osvědčené PID koeficienty
            const Kp = 0.02;
            const Ki = 0.001;
            const Kd = 0.005;
            integral += error * dt;
            if (integral > 5)
                integral = 5;
            if (integral < -5)
                integral = -5;
            const derivative = (error - lastError) / dt;
            lastError = error;
            // Zpětnovazební PID regulace
            const STEER_SIGN = 1;
            let curve = STEER_SIGN * (Kp * error + Ki * integral + Kd * derivative);
            // Limity korekce: max ±0.20
            if (curve > 0.20)
                curve = 0.20;
            if (curve < -0.20)
                curve = -0.20;
            robutek.setSpeed(speed);
            robutek.move(curve); // Voláme bez await, abychom neblokovali event loop!
        }
        await sleep(10);
    }
    // Konec pohybu - bez zastavení a bez sleep pro plynulý přechod na další stav
}
/**
 * Otáčení robota na místě o zadaný úhel s využitím zpětné vazby z gyroskopu (jednoduchý P-regulátor)
 * @param robutek Instancovaný objekt robutka
 * @param angleState Objekt obsahující aktuální integrovaný úhel angleZ (předávaný referencí)
 * @param targetAngleChange Relativní změna úhlu (kladná = doleva/CCW, záporná = doprava/CW)
 * @param speed Rychlost otáčení (SPEED_TURN)
 * @param emergencyPin Pin nouzového tlačítka (např. IO17)
 * @param leds Instancovaný LED pásek
 * @param emergencyStopCallback Funkce pro nouzové zastavení
 * @param isEmergencyLatched Funkce pro zjištění nouzového stavu
 */
export async function rotateAngle(robutek, angleState, targetAngleChange, speed, emergencyPin, leds, emergencyStopCallback, isEmergencyLatched) {
    if (isEmergencyLatched())
        return;
    const BLUE = 0x000030;
    const PURPLE = 0x300030;
    const setAllLeds = (color) => {
        // Nastavujeme pouze LED 2 až 7 (LED 0 a 1 jsou vyhrazeny pro sensory)
        for (let i = 2; i < 8; i++) {
            leds.set(i, color);
        }
        leds.show();
    };
    // Reset úhlu před zahájením otáčení
    angleState.angleZ = 0;
    const targetAngle = targetAngleChange;
    console.log(`Start otáčení na místě o: ${targetAngle.toFixed(1)} °`);
    setAllLeds(BLUE);
    robutek.setSpeed(speed);
    let lastLogTime = 0;
    while (!isEmergencyLatched()) {
        if (isPressed(emergencyPin)) {
            await emergencyStopCallback();
            break;
        }
        const error = angleState.angleZ - targetAngle;
        // P-regulátor zatáčení
        const Kp = 0.025;
        let curve = error * Kp;
        // Omezení curve na rozsah [-1.0, 1.0]
        if (curve > 1.0)
            curve = 1.0;
        if (curve < -1.0)
            curve = -1.0;
        // Logování průběhu otáčení každých 100 ms
        const nowLog = Date.now();
        if (nowLog - lastLogTime > 100) {
            lastLogTime = nowLog;
            console.log(`Otáčení: úhel ${angleState.angleZ.toFixed(1)}° / cíl ${targetAngle.toFixed(1)}° | Chyba: ${error.toFixed(1)}° | Výkon: ${curve.toFixed(2)}`);
        }
        // Pokud je chyba velmi malá (méně než 1.5 stupně), otáčení končí
        if (Math.abs(error) < 1.5) {
            console.log(`Otáčení úspěšně dokončeno. Koncový úhel: ${angleState.angleZ.toFixed(1)} °`);
            break;
        }
        // Voláme neblokující move() bez await
        robutek.move(curve);
        await sleep(10);
    }
    // Zastavení motorů
    try {
        await robutek.stop(true);
    }
    catch (e) { }
    try {
        robutek.setSpeed(0);
    }
    catch (e) { }
    setAllLeds(PURPLE); // Hotovo
    await sleep(300); // Krátká pauza na uklidnění po otočení
}
/**
 * Projede oblouk o daném poloměru a úhlu s využitím zpětné vazby z gyroskopu
 * @param robutek Instancovaný objekt robutka
 * @param angleState Objekt s integrovaným úhlem angleZ
 * @param radiusMm Poloměr oblouku v mm (např. 100 mm)
 * @param targetAngle Relativní úhel otočení v stupních (kladný = vlevo/CCW, záporný = vpravo/CW)
 * @param baseSpeed Základní rychlost jízdy v mm/s (SPEED_NORMAL)
 * @param emergencyPin Pin nouzového tlačítka (např. IO17)
 * @param leds LED pásek
 * @param emergencyStopCallback Nouzové zastavení
 * @param isEmergencyLatched Stav nouzového zastavení
 */
export async function driveArc(robutek, angleState, radiusMm, targetAngle, baseSpeed, emergencyPin, leds, emergencyStopCallback, isEmergencyLatched) {
    if (isEmergencyLatched())
        return;
    // Zastavíme předchozí pohyb a uvolníme motory pro nový start (předchází chybě Motor is already moving)
    try {
        await robutek.stop();
        await sleep(50); // Krátká pauza na zprocesování stopu v systému (zabraňuje race condition)
    }
    catch (e) { }
    const CYAN = 0x003030;
    const PURPLE = 0x300030;
    const setAllLeds = (color) => {
        // Nastavujeme pouze LED 2 až 7 (LED 0 a 1 jsou vyhrazeny pro sensory)
        for (let i = 2; i < 8; i++) {
            leds.set(i, color);
        }
        leds.show();
    };
    // Rozchod kol robota (track width) zadefinovaný výrobcem
    const d = 83; // mm
    // Reset úhlu na začátku oblouku
    angleState.angleZ = 0;
    const targetAbs = Math.abs(targetAngle);
    // Výpočet rychlostí kol (konstantní rychlost po celou dobu oblouku)
    let leftSpeed = 0;
    let rightSpeed = 0;
    const ratio = d / (2 * radiusMm);
    if (targetAngle > 0) {
        // Zatáčení vlevo
        leftSpeed = baseSpeed * (1 - ratio);
        rightSpeed = baseSpeed * (1 + ratio);
    }
    else {
        // Zatáčení vpravo
        leftSpeed = baseSpeed * (1 + ratio);
        rightSpeed = baseSpeed * (1 - ratio);
    }
    console.log(`Start oblouku R=${radiusMm} mm, úhel=${targetAngle.toFixed(1)}° | Rychlost: ${baseSpeed} mm/s`);
    setAllLeds(CYAN);
    // Nastavení rychlostí a nulových ramp pro okamžitý start
    robutek.leftMotor.setSpeed(leftSpeed);
    robutek.rightMotor.setSpeed(rightSpeed);
    robutek.leftMotor.setRamp(0);
    robutek.rightMotor.setRamp(0);
    // Spuštění motorů bez udání dráhy (jednou na začátku, bez await, aby se neblokoval event loop)
    robutek.leftMotor.move();
    robutek.rightMotor.move();
    let lastLogTime = 0;
    const startTime = Date.now();
    while (!isEmergencyLatched()) {
        if (isPressed(emergencyPin)) {
            await emergencyStopCallback();
            break;
        }
        // Bezpečnostní timeout 3 sekundy
        if (Date.now() - startTime > 3000) {
            console.log(`TIMEOUT: Oblouk nedokončen do 3 sekund! Nouzové přerušení. Poslední úhel: ${angleState.angleZ.toFixed(1)} °`);
            break;
        }
        const currentAngle = Math.abs(angleState.angleZ);
        const error = targetAbs - currentAngle;
        // Pokud jsme dosáhli cílového úhlu s tolerancí 1.0 stupně, zastavíme
        if (error <= 1.0) {
            console.log(`Oblouk dokončen. Koncový úhel: ${angleState.angleZ.toFixed(1)} °`);
            break;
        }
        // Logování každých 100 ms
        const now = Date.now();
        if (now - lastLogTime > 100) {
            lastLogTime = now;
            console.log(`Oblouk: úhel ${angleState.angleZ.toFixed(1)}° / cíl ${targetAngle.toFixed(1)}° | L: ${leftSpeed.toFixed(0)} mm/s, R: ${rightSpeed.toFixed(0)} mm/s`);
        }
        await sleep(10);
    }
    // Konec pohybu - bez zastavení a bez sleep pro plynulý přechod na další stav
}
