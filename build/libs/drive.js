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
export async function driveStraight(robutek, gyro, gyroZOffset, distanceMm, speed, emergencyPin, angleState, leds, emergencyStopCallback, isEmergencyLatched) {
    if (isEmergencyLatched())
        return;
    const GREEN = 0x003000;
    const PURPLE = 0x300030;
    const setAllLeds = (color) => {
        leds.clear();
        for (let i = 0; i < 8; i++) {
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
    console.log(`Start jízdy rovně na ${distanceMm} mm. Cílový úhel: ${targetAngle.toFixed(1)} °`);
    setAllLeds(GREEN);
    while (!isEmergencyLatched()) {
        if (isPressed(emergencyPin)) {
            await emergencyStopCallback();
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
            // Kladná curve v DifferentialDrive.move(curve) zatáčí doprava (corrective pro kladný error/vychýlení doleva)
            const STEER_SIGN = 1;
            let curve = STEER_SIGN * (Kp * error + Ki * integral + Kd * derivative);
            // Limity korekce: max ±0.20
            if (curve > 0.20)
                curve = 0.20;
            if (curve < -0.20)
                curve = -0.20;
            robutek.setSpeed(speed);
            await robutek.move(curve);
        }
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
    await sleep(1000);
}
/**
 * Otáčení robota na místě o zadaný úhel s využitím zpětné vazby z gyroskopu
 * @param robutek Instancovaný objekt robutka
 * @param angleState Objekt obsahující aktuální integrovaný úhel angleZ (předávaný referencí)
 * @param targetAngleChange Relativní změna úhlu (kladná = doleva/CCW, záporná = doprava/CW)
 * @param speed Maximální rychlost otáčení (SPEED_TURN)
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
        leds.clear();
        for (let i = 0; i < 8; i++) {
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
    while (!isEmergencyLatched()) {
        if (isPressed(emergencyPin)) {
            await emergencyStopCallback();
            break;
        }
        const error = angleState.angleZ - targetAngle;
        // Pokud je chyba velmi malá (méně než 1.5 stupně), otáčení končí
        if (Math.abs(error) < 1.5) {
            console.log(`Otáčení úspěšně dokončeno. Koncový úhel: ${angleState.angleZ.toFixed(1)} °`);
            break;
        }
        // P-regulátor zatáčení:
        // Pokud je chyba velká (např. > 40 stupňů), točíme plnou rychlostí (curve = ±1.0)
        // Před cílem plynule zpomalujeme.
        const Kp = 0.025;
        let curve = error * Kp;
        // Omezení curve na rozsah [-1.0, 1.0]
        if (curve > 1.0)
            curve = 1.0;
        if (curve < -1.0)
            curve = -1.0;
        // Voláme standardní neblokující move()
        await robutek.move(curve);
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
