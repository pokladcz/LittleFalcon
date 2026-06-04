import * as gpio from "gpio";

// Pomocné funkce pro vnitřní logiku pohybu
function isPressed(pin: number): boolean {
  return gpio.read(pin) == 0;
}

function sleep(ms: number): Promise<void> {
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
export async function driveStraight(
  robutek: any,
  gyro: any,
  gyroZOffset: number,
  distanceMm: number,
  speed: number,
  emergencyPin: number,
  angleState: { angleZ: number },
  leds: any,
  emergencyStopCallback: () => Promise<void>,
  isEmergencyLatched: () => boolean
): Promise<void> {
  if (isEmergencyLatched()) return;

  const GREEN = 0x003000;
  const PURPLE = 0x300030;
  
  const setAllLeds = (color: number) => {
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

  console.log(`Start jízdy rovně na ${distanceMm} mm. Rychlost: ${speed} mm/s. Cílový úhel: ${targetAngle.toFixed(1)} °`);
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
      if (integral > 5) integral = 5;
      if (integral < -5) integral = -5;

      const derivative = (error - lastError) / dt;
      lastError = error;

      // Zpětnovazební PID regulace
      const STEER_SIGN = 1; 
      let curve = STEER_SIGN * (Kp * error + Ki * integral + Kd * derivative);

      // Limity korekce: max ±0.20
      if (curve > 0.20) curve = 0.20;
      if (curve < -0.20) curve = -0.20;

      robutek.setSpeed(speed);
      robutek.move(curve); // Voláme bez await, abychom neblokovali event loop!
    }

    await sleep(10);
  }

  // Zastavení motorů
  try {
    await robutek.stop(true);
  } catch (e) {}
  try {
    robutek.setSpeed(0);
  } catch (e) {}

  setAllLeds(PURPLE); // Hotovo
  await sleep(1000);
}

/**
 * Otáčení robota na místě o zadaný úhel s využitím ramp pro plynulé zrychlení a zpomalení (podle Fotonu)
 * @param robutek Instancovaný objekt robutka
 * @param angleState Objekt obsahující aktuální integrovaný úhel angleZ (předávaný referencí)
 * @param targetAngleChange Relativní změna úhlu (kladná = doleva/CCW, záporná = doprava/CW)
 * @param maxSpeed Maximální rychlost otáčení (SPEED_TURN)
 * @param emergencyPin Pin nouzového tlačítka (např. IO17)
 * @param leds Instancovaný LED pásek
 * @param emergencyStopCallback Funkce pro nouzové zastavení
 * @param isEmergencyLatched Funkce pro zjištění nouzového stavu
 */
export async function rotateAngle(
  robutek: any,
  angleState: { angleZ: number },
  targetAngleChange: number,
  maxSpeed: number,
  emergencyPin: number,
  leds: any,
  emergencyStopCallback: () => Promise<void>,
  isEmergencyLatched: () => boolean
): Promise<void> {
  if (isEmergencyLatched()) return;

  const BLUE = 0x000030;
  const PURPLE = 0x300030;

  const setAllLeds = (color: number) => {
    leds.clear();
    for (let i = 0; i < 8; i++) {
      leds.set(i, color);
    }
    leds.show();
  };

  // Reset úhlu před zahájením otáčení
  angleState.angleZ = 0;
  const targetAngle = targetAngleChange;
  const targetAbs = Math.abs(targetAngle);

  console.log(`Start otáčení na místě o: ${targetAngle.toFixed(1)} °`);
  setAllLeds(BLUE);

  // Parametry rampy otáčení (hodnoty v mm/s přizpůsobené z Fotonu)
  const minSpeed = 80;      // Zvýšeno na 80 pro spolehlivé překonání tření
  const rampUpDeg = 8.0;    // Sníženo z 15.0 na 8.0 pro velmi agresivní rozjezd
  const rampDownDeg = 35.0; // Zvětšeno z 30.0 na 35.0 pro plynulé a přesné dobrzdění do cíle

  let lastLogTime = 0;

  while (!isEmergencyLatched()) {
    if (isPressed(emergencyPin)) {
      await emergencyStopCallback();
      break;
    }

    const currentAngle = Math.abs(angleState.angleZ);
    const error = targetAbs - currentAngle;

    // Rozjezdová rampa
    let speedAccel = maxSpeed;
    if (currentAngle < rampUpDeg) {
      let ratio = currentAngle / rampUpDeg;
      if (ratio < 0.0) ratio = 0.0;
      speedAccel = minSpeed + (maxSpeed - minSpeed) * ratio;
    }

    // Brzdná rampa
    let speedDecel = maxSpeed;
    if (error < rampDownDeg) {
      let ratio = error / rampDownDeg;
      if (ratio < 0.0) ratio = 0.0;
      speedDecel = minSpeed + (maxSpeed - minSpeed) * ratio;
    }

    // Výsledná rychlost je dána pomalejší z obou ramp
    const currentSpeed = Math.min(speedAccel, speedDecel);

    // Směr otáčení: CCW (kladný úhel) = -1.0 (točí vlevo), CW (záporný úhel) = 1.0 (točí vpravo)
    const curve = targetAngle > 0 ? -1.0 : 1.0;

    // Logování průběhu otáčení každých 100 ms
    const nowLog = Date.now();
    if (nowLog - lastLogTime > 100) {
      lastLogTime = nowLog;
      console.log(`Otáčení: úhel ${angleState.angleZ.toFixed(1)}° / cíl ${targetAngle.toFixed(1)}° | Rychlost: ${currentSpeed.toFixed(0)} mm/s | Výkon: ${(curve * (currentSpeed / maxSpeed)).toFixed(2)}`);
    }

    // Pokud jsme dosáhli cílového úhlu s tolerancí 1.0 stupně, otáčení končí
    if (error <= 1.0) {
      console.log(`Otáčení úspěšně dokončeno. Koncový úhel: ${angleState.angleZ.toFixed(1)} °`);
      break;
    }

    // Nastavení rychlosti a vyvolání pohybu (neblokující)
    robutek.setSpeed(currentSpeed);
    robutek.move(curve);

    await sleep(10);
  }

  // Zastavení motorů
  try {
    await robutek.stop(true);
  } catch (e) {}
  try {
    robutek.setSpeed(0);
  } catch (e) {}

  setAllLeds(PURPLE); // Hotovo
  await sleep(300); // Krátká pauza na uklidnění po otočení
}
