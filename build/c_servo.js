import * as pwm from "pwm";
const SERVO_PIN = 21;
// Střed serva (90°)
pwm.setServoPulse(SERVO_PIN, 1500);
while (true) {
    await sleep(1000);
}
