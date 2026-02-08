#include <Servo.h>
#include <HX711.h>

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int ESC_PIN = 9;

HX711 scale;
Servo esc;

int current_pwm = 1000;
int target_pwm = 1000;
unsigned long last_print = 0;

void setup() {
  // 1. FAST SERIAL to prevent blocking
  Serial.begin(115200); 

  // 2. Setup Scale
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-437); // Your calibration
  scale.tare();

  // 3. Setup ESC
  esc.attach(ESC_PIN);
  esc.writeMicroseconds(1000);
  delay(2000); // Wait for Arming Beeps
}

void loop() {
  // --- A. Handle Serial Input ---
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length() > 0) target_pwm = s.toInt();
    if (target_pwm < 1000) target_pwm = 1000;
    if (target_pwm > 2000) target_pwm = 2000;
  }

  // --- B. Smooth Ramping ---
  // Only update motor every 20ms (standard servo rate)
  // This prevents over-flooding the ESC with updates
  static unsigned long last_motor_update = 0;
  if (millis() - last_motor_update > 20) {
    last_motor_update = millis();
    
    if (current_pwm < target_pwm) current_pwm++;
    else if (current_pwm > target_pwm) current_pwm--;
    
    esc.writeMicroseconds(current_pwm);
  }

  // --- C. Telemetry (The Fix) ---
  // Only print every 500ms to reduce CPU load
  if (millis() - last_print > 500) {
    last_print = millis();
    
    // Check if ready FIRST to avoid freezing
    if (scale.is_ready()) {
       long reading = scale.read(); // Raw read is faster than get_units()
       // Manual math is faster than library math
       float thrust = (reading - scale.get_offset()) / scale.get_scale();
       
       Serial.print("PWM: ");
       Serial.print(current_pwm);
       Serial.print(" | Thrust: ");
       Serial.println(thrust);
    }
  }
}