#include <Servo.h>
#include <HX711.h>

// Pin definitions
#define LOADCELL_DOUT_PIN  3
#define LOADCELL_SCK_PIN   2
#define ESC_PIN            9

// Create objects
HX711 scale;
Servo esc;

// Calibration factor
float calibration_factor = -437; 

void setup() {
  Serial.begin(9600);
  scale.set_scale(calibration_factor);

  esc.attach(ESC_PIN);
  
  // --- Setup Messages (Plotter ignores these before the loop starts) ---
  Serial.println("Starting ESC calibration...");
  
  esc.writeMicroseconds(1000); 
  delay(1000);   
  Serial.println("ESC armed!");

  esc.writeMicroseconds(1000); // Neutral throttle
  delay(2000); 

  Serial.println("Initializing load cell...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(); 
  scale.tare();      

  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);

  Serial.println("Ramping up motor...");

  // Ramp up motor speed
  for (int i = 1000; i <= 1890; i++) {
    esc.writeMicroseconds(i);
    delay(20);
  }
  Serial.println("Thrust stand ready!");
}

void loop() {
  // 1. Maintain Motor Speed
  esc.writeMicroseconds(1890);

  // 2. Read and Output Data
  if (scale.is_ready()) {
    float thrust = scale.get_units(1); 

    Serial.print("Thrust_g:");
    Serial.print(thrust, 2);
    Serial.print(",");          // Separator
    Serial.print("Cal_Factor:");
    Serial.println(calibration_factor); // End of line
  }

  // 3. Handle Calibration Updates
  if (Serial.available()) {
    char temp = Serial.read();
    if (temp == '+' || temp == 'a') {
      calibration_factor += 10; 
    } else if (temp == '-' || temp == 'z') {
      calibration_factor -= 10; 
    }
    scale.set_scale(calibration_factor);
  }
}