#include <Servo.h>
#include <HX711.h>

// --- Pin Definitions ---
const uint8_t LOADCELL_DOUT_PIN = 3;
const uint8_t LOADCELL_SCK_PIN = 2;
const uint8_t ESC_PIN = 9;

// --- Constants ---
const unsigned long SERIAL_BAUD = 9600;
const long PRINT_INTERVAL = 200;    // Print data every 200ms
const int MIN_PULSE = 1100;         // Standard ESC Minimum
const int MAX_PULSE = 1980;         // Standard ESC Maximum
const int SLEW_STEP = 5;            // Pulse change per cycle (Ramping speed)
const int SLEW_DELAY = 10;          // Delay between steps in ms

// --- Global Objects ---
HX711 scale;
Servo esc;

// --- State Variables ---
float calibration_factor = -437.0; 

int target_pulse = 1100;            // Target speed (user input)
int current_pulse = 1100;           // Actual current speed (ramped)
unsigned long last_print_time = 0;
unsigned long last_ramp_time = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 1. Initialize Scale
  Serial.println(F("Initializing Load Cell..."));
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor);
  scale.tare(); // Reset scale to 0

  // 2. Initialize ESC
  Serial.println(F("Initializing ESC..."));
  // Attach with explicit min/max to ensure map matches standard protocol
  esc.attach(ESC_PIN, MIN_PULSE, MAX_PULSE); 
  
  // Send Minimum Signal to Arm ESC (Safety Critical)
  esc.writeMicroseconds(MIN_PULSE); 
  
  Serial.println(F("System Ready."));
  Serial.println(F("Enter PWM value (1100 - 1980) in the Serial Monitor."));
}

void loop() {
  // --- A. Serial Command Parsing ---
  if (Serial.available() > 0) {
    // parseInt looks for the next valid integer in the incoming serial stream
    int input = Serial.parseInt();
    
    // Clear the buffer of any newline characters to prevent '0' readings
    while(Serial.available() > 0) {
      Serial.read();
    }

    // Safety Check: Only update if input is within ESC bounds
    if (input >= MIN_PULSE && input <= MAX_PULSE) {
      target_pulse = input;
      Serial.print(F("Command Received: Target set to "));
      Serial.println(target_pulse);
    } 
    else if (input != 0) { 
      // Filter out 0 which often occurs from timeouts or empty lines
      Serial.print(F("Error: Input "));
      Serial.print(input);
      Serial.println(F(" is out of bounds (1000-2000)."));
    }
  }

  // --- B. Ramping Logic (Slew Rate Limiter) ---
  // We use millis() to create a non-blocking delay.
  // This slowly moves current_pulse toward target_pulse to prevent mechanical jerk.
  unsigned long current_time = millis();
  
  if (current_time - last_ramp_time >= SLEW_DELAY) {
    last_ramp_time = current_time;

    if (current_pulse < target_pulse) {
      current_pulse += SLEW_STEP;
      // Prevent overshooting
      if (current_pulse > target_pulse) current_pulse = target_pulse;
    } 
    else if (current_pulse > target_pulse) {
      current_pulse -= SLEW_STEP;
      // Prevent undershooting
      if (current_pulse < target_pulse) current_pulse = target_pulse;
    }
    
    // Write the actual signal to the ESC
    esc.writeMicroseconds(current_pulse);
  }

  // --- C. Telemetry Output ---
  if (current_time - last_print_time >= PRINT_INTERVAL) {
    last_print_time = current_time;
    
    Serial.print(F("PWM: "));
    Serial.print(current_pulse);
    Serial.print(F(" us | Thrust: "));
    Serial.print(scale.get_units(), 2); // 2 decimal places
    Serial.println(F(" units")); // usually grams or kg based on calibration
  }
}