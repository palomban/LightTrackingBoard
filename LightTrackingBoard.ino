// LightTrackingBoard.ino
// Copyright 2021 Nicholas F. Palomba
// https://www.thenicholaspalomba.com/
// Published on 1/4/2021
// This Arduino Uno code tracks a light source via a proportional-integral-derivative (PID) closed loop control system
// The position of the light source is measured and this error signal is processed by the PID algortihm such that
// the output signal to the servo motor can accurately follow the light source with minimul overshoot, rise time, and steady-state error.
// Refer to README.md for the required operating environment

#include <Servo.h>

// Initialize Global Variables
Servo theServo; // Initialize servo object
int LDRL; // Light Resistor 0 value (0 = no light, 1023 = maximum light)
int LDRR; // Light Resistor 1 value (0 = no light, 1023 = maximum light)
int diff; // Difference between the two light resistor values (-1023 = maximum on right, 1023 = maximum on left)
int error; // Position error
int lasterror = 0; // Position error from previous iteration
double pos; // Position (in degrees) to move the servo to
double last_pos; // Last position (in degrees) the servo was moved to
double mov = 0; // Degrees needed to move the servo
double errorP = 0; // Calculated proportional position error
double errorI = 0; // Calculated integral position error
double errorD = 0; // Calculated differential position error
double lasterrorI = 0; // Calculated integral position error from previous iteration
long past_time; // Value indicating last discrete PID output
long current_time; // Value indicating current discrete PID output
long elapsed_time; // Value indicating time between the last PID output and current PID output

// Define Global PID Gain Terms (Tuning may be required)
double kP = 0.5;
double kI = 0.001;
double kD = 1.3;

// Define Global Constants
double initial_pos = 90; // Initial position to move to after start up

// Define Arduino Pin I/O
int const servoPin = 11; // Servo signal pin
int const leftLDR = A1; // Left photoresistor input
int const rightLDR = A0; // Right photoresistor input

void setup() {  
    // Initialize serial communication between the PC and Arduino 
    Serial.begin(9600);
    // Attach servo signal pin
    theServo.attach(servoPin);
    // Move to the servo the intial position
    pos = initial_pos;
    theServo.write(pos);
    last_pos = pos;
    // Wait 3 seconds
    delay(3000);
    // Initialize time at beginning of PID control
    past_time = millis();
}

void loop() {
    // Read and print light resistor signals from analog inputs 0 and 1
    LDRL = analogRead(leftLDR);
    LDRR = analogRead(rightLDR);
    Serial.print(LDRL);
    Serial.print('\t');
    Serial.print(LDRR);
    Serial.print('\t');    
    // Calculate the photoresistor error signal
    diff = LDRL - LDRR;
    // Map light sensor error to a servo position error and print
    error = map(diff, -1023, 1023, -180, 180);
    Serial.print("error: ");
    Serial.print(error);
    Serial.print('\t'); 
    // Calculate the time elapsed
    current_time = millis();
    elapsed_time = current_time - past_time;
    Serial.print("Time: ");
    Serial.print(elapsed_time);
    Serial.print('\t');
    // Calculate the PID control terms
    errorP = error * kP;
    errorI = (lasterrorI + (error * elapsed_time)) * kI;
    errorD = ((error - lasterror) / elapsed_time) * kD;
    // Update the servo position
    mov = errorP + errorI + errorD;
    Serial.print("Total PID Error: ");
    Serial.print(mov);
    Serial.print('\t'); 
    pos = last_pos + mov;
    if (pos < 0) {
      pos = 0;
      if (last_pos == pos){
        error = 0;
        errorP = 0;
        errorI = 0;
        errorD = 0;
      }
    }
    if (pos > 180) {
      pos = 180;
      if (last_pos == pos){
        error = 0;
        errorP = 0;
        errorI = 0;
        errorD = 0;
      }
    }
    theServo.write(pos);
    // Print position information
    Serial.print("errorP: ");
    Serial.print(errorP);
    Serial.print('\t');
    Serial.print("errorI: ");
    Serial.print(errorI);
    Serial.print('\t');
    Serial.print("errorD: ");
    Serial.print(errorD);
    Serial.print('\t');
    Serial.print("pos: ");
    Serial.println(pos);
    // Remember last position
    last_pos = pos;
    // Remember the past time count variable
    past_time = current_time;
    // Remember error and integral error for next iteration
    lasterror = error;
    lasterrorI = errorI;
}
