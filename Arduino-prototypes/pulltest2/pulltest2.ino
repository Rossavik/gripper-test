#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include "HX711.h"

// Pin definitions
const int buttonPinStart = 4; // Start test sequence
const int buttonPin1 = 2;     // CNC control down
const int buttonPin2 = 3;     // CNC control up
const int servoPinAir = 5;    // Servo for air valve
const int servoPinVacuum = 6; // Servo for vacuum pump
const int xStep = 60, xDir = 61, yStep = 54, yDir = 55, zStep = 56, zDir = 57, enaPin = 13;
const int LOADCELL_DOUT_PIN = 7, LOADCELL_SCK_PIN = 8;
const int joystickXPin = A15, joystickYPin = A14; // Joystick analog pins

void performSequence();  // Declaration of the function
void handleCNCControls();

// CNC Steppers
AccelStepper stepperX(AccelStepper::DRIVER, xStep, xDir);
AccelStepper stepperY(AccelStepper::DRIVER, yStep, yDir);
AccelStepper stepperZ(AccelStepper::DRIVER, zStep, zDir);

// Servos
Servo servoAir, servoVacuum;

// Load cell
HX711 scale;
float maxForce = 0; // Track maximum force

void setup() {
  Serial.begin(9600);

  // Set servo positions:
  servoAir.write(180);
  servoVacuum.write(50);
  delay(250);
  servoVacuum.write(60);

  // Initialize steppers
  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH);
  stepperX.setMaxSpeed(1000); stepperX.setAcceleration(2000);
  stepperY.setMaxSpeed(1000); stepperY.setAcceleration(2000);
  stepperZ.setMaxSpeed(10000); stepperZ.setAcceleration(2000);

  // Initialize button inputs
  pinMode(buttonPinStart, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  // Initialize servos
  servoAir.attach(servoPinAir);
  servoVacuum.attach(servoPinVacuum);

  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(79060.0); // Calibration factor
  scale.tare();
}

void loop() {
  static bool sequenceActive = false;

  // CNC control from joystick and buttons
  handleCNCControls();

  // Check start sequence button
  if (digitalRead(buttonPinStart) == HIGH && !sequenceActive) {
    sequenceActive = true;
    performSequence();
    sequenceActive = false;
  }

  // Load cell reading
  float currentForce = scale.get_units();
  if (currentForce > maxForce) {
    maxForce = currentForce;
  }
  //Serial.print("Max force: ");
  //Serial.print(maxForce, 3);
  //Serial.println(" kg");

  stepperX.runSpeed();
  stepperY.runSpeed();
  stepperZ.run();
}

void handleCNCControls() {
    //Serial.println("Checking CNC controls"); // Debugging output

    int buttonState1 = digitalRead(buttonPin1);
    int buttonState2 = digitalRead(buttonPin2);

    //Serial.print("Button State 1 (up): "); Serial.println(buttonState1);  // Debugging output
    //Serial.print("Button State 2 (down): "); Serial.println(buttonState2);  // Debugging output

    if (buttonState1 == HIGH) {
        stepperZ.move(-200); // Move Z-axis up
        Serial.println("Moving Z-axis up"); // Debugging output
    } else if (buttonState2 == HIGH) {
        stepperZ.move(200); // Move Z-axis down
        Serial.println("Moving Z-axis down"); // Debugging output
    }

    // Ensure stepper commands are processed
    stepperZ.run();

    // Read the position of the joystick
    int joystickX = analogRead(joystickXPin);
    int joystickY = analogRead(joystickYPin);

    //Serial.print("Joystick X Position: "); Serial.println(joystickX); // Debugging output
    //Serial.print("Joystick Y Position: "); Serial.println(joystickY); // Debugging output

    const int deadZone = 30;
    const int centerValue = 511.5;

    int deltaX = joystickX - centerValue;
    int deltaY = joystickY - centerValue;

    if (abs(deltaX) > deadZone) {
        float speedX = map(abs(deltaX), deadZone, 512, 0, 2000);
        stepperX.setSpeed((deltaX > 0) ? speedX : -speedX);
        stepperX.runSpeed();
        Serial.print("Stepper X Speed: "); Serial.println(speedX); // Debugging output
    } else {
        stepperX.stop();
    }

    if (abs(deltaY) > deadZone) {
        float speedY = map(abs(deltaY), deadZone, 512, 0, 2000);
        stepperY.setSpeed((deltaY > 0) ? speedY : -speedY);
        stepperY.runSpeed();
        Serial.print("Stepper Y Speed: "); Serial.println(speedY); // Debugging output
    } else {
        stepperY.stop();
    }
}

bool vacuumPumpOn = false;  // Flag to track vacuum pump status

void performSequence() {
    // Move Z-axis up 5 turns
    stepperZ.move(5 * 200);
    while (stepperZ.distanceToGo() != 0) {
        stepperZ.run();
    }

    // Open air valve
    servoAir.write(0); 
    delay(2000); // Wait for 2 seconds
    servoAir.write(180); // Close valve

    // Move Z-axis down
    stepperZ.move(-5 * 200);
    while (stepperZ.distanceToGo() != 0) {
        stepperZ.run();
    }

    // Turn on vacuum pump
    vacuumPumpOn = true;  // Set vacuum pump status to ON
    servoVacuum.write(115);
    delay(1000); // Vacuum on for 1 second
    servoVacuum.write(105); // Adjust to prevent strain on servo

    // Measure and record the maximum force
    maxForce = 0; // Reset max force at the start of vacuum pump operation
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {  // Duration vacuum is on
        float currentForce = scale.get_units();
        if (currentForce > maxForce) {
            maxForce = currentForce;
        }
    }

    // Turn off vacuum pump
    vacuumPumpOn = false;  // Set vacuum pump status to OFF
    servoVacuum.write(50);
    delay(250); 
    servoVacuum.write(60); // Adjust to prevent strain on servo

    // Print max force after vacuum pump operation
    Serial.print("Max force during vacuum operation: ");
    Serial.print(maxForce, 3);
    Serial.println(" kg");
}
