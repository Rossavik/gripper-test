#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include "HX711.h"
#include <SoftwareSerial.h>

// Pin definitions
const int buttonPinStart = 4; // Start test sequence
const int buttonPin1 = 2;     // CNC control down
const int buttonPin2 = 3;     // CNC control up
const int servoPinAir = 5;    // Servo for air valve
const int servoPinVacuum = 6; // Servo for vacuum pump
const int xStep = 60, xDir = 61, yStep = 54, yDir = 55, zStep = 56, zDir = 57, enaPin = 13;
const int LOADCELL_DOUT_PIN = 7, LOADCELL_SCK_PIN = 8;
const int joystickXPin = A15, joystickYPin = A14; // Joystick analog pins
const int vacuumPumpRelayPin = 10; // Relay control pin for the vacuum pump

bool sequenceActive = false; // Declare sequenceActive globally

void performSequence();  // Declaration of the functions
void handleCNCControls();

// CNC Steppers
AccelStepper stepperX(AccelStepper::DRIVER, xStep, xDir);
AccelStepper stepperY(AccelStepper::DRIVER, yStep, yDir);
AccelStepper stepperZ(AccelStepper::DRIVER, zStep, zDir);

const int stepsPerRevolutionX = 200;  // Steps per revolution for X axis motor
const int stepsPerRevolutionY = 200;  // Steps per revolution for Y axis motor
const int stepsPerRevolutionZ = 200;  // Steps per revolution for Z axis motor

// Servos
Servo servoAir, servoVacuum;

// Load cell
HX711 scale;
float maxForce = 0; // Track maximum force

void setup() {
  Serial.begin(9600);

  // Set servo positions:
  servoAir.write(180);
  //servoVacuum.write(45);
  //delay(250);
  //servoVacuum.write(60);

  pinMode(vacuumPumpRelayPin, OUTPUT);  // Set the relay pin as an output
  digitalWrite(vacuumPumpRelayPin, LOW);  // Ensure relay is off at start

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
    handleCNCControls();

    if (digitalRead(buttonPinStart) == HIGH && !sequenceActive) {
        sequenceActive = true;  // Begin the sequence
    }

    if (sequenceActive) {
        performSequence();  // Continue running sequence
    }

    // Continuously process stepper movements outside of conditional checks
    stepperX.runSpeed();
    stepperY.runSpeed();
    stepperZ.run();
}

void handleCNCControls() {
    // Read joystick and button states
    int buttonState1 = digitalRead(buttonPin1);
    int buttonState2 = digitalRead(buttonPin2);
    int joystickX = analogRead(joystickXPin);
    int joystickY = analogRead(joystickYPin);

    // Handle Z-axis buttons
    if (buttonState1 == HIGH) {
        stepperZ.move(-stepsPerRevolutionZ/4); // Move Z-axis up
    } else if (buttonState2 == HIGH) {
        stepperZ.move(stepsPerRevolutionZ/4); // Move Z-axis down
    }

    // Calculate joystick deviations
    int deltaX = joystickX - 511.5;
    int deltaY = joystickY - 511.5;

    // Apply dead zone and control X and Y steppers
    if (abs(deltaX) > 15) {
        stepperX.setSpeed((deltaX > 0 ? 1 : -1) * map(abs(deltaX), 15, 512, 0, 2000));
    } else {
        stepperX.setSpeed(0);
    }
    if (abs(deltaY) > 15) {
        stepperY.setSpeed((deltaY > 0 ? 1 : -1) * map(abs(deltaY), 15, 512, 0, 2000));
    } else {
        stepperY.setSpeed(0);
    }
}

bool vacuumPumpOn = false;  // Flag to track vacuum pump status

void performSequence() {
    static int step = 0;
    static unsigned long timer = 0;
    static unsigned long airValveTimer = 0;

    switch (step) {
        case 0:
            Serial.println("Moving Z-axis up 15 turns");
            //stepperZ.move(15 * 200);  // Initiate moving Z-axis up 15 turns
            step++;
            break;
        case 1:
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Opening air valve");
                servoAir.write(0);
                timer = millis();  // Start timing for air valve operation
                step++;
            }
            break;
        case 2:
            if (millis() - timer > 750) {  // Wait 750 ms after opening the valve before starting descent
                stepperZ.move(-15 * 200);  // Move Z-axis down
                airValveTimer = millis();  // Start timer for closing air valve
                step++;
            }
            break;
        case 3:
            if (millis() - airValveTimer > 1100) {  // Close valve 1.1 seconds after starting descent
                servoAir.write(180);  // Close valve
            }
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Starting vacuum pump");
                digitalWrite(vacuumPumpRelayPin, HIGH);  // Turn on vacuum pump via relay
                timer = millis();  // Reset timer for vacuum operation
                step++;
            }
            break;
        case 4:
            if (millis() - timer < 1000) {  // Continue force measurements for 3 seconds
                float currentForce = scale.get_units();
                Serial.println(currentForce);
                if (currentForce > maxForce) {
                    maxForce = currentForce;
                }
            } else {
                stepperZ.move(15 * 200);  // Move Z-axis up again after measurements
                step++;
            }
            break;
        case 5:
            while (stepperZ.distanceToGo() > 0) {
                stepperZ.run();
                float currentForce = scale.get_units();
                if (currentForce > maxForce) {
                    maxForce = currentForce;
                    Serial.println(maxForce);
                }
            }
            digitalWrite(vacuumPumpRelayPin, LOW);  // Turn off vacuum pump
            Serial.print("Max force during vacuum operation: ");
            Serial.print(maxForce, 3);
            Serial.println(" kg");
            step++;  // Move to final step
            break;
        case 6:
            Serial.println("Sequence complete, ready for next operation");
            step = 0;  // Reset the sequence to allow it to be triggered again
            sequenceActive = false;
            break;
    }
    stepperZ.run();  // Continuously call to process stepper commands
}