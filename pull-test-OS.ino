#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include "HX711.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

// Pin definitions
const int buttonPinStart = 4; // Start test sequence
const int buttonPin1 = 2;     // CNC control down
const int buttonPin2 = 3;     // CNC control up
const int servoPin = 5;       // Universal servo pin for air valve, vacuum pump, or gripper
const int xStep = 60, xDir = 61, yStep = 54, yDir = 55, zStep = 56, zDir = 57, enaPin = 13;
const int LOADCELL_DOUT_PIN = 7, LOADCELL_SCK_PIN = 8;
const int joystickXPin = A15, joystickYPin = A14; // Joystick analog pins
const int vacuumPumpRelayPin = 9; // Relay control pin for the vacuum pump

enum GripperType {BALLOON, MAGNET, CLAW};
GripperType gripperAttached = BALLOON; // Default gripper

bool sequenceActive = false; 
int sequenceCount = 0;
const int maxSequences = 20;

AccelStepper stepperX(AccelStepper::DRIVER, xStep, xDir);
AccelStepper stepperY(AccelStepper::DRIVER, yStep, yDir);
AccelStepper stepperZ(AccelStepper::DRIVER, zStep, zDir);

Servo servo;

HX711 scale;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  pinMode(vacuumPumpRelayPin, OUTPUT);  
  digitalWrite(vacuumPumpRelayPin, LOW);  

  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH);
  stepperX.setMaxSpeed(1000); stepperX.setAcceleration(2000);
  stepperY.setMaxSpeed(1000); stepperY.setAcceleration(2000);
  stepperZ.setMaxSpeed(10000); stepperZ.setAcceleration(2000);

  pinMode(buttonPinStart, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);

  servo.attach(servoPin);

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(79060.0);
  scale.tare();
}

void loop() {
  handleCNCControls();

  if (digitalRead(buttonPinStart) == HIGH && !sequenceActive) {
    sequenceActive = true;
    performSequence();
  }
}

void performSequence() {
  switch (gripperAttached) {
    case BALLOON:
      performBalloonSequence();
      break;
    case MAGNET:
      performMagnetSequence();
      break;
    case CLAW:
      performClawSequence();
      break;
  }
}

void performBalloonSequence() {
    static int step = 0;
    static unsigned long timer = 0;
    static unsigned long airValveTimer = 0;

    switch (step) {
        case 0:
            Serial.println("Moving Z-axis up 15 turns");
            //stepperZ.move(15 * 200);  // Ensure this command is active to initiate moving
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
            if (millis() - timer > 750) {
                stepperZ.move(-15 * 200);  // Move Z-axis down
                airValveTimer = millis();  // Start timer for closing air valve
                step++;
            }
            break;
        case 3:
            if (millis() - airValveTimer > 1100) {
                servoAir.write(180);  // Close valve
            }
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Starting vacuum pump");
                digitalWrite(vacuumPumpRelayPin, HIGH);
                mySerial.write('S');  // Send start measuring signal to slave
                timer = millis();
                step++;
            }
            break;
        case 4:
            if (millis() - timer < 1000) {
                // Continue checking for max force value from slave
                if (mySerial.available()) {
                    maxForce = mySerial.parseFloat();  // Read the max force value sent back from Uno
                }
            } else {
                stepperZ.move(15 * 200);  // Move Z-axis up again after measurements
                //mySerial.write('E');  // Send stop measuring signal to slave
                step++;
            }
            break;
        case 5:
            if (stepperZ.distanceToGo() == 0) {
                mySerial.write('E');
                digitalWrite(vacuumPumpRelayPin, LOW);  // Turn off vacuum pump
                delay(250);
                Serial.print("Max force during vacuum operation: ");
                Serial.print(maxForce, 3);
                Serial.println(" kg");
                step++;  // Move to final step
            }
            break;
        case 6:
            Serial.println("Sequence complete, ready for next operation");
            step = 0;  // Reset the sequence to allow it to be triggered again
            //sequenceActive = false;
            sequenceCount += 1;
            if (sequenceCount >= maxSequences){
              Serial.println("All sequences completed");
              sequenceActive = false;
              sequenceCount = 0;
            }
            delay(500);
            break;
    }
    stepperZ.run();  // Continuously call to process stepper commands
}

void performMagnetSequence() {
    static int step = 0;
    static unsigned long timer = 0;
    static unsigned long airValveTimer = 0;

    switch (step) {
        case 0:
            Serial.println("Moving Z-axis up 15 turns");
            //stepperZ.move(15 * 200);  // Ensure this command is active to initiate moving
            step++;
            break;
        case 1:
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Opening air valve");
                //servoAir.write(0);
                timer = millis();  // Start timing for air valve operation
                step++;
            }
            break;
        case 2:
            if (millis() - timer > 750) {
                stepperZ.move(-15 * 200);  // Move Z-axis down
                airValveTimer = millis();  // Start timer for closing air valve
                step++;
            }
            break;
        case 3:
            if (millis() - airValveTimer > 1100) {
                //servoAir.write(180);  // Close valve
            }
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Starting vacuum pump");
                //digitalWrite(vacuumPumpRelayPin, HIGH);
                delay(1000);
                //Serial.println("Zero servo");
                //servoMagnet.write(0);
                //delay(2000);
                //Serial.println("Engage magnet");
                //servoMagnet.write(140);
                //servoAir.write(0);
                //delay(2000);
                mySerial.write('S');  // Send start measuring signal to slave
                timer = millis();
                step++;
            }
            break;
        case 4:
            if (millis() - timer < 1000) {
                // Continue checking for max force value from slave
                if (mySerial.available()) {
                    maxForce = mySerial.parseFloat();  // Read the max force value sent back from Uno
                }
                servoMagnet.write(140);
            } else {
                stepperZ.move(15 * 200);  // Move Z-axis up again after measurements
                //mySerial.write('E');  // Send stop measuring signal to slave
                step++;
            }
            break;
        case 5:
            if (stepperZ.distanceToGo() == 0) {
                mySerial.write('E');
                //digitalWrite(vacuumPumpRelayPin, LOW);  // Turn off vacuum pump
                servoMagnet.write(5);
                delay(250);
                Serial.print("Max force during vacuum operation: ");
                Serial.print(maxForce, 3);
                Serial.println(" kg");
                step++;  // Move to final step
            }
            break;
        case 6:
            Serial.println("Sequence complete, ready for next operation");
            step = 0;  // Reset the sequence to allow it to be triggered again
            //sequenceActive = false;
            sequenceCount += 1;
            if (sequenceCount >= maxSequences){
              Serial.println("All sequences completed");
              sequenceActive = false;
              sequenceCount = 0;
            }
            delay(500);
            break;
    }
    stepperZ.run();  // Continuously call to process stepper commands
}

void performClawSequence() {
    static int step = 0;
    static unsigned long timer = 0;
    static unsigned long airValveTimer = 0;

    switch (step) {
        case 0:
            Serial.println("Moving Z-axis up 15 turns");
            //stepperZ.move(15 * 200);  // Ensure this command is active to initiate moving
            step++;
            break;
        case 1:
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Opening air valve");
                //servoAir.write(0);
                timer = millis();  // Start timing for air valve operation
                step++;
            }
            break;
        case 2:
            if (millis() - timer > 750) {
                stepperZ.move(-15 * 200);  // Move Z-axis down
                airValveTimer = millis();  // Start timer for closing air valve
                step++;
            }
            break;
        case 3:
            if (millis() - airValveTimer > 1100) {
                //servoAir.write(180);  // Close valve
            }
            if (stepperZ.distanceToGo() == 0) {
                Serial.println("Starting vacuum pump");
                //digitalWrite(vacuumPumpRelayPin, HIGH);
                delay(1000);
                //Serial.println("Zero servo");
                //servoMagnet.write(0);
                //delay(2000);
                //Serial.println("Engage magnet");
                //servoMagnet.write(140);
                //servoAir.write(0);
                //delay(2000);
                mySerial.write('S');  // Send start measuring signal to slave
                timer = millis();
                step++;
            }
            break;
        case 4:
            if (millis() - timer < 1000) {
                // Continue checking for max force value from slave
                if (mySerial.available()) {
                    maxForce = mySerial.parseFloat();  // Read the max force value sent back from Uno
                }
                servoMagnet.write(140);
            } else {
                stepperZ.move(15 * 200);  // Move Z-axis up again after measurements
                //mySerial.write('E');  // Send stop measuring signal to slave
                step++;
            }
            break;
        case 5:
            if (stepperZ.distanceToGo() == 0) {
                mySerial.write('E');
                //digitalWrite(vacuumPumpRelayPin, LOW);  // Turn off vacuum pump
                servoMagnet.write(5);
                delay(250);
                Serial.print("Max force during vacuum operation: ");
                Serial.print(maxForce, 3);
                Serial.println(" kg");
                step++;  // Move to final step
            }
            break;
        case 6:
            Serial.println("Sequence complete, ready for next operation");
            step = 0;  // Reset the sequence to allow it to be triggered again
            //sequenceActive = false;
            sequenceCount += 1;
            if (sequenceCount >= maxSequences){
              Serial.println("All sequences completed");
              sequenceActive = false;
              sequenceCount = 0;
            }
            delay(500);
            break;
    }
    stepperZ.run();  // Continuously call to process stepper commands
}

void handleCNCControls() {
  int buttonState1 = digitalRead(buttonPin1);
  int buttonState2 = digitalRead(buttonPin2);
  int joystickX = analogRead(joystickXPin);
  int joystickY = analogRead(joystickYPin);

  if (buttonState1 == HIGH) {
    stepperZ.move(-stepsPerRevolutionZ/4);
  } else if (buttonState2 == HIGH) {
    stepperZ.move(stepsPerRevolutionZ/4);
  }

  int deltaX = joystickX - 511.5;
  int deltaY = joystickY - 511.5;

  if (abs(deltaX) > 30) {
    stepperX.setSpeed((deltaX > 0 ? 1 : -1) * map(abs(deltaX), 15, 512, 0, 2000));
  } else {
    stepperX.setSpeed(0);
  }
  if (abs(deltaY) > 30) {
    stepperY.setSpeed((deltaY > 0 ? 1 : -1) * map(abs(deltaY), 15, 512, 0, 2000));
  } else {
    stepperY.setSpeed(0);
  }
}
