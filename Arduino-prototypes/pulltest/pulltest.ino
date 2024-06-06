#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include "HX711.h"

// Define pin numbers for buttons
const int buttonPin1 = 2;
const int buttonPin2 = 3;
const int buttonPin3 = 4; // Button for initiating the test

// Define stepper motor parameters
const int stepsPerRevolutionX = 400;  // Steps per revolution for X axis motor
const int stepsPerRevolutionY = 400;  // Steps per revolution for Y axis motor
const int stepsPerRevolutionZ = 400;  // Steps per revolution for Z axis motor
const int xStep = 60;
const int xDir = 61;
const int yStep = 54;
const int yDir = 55;
const int zStep = 56;
const int zDir = 57;  // Corrected to a different pin for Z direction
const int enaPin = 13;

// Define servo pins
#define AIR_VALVE_SERVO_PIN 5
#define VACUUM_PUMP_SERVO_PIN 6

// Define load cell pins
#define LOADCELL_DOUT_PIN  7
#define LOADCELL_SCK_PIN  8

// Initialize stepper motors
AccelStepper stepperX(AccelStepper::DRIVER, xStep, xDir);
AccelStepper stepperY(AccelStepper::DRIVER, yStep, yDir);
AccelStepper stepperZ(AccelStepper::DRIVER, zStep, zDir);

// Create servo objects
Servo airValveServo;
Servo vacuumPumpServo;

// Create load cell object
HX711 scale;

// Load cell calibration factor
const float calibration_factor = 79060.0;

// Variable to track if the button was pressed
bool buttonPressed = false;

void setup() {
  // Initialize Serial communication at a baud rate of 9600
  Serial.begin(9600);
  
  // Configure the button pins as inputs
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(enaPin, OUTPUT);
  
  // Enable the stepper drivers by setting the enable pin LOW (or HIGH if required by your driver)
  digitalWrite(enaPin, HIGH);
  
  // Set the maximum speed and acceleration for the stepper motors
  stepperX.setMaxSpeed(2000); // Increased max speed
  stepperX.setAcceleration(2000); // Increased acceleration
  stepperY.setMaxSpeed(2000); // Increased max speed
  stepperY.setAcceleration(2000); // Increased acceleration
  stepperZ.setMaxSpeed(2000); // Increased max speed
  stepperZ.setAcceleration(2000); // Increased acceleration

  // Attach servo motors
  airValveServo.attach(AIR_VALVE_SERVO_PIN);
  vacuumPumpServo.attach(VACUUM_PUMP_SERVO_PIN);

  airValveServo.write(180);
  vacuumPumpServo.write(50);
  delay(250);
  vacuumPumpServo.write(60);

  // Initialize load cell
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(calibration_factor); // This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare(); // Assuming there is no weight on the scale at start up, reset the scale to 0
}

void loop() {
  // Read the state of the buttons
  int buttonState1 = digitalRead(buttonPin1);
  int buttonState2 = digitalRead(buttonPin2);
  int buttonState3 = digitalRead(buttonPin3); // Button for initiating the test
  
  // Control Z-axis stepper based on button presses
  if (buttonState1 == HIGH) {
    stepperZ.move(stepsPerRevolutionZ); // Move Z-axis up
  } else if (buttonState2 == HIGH) {
    stepperZ.move(-stepsPerRevolutionZ); // Move Z-axis down
  }
  
  // Read the position of the joystick
  int joystickX = analogRead(A0); // Read X-axis
  int joystickY = analogRead(A1); // Read Y-axis
  
  // Define the dead zone for the joystick
  const int deadZone = 100; // Increased dead zone
  const int centerValue = 512; // Adjusted center value

  // Calculate the distance from the center for both axes
  int deltaX = joystickX - centerValue;
  int deltaY = joystickY - centerValue;

  // Check if the joystick is outside the dead zone
  if (abs(deltaX) > deadZone) {
    // Map the joystick position to a speed value
    float speedX = map(abs(deltaX), deadZone, 512, 0, 2000); // Adjusted speed mapping
    stepperX.setSpeed((deltaX > 0) ? speedX : -speedX);
    stepperX.runSpeed();
  }

  if (abs(deltaY) > deadZone) {
    // Map the joystick position to a speed value
    float speedY = map(abs(deltaY), deadZone, 512, 0, 2000); // Adjusted speed mapping
    stepperY.setSpeed((deltaY > 0) ? speedY : -speedY);
    stepperY.runSpeed();
  }

  // Run the Z stepper motor to reflect the button presses
  stepperZ.run();

  // Check if the test button is pressed
  if (buttonState3 == HIGH && !buttonPressed) {
    buttonPressed = true; // Set buttonPressed flag to true

    // Move Z-axis up by 5 turns
    stepperZ.move(5 * stepsPerRevolutionZ);
    while (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    }

    // Open pressurized air valve for 2 seconds
    airValveServo.write(0); // Open valve with pressurized air
    delay(2000); // Wait for 2 seconds
    airValveServo.write(180); // Reset to resting position

    // Move Z-axis down
    stepperZ.move(-5 * stepsPerRevolutionZ);
    while (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    }

    // Turn on vacuum pump
    vacuumPumpServo.write(115); // Turn vacuum pump on
    delay(250);
    vacuumPumpServo.write(105);
    delay(1000); // Wait for 1 second

    // Move Z-axis up
    stepperZ.move(5 * stepsPerRevolutionZ);
    while (stepperZ.distanceToGo() != 0) {
      stepperZ.run();
    }

    // Turn off vacuum pump
    vacuumPumpServo.write(50); // Turn vacuum pump off
    delay(250);
    vacuumPumpServo.write(60);

    // Reset buttonPressed flag
    buttonPressed = false;
  }

  // Log load cell values
  Serial.print("Reading: ");
  Serial.print(scale.get_units(), 3); // scale.get_units() returns a float
  Serial.println(); // Print a newline
}
