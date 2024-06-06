#include <Servo.h>

// Define the pins
#define BUTTON_PIN 4
#define SERVO_PIN 5

// Create a servo object
Servo servoMotor;

// Variable to track if the button was pressed
bool buttonPressed = false;

void setup() {
  // Initialize the button pin as input
  pinMode(BUTTON_PIN, INPUT);

  // Attach the servo to its pin
  servoMotor.attach(SERVO_PIN);

  Serial.begin(9600);
}

void loop() {
  // Check if the button is pressed
  if (digitalRead(BUTTON_PIN) == HIGH && !buttonPressed) {
    // Button is pressed for the first time
    buttonPressed = true;
    Serial.println("Button pressed");
    // Rotate servo 30 degrees clockwise
    servoMotor.write(0); // Open valve with pressurized air
    delay(2000); // Wait for 2 seconds
    servoMotor.write(180); // Reset to resting position
    
    // Button is released
    buttonPressed = false;
  }
}


