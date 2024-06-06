#include <HX711.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo;

SoftwareSerial mySerial(10, 11); // RX, TX - Pins 10 and 11 are used for SoftwareSerial

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;
const int servoPin = 5;
HX711 scale;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(79060.0);  // Calibration factor
  scale.tare();  // Assuming there is no weight on the scale at start up
  servo.attach(servoPin);
  Serial.println("Hello World");
  servo.write(0);
}

void loop() {
  static bool measuring = false;
  static float maxForce = 0;

  if (mySerial.available()) {
    char cmd = mySerial.read();
    //Serial.print("Command recieved: ");
    //Serial.println(cmd);
    if (cmd == 'S') {
      measuring = true;
      servo.write(135);
      maxForce = 0;  // Reset max force at the start of measurement
      //Serial.println("Start Measuring...");
    } else if (cmd == 'E') {
      measuring = false;
      servo.write(0);
      delay(500);
      //Serial.println("Stop Measuring...");
      //Serial.print("Max Force: ");
      Serial.println(maxForce);  // Print the max force value
    }
  }

  if (measuring) {
    float force = scale.get_units();
    //Serial.println(force);  // Continuously print current force
    if (force > maxForce) {
      maxForce = force;
    }
  }
}
