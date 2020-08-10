//Libraries

#include <Servo.h>

// Motor Pins
const int RightMotorForwardPin = 5;
const int RightMotorBackwardPin = 6;
const int LeftMotorForwardPin = 10;
const int LeftMotorBackwardPin = 9;
// Servo Pins
const int ServoPin = 12;
Servo UltraSonicServo;
// Ultra Sonic Sensor
const int UltraSonicPinTrig = 0;
const int UltraSonicPinEcho = 0;
// Line Following Sensor
const int RightSensorPin = 0;
const int CenterSensorPin = 0;
const int LeftSensorPin = 0;
// Buttons
const int RightButtonPin = 0;
const int ButtonState;
// LED Pins
const int RightLEDPin = 0;
const int StartLEDPin = 13;
const int LeftLEDPin = 0;   

void setup() {
  // put your setup code here, to run once:
  // Servo Setup
  UltraSonicServo.attach(ServoPin);
  UltraSonicServo.write(90);

  pinMode(StartLEDPin, OUTPUT);
  
  BlinkStartLed();
  
  digitalWrite(5, HIGH);
  digitalWrite(10, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void BlinkStartLed() {
  digitalWrite(StartLEDPin, HIGH);
  delay(500);
  digitalWrite(StartLEDPin, LOW);
  delay(500);
  digitalWrite(StartLEDPin, HIGH);
  delay(500);
  digitalWrite(StartLEDPin, LOW);
  delay(500);
  digitalWrite(StartLEDPin, HIGH);
}