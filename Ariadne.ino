//Libraries

#include <Servo.h>

// Motor Pins
int RightMotorForwardPin = 5;
int RightMotorBackwardPin = 6;
int LeftMotorForwardPin = 10;
int LeftMotorBackwardPin = 9;
// Servo Pins
int ServoPin = 12;
Servo UltraSonicServo;
// Ultra Sonic Sensor
int UltraSonicPinTrig = 0;
int UltraSonicPinEcho = 0;
// Line Following Sensor
int RightSensorPin = 0;
int CenterSensorPin = 0;
int LeftSensorPin = 0;
// Button Pins
int RightButtonPin = 0;
int LeftButtonPin = 0;
// LED Pins
int RightLEDPin = 0;
int StartLEDPin = 13;
int LeftLEDPin = 0;   

void setup() {
  // put your setup code here, to run once:
  // Servo Setup
  UltraSonicServo.attach(ServoPin);
  UltraSonicServo.write(90);

  pinMode(StartLEDPin, OUTPUT);
  
  digitalWrite(StartLEDPin, HIGH);
  delay(500);
  digitalWrite(StartLEDPin, LOW);
  delay(500);
  digitalWrite(StartLEDPin, HIGH);
  delay(500);
  digitalWrite(StartLEDPin, LOW);
  delay(500);
  digitalWrite(StartLEDPin, HIGH);
  
  digitalWrite(5, HIGH);
  digitalWrite(10, HIGH);

}

void loop() {
  // put your main code here, to run repeatedly:
}
