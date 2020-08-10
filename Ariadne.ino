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
const int ButtonPin = 0;
// LED Pins
const int ButtonLEDPin = 0;
const int StartLEDPin = 13;

void setup() {
  // put your setup code here, to run once:
  // Servo Setup
  UltraSonicServo.attach(ServoPin);
  UltraSonicServo.write(90);

  // Led Setup
  pinMode(StartLEDPin, OUTPUT);
  pinMode(ButtonLEDPin, OUTPUT);

  //Button Setup
  pinMode(ButtonPin, INPUT);
  // Blinks LEDs indicating startup
  BlinkStartLed(3);
  
  digitalWrite(RightMotorForwardPin, HIGH);
  digitalWrite(LeftMotorForwardPin, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Pathfinding Modes
  if (ModeButtonRead() == HIGH){

  }else
  {
    
  }
  
}

void BlinkStartLed(int Length) {
  for(int i = 0; i<=Length; i++){
    digitalWrite(StartLEDPin, HIGH);
    delay(500);
    digitalWrite(StartLEDPin, LOW);
    delay(500);
  }
  digitalWrite(StartLEDPin, HIGH);
}

int ModeButtonRead() {
  // Sets the LED State based on whether the button is down
  int ModeButtonState = digitalRead(ButtonPin); 
  digitalWrite(ButtonLEDPin,ModeButtonState);
  // Returns the button state
  return ModeButtonState;
}
