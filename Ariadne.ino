//Libraries
#include <Servo.h>

// Motor Pins
const int RightMotorForwardPin = 5;
const int RightMotorBackwardPin = 6;
const int LeftMotorForwardPin = 10;
const int LeftMotorBackwardPin = 9;
// Servo Pins
const int ServoPin = 11;
Servo UltraSonicServo;
// Ultra Sonic Sensor
const int UltraSonicPinTrig = 12;
const int UltraSonicPinEcho = 13;
// Line Following Sensor
const int LeftSensorPin = A0;
const int CenterSensorPin = A1;
const int RightSensorPin = A2;
// Buttons
const int ButtonPin = 0;
// LED Pins
const int ButtonLEDPin = 1;
const int StartLEDPin = 2;

void setup() {
  Serial.begin(9600);

  // put your setup code here, to run once:
  // Servo Setup
  UltraSonicServo.attach(ServoPin);
  UltraSonicServo.write(90);

  // Led Setup
  pinMode(StartLEDPin, OUTPUT);
  pinMode(ButtonLEDPin, OUTPUT);

  // Line Following Sensor Setup
  pinMode(LeftSensorPin, INPUT);
  pinMode(CenterSensorPin, INPUT);
  pinMode(RightSensorPin, INPUT);

  //Button Setup
  pinMode(ButtonPin, INPUT);
  // Blinks LEDs indicating startup
  BlinkStartLed(3);
  //if (!Serial.avaliable){
    digitalWrite(RightMotorForwardPin, HIGH);
    digitalWrite(LeftMotorForwardPin, HIGH);
  //}
  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Pathfinding Modes
  Serial.println((digitalRead(ButtonPin) == HIGH)? "High":"Low");
  /*if (ModeButtonRead() == HIGH) {
    // Wall Avoiding Mode

  } else {
    // Line Following

  }*/
  
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


