// Libraries  
#include <Servo.h>

// Motor Pins
const int RightMotorForwardPin = 5;
const int RightMotorBackwardPin = 6;
const int LeftMotorForwardPin = 10;
const int LeftMotorBackwardPin = 9;

const int RightMotorSpeedPin = 3;
const int LeftMotorSpeedPin = 11;
// Servo Pins
const int ServoPin = 7;
Servo UltraSonicServo;
// Ultra Sonic Sensor
const int UltraSonicPinTrig = 12;
const int UltraSonicPinEcho = 13;
// Line Following Sensor
const int LeftSensorPin = A0;
const int CenterSensorPin = A1;
const int RightSensorPin = A2;
// Buttons
const int ButtonPin = A5;
// LED Pins
const int ButtonLEDPin = 1;
const int StartLEDPin = 2;
// Line Following Settings
// TODO : Reassign Time to turn
const int TimeToTurn = 215;
// Random Mode
int RandomCycles = 0;

int cm;

enum LineFollowModes {RANDOM, LEFT, RIGHT, FORWARD, ERROR};

void setup() {
  Serial.begin(9600);
  // Servo Setup
  UltraSonicServo.attach(ServoPin);
  UltraSonicServo.write(90);
  // Ultrasonic Sensor Setup
  pinMode(UltraSonicPinEcho, INPUT);
  pinMode(UltraSonicPinTrig, OUTPUT);

  // Led Setup
  pinMode(StartLEDPin, OUTPUT);
  pinMode(ButtonLEDPin, OUTPUT);

  // Blinks LEDs indicating startup
  BlinkStartLed(3);

  analogWrite(RightMotorSpeedPin,255);
  analogWrite(LeftMotorSpeedPin,255);

  digitalWrite(RightMotorForwardPin, HIGH);
  digitalWrite(LeftMotorForwardPin, HIGH);

  delay(2500);
}

void loop() {
  Serial.println((ModeButtonRead())? "High":"Low");
  if (ModeButtonRead() == HIGH) {
    logic();
    delay(1000);
  } else {
    // Line Following
    switch (LineFollowingRead())
    {
    case FORWARD:
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, HIGH);
      RandomCycles = 0;
      break;
    case LEFT:
      // TODO : Change This Based On How The Sensor Operates
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, LOW);
      delay(TimeToTurn);
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, HIGH);
      RandomCycles = 0;
      break;
    case RIGHT:
      // TODO : Change This Based On How The Sensor Operates
      digitalWrite(RightMotorForwardPin, LOW);
      digitalWrite(LeftMotorForwardPin, HIGH);
      delay(TimeToTurn);
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, HIGH);
      RandomCycles = 0;
      break;
    case RANDOM:
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, LOW);
      // TODO : Fill out with appropriate delay
      delay(100);
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorForwardPin, HIGH);

      delay(100+100*RandomCycles);
      digitalWrite(RightMotorForwardPin, LOW);
      digitalWrite(LeftMotorForwardPin, LOW);

      RandomCycles++;
      break;
    default:
      exit(1);
      break;
    }
  }

  // put your main code here, to run repeatedly:
  /*UltraSonicServo.write(50);
  delay(400);
  cm = GetDistance();
  logic();
  UltraSonicServo.write(90);
  delay(400);
  cm = GetDistance();
  logic();
  UltraSonicServo.write(130);
  delay(400);
  cm = GetDistance();
  logic();
  UltraSonicServo.write(90);
  delay(400);
  cm = GetDistance();
  logic();
  delay(1000);*/
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

void logic() {
  if(cm < 35){
    // Establish Variables
    int DistanceLeft,DistanceRight;
    Serial.println("Wall");
    // Back Robot Up
    digitalWrite(LeftMotorForwardPin, LOW);
    digitalWrite(RightMotorForwardPin, LOW);
    digitalWrite(LeftMotorBackwardPin, HIGH);
    digitalWrite(RightMotorBackwardPin, HIGH);
    delay(500);
    digitalWrite(LeftMotorBackwardPin, LOW);
    digitalWrite(RightMotorBackwardPin, LOW);
    // Scan
    UltraSonicServo.write(10);
    delay(400);
    DistanceRight = GetDistance();
    delay(400);
    UltraSonicServo.write(170);
    delay(400);
    DistanceLeft = GetDistance();
    delay(400);
    UltraSonicServo.write(90);

    if(DistanceRight > DistanceLeft) {
      Serial.println("Wall Left");
      digitalWrite(LeftMotorForwardPin, HIGH);
      digitalWrite(RightMotorBackwardPin, HIGH);
      delay(300);
    } else if (DistanceLeft > DistanceRight) {
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorBackwardPin, HIGH);
      delay(300);
    }
    if((DistanceRight < 5) && (DistanceLeft < 5) | (DistanceLeft = DistanceRight)) {
      Serial.println("Wall Close");
      digitalWrite(LeftMotorForwardPin, HIGH);
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorBackwardPin, LOW);
      digitalWrite(RightMotorBackwardPin, LOW);
      delay(140);
    } else
    {
      digitalWrite(LeftMotorForwardPin, HIGH);
      digitalWrite(RightMotorForwardPin, HIGH);
      digitalWrite(LeftMotorBackwardPin, LOW);
      digitalWrite(RightMotorBackwardPin, LOW);
    }
    
  }
}

int GetDistance() {
  // Clears the trigPin
  digitalWrite(UltraSonicPinTrig, LOW);
  delayMicroseconds(5);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(UltraSonicPinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltraSonicPinTrig, LOW);
  return pulseIn(UltraSonicPinEcho, HIGH)*0.034/2;
}

bool ModeButtonRead() {
  // Sets the LED State based on whether the button is down
  bool ModeButtonState = analogRead(ButtonPin)<100; 
  digitalWrite(ButtonLEDPin,ModeButtonState);

  // Returns the button state
  return ModeButtonState;
}

LineFollowModes LineFollowingRead(){
  // read input from sensors
  bool leftSensor = digitalRead(LeftSensorPin) == 0;
  bool centerSensor = digitalRead(CenterSensorPin) == 0;
  bool rightSensor = digitalRead(RightSensorPin) == 0;

  LineFollowModes Result = (leftSensor && rightSensor) || (!leftSensor && !centerSensor && !rightSensor) ? RANDOM: (leftSensor&&!rightSensor) ? LEFT: (rightSensor&&!leftSensor) ? RANDOM : (centerSensor)? FORWARD : ERROR;

  return Result;
}