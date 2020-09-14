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
//String AngleString;
// Line Following Sensor
const int LeftSensorPin = A0;
const int CenterSensorPin = A1;
const int RightSensorPin = A2;
// Buttons
const int ButtonPin = A5;
// LED Pins
const int ButtonLEDPin = 1;
const int StartLEDPin = 2;
// Ultrasonic variables
long duration;
int distance;

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
  Serial.println((ModeButtonRead())? "High":"Low");
  if (ModeButtonRead() == HIGH) {
    // Wall Avoiding Mode
    int TurnAngle = GetTurnDirection();
    
    //Serial.println(UltrasonicRead(90));
    //Serial.println(UltrasonicRead(0));
    //Serial.println(UltrasonicRead(180));
  } else {
    // Line Foellowing
    /*if (digitalRead(LeftSensorPin) == HIGH){
      digitalWrite(LeftMotorForwardPin, HIGH);
      digitalWrite (RightMotorForwardPin, LOW);
    }
  
    if (digitalRead(CenterSensorPin) == HIGH){
      digitalWrite (LeftMotorForwardPin, HIGH);
      digitalWrite (RightMotorForwardPin, HIGH);
    }
    if (digitalRead(RightSensorPin) == HIGH){
      digitalWrite (LeftMotorForwardPin, LOW);
      digitalWrite (RightMotorForwardPin, HIGH);
    }*/
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

bool ModeButtonRead() {
  // Sets the LED State based on whether the button is down
  bool ModeButtonState = analogRead(ButtonPin)<100; 
  digitalWrite(ButtonLEDPin,ModeButtonState);

  // Returns the button state
  return ModeButtonState;
}

int UltrasonicRead(int ServoAngle){
  UltraSonicServo.write(ServoAngle);

  // Clears the trigPin
  digitalWrite(UltraSonicPinTrig, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(UltraSonicPinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltraSonicPinTrig, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds(μs)
  duration = pulseIn(UltraSonicPinEcho, HIGH);
  
  // Calculating the distance
  distance = duration*0.034/2;
  //The duration is divided by 2 because the pulse travels to from the sensor to the surface back to the sensor meaning it travels twice the distance between the sensor and the surface and is multiplied by 0.034 because the speed of sound is 0.034 cm/μs and using the formula d=v*t
  return distance;
}
// Method that decides which direction to turn 
int GetTurnDirection(){
  for(int ServoAngle = 0; ServoAngle<=180; ServoAngle+=45){
    //AngleString = String(Angle);
    Serial.println(String(ServoAngle));
    UltrasonicRead(ServoAngle);
    delay(1000);
  }
  //TODO: Return Result
  return 1;
}
