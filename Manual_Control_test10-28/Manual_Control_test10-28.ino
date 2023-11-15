<<<<<<< HEAD
#include <Wire.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

//Define pins
SoftwareSerial BTSerial(0, 1); // RX | TX -->  0=blue, 1=brown

//Define ultrasonic sensors
#define TRIGGER_PIN1          48    // Front
#define TRIGGER_PIN2          49    // Left front
#define TRIGGER_PIN3          50    // Left back
#define TRIGGER_PIN4          51    // Right front
#define TRIGGER_PIN5          52    // Right back
#define TRIGGER_PIN6          53    // Back
#define ECHO_PIN1             8     // Front sensor
#define ECHO_PIN2             9     // Left front
#define ECHO_PIN3             10    // Left back
#define ECHO_PIN4             11    // Right front
#define ECHO_PIN5             12    // Right back
#define ECHO_PIN6             13    // Back
#define MaxDistance           200

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MaxDistance);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MaxDistance);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MaxDistance);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MaxDistance);
NewPing sonar5(TRIGGER_PIN5, ECHO_PIN5, MaxDistance);
NewPing sonar6(TRIGGER_PIN6, ECHO_PIN6, MaxDistance);

//Define functions
void InitMotors(void);
void InitInterrupts(void);
void DisableMotors(void);

float ReadUltrasonicSensor(int sensorNum, int numAvg);
int Rotate(float rotDegrees);
int MoveForward(float movInches);

void LeftMotorForward(void);
void LeftMotorBackward(void);
void RightMotorForward(void);
void RightMotorBackward(void);

//Define Variables
int LmotorSpeed = 100;
int RmotorSpeed = 100;
String cmdStr;
volatile long leftMotorCount = 0;
volatile long rightMotorCount = 0;

// Define motor connections
int leftMotorPin = 5;       // L DC Motor - interrupt pin (might not need it tho idk)
int rightMotorPin = 4;      // R DC Motor - interrupt pin

int rightMotorIn1 = 30;       // R DC Motor - IN1 on Motor Driver
int rightMotorIn2 = 31;       // R DC Motor - IN2 on Motor Driver
int leftMotorIn3 = 32;      // L DC Motor - IN3 on Motor Driver
int leftMotorIn4 = 33;      // L DC Motor - IN4 on Motor Driver

int rightEncA = 2;           // R DC Motor - encoder A signal (needs interrupt)
int rightEncB = 22;          // R DC Motor - encoder B signal
int leftEncA = 3;          // L DC Motor - encoder A signal (needs interrupt)
int leftEncB = 23;         // L DC Motor - encoder B signal


void setup() {
  // Initialize motors
  InitMotors();

  // attach interrupts
  InitInterrupts();
  
  // Initialize Serial communication
  Serial.begin(9600);
  //Serial.println("Enter AT commands:");
  // HC-05 default speed in AT command mode 
  BTSerial.begin(38400); 
  delay(1000);
}

void loop() {


  //get all sensor readings at once (with command ua)
  int numAvg = 2;       // total avg time
  float distanceBuffer[6];
  String strBuffer;
  for (int i=0; i<6; i++) {
    distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
    strBuffer += (String)i;
    strBuffer += "=";
    strBuffer += distanceBuffer[i];
    strBuffer += " | ";
  }
  Serial.println(strBuffer);
        
  // Read what is entered into serial monitor or bluetooth
  if (Serial.available()) {
    BTSerial.write(Serial.read());

    cmdStr = Serial.readString();
    Serial.println(cmdStr);
    
    if (cmdStr.charAt(0) == 'w') {
      // Remove first 3 characters to only get inch distance value (ex. w0-20 to 20, w0--10 to -10)
      cmdStr.remove(0,3);
      MoveForward(cmdStr.toFloat());
      
    } else if (cmdStr.charAt(0) == 'r') {
      cmdStr.remove(0,3);
      Rotate(cmdStr.toFloat());
      
    } else if (cmdStr.charAt(0) == 'u') {
      // Check which ultrasonic sensor we want to read from
      int numAvg = 2;       // total avg time
      cmdStr.remove(0,1);   // remove first char u to determine which sensor to read from

      if (cmdStr.charAt(0) == 'a') {
        //get all sensor readings at once (with command ua)
        float distanceBuffer[6];
        String strBuffer;
        for (int i=0; i<6; i++) {
          distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
          strBuffer += (String)i;
          strBuffer += "=";
          strBuffer += distanceBuffer[i];
          strBuffer += " | ";
        }
        Serial.println(strBuffer);
      } else {
        //get individual sensor readings
        float distance = ReadUltrasonicSensor(cmdStr.toInt(), numAvg);
        Serial.println(distance);
      }
      
    } else {
      Serial.println("Invalid command, please try again.");
    }
  }

//   // display encoder values
//   delay(100);
//   Serial.print("Left | Right - motor count: ");
//   Serial.print(leftMotorCount);
//   Serial.print(" | ");
//   Serial.println(rightMotorCount);
}

void InitMotors(void) {
  // Setting up motor pins
  pinMode(leftMotorPin, OUTPUT);        // pin 
  pinMode(rightMotorPin, OUTPUT);       // pin 
  
  pinMode(rightMotorIn1, OUTPUT);        // pin
  pinMode(rightMotorIn2, OUTPUT);        // pin 
  pinMode(leftMotorIn3, OUTPUT);        // pin 
  pinMode(leftMotorIn4, OUTPUT);        // 
}

void InitInterrupts(void) {
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncA), EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncB), EncoderEvent, CHANGE);
}

void DisableMotors(void) {
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, LOW);
  digitalWrite(leftMotorIn3, LOW);
  digitalWrite(leftMotorIn4, LOW);
}

void EncoderEvent() {
  
  // Left Motor
  if (digitalRead(leftEncA) == HIGH) {      // A high
    if (digitalRead(leftEncB) == LOW) {
      leftMotorCount++;
    } else {
      leftMotorCount--;
    }
  } else {                                  // A low
    if (digitalRead(leftEncB) == LOW) {
      leftMotorCount--;
    } else {
      leftMotorCount++;
    }
  }

  // Right Motor
  if (digitalRead(rightEncA) == HIGH) {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount--;
    } else {
      rightMotorCount++;
    }
  } else {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount++;
    } else {
      rightMotorCount--;
    }
  }
  
}

float ReadUltrasonicSensor(int sensorNum, int numAvg) {

  float tempVal = 0.0;

  for (int i=0; i< numAvg; i++) {
    delay(50);
    float echoCM = 0;
    if (sensorNum == 1) {
      echoCM = sonar1.ping_cm();
    } else if (sensorNum == 2) {
      echoCM = sonar2.ping_cm();
    } else if (sensorNum == 3) {
      echoCM = sonar3.ping_cm();
    } else if (sensorNum == 4) {
      echoCM = sonar4.ping_cm();
    } else if (sensorNum == 5) {
      echoCM = sonar5.ping_cm();
    } else if (sensorNum == 6) {
      echoCM = sonar6.ping_cm();
    }
    tempVal+= echoCM/2.54;    // convert to inches
  }
  return tempVal / ((float)numAvg);
}

int MoveForward(float movInches) {
  // ** need to calculate how much wheel rotation is needed per inch (need encoders working)
  float currentMillis = millis();

  // Calculate the distance in terms of encoder values, and once encoder value is reached must stop
  float encoderChange;
  
  if (movInches > 0.0) {
    // Move forward
    LeftMotorForward();
    RightMotorForward();
    Serial.print("moving forward: ");
    Serial.println(movInches);
    //DisableMotors();
    
  } else {
    // Move backward
    LeftMotorBackward();
    RightMotorBackward();
    Serial.print("moving backward: ");
    Serial.println(movInches);
    //DisableMotors();
  }
  
  delay(1000);
  TimeElapsed(currentMillis);
  DisableMotors();
  
}

int Rotate(float rotDegrees) {
  float currentMillis = millis();
  float encoderChange;
  
  if (rotDegrees > 0.0) {
    // Turn right
    LeftMotorForward();
    RightMotorBackward();
    Serial.print("turning right: ");
    Serial.println(rotDegrees);
    //DisableMotors();
    
  } else {
    // Turn left
    LeftMotorBackward();
    RightMotorForward();
    Serial.print("turning left: ");
    Serial.println(rotDegrees);
    //DisableMotors();
  }

  delay(600);
  TimeElapsed(currentMillis);
  DisableMotors();
}

void LeftMotorForward(void) {
  // testing code
  digitalWrite(leftMotorIn3, HIGH);
  digitalWrite(leftMotorIn4, LOW);
  analogWrite(leftMotorPin, LmotorSpeed);
  //delay(1000);
}

void LeftMotorBackward(void) {
  // testing code
  digitalWrite(leftMotorIn3, LOW);
  digitalWrite(leftMotorIn4, HIGH);
  analogWrite(leftMotorPin, LmotorSpeed);
  //delay(1000);
}

void RightMotorForward(void) {
  // testing code
  digitalWrite(rightMotorIn1, HIGH);
  digitalWrite(rightMotorIn2, LOW);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

void RightMotorBackward(void) {
  // testing code
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, HIGH);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

int TimeElapsed(float currentMillis) {
  float previousMillis = currentMillis;
  currentMillis = millis();
  float timeElapsed = (currentMillis - previousMillis);
  Serial.print("Time Elapsed: ");
  Serial.println(timeElapsed);      // prints the time elapsed
}
=======
#include <Wire.h>
#include <SoftwareSerial.h>
#include <NewPing.h>

//Define pins
SoftwareSerial BTSerial(0, 1); // RX | TX -->  0=blue, 1=brown

//Define ultrasonic sensors
#define TRIGGER_PIN1          48    // Front
#define TRIGGER_PIN2          49    // Left front
#define TRIGGER_PIN3          50    // Left back
#define TRIGGER_PIN4          51    // Right front
#define TRIGGER_PIN5          52    // Right back
#define TRIGGER_PIN6          53    // Front bottom (block detection)
#define ECHO_PIN1             8     // Front sensor
#define ECHO_PIN2             9     // Left front
#define ECHO_PIN3             10    // Left back
#define ECHO_PIN4             11    // Right front
#define ECHO_PIN5             12    // Right back
#define ECHO_PIN6             13    // Front bottom (block detection)
#define MaxDistance           200

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MaxDistance);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MaxDistance);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MaxDistance);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MaxDistance);
NewPing sonar5(TRIGGER_PIN5, ECHO_PIN5, MaxDistance);
NewPing sonar6(TRIGGER_PIN6, ECHO_PIN6, MaxDistance);

//Define functions
void InitMotors(void);
void InitInterrupts(void);
void DisableMotors(void);

float ReadUltrasonicSensor(int sensorNum, int numAvg);
int Rotate(float rotDegrees);
int MoveForward(float movInches);

void LeftMotorForward(void);
void LeftMotorBackward(void);
void RightMotorForward(void);
void RightMotorBackward(void);

//Define Variables
int LmotorSpeed = 100;
int RmotorSpeed = 100;
String cmdStr;
volatile long leftMotorCount = 0;
volatile long rightMotorCount = 0;

// Define motor connections
int leftMotorPin = 5;       // L DC Motor - interrupt pin (might not need it tho idk)
int rightMotorPin = 4;      // R DC Motor - interrupt pin

int rightMotorIn1 = 30;       // R DC Motor - IN1 on Motor Driver
int rightMotorIn2 = 31;       // R DC Motor - IN2 on Motor Driver
int leftMotorIn3 = 32;      // L DC Motor - IN3 on Motor Driver
int leftMotorIn4 = 33;      // L DC Motor - IN4 on Motor Driver

int rightEncA = 2;           // R DC Motor - encoder A signal (needs interrupt)
int rightEncB = 22;          // R DC Motor - encoder B signal
int leftEncA = 3;          // L DC Motor - encoder A signal (needs interrupt)
int leftEncB = 23;         // L DC Motor - encoder B signal


void setup() {
  // Initialize motors
  InitMotors();

  // attach interrupts
  InitInterrupts();
  
  // Initialize Serial communication
  Serial.begin(9600);
  //Serial.println("Enter AT commands:");
  // HC-05 default speed in AT command mode 
  BTSerial.begin(38400); 
  delay(1000);
}

void loop() {


  //get all sensor readings at once (with command ua)
  int numAvg = 2;       // total avg time
  float distanceBuffer[6];
  String strBuffer;
  for (int i=0; i<6; i++) {
    distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
    strBuffer += (String)i;
    strBuffer += "=";
    strBuffer += distanceBuffer[i];
    strBuffer += " | ";
  }
  Serial.println(strBuffer);
        
  // Read what is entered into serial monitor or bluetooth
  if (Serial.available()) {
    BTSerial.write(Serial.read());

    cmdStr = Serial.readString();
    Serial.println(cmdStr);
    
    if (cmdStr.charAt(0) == 'w') {
      // Remove first 3 characters to only get inch distance value (ex. w0-20 to 20, w0--10 to -10)
      cmdStr.remove(0,3);
      MoveForward(cmdStr.toFloat());
      
    } else if (cmdStr.charAt(0) == 'r') {
      cmdStr.remove(0,3);
      Rotate(cmdStr.toFloat());
      
    } else if (cmdStr.charAt(0) == 'u') {
      // Check which ultrasonic sensor we want to read from
      int numAvg = 2;       // total avg time
      cmdStr.remove(0,1);   // remove first char u to determine which sensor to read from

      if (cmdStr.charAt(0) == 'a') {
        //get all sensor readings at once (with command ua)
        float distanceBuffer[6];
        String strBuffer;
        for (int i=0; i<6; i++) {
          distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
          strBuffer += (String)i;
          strBuffer += "=";
          strBuffer += distanceBuffer[i];
          strBuffer += " | ";
        }
        Serial.println(strBuffer);
      } else {
        //get individual sensor readings
        float distance = ReadUltrasonicSensor(cmdStr.toInt(), numAvg);
        Serial.println(distance);
      }
      
    } else {
      Serial.println("Invalid command, please try again.");
    }
  }

  // display encoder values
  delay(100);
  Serial.print("Left | Right - motor count: ");
  Serial.print(leftMotorCount);
  Serial.print(" | ");
  Serial.println(rightMotorCount);
}

void InitMotors(void) {
  // Setting up motor pins
  pinMode(leftMotorPin, OUTPUT);        // pin 
  pinMode(rightMotorPin, OUTPUT);       // pin 
  
  pinMode(rightMotorIn1, OUTPUT);        // pin
  pinMode(rightMotorIn2, OUTPUT);        // pin 
  pinMode(leftMotorIn3, OUTPUT);        // pin 
  pinMode(leftMotorIn4, OUTPUT);        // 
}

void InitInterrupts(void) {
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncA), EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncB), EncoderEvent, CHANGE);
}

void DisableMotors(void) {
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, LOW);
  digitalWrite(leftMotorIn3, LOW);
  digitalWrite(leftMotorIn4, LOW);
}

void EncoderEvent() {
  
  // Left Motor
  if (digitalRead(leftEncA) == HIGH) {      // A high
    if (digitalRead(leftEncB) == LOW) {
      leftMotorCount++;
    } else {
      leftMotorCount--;
    }
  } else {                                  // A low
    if (digitalRead(leftEncB) == LOW) {
      leftMotorCount--;
    } else {
      leftMotorCount++;
    }
  }

  // Right Motor
  if (digitalRead(rightEncA) == HIGH) {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount--;
    } else {
      rightMotorCount++;
    }
  } else {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount++;
    } else {
      rightMotorCount--;
    }
  }
  
}

float ReadUltrasonicSensor(int sensorNum, int numAvg) {

  float tempVal = 0.0;

  for (int i=0; i< numAvg; i++) {
    delay(50);
    float echoCM = 0;
    if (sensorNum == 1) {
      echoCM = sonar1.ping_cm();
    } else if (sensorNum == 2) {
      echoCM = sonar2.ping_cm();
    } else if (sensorNum == 3) {
      echoCM = sonar3.ping_cm();
    } else if (sensorNum == 4) {
      echoCM = sonar4.ping_cm();
    } else if (sensorNum == 5) {
      echoCM = sonar5.ping_cm();
    } else if (sensorNum == 6) {
      echoCM = sonar6.ping_cm();
    }
    tempVal+= echoCM/2.54;    // convert to inches
  }
  return tempVal / ((float)numAvg);
}

int MoveForward(float movInches) {
  // ** need to calculate how much wheel rotation is needed per inch (need encoders working)
  float currentMillis = millis();

  // Calculate the distance in terms of encoder values, and once encoder value is reached must stop
  float encoderChange;
  
  if (movInches > 0.0) {
    // Move forward
    LeftMotorForward();
    RightMotorForward();
    Serial.print("moving forward: ");
    Serial.println(movInches);
    //DisableMotors();
    
  } else {
    // Move backward
    LeftMotorBackward();
    RightMotorBackward();
    Serial.print("moving backward: ");
    Serial.println(movInches);
    //DisableMotors();
  }
  
  delay(1000);
  TimeElapsed(currentMillis);
  DisableMotors();
  
}

int Rotate(float rotDegrees) {
  float currentMillis = millis();
  float encoderChange;
  
  if (rotDegrees > 0.0) {
    // Turn right
    LeftMotorForward();
    RightMotorBackward();
    Serial.print("turning right: ");
    Serial.println(rotDegrees);
    //DisableMotors();
    
  } else {
    // Turn left
    LeftMotorBackward();
    RightMotorForward();
    Serial.print("turning left: ");
    Serial.println(rotDegrees);
    //DisableMotors();
  }

  delay(600);
  TimeElapsed(currentMillis);
  DisableMotors();
}

void LeftMotorForward(void) {
  // testing code
  digitalWrite(leftMotorIn3, HIGH);
  digitalWrite(leftMotorIn4, LOW);
  analogWrite(leftMotorPin, LmotorSpeed);
  //delay(1000);
}

void LeftMotorBackward(void) {
  // testing code
  digitalWrite(leftMotorIn3, LOW);
  digitalWrite(leftMotorIn4, HIGH);
  analogWrite(leftMotorPin, LmotorSpeed);
  //delay(1000);
}

void RightMotorForward(void) {
  // testing code
  digitalWrite(rightMotorIn1, HIGH);
  digitalWrite(rightMotorIn2, LOW);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

void RightMotorBackward(void) {
  // testing code
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, HIGH);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

int TimeElapsed(float currentMillis) {
  float previousMillis = currentMillis;
  currentMillis = millis();
  float timeElapsed = (currentMillis - previousMillis);
  Serial.print("Time Elapsed: ");
  Serial.println(timeElapsed);      // prints the time elapsed
}
>>>>>>> ca281d60e67c6f5b9a34dc5edce8f55f9d5b7bfc
