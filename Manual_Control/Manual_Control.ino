#include <Wire.h>
#include <NewPing.h>

//Define pins
//SoftwareSerial BTSerial(0, 1); // RX | TX -->  0=blue, 1=brown

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

// LED pins
int R_LED = 40;
int B_LED = 41;
int G_LED = 42;

//Define functions
void InitLEDs(void);
int UpdateStage_LED(float stageNum);
void InitMotors(void);
void InitInterrupts(void);
void DisableMotors(void);

float ReadUltrasonicSensor(int sensorNum, int numAvg);
int Rotate(float rotDegrees);
int Rotate2(float rotDegrees);          // bigger rotate
int MoveForward(float movInches);
int MoveForward2(float movInches);      // bigger movement
float L_encValPerInch = 97.305;         // henry's experiment calculated this constant
float R_encValPerInch = 99;         // henry's trying something lol
int GetAllSensorReadings(float numAvg);
int numAvg = 2;       // total avg time

void LeftMotorForward(void);
void LeftMotorBackward(void);
void RightMotorForward(void);
void RightMotorBackward(void);

//Define Variables
int LmotorSpeed = 95;
int RmotorSpeed = 110;
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
//  BTSerial.begin(9600); 
  Serial.setTimeout(50);
//  BTSerial.setTimeout(50);
  delay(100);
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
//    BTSerial.write(Serial.read());

    cmdStr = Serial.readString();
    //Serial.println(cmdStr);
    
    if (cmdStr.charAt(0) == 'w') {
      // Remove first 3 characters to only get inch distance value (ex. w0-20 to 20, w0--10 to -10)
      cmdStr.remove(0,3);
      MoveForward(cmdStr.toFloat());
      GetAllSensorReadings(numAvg);
      
//    } else if (cmdStr.charAt(0) == 'q') {
//      cmdStr.remove(0,3);
//      MoveForward2(cmdStr.toFloat());
//      GetAllSensorReadings(numAvg);
//      
//    } else if (cmdStr.charAt(0) == 'e') {
//      cmdStr.remove(0,3);
//      Rotate2(cmdStr.toFloat());
//      GetAllSensorReadings(numAvg);
      
    } else if (cmdStr.charAt(0) == 'r') {
      cmdStr.remove(0,3);
      Rotate(cmdStr.toFloat());
      GetAllSensorReadings(numAvg);
      
    } else if (cmdStr.charAt(0) == 'u') {
      // Check which ultrasonic sensor we want to read from
      int numAvg = 1;       // # of times it reads the sensors
      cmdStr.remove(0,1);   // remove first char u to determine which sensor to read from
      leftMotorCount = 0;
      rightMotorCount = 0;
      
      if (cmdStr.charAt(0) == 'a') {
        GetAllSensorReadings(numAvg);
      } else {
        //get individual sensor readings
        float distance = ReadUltrasonicSensor(cmdStr.toInt(), numAvg);
        Serial.println(distance);
      }
      
    } else if (cmdStr.charAt(0) == 'x') {
      DisableMotors();
      
    } else if (cmdStr.charAt(0) == 's') {
      cmdStr.remove(0,1);
      UpdateStage_LED(cmdStr.toInt());      // takes commands of s1 (stage 1), s2, and s3
      
    } else {
      Serial.println("Invalid command, please try again.");
    }
  }

  // display encoder values (for manual control only)
//  delay(300);
//  Serial.print("Left | Right - motor count: ");
//  Serial.print(leftMotorCount);
//  Serial.print(" | ");
//  Serial.println(rightMotorCount);
  
}

void InitLEDs(void) {
  pinMode(R_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
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
  attachInterrupt(digitalPinToInterrupt(rightEncA), EncoderEvent, CHANGE);
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
      leftMotorCount--;
    } else {
      leftMotorCount++;
    }
  } else {                                  // A low
    if (digitalRead(leftEncB) == LOW) {
      leftMotorCount++;
    } else {
      leftMotorCount--;
    }
  }
  // Right Motor
  if (digitalRead(rightEncA) == HIGH) {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount++;
    } else {
      rightMotorCount--;
    }
  } else {
    if (digitalRead(rightEncB) == LOW) {
      rightMotorCount--;
    } else {
      rightMotorCount++;
    }
  }  
}

float ReadUltrasonicSensor(int sensorNum, int numAvg) {
  float tempVal = 0.0;

  for (int i=0; i< numAvg; i++) {
    delay(80);
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
    tempVal+= echoCM/2.54;    // sensor values -> convert to inches
  }
  return tempVal / ((float)numAvg);
}

int GetAllSensorReadings(float numAvg) {
  //get all sensor readings at once (with command ua)
  float distanceBuffer[6];
  String strBuffer;
  for (int i=0; i<6; i++) {
    distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
    strBuffer += " | ";
    strBuffer += (String)i;
    strBuffer += "=";
    strBuffer += distanceBuffer[i];
    
    // Send sensor values to OA code -> in order 1.front, 2.left-front, 3.left-back, 4.right-front, 5.right-back, 6. front-bottom
    // byte * b = (byte *) &distanceBuffer[i];
    // Serial.write(b, 8);
  }
  // Serial.print("String buffer: ");      // for debug
  Serial.println(strBuffer);              // sends this back to python 
}

int MoveForward(float movInches) {
  float currentMillis = millis();

  // Calculate the distance in terms of encoder values, and once encoder value is reached must stop
  float L_encoderChange = int(movInches*L_encValPerInch);
  float R_encoderChange = int(movInches*R_encValPerInch);
  int prev_leftMotorCount = leftMotorCount;
  int prev_rightMotorCount = rightMotorCount;
  
  if (movInches > 0.0) {
    // Move forward
//    Serial.print("moving forward: ");
//    Serial.println(movInches);
    while (leftMotorCount < prev_leftMotorCount+L_encoderChange && rightMotorCount < prev_rightMotorCount+R_encoderChange) {
      LeftMotorForward();
      RightMotorForward();
    }
    delay(10);
    DisableMotors();
    
  } else {
    // Move backward
//    Serial.print("moving backward: ");
//    Serial.println(movInches);
    while (leftMotorCount > prev_leftMotorCount+L_encoderChange && rightMotorCount > prev_rightMotorCount+R_encoderChange) {
      LeftMotorBackward();
      RightMotorBackward();
    }
    delay(10);
    DisableMotors();
  }
  
  delay(10);
  //TimeElapsed(currentMillis);
  //DisableMotors();
}

//int MoveForward2(float movInches) {
//  // ***only used for manual control, not autonomy (delays for 1 sec)***
//  float currentMillis = millis();
//  if (movInches > 0.0) {
//    // Move forward
//    LeftMotorForward();
//    RightMotorForward();
//    Serial.print("moving forward: ");
//    Serial.println(movInches);
//    //DisableMotors();
//  } else {
//    // Move backward
//    LeftMotorBackward();
//    RightMotorBackward();
//    Serial.print("moving backward: ");
//    Serial.println(movInches);
//    //DisableMotors();
//  }
//  delay(1000);
//  //TimeElapsed(currentMillis);
//  DisableMotors();
//}

int Rotate(float rotDegrees) {
  float currentMillis = millis();

  // Calculate the distance in terms of encoder values, and once encoder valueis reached must stop
  float L_encoderChange = int((rotDegrees*L_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
  float R_encoderChange = int((rotDegrees*R_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
  int prev_leftMotorCount = leftMotorCount;
  int prev_rightMotorCount = rightMotorCount;
  
  if (rotDegrees > 0.0) {
    // Turn right
//    Serial.print("turning right: ");
//    Serial.println(rotDegrees);
    while (leftMotorCount < prev_leftMotorCount+L_encoderChange && rightMotorCount > prev_rightMotorCount-R_encoderChange) {
      LeftMotorForward();
      RightMotorBackward();                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    }
    delay(10);
    DisableMotors();
    
  } else {
    // Turn left
//    Serial.print("turning left: ");
//    Serial.println(rotDegrees);
    while (leftMotorCount > prev_leftMotorCount+L_encoderChange && rightMotorCount < prev_rightMotorCount-R_encoderChange) {
      LeftMotorBackward();
      RightMotorForward();
    }
    delay(10);
    DisableMotors();
  }   

  delay(10);
  //TimeElapsed(currentMillis);
  //DisableMotors();
}

//int Rotate2(float rotDegrees) {
//  // ***only used for manual control, not autonomy (turns for 0.9 sec)***
//  float currentMillis = millis();
//  if (rotDegrees > 0.0) {
//    // Turn right
//    LeftMotorForward();
//    RightMotorBackward();
//    Serial.print("turning right: ");
//    Serial.println(rotDegrees);
//    //DisableMotors();
//  } else {
//    // Turn left
//    LeftMotorBackward();
//    RightMotorForward();
//    Serial.print("turning left: ");
//    Serial.println(rotDegrees);
//    //DisableMotors();
//  }
//  delay(900);
//  //TimeElapsed(currentMillis);
//  DisableMotors();
//}

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
  analogWrite(leftMotorPin, LmotorSpeed-8);
  //delay(1000);
}

void RightMotorForward(void) {
  // testing code
  digitalWrite(rightMotorIn1, HIGH);
  digitalWrite(rightMotorIn2, LOW);
  analogWrite(rightMotorPin, RmotorSpeed-6);
  //delay(1000);
}

void RightMotorBackward(void) {
  // testing code
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, HIGH);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

int UpdateStage_LED(float stageNum) {
  if (stageNum == 1) {
    digitalWrite(R_LED, HIGH);
  }
  if (stageNum == 2) {
    digitalWrite(B_LED, HIGH);
  }
  if (stageNum == 3) {
    digitalWrite(G_LED, HIGH);
  }
}

//int TimeElapsed(float currentMillis) {
//  float previousMillis = currentMillis;
//  currentMillis = millis();
//  float timeElapsed = (currentMillis - previousMillis);
//  Serial.print("Time Elapsed: ");
//  Serial.println(timeElapsed);      // prints the time elapsed
//}
