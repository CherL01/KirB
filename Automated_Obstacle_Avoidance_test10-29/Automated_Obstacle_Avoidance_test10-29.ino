void setup(){
// Start the Serial connection
 Serial.begin(9600);
}
void loop(){
 
 if (Serial.available()>0){
  String cmd_python;
  cmd_python = Serial.readString();
//  cmd_python.remove(0,1);
  if (cmd_python.indexOf('u') != -1){
      Serial.print("SENSOR: ");
      Serial.println(cmd_python);
      }
  if (cmd_python.indexOf('w') != -1){
      Serial.print("DRIVE: ");
      Serial.println(cmd_python);
      }
  if (cmd_python.indexOf('r') != -1){
      Serial.print("ROTATE: ");
      Serial.println(cmd_python);
      }
 }
}


//#include <Wire.h>
//#include <SoftwareSerial.h>
//
////Define pins
//SoftwareSerial BTSerial(0, 1); // RX | TX -->  0=blue, 1=brown
//
////Define functions
//void InitMotors(void);
//void InitInterrupts(void);
//void DisableMotors(void);
//int Rotate(float rotDegrees);
//int MoveForward(float movInches);
//
//void LeftMotorForward(void);
//void LeftMotorBackward(void);
//void RightMotorForward(void);
//void RightMotorBackward(void);
//
////Define Variables
//int motorSpeed = 50;
//String cmdStr;
//volatile long leftMotorCount = 0;
//volatile long rightMotorCount = 0;
//
//// Define motor connections
//int leftMotorPin = 4;       // L DC Motor - interrupt pin (might not need it tho idk)
//int rightMotorPin = 5;      // R DC Motor - interrupt pin
//
//int leftMotorIn1 = 6;       // L DC Motor - IN1 on Motor Driver
//int leftMotorIn2 = 7;       // L DC Motor - IN2 on Motor Driver
//int rightMotorIn3 = 8;      // R DC Motor - IN3 on Motor Driver
//int rightMotorIn4 = 9;      // R DC Motor - IN4 on Motor Driver
//
//int leftEncA = 2;           // L DC Motor - encoder A signal (needs interrupt)
//int leftEncB = 22;          // L DC Motor - encoder B signal
//int rightEncA = 3;          // R DC Motor - encoder A signal (needs interrupt)
//int rightEncB = 23;         // R DC Motor - encoder B signal
//
//
//void setup() {
//  // Initialize motors
//  InitMotors();
//
//  // attach interrupts
//  InitInterrupts();
//  
//  // Initialize Serial communication
//  Serial.begin(9600);
//  //Serial.println("Enter AT commands:");
//  // HC-05 default speed in AT command mode 
//  BTSerial.begin(38400); 
//  delay(1000);
//}
//
//void loop() {
//    
//  // Read what is entered into serial monitor or bluetooth
//  if (Serial.available()) {
////    BTSerial.write(Serial.read());

//    cmdStr = Serial.readString();
//    Serial.println(cmdStr);
//    
//    if (cmdStr.charAt(0) == 'w') {
//      // Remove first 3 characters to only get inch distance value (ex. w0-20 to 20, w0--10 to -10)
//      cmdStr.remove(0,3);
//      MoveForward(cmdStr.toFloat());
//      
//    } else if (cmdStr.charAt(0) == 'r') {
//      cmdStr.remove(0,3);
//      Rotate(cmdStr.toFloat());
//      
//    } else {
//      Serial.println("Invalid command, please try again.");
//    }
//  }
//
//  delay(50);
//  Serial.print("Left | Right - motor count: ");
//  Serial.print(leftMotorCount);
//  Serial.print(" | ");
//  Serial.println(rightMotorCount);
//}
//
//void InitMotors(void) {
//  // Setting up motor pins
//  pinMode(leftMotorPin, OUTPUT);        // pin 2 & ENA
//  pinMode(rightMotorPin, OUTPUT);       // pin 3 & ENB
//  
//  pinMode(leftMotorIn1, OUTPUT);        // pin 4 & IN1
//  pinMode(leftMotorIn2, OUTPUT);        // pin 5 & IN2
//  pinMode(rightMotorIn3, OUTPUT);        // pin 6 & IN1
//  pinMode(rightMotorIn4, OUTPUT);        // pin 7 & IN2
//}
//
//void InitInterrupts(void) {
//  pinMode(leftEncA, INPUT);
//  pinMode(leftEncB, INPUT);
//  pinMode(rightEncA, INPUT);
//  pinMode(rightEncB, INPUT);
//  attachInterrupt(digitalPinToInterrupt(leftEncA), EncoderEvent, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(rightEncB), EncoderEvent, CHANGE);
//}
//
//void DisableMotors(void) {
//  digitalWrite(leftMotorIn1, LOW);
//  digitalWrite(leftMotorIn2, LOW);
//  digitalWrite(rightMotorIn3, LOW);
//  digitalWrite(rightMotorIn4, LOW);
//}
//
//void EncoderEvent() {
//  // Need to figure out why the right motor not counting properly
//  
//  // Left Motor
//  if (digitalRead(leftEncA) == HIGH) {
//
//    if (digitalRead(leftEncB) == LOW) {
//      leftMotorCount++;
//    } else {
//      leftMotorCount--;
//    }
//    
//  } else {
//    if (digitalRead(leftEncB) == LOW) {
//      leftMotorCount--;
//    } else {
//      leftMotorCount++;
//    }
//  }
//
//  // Right Motor
//  if (digitalRead(rightEncA) == HIGH) {
//
//    if (digitalRead(rightEncB) == LOW) {
//      rightMotorCount++;
//    } else {
//      rightMotorCount--;
//    }
//    
//  } else {
//    if (digitalRead(rightEncB) == LOW) {
//      rightMotorCount--;
//    } else {
//      rightMotorCount++;
//    }
//  }
//  
//}
//
//int MoveForward(float movInches) {
//  // ** need to calculate how much wheel rotation is needed per inch (need encoders working)
//  float currentMillis = millis();
//  if (movInches > 0.0) {
//    // Move forward
//    
//    LeftMotorForward();
//    RightMotorForward();
//    Serial.print("moving forward: ");
//    Serial.println(movInches);
//    
//  } else {
//    // Move backward
//    
//    LeftMotorBackward();
//    RightMotorBackward();
//    Serial.print("moving backward: ");
//    Serial.println(movInches);
//  }
//
//  delay(1000);
//  TimeElapsed(currentMillis);
//  DisableMotors();
//  
//}
//
//int Rotate(float rotDegrees) {
//  float currentMillis = millis();
//  
//  if (rotDegrees > 0.0) {
//    // Turn right
//    
//    LeftMotorForward();
//    RightMotorBackward();
//    Serial.print("turning right: ");
//    Serial.println(rotDegrees);
//  } else {
//    // Turn left
//    
//    LeftMotorBackward();
//    RightMotorForward();
//    Serial.print("turning left: ");
//    Serial.println(rotDegrees);
//  }
//
//  delay(250);
//  TimeElapsed(currentMillis);
//  DisableMotors();
//}
//
//void LeftMotorForward(void) {
//  // testing code
//  digitalWrite(leftMotorIn1, LOW);
//  digitalWrite(leftMotorIn2, HIGH);
//  analogWrite(leftMotorPin, motorSpeed);
//  //delay(1000);
//}
//
//void LeftMotorBackward(void) {
//  // testing code
//  digitalWrite(leftMotorIn1, HIGH);
//  digitalWrite(leftMotorIn2, LOW);
//  analogWrite(leftMotorPin, motorSpeed);
//  //delay(1000);
//}
//
//void RightMotorForward(void) {
//  // testing code
//  digitalWrite(rightMotorIn3, LOW);
//  digitalWrite(rightMotorIn4, HIGH);
//  analogWrite(rightMotorPin, motorSpeed);
//  //delay(1000);
//}
//
//void RightMotorBackward(void) {
//  // testing code
//  digitalWrite(rightMotorIn3, HIGH);
//  digitalWrite(rightMotorIn4, LOW);
//  analogWrite(rightMotorPin, motorSpeed);
//  //delay(1000);
//}
//
//int TimeElapsed(float currentMillis) {
//  float previousMillis = currentMillis;
//  currentMillis = millis();
//  float timeElapsed = (currentMillis - previousMillis);
//  Serial.print("Time Elapsed: ");
//  Serial.println(timeElapsed);
//}
