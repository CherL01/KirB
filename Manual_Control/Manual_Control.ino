#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>

//Define pins
//SoftwareSerial BTSerial(0, 1); // RX | TX -->  0=blue, 1=brown

//Define ultrasonic sensors
#define TRIGGER_PIN1          48    // Front
#define TRIGGER_PIN2          49    // Left front
#define TRIGGER_PIN3          50    // Left back
#define TRIGGER_PIN4          51    // Right front
#define TRIGGER_PIN5          52    // Right back
#define TRIGGER_PIN6          53    // Back
#define TRIGGER_PIN7          43    // Front bottom  // Doublecheck
#define ECHO_PIN1             8     // Front sensor
#define ECHO_PIN2             9     // Left front
#define ECHO_PIN3             10    // Left back
#define ECHO_PIN4             11    // Right front
#define ECHO_PIN5             12    // Right back
#define ECHO_PIN6             13    // Back
#define ECHO_PIN7             6     // Front bottom  // Doublecheck
#define MaxDistance           200

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MaxDistance);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MaxDistance);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MaxDistance);
NewPing sonar4(TRIGGER_PIN4, ECHO_PIN4, MaxDistance);
NewPing sonar5(TRIGGER_PIN5, ECHO_PIN5, MaxDistance);
NewPing sonar6(TRIGGER_PIN6, ECHO_PIN6, MaxDistance);
NewPing sonar7(TRIGGER_PIN7, ECHO_PIN7, MaxDistance);

// LED pins
int R_LED = 40;
int B_LED = 41;
int G_LED = 42;

// Servo motors
Servo ArmServo;
Servo GripperServo;

//Define functions
void InitLEDs(void);
int UpdateStage_LED(float stageNum);
void InitMotors(void);
void InitInterrupts(void);
void DisableMotors(void);
void Parallel(void);
double sensorDifferenceLimit = 0.15;
int MoveArm(float rotDegrees);
void OpenGripper(void);
void CloseGripper(void);
bool Start = false;

int Rotate(float rotDegrees);
int MoveForward(float movInches);
float L_encValPerInch = 97.305;         // henry's experiment calculated this constant
float R_encValPerInch = 99;             // henry's experiment calculated this constant

float ReadUltrasonicSensor(int sensorNum, int numAvg);
int GetAllSensorReadings(float numAvg);
float distanceBuffer[7];
String strBuffer;
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
  delay(100);
}

void loop() {
  delay(100);
  // Always run parallel
  if (Start==true) {
    Parallel();
  } else if (Start==false) {
    DisableMotors();
  }
  
  // Read what is transmitted onto arduino from python code
  if (Serial.available()) {
//    BTSerial.write(Serial.read());

    cmdStr = Serial.readString();
    
    if (cmdStr.charAt(0) == 'w') {
      // Moves forward/backward depending on number of inches sent
      // Remove first 3 characters to only get inch distance value (ex. w0-20 to 20, w0--10 to -10)
      cmdStr.remove(0,3);
      MoveForward(cmdStr.toFloat());
      GetAllSensorReadings(numAvg);
      
    } else if (cmdStr.charAt(0) == 'r') {
      // Rotates depending on number of degrees sent
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

    } else if (cmdStr.charAt(0) == 'p') {
      // Runs parallel. This is also in the loop.
      Parallel();

    } else if (cmdStr.charAt(0) == 'a') {
      // Moves arm up and down depending on what degrees is sent
      cmdStr.remove(0,1);
      MoveArm(cmdStr.toInt());
      
    } else if (cmdStr.charAt(0) == 'g') {
      // Moves the gripper - 'go' for gripper open and 'gc' for gripper close
      cmdStr.remove(0,1);
      if (cmdStr.charAt(0) == 'o') {
        OpenGripper();
      } else if (cmdStr.charAt(0) == 'c') {
        CloseGripper();
      }
      
    } else if (cmdStr.charAt(0) == 's') {
      // Enables the parallel function in the loop
      Start = true;

    } else if (cmdStr.charAt(0) == 'x') {
      // Emergency stop, also disables the parallel function in the loop
      DisableMotors();
      Start = false;
      
    } else if (cmdStr.charAt(0) == 'l') {
      // Lights up the LED depending on which stage 
      // 1. localized in Loading Zone (RED), 2. Picked up Block (BLUE), 3. Dropped Block in Drop-off Zone (GREEN)
      cmdStr.remove(0,3);
      UpdateStage_LED(cmdStr.toInt());      // turns on leds
      
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
  // Setting up DC motor pins
  pinMode(leftMotorPin, OUTPUT);         
  pinMode(rightMotorPin, OUTPUT);        
  
  pinMode(rightMotorIn1, OUTPUT);       
  pinMode(rightMotorIn2, OUTPUT);        
  pinMode(leftMotorIn3, OUTPUT);         
  pinMode(leftMotorIn4, OUTPUT);        

//  //Servo motors pins
//  ArmServo.attach(6);
//  GripperServo.attach(7);
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
    delay(10);
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
    } else if (sensorNum == 7) {
      echoCM = sonar7.ping_cm();
    }
    tempVal+= echoCM/2.54;    // sensor values -> convert to inches
  }
  return tempVal / ((float)numAvg);
}

int GetAllSensorReadings(float numAvg) {
  //get all sensor readings at once (with command ua)
  strBuffer = "";
  for (int i=0; i<7; i++) {
    distanceBuffer[i] = ReadUltrasonicSensor(i+1, numAvg);
    strBuffer += " | ";
    strBuffer += (String)i;
    strBuffer += "=";
    strBuffer += distanceBuffer[i];
  }
  // Send sensor values to OA code -> in order 0.front, 1.left-front, 2.left-back, 3.right-front, 4.right-back, 5. back, 6.front-bottom
  Serial.println(strBuffer);              // sends this back to python 
}

int MoveForward(float movInches) {
  // Calculate the distance in terms of encoder values, and once encoder value is reached must stop
  float L_encoderChange = int(movInches*L_encValPerInch);
  float R_encoderChange = int(movInches*R_encValPerInch);
  int prev_leftMotorCount = leftMotorCount;
  int prev_rightMotorCount = rightMotorCount;
  
  if (movInches > 0.0) {
    // Move forward
    while (leftMotorCount < prev_leftMotorCount+L_encoderChange && rightMotorCount < prev_rightMotorCount+R_encoderChange) {
      LeftMotorForward();
      RightMotorForward();
    }
    delay(10);
    DisableMotors();
  } else {
    // Move backward
    while (leftMotorCount > prev_leftMotorCount+L_encoderChange && rightMotorCount > prev_rightMotorCount+R_encoderChange) {
      LeftMotorBackward();
      RightMotorBackward();
    }
    delay(10);
    DisableMotors();
  }
  delay(10);
  //DisableMotors();
}

int Rotate(float rotDegrees) {
  // Calculate the distance in terms of encoder values, and once encoder valueis reached must stop
  float L_encoderChange = int((rotDegrees*L_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
  float R_encoderChange = int((rotDegrees*R_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
  int prev_leftMotorCount = leftMotorCount;
  int prev_rightMotorCount = rightMotorCount;
  
  if (rotDegrees > 0.0) {
    // Turn right
    while (leftMotorCount < prev_leftMotorCount+L_encoderChange && rightMotorCount > prev_rightMotorCount-R_encoderChange) {
      LeftMotorForward();
      RightMotorBackward();                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    }
    delay(10);
    DisableMotors();
  } else {
    // Turn left
    while (leftMotorCount > prev_leftMotorCount+L_encoderChange && rightMotorCount < prev_rightMotorCount-R_encoderChange) {
      LeftMotorBackward();
      RightMotorForward();
    }
    delay(10);
    DisableMotors();
  }   
  delay(10);
  //DisableMotors();
}

void LeftMotorForward(void) {
  // left motor rotates forward
  digitalWrite(leftMotorIn3, HIGH);
  digitalWrite(leftMotorIn4, LOW);
  analogWrite(leftMotorPin, LmotorSpeed);
  //delay(1000);
}

void LeftMotorBackward(void) {
  // left motor rotates backward
  digitalWrite(leftMotorIn3, LOW);
  digitalWrite(leftMotorIn4, HIGH);
  analogWrite(leftMotorPin, LmotorSpeed-8);
  //delay(1000);
}

void RightMotorForward(void) {
  // right motor rotates forward
  digitalWrite(rightMotorIn1, HIGH);
  digitalWrite(rightMotorIn2, LOW);
  analogWrite(rightMotorPin, RmotorSpeed-6);
  //delay(1000);
}

void RightMotorBackward(void) {
  // right motor rotates backward
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, HIGH);
  analogWrite(rightMotorPin, RmotorSpeed);
  //delay(1000);
}

int UpdateStage_LED(float stageNum) {
  // LED lights up
  // 1. localized in Loading Zone (RED), 2. Picked up Block (BLUE), 3. Dropped Block in Drop-off Zone (GREEN)
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

void Parallel(void) {
  // get sensor readings
  GetAllSensorReadings(numAvg);
  
  // get difference btw left sensors and difference btw right sensors
  // distanceBuffer[7] --> 0.front, 1.left-front, 2.left-back, 3.right-front, 4.right-back, 5. back, 6.front-bottom
  float L_sensor_diff = abs(distanceBuffer[1]-distanceBuffer[2]);
  float R_sensor_diff = abs(distanceBuffer[3]-distanceBuffer[4]);
  
  // get closest sensor
  float closestSensor = 100;
  int closestSensorIndex = 0;
  for (int i=1; i<=4; i++) {
    if (closestSensor>distanceBuffer[i]) {
       closestSensor = distanceBuffer[i];       // value of closest sensor
       closestSensorIndex = i;                  // index of closest sensor
    }
  }
 
  // calculate avg left sensor and avg right sensor values
  float L_sense = (distanceBuffer[1]+distanceBuffer[2])/2;
  float R_sense = (distanceBuffer[3]+distanceBuffer[4])/2;
  
  // check left and right side difference to center itself. if kirb in a hallway (L+R < 10)
  if (L_sense + R_sense < 10) {
    if (L_sense > R_sense+2) {
      Rotate(-6);         // doublecheck if this works
    } else if (R_sense > L_sense+2) {
      Rotate(6);
    }
  }

  // both sides over the sensor difference limit (not parallel)
  if (L_sensor_diff > sensorDifferenceLimit && R_sensor_diff > sensorDifferenceLimit) {
    if (closestSensorIndex == 1) {
      Rotate(6);
    } else if (closestSensorIndex == 2) {
      Rotate(-6);
    } else if (closestSensorIndex == 3) {
      Rotate(-6);
    } else if (closestSensorIndex == 4) {
      Rotate(6);
    }
  }
  
  // if too close to a wall (less than estop limit, check_turn_clearance()

  // if not initialized yet, need to keep swimming, move forward 4 inches or backward 2 inches
  
}

int MoveArm(float rotDegrees) {
  ArmServo.write(rotDegrees); // sets the servo position according to the scaled value
  delay(2000);                // waits for the servo to get there
  return rotDegrees;
}

void OpenGripper(void) {
  // this only sets the speed. To stop, use GripperServo.write(90)
  //To open
  //GripperServo.write();   // calibrate
  delay(2000);
  GripperServo.write(90);
}

void CloseGripper(void) {
  // this only sets the speed. To stop, use GripperServo.write(90)
  //To close
  //GripperServo.write();     // calibrate
  delay(2000);              // calibrate
  GripperServo.write(90);   // might not need to stop, keep "closing" to hold grip
}
