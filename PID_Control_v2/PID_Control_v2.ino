#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

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
// volatile long posiL = 0;
// volatile long posiR = 0;

// Define motor connections
int leftMotorPin = 5;       // L DC Motor - interrupt pin (might not need it tho idk)
int rightMotorPin = 4;      // R DC Motor - interrupt pin

int rightMotorIn1 = 30;       // R DC Motor - IN1 on Motor Driver - green en1
int rightMotorIn2 = 31;       // R DC Motor - IN2 on Motor Driver - black en2
int leftMotorIn1 = 32;      // L DC Motor - IN3 on Motor Driver - orange en3
int leftMotorIn2 = 33;      // L DC Motor - IN4 on Motor Driver - purple en4

int rightEncA = 2;           // R DC Motor - encoder A signal (needs interrupt)
int rightEncB = 22;          // R DC Motor - encoder B signal
int leftEncA = 3;          // L DC Motor - encoder A signal (needs interrupt)
int leftEncB = 23;         // L DC Motor - encoder B signal

const int encoders[] = {rightEncB, leftEncB};   // initialize pins for templates

// Define PID control parameters, motor parameters, and PID variables
struct PID_params {
  float kp;
  float kd;
  float ki;
  float eprev; // default 0
  float eintegral; // default 0
  long prevT; // default 0
} ;

struct motor_params {
  int dir;
  float pwr;
} ;

volatile long posiR = 0; // volatile position since interrupt
volatile long posiL = 0; 
volatile long posi[] = {posiR, posiL};

PID_params right_motor_PID_params = {1, 0.05, 0.05, 0, 0, 0};
PID_params left_motor_PID_params = {1, 0.05, 0.05, 0, 0, 0};

long targetL = 0; // positive = forward
long targetR = 0; // negative = forward

void RunPID(float L_enc_change, float R_enc_change);
int L_goal_pos = 0;
int R_goal_pos = 0;

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
  targetL = read_posL(); // positive = forward
  targetR = read_posR(); // negative = forward
  delay(100);
}

void loop() {
  delay(300);
  // Always run parallel
  if (Start==true) {
    Parallel();
  } else if (Start==false) {
    DisableMotors();
  }

  GetAllSensorReadings(numAvg);
  
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
      CheckTurnClearance();
      Rotate(cmdStr.toFloat());
      GetAllSensorReadings(numAvg);
      
    } else if (cmdStr.charAt(0) == 'u') {
      // Check which ultrasonic sensor we want to read from
      int numAvg = 1;       // # of times it reads the sensors
      cmdStr.remove(0,1);   // remove first char u to determine which sensor to read from
      
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

//   // display encoder values (for manual control only)
//  delay(300);
//  Serial.print("Left | Right - motor count: ");
//  Serial.print(posiL);
//  Serial.print(" | ");
//  Serial.println(posiR);
  
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
  pinMode(leftMotorIn1, OUTPUT);         
  pinMode(leftMotorIn2, OUTPUT);        
//  //Servo motors pins
  ArmServo.attach(45);        // needs pwm pins
//  GripperServo.attach(46);    // needs pwm pins
}

void InitInterrupts(void) {
  pinMode(rightEncA, INPUT);
  pinMode(rightEncB, INPUT);
  pinMode(leftEncA, INPUT);
  pinMode(leftEncB, INPUT);

  // read encoder 0 = right, 1 = left
  attachInterrupt(digitalPinToInterrupt(rightEncA),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(leftEncA),readEncoder<1>,RISING);
}

template <int j>
void readEncoder(){
  int b = digitalRead(encoders[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}

void DisableMotors(void) {
  digitalWrite(rightMotorIn1, LOW);
  digitalWrite(rightMotorIn2, LOW);
  digitalWrite(leftMotorIn1, LOW);
  digitalWrite(leftMotorIn2, LOW);
}

float ReadUltrasonicSensor(int sensorNum, int numAvg) {
  float tempVal = 0.0;

  for (int i=0; i< numAvg; i++) {
    delay(15);
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
  float L_encoderChange = int(movInches*L_encValPerInch); // positive
  float R_encoderChange = int(movInches*R_encValPerInch); // positive
  
  int prev_posiL = read_posL();
  int prev_posiR = read_posR();
  
  if (movInches > 0.0) {
    // Move forward
    while (read_posL() < prev_posiL+L_encoderChange && read_posR() < prev_posiR+R_encoderChange) {
      RunPID(L_encoderChange, R_encoderChange);
      //LeftMotorForward();
      //RightMotorForward();
    }
    delay(10);
    DisableMotors();
  } else {
    // Move backward
    while (read_posL() > prev_posiL+L_encoderChange && read_posR() > prev_posiR+R_encoderChange) {
      RunPID(L_encoderChange, R_encoderChange);
      //LeftMotorBackward();
      //RightMotorBackward();
    }
    delay(10);
    DisableMotors();
  }
  delay(10);
  //DisableMotors();
}

int Rotate(float rotDegrees) {
  
  if (rotDegrees > 0.0) {
    // Turn right

    // Calculate the distance in terms of encoder values, and once encoder valueis reached must stop
    float L_encoderChange = int((rotDegrees*L_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
    float R_encoderChange = int(-1*(rotDegrees*R_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
    
    
    L_goal_pos = L_encoderChange + read_posL();
    R_goal_pos = R_encoderChange + read_posR();

    while (read_posL() < L_goal_pos && read_posR() > R_goal_pos) {
      RunPID(L_encoderChange, R_encoderChange);
      //LeftMotorForward();
      //RightMotorBackward();                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    }
    delay(10);
    DisableMotors();
  } else {
    // Turn left

    // Calculate the distance in terms of encoder values, and once encoder valueis reached must stop
    float L_encoderChange = int(-1*(rotDegrees*L_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
    float R_encoderChange = int((rotDegrees*R_encValPerInch*3.14159265359)/(20*2.54));     // henry's calculations for turning
    L_goal_pos = L_encoderChange + read_posL();
    R_goal_pos = R_encoderChange + read_posR();

    while (read_posL() > L_goal_pos && read_posR() < R_goal_pos) {
      RunPID(L_encoderChange, R_encoderChange);
      //LeftMotorBackward();
      //RightMotorForward();
    }
    delay(10);
    DisableMotors();
  }   
  delay(10);
  //DisableMotors();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

// void LeftMotorForward(void) {
//   // left motor rotates forward
//   digitalWrite(leftMotorIn1, HIGH);
//   digitalWrite(leftMotorIn2, LOW);
//   analogWrite(leftMotorPin, LmotorSpeed);
//   //delay(1000);
// }

// void LeftMotorBackward(void) {
//   // left motor rotates backward
//   digitalWrite(leftMotorIn1, LOW);
//   digitalWrite(leftMotorIn2, HIGH);
//   analogWrite(leftMotorPin, LmotorSpeed-8);
//   //delay(1000);
// }

// void RightMotorForward(void) {
//   // right motor rotates forward
//   digitalWrite(rightMotorIn1, HIGH);
//   digitalWrite(rightMotorIn2, LOW);
//   analogWrite(rightMotorPin, RmotorSpeed-6);
//   //delay(1000);
// }

// void RightMotorBackward(void) {
//   // right motor rotates backward
//   digitalWrite(rightMotorIn1, LOW);
//   digitalWrite(rightMotorIn2, HIGH);
//   analogWrite(rightMotorPin, RmotorSpeed);
//   //delay(1000);
// }

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

void CheckTurnClearance(void) {

  // boolean cleared
  bool cleared = false;
  // front turn limit, side turn limit, estop limit (aka back limit)
  float frontTurnLimit = 2.0;
  float sidesTurnLimit = 2.76;
  float estopLimit = 1.18;
  float wallLimit = 4.5;
  float sensorTolerance = 2;
  
  // get sensor readings and put into list -> distanceBuffer[7]
  GetAllSensorReadings(numAvg);

  // calculate avg left sensor and avg right sensor values
  float L_sense = (distanceBuffer[1]+distanceBuffer[2])/2;
  float R_sense = (distanceBuffer[3]+distanceBuffer[4])/2;

  // if no wall in front
  // set front turn limit -> 1 tile, 2 tile, 3 tile
  // distanceBuffer[7] --> 0.front, 1.left-front, 2.left-back, 3.right-front, 4.right-back, 5. back, 6.front-bottom
  if (distanceBuffer[0] > wallLimit + sensorTolerance) {
    if (distanceBuffer[0] < 24) {
      frontTurnLimit = 14.5;
    } else if (distanceBuffer[0] < 36) {
      frontTurnLimit = 26.5;
    } else if (distanceBuffer[0] < 48) {
      frontTurnLimit = 38.5;
    }
  }

  // while front < front turn limit or back < estop limit, move forward or backward
      // set front turn limit -> 1 tile, 2 tile, 3 tile
  while (distanceBuffer[0] < frontTurnLimit || distanceBuffer[5] < estopLimit) {
    
    if (distanceBuffer[0] < frontTurnLimit) {
      MoveForward(-1);
    } else if (distanceBuffer[5] < estopLimit) {
      MoveForward(0.75);
    }

    GetAllSensorReadings(numAvg);

    if (distanceBuffer[0] > wallLimit + sensorTolerance) {
        if (distanceBuffer[0] < 24) {
        frontTurnLimit = 14.5;
      } else if (distanceBuffer[0] < 36) {
        frontTurnLimit = 26.5;
      } else if (distanceBuffer[0] < 48) {
        frontTurnLimit = 38.5;
      }
    }
  }
  
  // while left < side turn limit, do left adjustment
  // while right < side turn limit, do right adjustment
      // set front turn limit -> 1 tile, 2 tile, 3 tile
  while (L_sense < sidesTurnLimit || R_sense < sidesTurnLimit) {
    if (L_sense < sidesTurnLimit) {
      // do the turns
      Rotate(-15);
      MoveForward(-1);
      Rotate(12);
      MoveForward(1);
    } else if (R_sense < sidesTurnLimit) {
      // do the turns
      Rotate(15);
      MoveForward(-1);
      Rotate(-12);
      MoveForward(1);
    }

    GetAllSensorReadings(numAvg);

    if (distanceBuffer[0] > wallLimit + sensorTolerance) {
      if (distanceBuffer[0] < 24) {
        frontTurnLimit = 14.5;
      } else if (distanceBuffer[0] < 36) {
        frontTurnLimit = 26.5;
      } else if (distanceBuffer[0] < 48) {
        frontTurnLimit = 38.5;
      }   
    }

  // set true clear once done
  cleared = true;
  
  }
}

int read_posL(void){
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posiL;   // comes from encoder values caluclated in interrupts
  }
  return pos;
}

int read_posR(void){
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posiR;   // comes from encoder values caluclated in interrupts
  }
  return pos;
}
  

void RunPID(float L_enc_change, float R_enc_change) {
  
  targetR = targetR + R_enc_change/50;
  targetL = targetL + L_enc_change/50;

  // enc values to move to
  // Serial.print("Left Target: ");
  // Serial.println(targetL);
  // Serial.print("Right Target: ");
  // Serial.println(targetR);

  // Serial.print("Left pos: ");
  // Serial.println(read_posL());
  // Serial.print("Right pos: ");
  // Serial.println(read_posR());
  
  // set positions steady to use in calcs
  int posR = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posR = posiR;   // comes from encoder values caluclated in interrupts
  }
  
  int posL = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posL = posiL;   // comes from encoder values calcuolated in interrupts
  }

  motor_params right_motor;   
  right_motor = calc_PID(posR, targetR, right_motor_PID_params);
  motor_params left_motor;
  left_motor = calc_PID(posL, targetL*-1, left_motor_PID_params);

  // debug
  // Serial.print("Right motor direction, speed: ");
  // Serial.print(right_motor.dir);
  // Serial.print(", ");
  // Serial.println(right_motor.pwr);
  // Serial.print("Left motor direction, speed: ");
  // Serial.print(left_motor.dir);
  // Serial.print(", ");
  // Serial.println(left_motor.pwr);

  // signal the motor
  setMotor(right_motor.dir,right_motor.pwr,rightMotorPin,rightMotorIn1,rightMotorIn2);
  setMotor(left_motor.dir,left_motor.pwr,leftMotorPin,leftMotorIn1,leftMotorIn2);

}

motor_params calc_PID(int pos, int target, PID_params &params){
  motor_params out;

  long currT = micros();
  float deltaT = ((float) (currT - params.prevT))/( 1.0e6 );
  params.prevT = currT;

  // error
  int e = target - pos;

  // derivative
  float dedt = (e-params.eprev)/(deltaT);

  // integral
  params.eintegral = params.eintegral + e*deltaT;

  // control signal
  float u = params.kp*e + params.kd*dedt + params.ki*params.eintegral;
  // Serial.println(deltaT);
  // Serial.println(pos);
  // Serial.println(target);
  // Serial.println(u);

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  } 

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  
  // store previous error
  params.eprev = e;

  out.dir = dir;
  out.pwr = pwr;
  
  return out;
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
