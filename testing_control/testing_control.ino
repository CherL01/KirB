#include <util/atomic.h> // For the ATOMIC_BLOCK macro

//#define ENCA 2 // YELLOW
//#define ENCB 22 // WHITE
//#define PWM 4
//#define IN2 31
//#define IN1 30

// #define ENCA 3 // YELLOW
// #define ENCB 23 // WHITE
// #define PWM 5
// #define IN2 32
// #define IN1 33

// Define motor connections
int leftMotorPin = 5;       // L DC Motor - interrupt pin (might not need it tho idk)
int rightMotorPin = 4;      // R DC Motor - interrupt pin

int rightMotorIn1 = 30;       // R DC Motor - IN1 on Motor Driver
int rightMotorIn2 = 31;       // R DC Motor - IN2 on Motor Driver
int leftMotorIn2 = 32;      // L DC Motor - IN3 on Motor Driver
int leftMotorIn1 = 33;      // L DC Motor - IN4 on Motor Driver

int rightEncA = 2;           // R DC Motor - encoder A signal (needs interrupt)
int rightEncB = 22;          // R DC Motor - encoder B signal
int leftEncA = 3;          // L DC Motor - encoder A signal (needs interrupt)
int leftEncB = 23;         // L DC Motor - encoder B signal

struct PID_params {
  float kp;
  float kd;
  float ki;
  float eprev; // default 0
  float eintegral; // default 0
  long prevT; // default 0
};

struct motor_params {
  int dir;
  float pwr;
};

volatile int posiR = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int posiL = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  
PID_params right_motor_PID_params = {1, 0.05, 0.05, 0, 0, 0};
PID_params left_motor_PID_params = {1, 0.05, 0.05, 0, 0, 0};

int target = 0;

void setup() {
  Serial.begin(9600);

  pinMode(rightEncA,INPUT);
  pinMode(rightEncB,INPUT);
  attachInterrupt(digitalPinToInterrupt(rightEncA),readEncoderR,RISING);
  
  pinMode(rightMotorPin,OUTPUT);
  pinMode(rightMotorIn1,OUTPUT);
  pinMode(rightMotorIn2,OUTPUT);

  pinMode(leftEncA,INPUT);
  pinMode(leftEncB,INPUT);
  attachInterrupt(digitalPinToInterrupt(leftEncA),readEncoderL,RISING);
  
  pinMode(leftMotorPin,OUTPUT);
  pinMode(leftMotorIn1,OUTPUT);
  pinMode(leftMotorIn2,OUTPUT);
  
  Serial.println("target pos");
}

void loop() {

  // set target position
//  int target = 250*sin(prevT/1e6);
  target += 10;
  // PID constants
  // right wheel
//  float kp = 1;
//  float kd = 0.1;
//  float ki = -0.078;

  // float kp = 1;
  // float kd = 0.1;
  // float ki = -0.045;


  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int posR = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posR = posiR;
  }
  
  int posL = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posL = posiL;
  }

  motor_params right_motor;
  motor_params left_motor;
  right_motor = calc_PID(posR, target*-1, right_motor_PID_params);
  left_motor = calc_PID(posL, target, left_motor_PID_params);

  // signal the motor
  setMotor(right_motor.dir,right_motor.pwr,rightMotorPin,rightMotorIn1,rightMotorIn2);
  setMotor(left_motor.dir,left_motor.pwr,leftMotorPin,leftMotorIn1,leftMotorIn2);

  Serial.print("target:");
  Serial.print(target);
  Serial.print(" ");
  Serial.print("posR:");
  Serial.print(posR*-1);
  Serial.print(" ");
  Serial.print("posL:");
  Serial.print(posL);
  Serial.println();
}

motor_params calc_PID(int pos, int target, PID_params &params){
  motor_params out;

  long currT = micros();
  float deltaT = ((float) (currT - params.prevT))/( 1.0e6 );
  params.prevT = currT;

  // error
  int e = pos - target;

  // derivative
  float dedt = (e-params.eprev)/(deltaT);

  // integral
  params.eintegral = params.eintegral + e*deltaT;

  // control signal
  float u = params.kp*e + params.kd*dedt + params.ki*params.eintegral;

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
  
  out.dir = dir;
  out.pwr = pwr;
  
  return out;
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

void readEncoderR(){
  int b = digitalRead(rightEncB);
  if(b > 0){
    posiR++;
  }
  else{
    posiR--;
  }
}

void readEncoderL(){
  int b = digitalRead(leftEncB);
  if(b > 0){
    posiL++;
  }
  else{
    posiL--;
  }
}
