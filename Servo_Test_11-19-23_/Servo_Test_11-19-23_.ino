#include <Servo.h>

//Servo motors
Servo ArmServo;
Servo GripperServo;
int pos = 0;

int MoveArm(float rotDegrees);
void MoveGripper(void);
void OpenGripper(void);
void CloseGripper(void);

void setup() {
  // put your setup code here, to run once:
  ArmServo.attach(3);
  GripperServo.attach(5);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  MoveArm(60);
  delay(1000);
  MoveArm(-60);
  delay(1000);

//  MoveArm(180);
//  delay(1000);
//  MoveArm(-180);
//  delay(1000);

  CloseGripper();
  delay(1000);
  OpenGripper();
  delay(1000);
}

int MoveArm(float rotDegrees) {
  //rotDegrees = 180;        // set degrees needed to move arm up and down

  ArmServo.write(rotDegrees); // sets the servo position according to the scaled value
  delay(2000); // waits for the servo to get there
  return rotDegrees;
}

void MoveGripper(void) {
//  for (pos = 0; pos <= 180; pos += 1) {
//    GripperServo.write(pos); 
//    delay(25); 
//  }
  GripperServo.write(75);
  delay(1000);
  GripperServo.write(90);
  delay(800);
  GripperServo.write(115);
  delay(2000);

//  for (pos = 180; pos >= 0; pos -= 1) { 
//    GripperServo.write(pos); 
//    delay(25); 
//  }
  GripperServo.write(90);
  delay(1000);
}

void OpenGripper(void) {
  // this only sets the speed. To stop, use GripperServo.write(90)
  //To open
  GripperServo.write(65);   // calibrate
  delay(2000);              // calibrate
  GripperServo.write(90);
}

void CloseGripper(void) {
  // this only sets the speed. To stop, use GripperServo.write(90)
  //To close
  GripperServo.write(115);     // close
  delay(2000);              // closed?
  GripperServo.write(90);   // might not need to stop, keep "closing" to hold grip
}
