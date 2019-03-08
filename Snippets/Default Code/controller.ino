#include <Servo.h>

const int MOVE_FOWARD = 0;
const int MOVE_BACKWARD = 1;
const int STOP = 2;

const int LEFT_MOTOR_PIN = 13;
const int RIGHT_MOTOR_PIN = 12;

Servo left_motor;
Servo right_motor;

int STATE;

void setup() {
  // put your setup code here, to run once:
  STATE = STOP;
  Serial.begin(9600);
  
  left_motor.attach(LEFT_MOTOR_PIN);
  right_motor.attach(RIGHT_MOTOR_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(STATE);
  //left_motor.write(100);
  //right_motor.write(100);
}
