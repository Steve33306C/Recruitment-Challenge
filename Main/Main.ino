// --- Libraries Included ---
#include <Servo.h>

// --- IO Pin Connections ---
const int IN1 = ;
const int IN2 = ;
const int ENA = ;

const int IN3 = ;
const int IN4 = ;
const int ENB = ;

const int servo = ;

const int IR[] = {};

Servo servo1;

// --- Global Variables ---


void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  for(int i=0, i<5, i++){
    pinMode(IR[i], INPUT);
  }

  servo1.attach(servo);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// Controlling both motor with different PWM values
void driveRAW(int leftPWM, int rightPWM, bool stop){
  // PWM value of both left and right motor => leftPWM, rightPWm, value range (-255 to 255)
  // stop = true, break motor when PWM = 0

  // Constrain input
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rigthPWM, -255, 255);

  // Configurate left motor
  if(leftPWM > 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, leftPWM);
  }
  else if(leftPWM <0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(leftPWM));
  }
  else if(leftPWM == 0 && stop == true){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Configurate right motor
  if(rightPWM > 0){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftPWM);
  }
  else if(rightPWM <0){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(leftPWM));
  }
  else if(rightPWM == 0 && stop == true){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
  }
  else{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

// Drive motor with a single steer value
void driveSteer(int power, int steer){
  // Control PWM of both motor with power, value range(-255 to 255)
  // Steer adjust power ratio of both motor, value range(-100 - 100)

  // Constrain inputs
  power = constrain(power, -255, 255);
  steer = constrain(steer, -100, 100);

  // Motor PWM ratio algorithm
  int control = abs(power) - abs((power * steer) / 50);

  if (power < 0){
    control = -control;
  }

  if (steer >= 0){
    driveRAW(power, control, true);
  }

  else{
    driveRAW(control, power, true);
  }
}





