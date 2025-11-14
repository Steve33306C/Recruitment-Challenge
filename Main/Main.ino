// ========== Libraries Included ==========
#include <Servo.h>

// ========== Structures ==========
struct PDGain{
  float Kp;
  float Kd;
};

// ========== IO Pin Connections ==========
const int IN1 = 2;
const int IN2 = 7;
const int ENA = 11;

const int IN3 = 4;
const int IN4 = 8;
const int ENB = 10;

const int servo = 3;

const int IR[5] = {A1, A2, A3, A4, A5};

Servo servo1;

// ========== Global Variables ==========
int SENS_MIN[5] = {200,200,200,200,200}; 
int SENS_MAX[5] = {900,900,900,900,900};

const bool INVERT_DARK = true;

PDGain fastGain = {18.0, 4.5};

float prevErr = 0.0;
unsigned long prevT = 0;

// ========== Functions ==========

// Controlling both motor with different PWM values
void driveRAW(int leftPWM, int rightPWM, bool stop){
  // PWM value of both left and right motor => leftPWM, rightPWm, value range (-255 to 255)
  // stop = true, break motor when PWM = 0

  // Constrain input
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

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
  int control = map(steer, -100, 100, -(power*2), power*2);

  int L = power + control;
  int R = power - control;

  L = constrain(L, -power, power);
  R = constrain(R, -power, power);

  driveRAW(L, R, true); 
}

// Mapping all IR sensors value to 0 and 1 (0 for white; 1 for black)
float normSensor(int raw, int sMin, int sMax, bool invertDark) {
  // x = ratio of the current sensor value and the difference between the sensor's max and min input
  float x = float(raw - sMin) / float(max(1, sMax - sMin));
  x = constrain(x, 0.0, 1.0);
  return invertDark ? (1.0 - x) : x;
}

// Checking if line is currently present and update the position of it
bool getLine(float &position, float &signalStrength) {
  static const float W[5] = {-2, -1, 0, +1, +2}; // Weight of each sensors form left to right
  float num = 0.0, den = 0.0; // num = sum of weighted input; den = sum of inout

  for (int i = 0; i < 5; ++i) {
    int raw = analogRead(IR[i]);
    float v = normSensor(raw, SENS_MIN[i], SENS_MAX[i], INVERT_DARK);
    num += v * W[i];
    den += v;
  }

  signalStrength = den;
  if (den < 0.02) {
    return false;
  }

  position = num / den;
  return true;
}

float PDSteer(float position, float &prevErr, float dtSec, const PDGain &g) {
  float err = 0.0f - position;
  float deriv = (dtSec > 1e-4) ? (err - prevErr) / dtSec : 0.0f;
  float u = g.Kp * err + g.Kd * deriv;
  prevErr = err;
  return u; // positive u => steer left wheel faster, right slower
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  for(int i=0; i<5; i++){
    pinMode(IR[i], INPUT);
  }

  servo1.attach(servo);

  prevT = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}



