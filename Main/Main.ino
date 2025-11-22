// ========== Libraries Included ==========
#include <Servo.h>

// ========== Tuning Values ==========
#define turnRAW 2.7
#define straightTime 130
#define turnPwr 150
#define servoDelay 400

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
const int ENB = 6;

const int servo = 10;

const int IR[5] = {A1, A2, A3, A4, A5};

const int button = 12;

Servo servo1;

// ========== Global Variables ==========
int SENS_MIN[5] = {100,100,100,100,100}; 
int SENS_MAX[5] = {550,550,550,550,550};

const bool INVERT_DARK = false;

PDGain slowGain = {-85.0, 0.0};

int steer = 0;
const int desiredPWM = 200;
int dynamicPWM = desiredPWM;

float pos, density, raw;
bool line;
bool checkCentre = true;
int checkOpp = 0;

int lineCount = 0;
int servoPos = 0;
int currentPos = 0;

float prevErr = 0.0;
unsigned long prevT = 0;
unsigned long lastBreak = 0;
unsigned long dropDelay = 0;
bool servoON = false;

unsigned long runDebug = 0;

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
    analogWrite(ENB, rightPWM);
  }
  else if(rightPWM <0){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(rightPWM));
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

  if (power >= 0){
    L = constrain(L, -power, power);
    R = constrain(R, -power, power);
  }
  else{
    L = constrain(L, power, -power);
    R = constrain(R, power, -power);
  }

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
bool getLine(float &position, float &signalStrength, float &val) {
  static const float W[5] = {-6, -2, 0, +2, +6}; // Weight of each sensors form left to right
  float num = 0.0, den = 0.0; // num = sum of weighted input; den = sum of inout

  for (int i = 0; i < 5; ++i) {
    int raw = analogRead(IR[i]);
    raw = constrain(raw, SENS_MIN[i], SENS_MAX[i]);
    float v = normSensor(raw, SENS_MIN[i], SENS_MAX[i], INVERT_DARK);
    num += v * W[i];
    den += v;
  }

  val = num;

  signalStrength = den;
  if (den < 0.15) {
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

  pinMode(LED_BUILTIN, OUTPUT);

  for(int i=0; i<5; i++){
    pinMode(IR[i], INPUT);
  }

  Serial.begin(9600);

  servo1.attach(servo);

  lastBreak = millis();

  servo1.write(0);
  delay(1000);

// Sensor values configuration
  while(true){
    if (digitalRead(button)){
      if (millis() - lastBreak >= 2500){
        while(digitalRead(button)){
          digitalWrite(LED_BUILTIN, HIGH);
          delay(200);
          digitalWrite(LED_BUILTIN, LOW);
          delay(200);
        }
        break;
      }
    }
    else{
      for (int i = 0; i <= 4; i++){
        int v = analogRead(IR[i]);

        if (v > SENS_MAX[i]){
          SENS_MAX[i] = v;
        }
        if (v < SENS_MIN[i]){
          SENS_MIN[i] = v;
        }
      }

      if (millis() - lastBreak >= 1000){
        for (int i = 0; i <= 4; i++){
          Serial.print(SENS_MAX[i]);
          Serial.print("  ");
        }

        Serial.println();
        
        for (int i = 0; i <= 4; i++){
          Serial.print(SENS_MIN[i]);
          Serial.print("  ");
        }

        Serial.println();

        for (int i = 0; i <= 4; i++){
          Serial.print(analogRead(IR[i]));
          Serial.print(" ");
        }
        Serial.println();
        Serial.println("===================================");
        Serial.println();
        
        lastBreak = millis();

      }
    }
  }

  delay(2000);

  driveSteer(desiredPWM, 0);

  delay(200);

  prevT = micros();
  lastBreak = millis();
  dropDelay = millis();
}

void loop() {
  // Drop cubes or sphere    
  if (servoON == true && millis() - dropDelay >= servoDelay && lineCount <= 3){
    servo1.write(36 * lineCount);
    servoON = false;
    dropDelay = millis();
  }


  // Running debug monitor
  // runDebug++;

  // if (runDebug % 2000 == 0){
  //   Serial.print("Line Position: ");
  //   Serial.println(pos);
  //   Serial.print("Density: ");
  //   Serial.println(density);
  //   Serial.print("Raw Value: ");
  //   Serial.println(raw);
  //   Serial.print("Line Presence: ");
  //   Serial.println(line);
  //   Serial.print("Error: ");
  //   Serial.println(prevErr);
  //   Serial.print("dynamicPWM: ");
  //   Serial.println(dynamicPWM);
  //   Serial.print("Steer Value: ");
  //   Serial.println(steer);
  //   Serial.println();
  //   Serial.println("==============================================");
  //   Serial.println();

  //   runDebug = 0;
  // }

  if (lineCount == 4){
    driveSteer(255, 0);
    delay(250);
    driveSteer(0, 0);
    lineCount++;
    return;
  }

  if (lineCount == 5){
    driveRAW(0, 0, false);
    delay(2);
    return;
  }

  unsigned long currentT = micros();
  float dt = (currentT - prevT)/ 1e6;
  prevT = currentT;

  line = getLine(pos, density, raw);

  if (!line){
    driveSteer(dynamicPWM, steer);
    return;
  }

  if (abs(raw) > turnRAW && millis() - lastBreak >= straightTime){
    driveSteer(desiredPWM, 0);
    if(raw > 0){
      checkOpp = -1;
    }
    else{
      checkOpp = 1;
    }
  }

  if (density > 4.0 && pos <= 1.2 && millis() - lastBreak >= straightTime){
    lineCount++;
    checkOpp = 0;
    servoON = true;
    dropDelay = millis();
    lastBreak = millis();
  }

  if(checkOpp == -1){
    lastBreak = millis();
    checkCentre = false;
    while(millis() - lastBreak < straightTime){
      if (analogRead(IR[0]) > (SENS_MAX[0] + SENS_MIN[0]) / 2){
        checkCentre = true;
        servoON = true;
        dropDelay = millis();
        break;
      }
    }

    if(checkCentre == true){
      lineCount++;
      checkCentre = false;
      checkOpp = 0;
      lastBreak = millis();
      if (lineCount == 4){
        return;
      }
    }
    else{      
      while(analogRead(IR[3]) < (SENS_MAX[3] + SENS_MIN[3]) / 2){
        driveSteer(turnPwr, 100);
      }
    }
    checkOpp = 0;
  }

  if(checkOpp == 1){
    lastBreak = millis();
    checkCentre = false;
    while(millis() - lastBreak < straightTime){
      if (analogRead(IR[4]) > (SENS_MAX[4] + SENS_MIN[4]) / 2){
        checkCentre = true;
        servoON = true;
        dropDelay = millis();
        break;
      }
    }

    if(checkCentre == true){
      lineCount++;
      lastBreak = millis();
      checkCentre = false;
      checkOpp = 0;
      if (lineCount == 5){
        return;
      }
    }
    else{
      while(analogRead(IR[1]) < (SENS_MAX[1] + SENS_MIN[1]) / 2){
        driveSteer(turnPwr, -100);
      }
    }
    checkOpp = 0;
  }

  steer = PDSteer(pos, prevErr, dt, slowGain);

  dynamicPWM = desiredPWM - int(min(90, fabs(steer * 1.8)));

  driveSteer(dynamicPWM, steer);
}