const int IR[5] = {A1, A2, A3, A4, A5};

int SENS_MIN[5] = {200,200,200,200,200}; 
int SENS_MAX[5] = {500,500,500,500,500};

long p = 0;

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<5; i++){
    pinMode(IR[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i = 0; i <= 4; i++)
  {
    int val = analogRead(IR[i]);
    if (val > SENS_MAX[i]){
      SENS_MAX[i] = val;
    }
    if (val < SENS_MIN[i]){
      SENS_MIN[i] = val;
    }
  }

  p++;

  if (p % 1500 == 0){
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
    p = 0;
  }
}