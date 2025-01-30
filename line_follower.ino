#include <Arduino.h>

#define l_motor 3
#define r_motor 5
#define l_motor_in1 6
#define l_motor_in2 7
#define r_motor_in1 8
#define r_motor_in2 9

int base = 150;

float kp = 32;
float ki = 0;
float kd = 0.9;

int pos;
int prev_error = 0;
int pot_limite = 255;

int v_s_min[] = {1023, 1023, 1023, 1023, 1023};
int v_s_max[] = {0, 0, 0, 0, 0};
int s_p[5];
int threshold[5];

int l_pos;

void setup(){
  Serial.begin(9600);
  pinMode(l_motor_in1, OUTPUT);
  pinMode(l_motor_in2, OUTPUT);
  pinMode(r_motor_in1, OUTPUT);
  pinMode(r_motor_in2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(l_motor_in1, LOW);
  digitalWrite(l_motor_in2, LOW);
  digitalWrite(r_motor_in1, LOW);
  digitalWrite(r_motor_in2, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  calibration();  
}

void loop(){
  int line_position = GetPos();
  int correction_power = Calc_PID(line_position, kp, kd, ki);
  Motor_Control(base + correction_power, base - correction_power);
}

void calibration(){
  int v_s[5];
  for(int i=0;i<200;i++){
    delay(10);
    for(int j=0;j<5;j++){
      v_s[j] = analogRead(j);
    }
    for(int j=0;j<5;j++){
      if(v_s[j] < v_s_min[j]){
        v_s_min[j] = v_s[j];
      }
    }
    for(int j=0;j<5;j++){
      if(v_s[j] > v_s_max[j]){
        v_s_max[j] = v_s[j];
      }
    }
  }
  for(int i=0;i<5;i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
  for(int i=0;i<5;i++){
    threshold[i] = (v_s_max[i] - v_s_min[i]) / 2;
  }
  for(int i=0;i<5;i++){
    Serial.print(v_s_min[i]);
    Serial.print(" ");
  }
  Serial.println();
  for(int i=0;i<5;i++){
    Serial.print(v_s_max[i]);
    Serial.print(" ");
  }
  Serial.println();
  for(int i=0;i<5;i++){
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
}

int Calc_PID(int POS, float Kp, float Kd, float Ki){
  int error = pos;
  int D = error - prev_error;
  prev_error = error;
  int pot_giro = (error*Kp + D*kd);

  if(pot_giro > pot_limite){
    pot_giro = pot_limite;
  }
  else if(pot_giro < -pot_limite){
    pot_giro = -pot_limite;
  }
  return pot_giro;
}

void Sensor_Read(){
  int s[5];
  for(int i=0;i<5;i++){
    s[i] = analogRead(i);
  }
  for(int i=0;i<5;i++){
    if(s[i] > threshold[i]){
      s_p[i] = 0;
    }
    else{
      s_p[i] = 1;
    }
  }
  // for(int i=0;i<5;i++){
  //   Serial.print(s_p[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
}

int GetPos(){
  Sensor_Read();
  if(!s_p[0] && !s_p[1] && s_p[2] && s_p[3] && s_p[4]){
    pos = 5;
  }
  else if(!s_p[0] && !s_p[1] && !s_p[2] && !s_p[3] && s_p[4]){
    pos = 4;
  }
  else if(!s_p[0] && !s_p[1] && !s_p[2] && s_p[3] && s_p[4]){
    pos = 3;
  }
  else if(!s_p[0] && !s_p[1] && !s_p[2] && s_p[3] && !s_p[4]){
    pos = 2;
  }
  else if(!s_p[0] && !s_p[1] && s_p[2] && s_p[3] && !s_p[4]){
    pos = 1;
  }
  else if(!s_p[0] && !s_p[1] && s_p[2] && !s_p[3] && !s_p[4]){
    pos = 0;
  }
  else if(!s_p[0] && s_p[1] && s_p[2] && !s_p[3] && !s_p[4]){
    pos = -1;
  }
  else if(!s_p[0] && s_p[1] && !s_p[2] && !s_p[3] && !s_p[4]){
    pos = -2;
  }
  else if(s_p[0] && s_p[1] && !s_p[2] && !s_p[3] && !s_p[4]){
    pos = -3;
  }
  else if(s_p[0] && !s_p[1] && !s_p[2] && !s_p[3] && !s_p[4]){
    pos = -4;
  }
  else if(s_p[0] && s_p[1] && s_p[2] && !s_p[3] && !s_p[4]){
    pos = -5;
  }
  else if(!s_p[0] && !s_p[1] && !s_p[2] && !s_p[3] && !s_p[4]){
    if(l_pos == -4) pos = -5;
    else if(l_pos == 4) pos = 5;
    else if(l_pos == -5) pos = -6;
    else if(l_pos == 5) pos = 6;
  }
  l_pos = pos;
  return pos;
}

void l_motor_control(int val){
  if(val >= 0){
    analogWrite(l_motor, abs(val));
    digitalWrite(l_motor_in1, HIGH);
    digitalWrite(l_motor_in2, LOW);
  }
  else{
    analogWrite(l_motor, abs(val));
    digitalWrite(l_motor_in1, LOW);
    digitalWrite(l_motor_in2, HIGH);  
  }
}

void r_motor_control(int val){
  if(val >= 0){
    analogWrite(r_motor, abs(val));
    digitalWrite(r_motor_in1, LOW);
    digitalWrite(r_motor_in2, HIGH);
  }
  else{
    analogWrite(r_motor, abs(val));
    digitalWrite(r_motor_in1, HIGH);
    digitalWrite(r_motor_in2, LOW);  
  }
}

void Motor_Control(int left, int right){
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  l_motor_control(left);
  r_motor_control(right);
  Serial.print(left);
  Serial.print(" ");
  Serial.print(right);
  Serial.println();
}