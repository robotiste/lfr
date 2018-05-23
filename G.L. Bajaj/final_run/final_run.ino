#include <QTRSensors.h>

#define rightMotorF 8
#define rightMotorB 7
#define rightMotorPWM 9
#define leftMotorF 5
#define leftMotorB 4
#define leftMotorPWM 3
#define stby 6
int count = 0;
int count_blank = 0;
int a = 150;
int b = 120;

QTRSensorsRC qtr((unsigned char[]) {A0, 11, A1, A2, A3, A4, 12, A5}, 8, 2500);
                           
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  Serial.begin(9600);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int lastError = 0;
float kp = 0.12;  // 0.08 // for small = 0.1
float kd = 1; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
int error, power_difference;
 
void loop()
{
  unsigned int sensors[8];
  
  int position = qtr.readLine(sensors, QTR_EMITTERS_ON);
 
  error = 3500 - int(position);
  integral += error;
  derivative = error - lastError;
  power_difference = kp * error + ki * integral + kd * derivative;  
  lastError = error;
  //Serial.println(power_difference);
  /*
  for(int i = 0; i<8; i++){
    Serial.print(sensors[i]);
    Serial.print("         ");
  }
  Serial.println();
  */
  
  
  const int maximum = a;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;  
    
  if (power_difference < 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum + power_difference);
    digitalWrite(stby,HIGH);
  } 
  else { 
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum);
    digitalWrite(stby,HIGH);
  }
 
  if(sensors[7] > 700){
    count+=1;
  }
  else count = 0;
  
  if(count > 8){
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,b);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, b);
    digitalWrite(stby,HIGH);
    delay(200);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, b);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, b);
    digitalWrite(stby,HIGH);
    delay(150);
    do{
    int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, b);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, b);
    digitalWrite(stby,HIGH);
    }
    while(sensors[2] < 700);
    
  }
  if(sensors[1] < 300 && sensors[2] < 300 && sensors[3] < 300 && sensors[4] < 300 && sensors[5] < 300 && sensors[6] < 300){
    count_blank+=1;
  }
  else count_blank = 0;

  if(count_blank > 10){
   digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, b);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, b);
    digitalWrite(stby,HIGH);
    delay(200);
    do{
    int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, b);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, b);
    digitalWrite(stby,HIGH);
    }
    while(sensors[2] < 700);
}
}
