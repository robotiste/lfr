#include <QTRSensors.h>

#define rightMotorF 8
#define rightMotorB 7
#define rightMotorPWM 9
#define leftMotorF 5
#define leftMotorB 4
#define leftMotorPWM 3
#define stby 6

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
  
  int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
 
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
  
  
  const int maximum = 200;
  
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
  
  if(sensors[0] < 500){
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,150);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 150);
    digitalWrite(stby,HIGH);
    delay(100);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, 100);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(300);
    do{
    int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    }
    while(sensors[4] > 500);
    
  }
}
