//2018.06.11 Kim Seok Min

#include <MPU6050.h>
#include <I2Cdev.h>
#include <SoftwareSerial.h>
#include<Wire.h>
#include<Math.h>
#define g 9.80665

//여기부터 새로운 각도 재기파트의 변수
MPU6050 accelgyro;

int accel_reading;
int accel_corrected;
int accel_offset = 200;
float accel_angle;
float accel_scale = 1;
 
int gyro_offset = 131;
float dgy_x,deg;
float angle;
float angle_d;
float last_read_time;
float last_x_angle,last_y_angle,last_z_angle;
//여기까지

int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float emgin = 0;
float tcount;

char SEND[35];
char SEND2[30];
char SEND_SEN[25];
String temp;

int FSRpin = 2;
int FSRpin2 = 3;
int EMG = 0;

int Vo;
int Vo2;
float Rfsr;
float Rfsr2;

//MPU6050 accelgyro(0x68);
SoftwareSerial mySerial(2, 3);   //bluetooth module Tx:Digital 2 Rx:Digital 3

void setup() {
  Wire.begin();      //Wire 라이브러리 초기화
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
  pinMode(8, OUTPUT);    //HC-05
  digitalWrite(8,HIGH);
  
  Serial.begin(9600);
  Serial.println("Start");  //ATcommand Start
  mySerial.begin(38400);
}
void loop() {
  tcount = millis();
  Vo = analogRead(FSRpin);
  Vo2 = analogRead(FSRpin2);
  Rfsr = ((9.78*Vo)/(1-(Vo/1023.0)));
  Rfsr2 = ((9.78*Vo2)/(1-(Vo2/1023.0)));
  String FSR(Rfsr);
  String FSR2(Rfsr2);
  
  int emg = analogRead(EMG);
  float voltage = float(emg)*5/1023; //- 1/6569;
  String VOL(voltage);
  
  accelgyro.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  float Acx = ((float)AcX/16384) * g * (-1);
  float Acy = ((float)AcY/16384) * g * (-1);
  float Acz = ((float)AcZ/16384) * g * (-1);
  
  accel_reading = AcY;
  accel_corrected = accel_reading - accel_offset;
  accel_corrected = map(accel_corrected, -16800, 16800, -90, 90);
  accel_corrected = constrain(accel_corrected, -90, 90);
  accel_angle = (float)(accel_corrected * accel_scale);

  deg = atan2(AcX, AcZ) * 180 / PI;     // rad to deg
  
  // 자이로+가속도 조합한 각도
  dgy_x = GyY / gyro_offset;
  angle_d = (0.95 * (angle_d + (dgy_x * 0.001))) + (0.05 * deg);
  angle = angle_d *PI /180;

  String ANGLE(angle_d);
  float time_sen = tcount * 0.001;
  String TIME(time_sen);
  String send_sen = TIME + " " + FSR + " " + FSR2 + " " + VOL + " " + ANGLE + " ";
  send_sen.toCharArray(SEND_SEN, send_sen.length());
  Serial.write(SEND_SEN);
  Serial.write('\n');
  mySerial.write(SEND_SEN);
  mySerial.write('\n');

  delay(20);
}
