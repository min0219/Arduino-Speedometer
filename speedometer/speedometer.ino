//2018.06.22 Kim Seok Min

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

const double ts = 0.001*0.001;
unsigned long tCount = 0;
unsigned long tCountPre = 0;
unsigned long tCounting = 0;
float vx = 0, vy = 0, vz = 0;

//const int MPU=0x68;  //MPU 6050 의 I2C 기본 주소
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float t,dt,ax,az,aX;//시간, 미소시간, 센서를 축으로한 x,z축 방향의 가속도, 지면을 수평방향으로 지정한 가속도이다.
float vX = 0;  //적분결과: 수평방향 속도이다.
float angle_g;//걷기 시작할때 반환되어야하는 값으로, 보행 시작시 지정되는 가속도센서의 지면에 대한 기울어진 각을 반환해준다. 
float ref_angle, angle_dif;
float distance;//이건 첫번째알고리즘이랑은 상관 없는데 두번째 알고리즘에서 보폭을 알려주기 위해서 만들어둔 보행 시작시 발의 좌표를 기준으로 한 축이다
float vX_avg=0;//이것도 두번재 알고리즘에 사용되는 값으로 평균 보행속도를 반환해준다

float emgin = 0;
float tcount;

char SEND[35];
char SEND2[30];
char SEND_SEN[25];
//char Acx[],Acy[],Acz[],Gyx[],Gyy[],Gyz[];
String temp;

int FSRpin = 2;
int FSRpin2 = 3;
int EMG = 0;

int Vo;
int Vo2;
float Rfsr;
float Rfsr2;

boolean isWalk = false;
int cnt = 1;//걸음 수 측정

//MPU6050 accelgyro(0x68);
SoftwareSerial mySerial(2, 3);   //bluetooth module Tx:Digital 2 Rx:Digital 3

void setup() {
  Wire.begin();      //Wire 라이브러리 초기화
  accelgyro.initialize();
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
  
  //Wire.beginTransmission(MPU); //MPU로 데이터 전송 시작
  //Wire.write(0x6B);  // PWR_MGMT_1 register
  //Wire.write(0);     //MPU-6050 시작 모드로
  //Wire.endTransmission(true);
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
  //String send_sen = TIME + " " + FSR + " " + FSR2 + " " + VOL + " " + ANGLE;
  String send_sen = TIME + " " + VOL + "  ";
  //send_sen.toCharArray(SEND_SEN, send_sen.length());
  
  if(cnt==1){
    ref_angle = angle;
  }

  if(Rfsr<200){
    if(isWalk==false){
      tCount = micros();
      tCountPre = tCount;
      tCounting = tCount;
      isWalk=true;
      vX = 0;
      distance = 0;
      emgin = 0;
    }else if(isWalk==true){
      tCount = micros();
      dt = (float)(tCount-tCounting)*ts;
      tCounting = tCount;
      aX = Acx*cos(angle)-Acz*sin(angle);
      vX += aX*dt;
      distance += vX*dt;
      emgin += voltage*dt;
    }
  }else{
    if(isWalk==true){
      isWalk=false;
      angle_g = angle;
      angle_dif = angle_g-ref_angle;
      tCount = micros();
      vX_avg = distance/((tCount-tCountPre)*ts)*3.6;
      String TEMP(vX_avg);
      String TEMP2(emgin);
      String TEMP3 = "v : " + TEMP + " emg : " +TEMP2 + " ";
      TEMP3.toCharArray(SEND2, TEMP3.length());
      mySerial.write(SEND2 );
      mySerial.write('\n');
      Serial.println(SEND2);
    }
  }
  //값 보정
  cnt+=1;
  delay(20);
}
