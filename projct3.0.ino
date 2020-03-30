#include <FlexiTimer2.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
/*oled 引脚定义*/
#define OLED_MOSI  26
#define OLED_CLK   28
#define OLED_DC    22
#define OLED_CS    12
#define OLED_RESET 24
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
MPU6050  accelgyro;; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    //角度变量
int16_t ax, ay, az, gx, gy, gz;             //加速度计陀螺仪原始数据
long gxo = 0, gyo = 0, gzo = 0;             //陀螺仪偏移量
float pi = 3.1415926;
float GyroRatio = 131.0;                    //陀螺仪比例系数
unsigned long now, lastTime = 0;
float dt;  //微分时间
/*二倍频编码器值读取变量*/

byte encoder0PinALast,encoder1PinALast,encoder2PinALast;
double duration0,abs_duration0;//the number of the pulses
double duration1,abs_duration1;//the number of the pulses
double duration2,abs_duration2;//the number of the pulses
boolean Direction0;//the rotation direction 
boolean Direction1;//the rotation direction 
boolean Direction2;//the rotation direction
boolean result0,result1,result2,result3,result4;   
double val_output0,val_output1,val_output2;//速度环PID输出值，用于提供给电机PWM功率值。
double Setpoint0,Setpoint1,Setpoint2;   //PID设定值
double w=0,desire_theta=0,theta_out,w_output;//角速度与姿态环PID输入输出参数定义
double desire_w = 0.00;
static double theta=0;
String angle;
//PID参数
double Kp1=0.5,Ki1=7,Kd1=0;//速度环PID参数
double Kp2=0.5,Ki2=7,Kd2=0;
double Kp3=0.5,Ki3=4.1,Kd3=0;
double kp=1,ki=1,kd;    //角速度环PID参数
double p=0.5,i=8,d;   //theta环PID参数
//五个PID初始化
PID myPID0(&abs_duration0,&val_output0,&Setpoint0,Kp1,Ki1,Kd1,P_ON_M,DIRECT);
PID myPID1(&abs_duration1,&val_output1,&Setpoint1,Kp2,Ki2,Kd2,P_ON_M,DIRECT);
PID myPID2(&abs_duration2,&val_output2,&Setpoint2,Kp3,Ki3,Kd3,P_ON_M,DIRECT);
PID PID_w(&w, &w_output, &desire_w, kp, ki, kd,P_ON_M,DIRECT);
PID PID_theta(&theta,&theta_out,&desire_theta,p,i,d,P_ON_M,DIRECT);
//******************PWM引脚和电机驱动引脚***************************
int AIN1 = 5;//A电机PWM波
int AIN2 = 6;//A电机PWM波
int BIN1 = 8;//B电机PWM波
int BIN2 = 7;//B电机PWM波
int CIN1 = 44;//C电机PWM波
int CIN2 = 46;//C电机PWM波
////////编码器引脚///////////
#define encoder0pinA 2//A路电机编码器引脚
#define encoder1pinA 3//B路电机编码器引脚
#define encoder2pinA 19//C路电机编码器引脚
#define encoder0pinB 51//A路电机编码器引脚
#define encoder1pinB 53//B路电机编码器引脚
#define encoder2pinB 50//C路电机编码器引脚
//各种变量定义
#define L  23.5 //轮子到定位中心距离
float x0,x1,x2;//三个轮子编码器里程
double sx,sy; //X轴上编码器实际里程 Y轴上编码器实际里程，脉冲数
double a,b,c; //PID输出正负转换参数
double desire_speed_x=0,desire_speed_y=0;//期望的全局速度
int flag=1;//状态标志位
String qr_data = ""; //存储二维码顺序
String get_data = ""; //存储抓取顺序
int get_order[3];//存储抓取顺序
int put_order[3];//存储放置顺序
#define PUTORDER "123"
static int t=180;
static  int F=1;
static int mill=100;
String color_data="";
void setup()
{  //pwm.begin();
   //pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
   Serial.begin(115200);
   Serial2.begin(115200);
   oled_init();
   //舵机初始化
   //pwm.setPWM(0,0,120);
   //delay(500);
   //pwm.setPWM(1,0,150);
   oled_init();
   //accelgyro.initialize();                 //MPU6050初始化
        // unsigned short times = 200; //采样次数
    //for(int i=0;i<times;i++)
    //{
       // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值 
       // gxo += gx; gyo += gy; gzo += gz;
   // }
    
  // gxo /= times; gyo /= times; gzo /= times;//计算陀螺仪偏移
  // motor_init();
   // delay(300);
  // PID_init();
}
void loop()
{   //Serial.write('a');
    //delay(500);
    color_Detect();
    /*
    if(color_data=="a")
    {//roll(-90);
     //Y__(0,4.0,0,-550,-80); 
       analogWrite(CIN2,200), analogWrite(CIN1, 0);
       analogWrite(AIN1,200), analogWrite(AIN2, 0); 
       analogWrite(BIN1,200), analogWrite(BIN2, 0); 
    }
    if(color_data=="ab")
    {   
       analogWrite(CIN2,200), analogWrite(CIN1, 0);
       analogWrite(AIN1,200), analogWrite(AIN2, 0); 
       analogWrite(BIN1,200), analogWrite(BIN2, 0); 
       //roll_(90);
    }
    
 */
    
   // control();
    /*
    if(flag==1)      {Y__(0,4.0,0,-550,-80);} 
    if(flag==2)      {  track_Y(6700);   color_Detect();  }
    if(flag==3)      {  delay(1000); 
                        oled_show(color_data);
                        delay(1000);
                        flag++;
                        }
    if(flag==4)        track_Y_(-2700);
    if(flag==5)        roll(-90);
    if(flag==6)        track_Y(600);
    if(flag==7)        zhua();
    if(color_data=="123")
    {   if(flag==8)    track_Y_(-300);
        if(flag==9)     roll(-90);
        if(flag==10)    track_Y(620);
        if(flag==11)    roll(-94);
        if(flag==12)    adjust();
        if(flag==13)    track_Y(2000);
        if(flag==14)    fang();
        if(flag==15)    roll(-195);
        if(flag==16)    adjust();
        if(flag==17)    track_Y(2000);
        if(flag==18)    roll_(82);
        if(flag==19)    track_Y(1300);
        if(flag==20)    roll(-90);
        if(flag==21)    track_Y(800);
        if(flag==22)    zhua();
        if(flag==23)    track_Y_(-300);
        if(flag==24)    roll(-180);
        if(flag==25)    track_Y_(-800);
        if(flag==26)    track_Y(2400);
        if(flag==27)    fang();
        if(flag==28)    roll(-186);
        if(flag==29)    adjust();
        if(flag==30)    track_Y(2600);
        if(flag==31)    track_Y_(-300);
        if(flag==32)    roll(-90);
        if(flag==33)    track_Y_(-460);
        if(flag==34)    roll_(85);
        if(flag==35)    track_Y(800);
        if(flag==36)    zhua();
        if(flag==37)    roll(-180);
        if(flag==38)    Y(0.5,10,0,550,20);
        if(flag==39)    adjust();
        if(flag==40)    track_Y(2300);
        if(flag==41)    fang();
        if(flag==42)    roll_(100);
        if(flag==43)    track_Y(9000);
    }
   if(color_data=="132")
   {    if(flag==8)     track_Y_(-300);
        if(flag==9)     roll(-90);
        if(flag==10)    track_Y(600);
        if(flag==11)    roll(-94);
        if(flag==12)    adjust();
        if(flag==13)    track_Y(1900);
        if(flag==14)    fang();
        if(flag==15)    roll(-189);
        if(flag==16)    adjust();
        if(flag==17)    track_Y(1900);
        if(flag==18)    roll_(80);
        if(flag==19)    track_Y(1100);
        if(flag==20)    roll(-90);
        if(flag==21)    track_Y(800);
        if(flag==22)    zhua();
        if(flag==23)    track_Y_(-300);
        if(flag==24)    roll(-90);
        if(flag==25)    track_Y_(-1100);
        if(flag==26)    roll(-90);
        if(flag==27)    adjust();      
        if(flag==28)    track_Y(1900);
        if(flag==29)    fang();
        if(flag==30)    roll(-189);
        if(flag==31)    adjust();
        if(flag==32)    track_Y(1900);
        if(flag==33)    roll(-90);
        if(flag==34)    track_Y(800);
        if(flag==35)    roll_(90);
        if(flag==36)    track_Y(900);
        if(flag==37)    zhua();
        if(flag==38)    track_Y_(-300);
        if(flag==39)    roll(-90);
        if(flag==40)    track_Y(650);
        if(flag==41)    roll(-90);
        if(flag==42)    adjust();
        if(flag==43)    track_Y_(-800);
        if(flag==44)    track_Y(2400);
        if(flag==45)    fang();
        if(flag==46)    roll_(100);
        if(flag==47)    track_Y(9000);
}
  if(color_data=="213")
   {if(flag==8)     track_Y_(-300);
    if(flag==9)     roll_(85);
    if(flag==10)    track_Y(370);
    if(flag==11)    roll_(80);
    if(flag==12)    track_Y_(-800);
    if(flag==13)    adjust();
    if(flag==14)    track_Y(2300);
    if(flag==15)    fang();
    if(flag==16)    roll(-189);
    if(flag==17)    adjust();
    if(flag==18)    track_Y(2500);
    if(flag==19)    zhua();
    if(flag==20)    track_Y_(-300);
    if(flag==21)    roll_(86);
    if(flag==22)    track_Y_(-1600);
    if(flag==23)    roll_(80);
    if(flag==24)    adjust();
    if(flag==25)    track_Y(2100);
    if(flag==26)    fang();
    if(flag==27)    roll(-185);
    if(flag==28)    adjust();
    if(flag==29)    track_Y(2100);
    if(flag==30)    roll_(83);
    if(flag==31)    track_Y(1700);
    if(flag==32)    roll(-95);
    if(flag==33)    track_Y(800);
    if(flag==34)    zhua();
    if(flag==35)    roll(-181);
    if(flag==36)    Y(0.5,10,0,550,20);
    if(flag==37)    adjust();
    if(flag==38)    track_Y(2000);
    if(flag==39)    fang();
    if(flag==40)    roll_(100);
    if(flag==41)    track_Y(9000);
   }
  if(color_data=="231")
   {if(flag==8)     track_Y_(-300);
    if(flag==9)     roll_(85);
    if(flag==10)    track_Y(370);  
    if(flag==11)    roll_(80);
    if(flag==12)    track_Y_(-800);
    if(flag==13)    adjust();
    if(flag==14)    track_Y(2300);
    if(flag==15)    fang();
    if(flag==16)    roll(-190);
    if(flag==17)    adjust();
    if(flag==18)    track_Y(2500);
    if(flag==19)    zhua();
    if(flag==19)    track_Y_(-300);
    if(flag==20)    roll(-90);
    if(flag==21)    track_Y_(-1100);
    if(flag==22)    roll(-93);
    if(flag==23)    adjust();
    if(flag==24)    track_Y(2100);
    if(flag==25)    fang();
    if(flag==26)    roll(-187);
    if(flag==27)    adjust();
    if(flag==28)    track_Y(2100);
    if(flag==29)    roll(-90);
    if(flag==30)    track_Y(750);
    if(flag==31)    roll_(90);
    if(flag==32)    track_Y(1000);
    if(flag==33)    zhua();
    if(flag==34)    track_Y_(-300);
    if(flag==35)    roll(-90);
    if(flag==36)    track_Y(2000);
    if(flag==37)    roll(-90);
    if(flag==38)    adjust();
    if(flag==39)    track_Y(2000);
    if(flag==40)    fang();
    if(flag==41)    roll_(100);
    if(flag==42)    track_Y(9000);

   }
   if(color_data=="312")
   {  if(flag==8)     track_Y_(-300);
      if(flag==9)     roll(-90);
      if(flag==10)    track_Y_(-1700);
      if(flag==11)    roll(-90);
      if(flag==12)    adjust();
      if(flag==13)    track_Y(1900);
      if(flag==14)    fang();
      if(flag==15)    roll(-189);
      if(flag==16)    adjust();
      if(flag==17)    track_Y(1900);
      if(flag==18)    roll(-90);
      if(flag==19)    track_Y(1400);
      if(flag==20)    roll_(85);
      if(flag==21)    track_Y(1000);
      if(flag==22)    zhua();
      if(flag==23)    track_Y_(-300);
      if(flag==24)    roll(-90);
      if(flag==25)    track_Y(1300);
      if(flag==26)    roll(-90);
      if(flag==27)    adjust();
      if(flag==28)    track_Y(2000);
      if(flag==29)    fang();
      if(flag==30)    roll(-190);
      if(flag==31)    adjust();
      if(flag==32)    track_Y(2000);
      if(flag==33)    roll_(85);
      if(flag==34)    track_Y(1700);
      if(flag==35)    roll(-95);
      if(flag==36)    track_Y(1200);
      if(flag==37)    zhua();
      if(flag==38)    track_Y_(-300);
      if(flag==39)    roll(-90);
      if(flag==40)    track_Y(600);
      if(flag==41)    roll(-90);
      if(flag==42)    adjust();
      if(flag==43)    track_Y_(-800);
      if(flag==43)    track_Y(2200);
      if(flag==44)    fang();
      if(flag==45)    roll_(100);
      if(flag==46)    track_Y(9000);
   }
  if(color_data=="321")
  {   if(flag==8)     track_Y_(-300);
      if(flag==9)     roll(-90);
      if(flag==10)    track_Y_(-1700);
       if(flag==11)    roll(-90);
      if(flag==12)    adjust();
      if(flag==13)    track_Y(1900);
      if(flag==14)    fang();
      if(flag==15)    roll(-189);
      if(flag==16)    adjust();
      if(flag==17)    track_Y(1900);
      if(flag==18)    roll(-90);
      if(flag==19)    track_Y(1400);
      if(flag==20)    roll_(85);
      if(flag==21)    track_Y(1000);
      if(flag==22)    zhua();
      if(flag==23)    roll(-181);
      if(flag==24)    adjust();
      if(flag==25)    track_Y_(-800);
      if(flag==26)    track_Y(2400);
      if(flag==27)    fang();
      if(flag==28)    roll(-189);
      if(flag==29)    adjust();
      if(flag==30)    track_Y(2600);
      if(flag==31)    track_Y_(-300);
      if(flag==32)    roll(-85);
      if(flag==33)    track_Y_(-450);
      if(flag==34)    roll_(81);
      if(flag==35)    track_Y(800);
      if(flag==36)    zhua();
      if(flag==37)    track_Y_(-300);
      if(flag==38)    roll(-90);
      if(flag==39)    track_Y(1950);
      if(flag==40)    roll(-90);
      if(flag==41)    adjust();
      if(flag==42)    track_Y(1900);
      if(flag==43)    fang();
      if(flag==44)    roll_(100);
      if(flag==45)    track_Y(9000);
} */
}

void zhua()
{  for( int pos=120;pos<=160;pos++)
   { pwm.setPWM(0,0,pos); delay(10);}
   delay(500);
   for(int pos=150;pos<=230;pos++)
   {pwm.setPWM(1,0,pos);delay(10);}
   delay(500);
   flag++;
   
}
void fang()
{   for(int pos1=160;pos1<=230;pos1++)
    {pwm.setPWM(0,0,pos1); delay(10);}
    delay(500);
    for(int pos2=230;pos2>=150;pos2--)
    {pwm.setPWM(1,0,pos2);delay(10);}
    delay(500);
    for(int pos3=230;pos3>=120;pos3--)
    {pwm.setPWM(0,0,pos3); delay(10);}
    delay(500);
    flag++;
}

   
 void mpu6050()
 {     unsigned long now = millis();             //当前时间(ms)
    dt = (now - lastTime) / 1000.0;           //微分时间(s)
    lastTime = now;                           //上一次采样时间(ms)
 
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //读取六轴原始数值
    float gyrox =  (gx-gxo) / GyroRatio * dt; //x轴角速度
    float gyroy =  (gy-gyo) / GyroRatio * dt; //y轴角速度
    float gyroz =  (gz-gzo) / GyroRatio * dt; //z轴角速度
    w=gyroz ;
    theta+=w;
    
 }
 void control() 
{ mpu6050();
  abs_duration0=abs(duration0);  
  abs_duration1=abs(duration1);
  abs_duration2=abs(duration2);
  a=get_wheel_speed1(desire_speed_x,desire_speed_y,w_output);
  Setpoint0=abs(a);
  b=get_wheel_speed2(desire_speed_x,desire_speed_y,w_output);
  Setpoint1=abs(b);
  c=get_wheel_speed3(desire_speed_x,desire_speed_y,w_output);
  Setpoint2=abs(c);
        result0=myPID0.Compute();//PID转换完成返回值为1
      if(result0)
      {  x0+= duration0;
        duration0 = 0; //计数清零等待下次计数
      }
        result1=myPID1.Compute();//PID转换完成返回值为1
      if(result1)
      { x1+=duration1;
        duration1 = 0; //计数清零等待下次计数
      }
        result2=myPID2.Compute();//PID转换完成返回值为1
      if(result2)
      { x2+=duration2;
        duration2 = 0; //计数清零等待下次计数
      }
      sx=(x1-x0)/1.732;
      sy=(2*x2-x0-x1)/3;
}
void turn_adjust()
{ 
  if(F%2==1)
  { 
  if(x2>-mill)
     { adjust_Y_();}
     else
     {
     mill+=50;
     F++;}

    
    F++;}
  if(F%2==0)
  {  if(x2<mill)
     { adjust_Y();}
     else
     {
      mill+=50;
     F++;}
  }
}
void adjust_Y_()
{         desire_speed_x=0;
          desire_speed_y=-5;
          myPID0.SetTunings(0.5,7,0);
          myPID1.SetTunings(0.5,7,0);
          myPID2.SetTunings(1,15,0);
          
          myPID0.SetMode(AUTOMATIC);
          myPID1.SetMode(AUTOMATIC);
          myPID2.SetMode(AUTOMATIC);
          analogWrite(CIN2,val_output2+5),analogWrite(CIN1, 0);
          delay(5);
        analogWrite(AIN1, val_output0), analogWrite(AIN2, 0); 
        analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
       
          }
void adjust_Y()
{         desire_speed_x=0;
          desire_speed_y=5;
          myPID0.SetTunings(0.5,7,0);
          myPID1.SetTunings(0.5,7,0);
          myPID2.SetTunings(1,15,0);
          myPID0.SetMode(AUTOMATIC);
          myPID1.SetMode(AUTOMATIC);
          myPID2.SetMode(AUTOMATIC);
          analogWrite(CIN1,val_output2+5),analogWrite(CIN2, 0);
          delay(5);
       analogWrite(AIN2, val_output0), analogWrite(AIN1, 0); 
       analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
       }
void Y__(float pp,float ii,float dd,int x,int v)//负Y轴平移,X代表距离
{  desire_speed_x=0;
   desire_speed_y=v;
   //PID_theta.SetMode(AUTOMATIC);
   myPID2.SetTunings(pp,ii,dd);
   myPID0.SetMode(AUTOMATIC);
   myPID1.SetMode(AUTOMATIC);
   myPID2.SetMode(AUTOMATIC);
    if(sy>x)
    {   analogWrite(CIN2,val_output2),analogWrite(CIN1, 0); 
    delay(10);
      analogWrite(AIN1, val_output0), analogWrite(AIN2, 0); 
       analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
       }
     if(sy<x)
      { myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
       // PID_theta.SetMode(MANUAL);
       desire_speed_x=0;
       desire_speed_y=0;
       w_output=0;
       x0=0,x1=0,x2=0,theta=0; 
       analogWrite(CIN1,200), analogWrite(CIN2, 0);
       analogWrite(AIN2,200), analogWrite(AIN1, 0); 
       analogWrite(BIN2,200), analogWrite(BIN1, 0); 
       delay(100);  
       flag++;  
      }
}
void Y(float pp,float ii,float dd,int x,int v)//Y轴正向平移,x代表距离
{  desire_speed_x=0;
   desire_speed_y=v;
    myPID2.SetTunings(pp,ii,dd);
    PID_theta.SetMode(AUTOMATIC);
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    if(sy<x)
    {  analogWrite(CIN1,val_output2),analogWrite(CIN2, 0);
       delay(10);
       analogWrite(AIN2, val_output0), analogWrite(AIN1, 0); 
       analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
         }
             if(sy>x)
      { myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        PID_theta.SetMode(MANUAL);
       desire_speed_x=0;
       desire_speed_y=0;
       w_output=0;
       x0=0,x1=0,x2=0,theta=0;
       analogWrite(CIN2,135), analogWrite(CIN1, 0);
       analogWrite(AIN1,140), analogWrite(AIN2, 0); 
       analogWrite(BIN1,140), analogWrite(BIN2, 0); 
       delay(100);  
       flag++;  
      }
}
void motor_init()
{  int fff = 1;
  TCCR1B =(TCCR1B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  TCCR3B =(TCCR3B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  TCCR4B =(TCCR4B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ 
  TCCR5B =(TCCR5B & 0xF8) | fff;//调整计数器分频，频率调高至31.374KHZ
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(35,INPUT);
  pinMode(33,INPUT);
  pinMode(31,INPUT);
  pinMode(37,INPUT);
  pinMode(23,INPUT);
  pinMode(encoder0pinA, INPUT);
  pinMode(encoder1pinA, INPUT);
  pinMode(encoder2pinA, INPUT);
  pinMode(encoder0pinB, INPUT);
  pinMode(encoder1pinB, INPUT);
  pinMode(encoder2pinA, INPUT);
  delay(300);
   Direction0 = true;
  attachInterrupt(0,wheelSpeed0 , CHANGE);
  Direction1 = true;
  attachInterrupt(1,wheelSpeed1, CHANGE);
  Direction2 = true;
  attachInterrupt(4,wheelSpeed2, CHANGE);
}
void track_X(int x)//X轴正向循迹
{   if(sx<x)
    desire_speed_x=80;
    desire_speed_y=0;
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    if(digitalRead(35)==HIGH&&digitalRead(33)==HIGH&&digitalRead(31)==HIGH)
    {     analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0), analogWrite(AIN1, 0);      } //直走
    if(digitalRead(35)==LOW&&digitalRead(33)==LOW&digitalRead(31)==HIGH)
    {      analogWrite(BIN1,val_output1+30),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0-10), analogWrite(AIN1, 0);     } //左转
    if(digitalRead(35)==LOW&&digitalRead(33)==LOW&&digitalRead(31)==LOW)
    {     analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0), analogWrite(AIN1, 0);      } //直走
    if(digitalRead(35)==HIGH&&digitalRead(33)==LOW&&digitalRead(31)==LOW)
    {       analogWrite(BIN1,val_output1-10),analogWrite(BIN2, 0);
           analogWrite(AIN2, val_output0+30), analogWrite(AIN1, 0);    } //右转
     
    if(sx>x)
    {   myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        analogWrite(BIN2,140),analogWrite(BIN1, 0);
        analogWrite(AIN1,140), analogWrite(AIN2, 0); 
        x0=0,x1=0,x2=0,theta=0;
        flag++; 
        delay(500);
}
}
void track_X_(int x) //X轴负向循迹
{   if(sx>x)
    desire_speed_x=80;
    desire_speed_y=0;
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    if(digitalRead(35)==HIGH&&digitalRead(33)==HIGH&&digitalRead(31)==HIGH)
    {     analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0), analogWrite(AIN2, 0);      } //直走
    if(digitalRead(35)==LOW&&digitalRead(33)==LOW&digitalRead(31)==HIGH)
    {      analogWrite(BIN2,val_output1+30),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0-10), analogWrite(AIN2, 0);     } //左转
    if(digitalRead(35)==LOW&&digitalRead(33)==LOW&&digitalRead(31)==LOW)
    {     analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0), analogWrite(AIN2, 0);      } //直走
    if(digitalRead(35)==HIGH&&digitalRead(33)==LOW&&digitalRead(31)==LOW)
    {       analogWrite(BIN2,val_output1-10),analogWrite(BIN1, 0);
           analogWrite(AIN1, val_output0+30), analogWrite(AIN2, 0);    } //右转
    if(sx<x)
    {   myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        analogWrite(BIN1,140),analogWrite(BIN2, 0);
        analogWrite(AIN2,140), analogWrite(AIN1, 0); 
        x0=0,x1=0,x2=0,theta=0;
        flag++; 
        delay(500);
}
}
void track_Y(int x)//y轴正向循迹
{   if(sx<x)
    desire_speed_x=60;
    desire_speed_y=0;
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    char num1,num2,num3,num4,num5;
    num1=digitalRead(37)*2;
    num2=digitalRead(35);
    num4=digitalRead(31);
    num5=digitalRead(23)*2;
    if( (num1+num2)==(num4+num5)      )
    {     analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0), analogWrite(AIN1, 0);      } //直走
         
    if( (num1+num2)-(num4+num5)==1       )
    {      analogWrite(BIN1,val_output1-20),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0+20), analogWrite(AIN1, 0);     } //左转
     
    if( (num1+num2)-(num4+num5)>=2       )
    {      analogWrite(BIN1,val_output1-50),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0+80), analogWrite(AIN1, 0);     } //左转
    if( (num4+num5)-(num1+num2)==1       )
    {      analogWrite(BIN1,val_output1+20),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0-20), analogWrite(AIN1, 0);     } //左转
    if( (num4+num5)-(num1+num2)>=2       )
    {      analogWrite(BIN1,val_output1+80),analogWrite(BIN2, 0);
          analogWrite(AIN2, val_output0-50), analogWrite(AIN1, 0);     } //左转//右转

     
    if(sx>x)
    {   myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        analogWrite(BIN2,140),analogWrite(BIN1, 0);
        analogWrite(AIN1,140), analogWrite(AIN2, 0); 
        x0=0,x1=0,x2=0,theta=0;
        flag++; 
        delay(500);
}
}

void track_Y_(int x) //X轴负向循迹
{   if(sx>x)
    desire_speed_x=60;
    desire_speed_y=0;
    myPID0.SetMode(AUTOMATIC);
    myPID1.SetMode(AUTOMATIC);
    myPID2.SetMode(AUTOMATIC);
    char num1,num2,num3,num4,num5;
    num1=digitalRead(37)*2;
    num2=digitalRead(35);
    num4=digitalRead(31);
    num5=digitalRead(23)*2;
    if( (num1+num2)==(num4+num5)      )
    {     analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0), analogWrite(AIN2, 0);      } //直走
         
    if( (num1+num2)-(num4+num5)==1       )
    {      analogWrite(BIN2,val_output1-20),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0+20), analogWrite(AIN2, 0);     } //左转
     
    if( (num1+num2)-(num4+num5)>=2       )
    {      analogWrite(BIN2,val_output1-30),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0+50), analogWrite(AIN2, 0);     } //左转
    if( (num4+num5)-(num1+num2)==1       )
    {      analogWrite(BIN2,val_output1+20),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0-20), analogWrite(AIN2, 0);     } //左转
    if( (num4+num5)-(num1+num2)>=2       )
    {      analogWrite(BIN2,val_output1+30),analogWrite(BIN1, 0);
          analogWrite(AIN1, val_output0-50), analogWrite(AIN2, 0);     } //左转//右转
    if(sx<x)
    {   myPID0.SetMode(MANUAL);
        myPID1.SetMode(MANUAL);
        myPID2.SetMode(MANUAL);
        analogWrite(BIN1,140),analogWrite(BIN2, 0);
        analogWrite(AIN2,140), analogWrite(AIN1, 0); 
        x0=0,x1=0,x2=0,theta=0;
        flag++; 
        delay(500);
}
}


void roll(int x)
{
myPID0.SetMode(AUTOMATIC);
myPID1.SetMode(AUTOMATIC);
myPID2.SetMode(AUTOMATIC);
myPID2.SetTunings(0.5,8.0,0);
   w_output=1;
   desire_speed_x=0;
   desire_speed_y=0;
   if(theta>x)
   {  analogWrite(AIN2, val_output0), analogWrite(AIN1, 0); 
      analogWrite(BIN2,val_output1),analogWrite(BIN1, 0);
      analogWrite(CIN2,val_output2),analogWrite(CIN1, 0);}
   if(theta<x)
   { 
     
      myPID0.SetMode(MANUAL);
      myPID1.SetMode(MANUAL);
      myPID2.SetMode(MANUAL);
      w_output=0;
       analogWrite(AIN1,200), analogWrite(AIN2, 0); 
       analogWrite(BIN1,200),analogWrite(BIN2, 0);
       analogWrite(CIN1,200),analogWrite(CIN2, 0);
       flag++;
       x0=0,x1=0,x2=0,theta=0;
       delay(500);
        }
}

void roll_(int x)
{
myPID0.SetMode(AUTOMATIC);
myPID1.SetMode(AUTOMATIC);
myPID2.SetMode(AUTOMATIC);
myPID2.SetTunings(0.5,8.0,0);
   w_output=1;
   desire_speed_x=0;
   desire_speed_y=0;
   if(theta<x)
   {  analogWrite(AIN1, val_output0), analogWrite(AIN2, 0); 
      analogWrite(BIN1,val_output1),analogWrite(BIN2, 0);
      analogWrite(CIN1,val_output2),analogWrite(CIN2, 0);}
   if(theta>x)
   { 
     
      myPID0.SetMode(MANUAL);
      myPID1.SetMode(MANUAL);
      myPID2.SetMode(MANUAL);
      w_output=0;
       analogWrite(AIN2,200), analogWrite(AIN1, 0); 
       analogWrite(BIN2,200),analogWrite(BIN1, 0);
       analogWrite(CIN2,200),analogWrite(CIN1, 0);
       flag++;
       x0=0,x1=0,x2=0,theta=0;
       delay(500);
        }
}
void adjust()
 {       
  char num1,num2,num3,num4,num5;
         num1=digitalRead(37);
         num2=digitalRead(35);
         num3=digitalRead(33);
         num4=digitalRead(31);
         num5=digitalRead(23);
         if(num3!=0)
         { if(num2==0&&num1!=0&&num5!=0&&num4!=0)adjust_Y_();else if(num2!=0&&num1!=0&&num4!=0&&num5!=0) turn_adjust();
           if(num4==0&&num5!=0&&num1!=0&&num2!=0) adjust_Y();else if(num2!=0&&num1!=0&&num4!=0&&num5!=0) turn_adjust();
           if(num1==0&&num2!=0&&num4!=0&&num5!=0) adjust_Y_();else if(num2!=0&&num1!=0&&num4!=0&&num5!=0) turn_adjust();
           if(num5==0&&num1!=0&&num2!=0&&num4!=0) adjust_Y();else if(num2!=0&&num1!=0&&num4!=0&&num5!=0) turn_adjust();
        
                                       
         }                           
         if(num3==0) {  
                  
                        if(x2>0)
                     {                myPID0.SetMode(MANUAL);
                                      myPID1.SetMode(MANUAL);
                                      myPID2.SetMode(MANUAL);
                                
                              
         analogWrite(CIN2,100), analogWrite(CIN1, 0);
         analogWrite(AIN1,140), analogWrite(AIN2, 0); 
         analogWrite(BIN1,140), analogWrite(BIN2, 0);    
     
}
                     if(x2<0)
                     {                
                                       myPID0.SetMode(MANUAL);
                                       myPID1.SetMode(MANUAL);
                                       myPID2.SetMode(MANUAL);
       analogWrite(CIN1,140), analogWrite(CIN2, 0);
       analogWrite(AIN2,140), analogWrite(AIN1, 0); 
       analogWrite(BIN2,140), analogWrite(BIN1, 0); 
                    }
                    if(x2=0)
                    {           analogWrite(CIN1,0), analogWrite(CIN2, 0);
                             analogWrite(AIN2,0), analogWrite(AIN1, 0); 
                           analogWrite(BIN2,0), analogWrite(BIN1, 0); }

                 x0=0,x1=0,x2=0; 
                 flag++;         
                     

         }
 }

                  
                  
 void wheelSpeed0()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction0)
    {
      Direction0 = false; //Reverse
    }
    else if(val == HIGH && !Direction0)
    {
      Direction0 = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;
 
  if(!Direction0)  duration0++;
  else  duration0--;

}
void wheelSpeed1()
{
  int Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder1pinB);
    if(val == LOW && Direction1)
    {
      Direction1 = false; //Reverse
    }
    else if(val == HIGH && !Direction1)
    {
      Direction1 = true;  //Forward
    }
  }
  encoder1PinALast = Lstate;
 
  if(!Direction1)  duration1++;
  else  duration1--;

}
void wheelSpeed2()
{
  int Lstate = digitalRead(encoder2pinA);
  if((encoder2PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder2pinB);
    if(val == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder2PinALast = Lstate;
 
  if(!Direction2)  duration2++;
  else  duration2--;

}
float get_wheel_speed1(float X,float Y,float W)  //LEFT
{ 
  return -X*0.886 -Y  * 0.5 + W * L;
}
 
float get_wheel_speed2(float X, float Y,float W) //right
{

   return X *0.886 -Y* 0.5 + W * L;
}
 
float get_wheel_speed3(float X, float Y,float W) //middle
{
  return  Y + W * L;
}
void PID_init()
{
   myPID0.SetMode(AUTOMATIC);//设置PID为自动模式
   myPID0.SetSampleTime(50);//设置PID采样频率为100ms
   myPID1.SetMode(AUTOMATIC);
   myPID1.SetSampleTime(50);
   myPID2.SetMode(AUTOMATIC);
   myPID2.SetSampleTime(50);
   PID_w.SetMode(MANUAL);
   PID_w.SetSampleTime(50);
   PID_theta.SetMode(MANUAL);
   PID_theta.SetSampleTime(10);
}
void oled_init()
{   
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
    delay(500);
}
void oled_show(String data) {
  display.clearDisplay();
  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println(data);
  display.display();      // Show initial text
  delay(100);
}



/*--------------------------------------*
                颜色扫描函数
  --------------------------------------*/
  void color_Detect() {
     
  if(Serial2.available())
  { char a;
    //color_data='a';
    a=Serial2.read();
    /*oled_show(a);
      if(a==49)
    color_data +="1";
       if(a==50)
    color_data +="2";
     if(a==51)
    color_data +="3"; 
    //oled_show(b);*/
     oled_show(a);
    //color_data+=a;
   Serial.println(a);
   // oled_show(a);
  }
  //a='s';
  //oled_show(color_data);
 // Serial.println(a);
 
  
}



     
