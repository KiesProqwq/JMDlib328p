#include <JMDlib.h>

uint16_t NUM=0;
int LX=0,RY=0;

void setup() {
  // put your setup code here, to run once:
  //setRFPassWordPin(I1,1234567);
}


void loop() {
  NUM=getRFModuleRemoteButtonPin(I1);
  LX=getRFModuleRemoteRockerPin(I1,ROCKER_LEFT,ROCKER_X);
  RY=getRFModuleRemoteRockerPin(I1,ROCKER_RIGHT,ROCKER_Y);
  
  if(LX<800) LX=(int)(LX*2.55);else LX=0;
  if(RY<800) RY=(int)(RY*2.55);else RY=0;
  
 if(NUM==9){
  setMotorPin(M3,230);//控制M1电机端口兼容带编码器端口  
  }
  else if(NUM==10){
    setMotorPin(M3,-120);//控制M1电机端口兼容带编码器端口 
  }else{
    setMotorPin(M3,0);//控制M1电机端口兼容带编码器端口 
  }      

  if(NUM==0){
  setServoPin(4,130);//接入PWM端口2,输入角度值100°
  delay(500);//等舵机转到100°的时间
  }
  else if(NUM==11){
  setServoPin(4,45);//接入PWM端口2,输入角度值50°
  delay(500);//等舵机转到50°的时间
  }
 
  setMotorPin(M1,-RY+LX);//控制M1电机端口兼容带编码器端口
  setMotorPin(M2,RY+LX);//控制M2电机端口兼容带编码器端口

}

