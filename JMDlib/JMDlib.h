#ifndef JMDLIB_H
#define JMDliB_H

#include <Arduino.h>
#include <Servo.h>
#include <PWMServo.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
/****模拟IIC端口数量****/

typedef enum		
{
    I1=0,
    I2,
    I3,
    I4,
    I5,
    I6,
    I7,
    I8,
    I9,
    I10
}I2C_ENUM;

/****RGB颜色****/

typedef enum		
{
    RED=0,		//红
    GREEN,		//绿
    BLUE,		//蓝
    YELLOW,		//黄
    PURPLE,		//紫
    CYAN,		//青
    WHITE,		//白
	CLOSE		//关（黑）
}RGB_ENUM;

/****温湿传感器命令****/

typedef enum    
{
    HUMIDITY=0,		//湿度
    CELSIUS,		//摄氏度
    FAHRENHEIT,		//华氏度
    SUM				//总和=湿度+摄氏度
}DHT11_ENUM;

/****字节转换****/

union INT_TO_BYTE
{ 
	short int j; 
	char buf[2]; 
};

//随机数
#define _MULTIPLIER   0x015a4e35L 
#define _INCREMENT      1 

/****模拟IIC端口初始化****/
#define SET_SCL_DDR pinMode(_SCL,OUTPUT)		
#define CLS_SCL_DDR pinMode(_SCL,INPUT)
#define SET_SCL     digitalWrite(_SCL,HIGH)
#define CLS_SCL     digitalWrite(_SCL,LOW)
#define SCL_HIGH    digitalRead(_SCL)

#define SET_SDA_DDR pinMode(_SDA,OUTPUT)
#define CLS_SDA_DDR pinMode(_SDA,INPUT)
#define SET_SDA     digitalWrite(_SDA,HIGH)
#define CLS_SDA     digitalWrite(_SDA,LOW)
#define SDA_HIGH    digitalRead(_SDA)

/****传感器地址****/
#define _ULTRASOUND_ADDR_	0X04	//超声波
#define _COMPOI_ADDR_		0X04	//摇杆
#define _COLOR_ADDR_		0x50	//颜色传感器

/****电机端口****/
#define M1 1	
#define M2 2
#define M3 3
#define M4 4

/****编码对应电机端口****/
#define m1 1 		
#define m2 2
#define m3 3
#define m4 4 

/****限制速度****/
#define SPEED_MIN -255			
#define SPEED_MAX 255

/****串口****/
#define uart0 0 		
#define uart1 1
#define uart2 2
#define uart3 3 
	
//----------------------------------------------音调
#define TONE_0  -1		//代表音阶0
//----------------------------------------------
#define TONE_AL1 221	//代表A音阶 1
#define TONE_AL2 248	//          .
#define TONE_AL3 278
#define TONE_AL4 294
#define TONE_AL5 330
#define TONE_AL6 371
#define TONE_AL7 416

#define TONE_BL1 248
#define TONE_BL2 278
#define TONE_BL3 294
#define TONE_BL4 330
#define TONE_BL5 371
#define TONE_BL6 416
#define TONE_BL7 467

#define TONE_CL1 131
#define TONE_CL2 147
#define TONE_CL3 165
#define TONE_CL4 175
#define TONE_CL5 221
#define TONE_CL6 248
#define TONE_CL7 278

#define TONE_DL1 147
#define TONE_DL2 165
#define TONE_DL3 175
#define TONE_DL4 196
#define TONE_DL5 221
#define TONE_DL6 248
#define TONE_DL7 178

#define TONE_EL1 165
#define TONE_EL2 175
#define TONE_EL3 196
#define TONE_EL4 221
#define TONE_EL5 248
#define TONE_EL6 278
#define TONE_EL7 312

#define TONE_FL1 175
#define TONE_FL2 196
#define TONE_FL3 221
#define TONE_FL4 234
#define TONE_FL5 262
#define TONE_FL6 294
#define TONE_FL7 330

#define TONE_GL1 196
#define TONE_GL2 221
#define TONE_GL3 234
#define TONE_GL4 262
#define TONE_GL5 294
#define TONE_GL6 330
#define TONE_GL7 371
//----------------------------------------------
#define TONE_A1 441		//代表A音阶1
#define TONE_A2 495
#define TONE_A3 556
#define TONE_A4 589
#define TONE_A5 661
#define TONE_A6 742
#define TONE_A7 833

#define TONE_B1 495
#define TONE_B2 556
#define TONE_B3 624
#define TONE_B4 661
#define TONE_B5 742
#define TONE_B6 833
#define TONE_B7 935

#define TONE_C1 262
#define TONE_C2 294
#define TONE_C3 330
#define TONE_C4 350
#define TONE_C5 393
#define TONE_C6 441
#define TONE_C7 495

#define TONE_D1 294
#define TONE_D2 330
#define TONE_D3 350
#define TONE_D4 393
#define TONE_D5 441
#define TONE_D6 495
#define TONE_D7 556

#define TONE_E1 330
#define TONE_E2 350
#define TONE_E3 393
#define TONE_E4 441
#define TONE_E5 495
#define TONE_E6 556
#define TONE_E7 624

#define TONE_F1 350
#define TONE_F2 393
#define TONE_F3 441
#define TONE_F4 495
#define TONE_F5 556
#define TONE_F6 624
#define TONE_F7 661

#define TONE_G1 393
#define TONE_G2 441
#define TONE_G3 495
#define TONE_G4 556
#define TONE_G5 624
#define TONE_G6 661
#define TONE_G7 742
//----------------------------------------------
#define TONE_AH1 882		//代表A音阶 .
#define TONE_AH2 990		//			1
#define TONE_AH3 1112
#define TONE_AH4 1178
#define TONE_AH5 1322
#define TONE_AH6 1484
#define TONE_AH7 1665

#define TONE_BH1 990
#define TONE_BH2 1112
#define TONE_BH3 1178
#define TONE_BH4 1322
#define TONE_BH5 1484
#define TONE_BH6 1665
#define TONE_BH7 1869

#define TONE_CH1 525
#define TONE_CH2 589
#define TONE_CH3 661
#define TONE_CH4 700
#define TONE_CH5 786
#define TONE_CH6 882
#define TONE_CH7 990

#define TONE_DH1 589
#define TONE_DH2 661
#define TONE_DH3 700
#define TONE_DH4 786
#define TONE_DH5 882
#define TONE_DH6 990
#define TONE_DH7 1112

#define TONE_EH1 661
#define TONE_EH2 700
#define TONE_EH3 786
#define TONE_EH4 882
#define TONE_EH5 990
#define TONE_EH6 1112
#define TONE_EH7 1284

#define TONE_FH1 700
#define TONE_FH2 786
#define TONE_FH3 882
#define TONE_FH4 935
#define TONE_FH5 1049
#define TONE_FH6 1178
#define TONE_FH7 1322

#define TONE_GH1 786
#define TONE_GH2 882
#define TONE_GH3 990
#define TONE_GH4 1049
#define TONE_GH5 1178
#define TONE_GH6 1322
#define TONE_GH7 1484
//----------------------------------------------
#define DOUBLE		2 
#define WHOLE 		1
#define HALF 		0.5
#define QUARTER 	0.25
#define EIGHTH 		0.125
#define SIXTEENTH 	0.0625
//这部分是用英文对应了拍子，这样后面也比较好看
//duration[]直接使用数字
//----------------------------------------------

/****重力方向****/
#define GRAVITY_X 		0x01	//X轴
#define GRAVITY_Y 		0x02	//Y轴	
#define GRAVITY_Z 		0x03	//Z轴	

/****左/右摇杆****/
#define ROCKER_LEFT 	0		//左
#define ROCKER_RIGHT	1		//右

/****摇杆方向****/
#define ROCKER_X		0		//X轴
#define ROCKER_Y		1		//Y轴
#define BUTTON			2		//按键

/****开关*****/
#define ON 1		 	
#define OFF 0

/****LCD1602_IIC address*****/
#define AT 0x3F		 	
#define T  0x27

/****蓝牙模块*****/
#define RF_NUM 20  		//包长度
#define DOUBLE_CODE 1	//键码
#define SINGLE_VAL  0	//键值

/****函数声明*****/
uint32_t getSystemTime_ms();														//读取系统时间 ms（毫秒）
uint32_t getSystemTime_us();														//读取系统时间 us（微秒）	
void setDelay_ms(uint32_t _time_ms);												//设置延时时间 ms（毫秒）
void setDelay_us(uint32_t _time_us);												//设置延时时间 us（微秒）
long GetRandom(long min, long max);                                                 //随机数
void setUart(uint32_t baud,const char *str);										//设置串口0输出--字符串
void setUart(uint32_t baud,long longNum);											//设置串口0输出--变量/常量	
void setDigitalPin(uint8_t pin,uint8_t HIGH_LOW);									//设置数字IO输出
int getDigitalPin(uint8_t pin);														//读取数字IO输入
int getAnalogPin(uint8_t pin);														//读取模拟IO输入
void setPWMPin(uint8_t pin, uint8_t value);											//设置PWM输出

void setMotorPin(uint8_t motor, int speed);						                   //设置电机输出
void setAllMotorDir( uint8_t direction);                                             //设置全部马达方向是否与原始方向相反
//void setFanPin(uint8_t motor, uint8_t motorDir, int speed);                        //设置风扇输出
long getCodePin(uint8_t code);														//读取编码器输入
void setClearCodePin(uint8_t code);													//设置清除编码器数值
void setCloseCodeInterruptPin(uint8_t code);										//设置关闭编码器

int getKey();																		//读取板载按键输入
int getInfraredPin(uint8_t pin);													//读取人体红外输入					
int getTouchPin(uint8_t pin);														//读取触碰输入
int getTrackingPin(uint8_t pin);													//读取循迹输入
void setRGB(uint8_t color);															//设置板载RGB彩灯输出
void setRGBPin(uint8_t R_pin, uint8_t G_pin, uint8_t B_pin, uint8_t color);			//设置RGB彩灯输出
int getDHT11Pin(uint8_t pin,uint8_t command);										//读取温/湿度输入
int getIRPin(uint8_t pin);															//读取红外接收输入
//SK6812

int getKeyPin(uint8_t pin);															//读取按键输入
int getIRDistancePin(uint8_t pin);													//读取红外测距输入
int getLightSensorPin(uint8_t pin);													//读取光电输入
int getPhotosensitivePin(uint8_t pin);                                              //读取光敏输入
int getSound();																		//读取板载声音输入
int getSoundPin(uint8_t pin);														//读取声音输入
int getFlamePin(uint8_t pin);														//读取火焰输入
int getPotentiometerPin(uint8_t pin);												//读取电位器输入
int getSliderPin(uint8_t pin);														//读取滑杆电阻输入

void setServoPin(uint8_t pin,uint8_t angle);										//设置舵机输出角度
void setMusic();																	//设置音乐输出（歌曲：葫芦娃）
void setMusicOUT(int _tune[],float _duration[]);									//设置自定义音乐输出
void setMusicOUT(int _tune, float _duration);
void setBeep(uint8_t turn);															//设置板载蜂鸣器输出
void setBeepPin(uint8_t pin ,uint8_t turn);											//设置蜂鸣器输出

uint16_t getUltrasoundPin(uint8_t pin);												//读取超声波传感器输入
void setColorSensorPin(uint8_t pin,uint8_t cmd,int parameter);						//设置颜色传感器输出
uint32_t getColorSensorPin(uint8_t pin,uint8_t cmd);								//读取颜色传感器输入
void setAHRSPin(uint8_t pin,uint8_t command);										//设置姿态初始化输出
short int getAHRSRecBuf(uint8_t pin,uint8_t cnt);
short int getAHRSPin(uint8_t pin,uint8_t command1,uint8_t command2);				//读取姿态角度输入			
int getJoystickCmdPin(uint8_t pin, uint8_t command);								//读取摇杆值输入

void setRFPassWordPin(uint8_t pin,uint32_t password);								//设置2.4G遥控通讯密码输出
uint16_t getRFModuleBytePin(uint8_t pin);											//读取2.4G遥控通讯数据输入
uint8_t setRFModuleBytePin(uint8_t pin,uint8_t _data);								//设置2.4G遥控通讯数据输出
uint16_t getRFModuleRemoteButtonPin(uint8_t pin);									//读取2.4G遥控手柄键值输入
uint32_t getRFModuleRemoteCodePin(uint8_t pin);										//读取2.4G遥控手柄键码输入
short int getRFModuleRemoteGravityPin(uint8_t pin,uint8_t axis);					//读取2.4G遥控手柄重力值输入
short int getRFModuleRemoteRockerPin(uint8_t pin,uint8_t direction,uint8_t axis);	//读取2.4G遥控手柄摇杆值输入

//uint8_t getIRPin(uint8_t _uart);													//读取红外接收输入(保留)
uint16_t getBluetoothPin(uint8_t _uart,uint8_t mode);								//读取蓝牙接收输入

void setLCDWriteStr(uint8_t addr, uint8_t x,uint8_t y,char *str);					//设置输出字符到LCD1602
void setLCDWriteStr(uint8_t addr, uint8_t x,uint8_t y,String str);
void setLCDWriteInt(uint8_t addr, uint8_t x,uint8_t y,long _intNum);				//设置输出4位有效数字 整数变量到LCD1602
void setLCDWriteFloat(uint8_t addr, uint8_t x,uint8_t y,float _floatNum);			//设置输出6位有效数字 浮点(带2位小数)变量到LCD1602
void setLCDClear(uint8_t addr);														//设置清屏

short int getRFModuleRemoteRockerPin_Balance(uint8_t pin,uint8_t flag);				//平衡车遥控专用，2个字节
//EEPROM
int GetData(int address);                                                          //读EEPROM
void SetData(int addr,int val);                                                     //写EEPROM

//AI摄像头模块
uint8_t GetAICamData(uint8_t _uart,uint8_t cmd);                                    //获取AI摄像头数据
void SetAICamData(uint8_t _uart,uint8_t cmd0,uint8_t cmd1);                         //设置AI摄像头状态
void SetWaitForAICamData(uint8_t _uart,uint8_t cmd0,uint8_t cmd1);                  //等待AI视觉模块模式
void SetAICamLED(uint8_t _uart,uint8_t cmd);                                        //设置补光灯 开/关
#endif
//-------------------------------------------------------------------------------------------------------------------