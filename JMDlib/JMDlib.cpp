                                   /************BE5141v15---Mega2560************/

#include <JMDlib.h>    
//System----------------------------------------------------------------------------
/****读取系统时间****/
uint32_t getSystemTime_ms()	//毫秒		
{
	return millis();		
}
uint32_t getSystemTime_us()	//微秒			
{
	return micros();		
}

/****延时时间****/
void setDelay_ms(uint32_t _time_ms)	//毫秒
{
	delay(_time_ms);
}

void setDelay_us(uint32_t _time_us)	//微秒
{
	delayMicroseconds(_time_us);
}

/*****随机数*****/
uint8_t RandomFlag=0;
long GetRandom(long min, long max)
{
	if(RandomFlag==0)
	{
		randomSeed(analogRead(0));
		RandomFlag=1;
	}
	return random(min,max);	
}

/****串口0输出（函数重载）****/
uint8_t strFlag = 0;
uint8_t numFlag = 0;
void setUart(uint32_t baud, const char *str)			//字符-字符串
{
	if(strFlag==0)
	{
		strFlag = 1;
		Serial.begin(baud);
	}
	Serial.println(str);	
}

void setUart(uint32_t baud, long longNum)			//变量-常量
{
	if(numFlag==0)
	{
		numFlag = 1;
		Serial.begin(baud);
	}
	Serial.println(longNum);	
}

/****数字端口****/
void setDigitalPin(uint8_t pin, uint8_t HIGH_LOW)	//DIGITAL输出
{
	pinMode(pin,OUTPUT);
	digitalWrite(pin,HIGH_LOW);
}
    
int getDigitalPin(uint8_t pin)						//DIGITAL输入				
{
	pinMode(pin,INPUT);
	return digitalRead(pin);
}

/****模拟端口****/
int getAnalogPin(uint8_t pin)						//ANALOG输入				
{
	int value;
    value = analogRead(pin);
    return value;
}

/****PWM输出****/
/*
 *	pin：	4-8/44-46/13
 *	value:	0-255
 */
void setPWMPin(uint8_t pin, uint8_t value)		
{   
	if(value>255) value = 255;		
	else if(value<0) value = 0;
	analogWrite(pin,value);		//0-255			
}

/****模拟IIC****/
const uint8_t _iic_pin[10][2] = {{15,14},{17,16},{0,1},{2,38},{3,39},
								 {18,40},{19,41},{0,0},{0,0}, {0,0}};//IIC端口
								
uint8_t _SCL;
uint8_t _SDA;
uint8_t I2C_SPEED;
uint8_t I2C_PIC;

void set_I2C_Init(uint8_t pin)	//初始化			
{
	if(pin>=10) return;
	if(pin<10)
	{
		_SDA = _iic_pin[pin][1];
		_SCL = _iic_pin[pin][0];
	}
}

void Soft_I2C_Delay(void)		//延时10KHz				
{	
	int ri;
	for(ri=0; ri<50; ri++);
	/*/
	int ri;
	for( ri=0; ri<300; ri++ );
	if(I2C_SPEED==50)
		for( ri=0; ri<700; ri++ );
	else if(I2C_PIC==1)
		for( ri=0; ri<200; ri++ );
	//*/
}

void Soft_I2C_Start(void)		//启动IIC				
{
	SET_SDA;
	SET_SDA_DDR;
	SET_SCL_DDR;
	SET_SCL;
	Soft_I2C_Delay();
	SET_SCL;
	Soft_I2C_Delay();
	CLS_SDA;
	Soft_I2C_Delay();
	CLS_SCL;
	Soft_I2C_Delay();
}

void Soft_I2C_Stop(void)		//停止IIC					
{
	SET_SDA_DDR;
	SET_SCL_DDR;
	CLS_SDA;
	Soft_I2C_Delay();
	SET_SCL;
	Soft_I2C_Delay();
	SET_SDA;
	Soft_I2C_Delay();
}

uint8_t Soft_I2C_Write(uint8_t wb)//写数据
{
	uint8_t ack = 0;
	int i;
	SET_SDA_DDR;
	SET_SCL_DDR;
	for(i=0; i<8; i++)
	{
		if( wb&0x80 ) SET_SDA;
		else CLS_SDA;
		wb = wb<<1;
		Soft_I2C_Delay();
		SET_SCL;
		Soft_I2C_Delay();
		CLS_SCL;
	}
	CLS_SDA_DDR;                              
	SET_SDA;                                  
	Soft_I2C_Delay();
	SET_SCL;
	Soft_I2C_Delay();
	if(SDA_HIGH) ack = 0;
	else ack = 1;
	CLS_SCL;
	Soft_I2C_Delay();
	return ack;
}

uint8_t Soft_I2C_Read(uint8_t rb)//读数据
{
	uint8_t i,mb;
	mb = 0;
	CLS_SDA_DDR;
	SET_SDA;
	SET_SCL_DDR;
	for(i=0; i<8; i++)
	{
		Soft_I2C_Delay();
		CLS_SCL;
		Soft_I2C_Delay();
		SET_SCL;
		mb = mb<<1;
		if( SDA_HIGH )
			mb++;
	}
	Soft_I2C_Delay();
	CLS_SCL;
	SET_SDA_DDR;
	if( rb )
		SET_SDA;
	else
		CLS_SDA;
	Soft_I2C_Delay();
	SET_SCL;
	Soft_I2C_Delay();
	CLS_SCL;
	Soft_I2C_Delay();
	return mb;
}
//----------------------------------------------------------------------------

//Sensor----------------------------------------------------------------------------
//Motor
/****电机输出v11****/
uint8_t MotorFlag1 = 0;//M1初始化标志位
uint8_t MotorFlag2 = 0;//M2初始化标志位
uint8_t MotorFlag3 = 0;//M3初始化标志位
uint8_t MotorFlag4 = 0;//M4初始化标志位

//motor 1
uint8_t PWMA = 10; //speed control
uint8_t AIN1 = 22; //Direction
uint8_t AIN2 = 23; //Direction

//motor 2
uint8_t PWMB = 11; //speed control
uint8_t BIN1 = 24; //Direction
uint8_t BIN2 = 25; //Direction

//motor 3
uint8_t PWMC = 12; //speed control
uint8_t CIN1 = 28; //Direction
uint8_t CIN2 = 29; //Direction

//motor 4
uint8_t PWMD = 13; //speed control
uint8_t DIN1 = 26; //Direction
uint8_t DIN2 = 27; //Direction 


/****电机输出v15****/
uint8_t PWMA1 = 10;
uint8_t PWMA2 = 8;

uint8_t PWMB1 = 11;
uint8_t PWMB2 = 44;

uint8_t PWMC1 = 12;
uint8_t PWMC2 = 45;

uint8_t PWMD1 = 7;
uint8_t PWMD2 = 46;


/*
 * motor：M1-M4
 * speed：0 - 255
 * motorDir: 0按照原始方向，1表示全部马达与原始方向相反
 */
 
uint8_t motorinit = 0;
uint8_t motorinitflag = 0;
uint8_t motorDir = 0;
void setMotorPin(uint8_t motor, int speed)
{	
	if(speed > SPEED_MAX) speed = SPEED_MAX;
	else if(speed < SPEED_MIN) speed = SPEED_MIN;	
	
	if (motorinit == 0)
	{
		motorinitflag = getDigitalPin(24);
		motorinit = 1;
	}
	
	if( motorDir == 1) speed = -speed;
	/*BE5141V11motor*/
	if (motorinitflag == 0)
	{
		switch (motor)
		{
		case M1:
			if (MotorFlag1 == 0)
			{
				MotorFlag1 = 1;
				pinMode(PWMA, OUTPUT);
				pinMode(AIN1, OUTPUT);
				pinMode(AIN2, OUTPUT);
			}
			break;

		case M2:
			if (MotorFlag2 == 0)
			{
				MotorFlag2 = 1;
				pinMode(PWMB, OUTPUT);
				pinMode(BIN1, OUTPUT);
				pinMode(BIN2, OUTPUT);
			}
			break;

		case M3:
			if (MotorFlag3 == 0)
			{
				MotorFlag3 = 1;
				pinMode(PWMC, OUTPUT);
				pinMode(CIN1, OUTPUT);
				pinMode(CIN2, OUTPUT);
			}
			break;

		case M4:
			if (MotorFlag4 == 0)
			{
				MotorFlag4 = 1;
				pinMode(PWMD, OUTPUT);
				pinMode(DIN1, OUTPUT);
				pinMode(DIN2, OUTPUT);
			}
			break;

		default:break;
	}

		switch (motor)
		{
		case M1:
			if (speed == 0)
			{
				digitalWrite(AIN2, HIGH);
				digitalWrite(AIN1, HIGH);
			}
			else if (speed>0)
			{
				digitalWrite(AIN2, HIGH);
				digitalWrite(AIN1, LOW);
				analogWrite(PWMA, speed);
			}
			else if (speed<0)
			{
				digitalWrite(AIN1, HIGH);
				digitalWrite(AIN2, LOW);
				analogWrite(PWMA, -speed);
			}
			break;

		case M2:
			if (speed == 0)
			{
				digitalWrite(BIN1, HIGH);
				digitalWrite(BIN2, HIGH);
			}
			else if (speed>0)
			{
				digitalWrite(BIN1, HIGH);
				digitalWrite(BIN2, LOW);
				analogWrite(PWMB, speed);
			}
			else if (speed<0)
			{
				digitalWrite(BIN2, HIGH);
				digitalWrite(BIN1, LOW);
				analogWrite(PWMB, -speed);
			}
			break;

		case M3:
			if (speed == 0)
			{
				digitalWrite(CIN2, HIGH);
				digitalWrite(CIN1, HIGH);
			}
			else if (speed>0)
			{
				digitalWrite(CIN2, HIGH);
				digitalWrite(CIN1, LOW);
				analogWrite(PWMC, speed);
			}
			else if (speed<0)
			{
				digitalWrite(CIN1, HIGH);
				digitalWrite(CIN2, LOW);
				analogWrite(PWMC, -speed);
			}
			break;

		case M4:
			if (speed == 0)
			{
				digitalWrite(DIN1, HIGH);
				digitalWrite(DIN2, HIGH);
			}
			else if (speed>0)
			{
				digitalWrite(DIN1, HIGH);
				digitalWrite(DIN2, LOW);
				analogWrite(PWMD, speed);
			}
			else if (speed<0)
			{
				digitalWrite(DIN2, HIGH);
				digitalWrite(DIN1, LOW);
				analogWrite(PWMD, -speed);
			}
			break;
			
		default:break;
			}
	}
	
	
	/*BE5141V15motor*/
	if (motorinitflag == 1)
	{
		switch (motor)
		{
		case M1:
			if (MotorFlag1 == 0)
			{
				MotorFlag1 = 1;
				pinMode(PWMA1, OUTPUT);
				pinMode(PWMA2, OUTPUT);							
			}
			break;

		case M2:
			if (MotorFlag2 == 0)
			{
				MotorFlag2 = 1;
				pinMode(PWMB1, OUTPUT);
				pinMode(PWMB2, OUTPUT);							
			}
			break;

		case M3:
			if (MotorFlag3 == 0)
			{
				MotorFlag3 = 1;
				pinMode(PWMC1, OUTPUT);
				pinMode(PWMC2, OUTPUT);						
			}
			break;

		case M4:
			if (MotorFlag4 == 0)
			{
				MotorFlag4 = 1;
				pinMode(PWMD1, OUTPUT);
				pinMode(PWMD2, OUTPUT);						
			}
			break;

		default:break;
		}
		
		switch (motor)
		{
			case M1:
				if (speed == 0)
				{
					analogWrite(PWMA1, 255);
					analogWrite(PWMA2, 255);
				}
				else if (speed>0)
				{
					analogWrite(PWMA1, speed);
					analogWrite(PWMA2, 0); 
				}
				else if (speed<0)
				{
					analogWrite(PWMA2, -speed);
					analogWrite(PWMA1, 0); 
				}
				break;

			case M2:
				if (speed == 0)
				{
					analogWrite(PWMB1, 255);
					analogWrite(PWMB2, 255);
				}
				else if (speed>0)
				{
					analogWrite(PWMB1, speed);
					analogWrite(PWMB2, 0);
				}
				else if (speed<0)
				{
					analogWrite(PWMB2, -speed);
					analogWrite(PWMB1, 0);
				}
				break;

			case M3:
				if (speed == 0)
				{
					analogWrite(PWMC1, 255);
					analogWrite(PWMC2, 255);
				}
				else if (speed>0)
				{
					analogWrite(PWMC1, speed);
					analogWrite(PWMC2, 0);
				}
				else if (speed<0)
				{
					analogWrite(PWMC2, -speed);
					analogWrite(PWMC1, 0);
				}
				break;

			case M4:
				if (speed == 0)
				{
					analogWrite(PWMD1, 255);
					analogWrite(PWMD2, 255);
				}
				else if (speed>0)
				{
					analogWrite(PWMD1, speed);
					analogWrite(PWMD2, 0);
				}
				else if (speed<0)
				{
					analogWrite(PWMD2, -speed);
					analogWrite(PWMD1, 0);
				}
				break;

			default:break;
		}	
	}	
	
}

void setAllMotorDir( uint8_t direction)
{
	motorDir = direction;
}


/*
 * motor：M1-M4
 * speed：-255 - 255
 
void setFanPin(uint8_t motor, uint8_t motorDir, int speed)
{
	setMotorPin(motor, motorDir, speed);
}
*/


/****编码****/
/*
 *attachInterrupt(interrupt, function, mode);//配置外部中断函数
 *detachInterrupt(interrupt);				 //关闭外部中断
 *interrupt	中断通道编号0-5,Atmega2560使用0 1 5 4 (对应引脚2 3 18 19),中断通道编号2 3(对应引脚21 20--SCL/SDA)
 *function	中断服务函数(void 任意函数名(void){})
 *mode		为中断触发模式
 **LOW		低电平触发
 **CHANGE	电平变化，高电平变低电平、低电平变高电平
 **RISING	上升沿触发
 **FALLING	下降沿触发
 */
uint8_t CodeFlag1  = 0; //编码1初始化标志位	
uint8_t CodeFlag2  = 0;	//编码2初始化标志位
uint8_t CodeFlag3  = 0; //编码3初始化标志位
uint8_t CodeFlag4  = 0; //编码4初始化标志位

long  MotorCodeCnt1= 0;	//M1编码计数
long  MotorCodeCnt2= 0;	//M2编码计数
long  MotorCodeCnt3= 0;	//M3编码计数
long  MotorCodeCnt4= 0;	//M4编码计数

/****ISR中断服务函数****/
void getCode1Interrupt()	//中断函数ISP: Interrupt 0--Code1
{
	uint8_t m_ISR,m_Dir;	//读取两个的方向
	m_ISR = getDigitalPin(2);	//中断IO
	m_Dir = getDigitalPin(38);	//数字输入IO
	if(m_ISR^m_Dir)			//正交脉冲
		MotorCodeCnt1--;	//正转--
	else
		MotorCodeCnt1++;	//反转++ 
} 
void getCode2Interrupt()	//中断函数ISP: Interrupt 1--Code2
{
	uint8_t m_ISR,m_Dir;	//读取两个的方向
	m_ISR = getDigitalPin(3);	//中断IO
	m_Dir = getDigitalPin(39);	//数字输入IO
	if(m_ISR^m_Dir)			//正交脉冲
		MotorCodeCnt2--;	//正转--
	else
		MotorCodeCnt2++;	//反转++ 
}
void getCode3Interrupt()	//中断函数ISP: Interrupt 5--Code3
{
	uint8_t m_ISR,m_Dir;	//读取两个的方向
	m_ISR = getDigitalPin(18);	//中断IO
	m_Dir = getDigitalPin(40);	//数字输入IO
	if(m_ISR^m_Dir)			//正交脉冲
		MotorCodeCnt3--;	//正转--
	else
		MotorCodeCnt3++;	//反转++ 
}
void getCode4Interrupt()	//中断函数ISP: Interrupt 4--Code4
{
	uint8_t m_ISR,m_Dir;	//读取两个的方向
	m_ISR = getDigitalPin(19);	//中断IO
	m_Dir = getDigitalPin(41);	//数字输入IO
	if(m_ISR^m_Dir)			//正交脉冲
		MotorCodeCnt4--;	//正转--
	else
		MotorCodeCnt4++;	//反转++ 
}  

/****编码初始化****/
void setCodeInit(uint8_t interrupt)
{	
	switch(interrupt)
	{
		case m1: attachInterrupt(0, getCode1Interrupt, CHANGE); break;//当Init.0电平改变时,触发中断函数getCode1Interrupt
		case m2: attachInterrupt(1, getCode2Interrupt, CHANGE); break;//当Init.1电平改变时,触发中断函数getCode2Interrupt
		case m3: attachInterrupt(5, getCode3Interrupt, CHANGE); break;//当Init.5电平改变时,触发中断函数getCode3Interrupt
		case m4: attachInterrupt(4, getCode4Interrupt, CHANGE); break;//当Init.4电平改变时,触发中断函数getCode4Interrupt
		default:break;		
	}
}
/****读取编码数值****/
/*
 * code: m1-m4
 */
long getCodePin(uint8_t code)
{	
	if(CodeFlag1==0 && code==m1)
	{
		CodeFlag1 = 1;
		setCodeInit(code);		//初始化
	}
	else if(CodeFlag2==0 && code==m2)
	{
		CodeFlag2 = 1;
		setCodeInit(code);		//初始化		
	}
	else if(CodeFlag3==0 && code==m3)
	{
		CodeFlag3 = 1;
		setCodeInit(code);		//初始化		
	}
	else if(CodeFlag4==0 && code==m4)
	{
		CodeFlag4 = 1;
		setCodeInit(code);		//初始化		
	}
	switch(code)
	{
		case m1: return	MotorCodeCnt1; break; //返回编码1数值
		case m2: return	MotorCodeCnt2; break; //返回编码2数值
		case m3: return	MotorCodeCnt3; break; //返回编码3数值
		case m4: return	MotorCodeCnt4; break; //返回编码4数值
		default:break;
	}
}

/****清零编码数值****/
/*
 * code: m1-m4
 */
void setClearCodePin(uint8_t code)	
{	
	switch(code)
	{
		case m1: MotorCodeCnt1 = 0; break; //清零编码1数值
		case m2: MotorCodeCnt2 = 0; break; //清零编码2数值
		case m3: MotorCodeCnt3 = 0; break; //清零编码3数值
		case m4: MotorCodeCnt4 = 0; break; //清零编码4数值
		default:break;		
	}
}

/****关闭编码中断****/
/*
 * code: m1-m4
 */
void setCloseCodeInterruptPin(uint8_t code)
{
	switch(code)
	{
		case m1: detachInterrupt(0); MotorCodeCnt1 = 0; CodeFlag1 = 0; break; //禁用外部中断0,清零编码1数值	
		case m2: detachInterrupt(1); MotorCodeCnt2 = 0; CodeFlag2 = 0; break; //禁用外部中断1,清零编码2数值		
		case m3: detachInterrupt(5); MotorCodeCnt3 = 0; CodeFlag3 = 0; break; //禁用外部中断5,清零编码3数值	
		case m4: detachInterrupt(4); MotorCodeCnt4 = 0; CodeFlag4 = 0; break; //禁用外部中断4,清零编码4数值		
		default:break;
	}		
}

//数字
uint8_t Key_Flag = 0;
/****板载按键****/
/*
 * pin：DIGITAL端口53
 */
int getKey()
{
	if(Key_Flag==0)
	{
		Key_Flag = 1;		
		pinMode(53,INPUT_PULLUP);//上拉输入
	}
	return digitalRead(53);
}

/****人体红外****/
/*
 * pin：DIGITAL端口
 */
int getInfraredPin(uint8_t pin)
{
    return getDigitalPin(pin); 
}

/****触碰****/
/*
 * pin：DIGITAL端口
 */
int getTouchPin(uint8_t pin)
{
    if(getDigitalPin(pin)==0)
		return 1;
	else return 0;
}

/****Tracking循迹****/
/*
 * pin：DIGITAL端口
 */
int getTrackingPin(uint8_t pin)
{
    return getDigitalPin(pin); 
}

/****RGB全彩灯****/
uint8_t RGBFlag = 0;//内置RGB初始化标志位

/*
 * color: 红绿蓝 黄紫青 白黑
 */
void setRGB(uint8_t color)//内置
{
	uint8_t red; uint8_t blue; uint8_t green;

	if(RGBFlag==0)
	{
		RGBFlag = 1;
        pinMode(47,OUTPUT);
        pinMode(48,OUTPUT);
        pinMode(49,OUTPUT);
    }
	switch(color)
	{
		case 0: red = LOW; 	green = HIGH; blue = HIGH; break;//红
		case 1: red = HIGH; green = LOW;  blue = HIGH; break;//绿
		case 2: red = HIGH; green = HIGH; blue = LOW;  break;//蓝
		
		case 3: red = LOW; 	green = LOW;  blue = HIGH; break;//黄
		case 4: red = LOW; 	green = HIGH; blue = LOW;  break;//紫
		case 5: red = HIGH; green = LOW;  blue = LOW;  break;//青
		
		case 6: red = LOW; 	green = LOW;  blue = LOW;  break;//白
		case 7: red = HIGH; green = HIGH; blue = HIGH; break;//黑		
		default: break;				
	}
    digitalWrite(47,blue);
    digitalWrite(48,green);
    digitalWrite(49,red);
}

uint8_t RGBPinFlag = 0;//外置RGB初始化标志位
/*
 * R_pin/G_pin/B_pin： DIGITAL端口
 * color: 红绿蓝 黄紫青 白黑
 */
void setRGBPin(uint8_t R_pin, uint8_t G_pin, uint8_t B_pin, uint8_t color)//外置
{
	uint8_t red; uint8_t blue; uint8_t green;

	if(RGBPinFlag==0)
	{
		RGBPinFlag = 1;
        pinMode(B_pin,OUTPUT);
        pinMode(G_pin,OUTPUT);
        pinMode(R_pin,OUTPUT);
    }
	switch(color)
	{
		case 0: red = HIGH;green = LOW; blue = LOW; break;//红
		case 1: red = LOW; green = HIGH;blue = LOW; break;//绿
		case 2: red = LOW; green = LOW; blue = HIGH;break;//蓝
		
		case 3: red = HIGH;green = HIGH;blue = LOW; break;//黄
		case 4: red = HIGH;green = LOW; blue = HIGH;break;//紫
		case 5: red = LOW; green = HIGH;blue = HIGH;break;//青
		
		case 6: red = HIGH;green = HIGH;blue = HIGH;break;//白
		case 7: red = LOW; green = LOW; blue = LOW; break;//黑			
		default:break;
	}
    digitalWrite(B_pin,blue);	//蓝
    digitalWrite(G_pin,green);	//绿
    digitalWrite(R_pin,red);	//红
}
//--------------------------------------------SK6812

/****DHT11温湿度****/
/*
 * pin：DIGITAL端口
 * command:    
 **TEMPERATURE  0	//湿度
 **CELSIUS		1	//摄氏度
 **FAHRENHEIT	2	//华氏度
 **SUM			3	//总和=湿度+摄氏度
 */
int getDHT11Pin(uint8_t pin, uint8_t command)
{
    int temp; //温度
	int humi; //湿度
	int sum;  //校对码=湿度+温度
	int chr[40] = {0};//创建数字数组，用来存放40个bit
	unsigned int loopCnt,_loopCnt;//响应时间
	unsigned long time;//收集数据时间

	setDelay_ms(700);       //采样时间
	setDigitalPin(pin,LOW); //输出模式--低电平
	setDelay_ms(20);        //输出低电平20ms（>18ms）
	setDigitalPin(pin,HIGH);//输出模式--高电平
	setDelay_us(40);         //输出高电平40μs
	setDigitalPin(pin,LOW); //输出模式--低电平
	pinMode(pin,INPUT);     //输入模式

	loopCnt=getSystemTime_us();         //低电平响应信号
	_loopCnt=loopCnt;
	while(getDigitalPin(pin) != HIGH)
	{
		if(loopCnt++ == _loopCnt+80)	//80us
			break;
	} 
	loopCnt  = getSystemTime_us();		//高电平响应信号
	_loopCnt = loopCnt;
	while(getDigitalPin(pin) != LOW)
	{
		if(loopCnt++ == _loopCnt+80)	//80us
			break;
	}
	for(int i=0; i<40; i++) //开始读取bit1-40的数值  
	{
		/*
		 *当出现高电平时，记下时间“time”
		 *当出现低电平，记下时间，再减去刚才储存的time
		 *得出的值若大于50μs，则为‘1’，否则为‘0’
		 *并储存到数组里去
		*/
		while(getDigitalPin(pin)==LOW);
		time = getSystemTime_us();
		while(getDigitalPin(pin)==HIGH);
		if (getSystemTime_us() - time >50)
		{
			chr[i] = 1;
		}
		else
		{
			chr[i] = 0;
		}
	}
	
	/*8位的bit转换为十进制*/
		//湿度
	humi=chr[0]*128+chr[1]*64+chr[2]*32+chr[3]*16+chr[4]*8+chr[5]*4+chr[6]*2+chr[7];   
		//温度
	temp=chr[16]*128+chr[17]*64+chr[18]*32+chr[19]*16+chr[20]*8+chr[21]*4+chr[22]*2+chr[23];
		//校对码总和
	sum =chr[32]*128+chr[33]*64+chr[34]*32+chr[35]*16+chr[36]*8+chr[37]*4+chr[38]*2+chr[39];

	switch(command)
	{
		case 0:	return humi;break;//湿度
      
		case 1:	return temp;break;//摄氏
      
		case 2:	temp=(1.8 * temp + 32.0);
				return temp;break;//华氏
              
		case 3:	return sum;break;//总和
      
		default: return 999;break;
	}    
}
//红外接收头-NEC
/*
 * pin： DIGITAL端口
 */
int getIRPin(uint8_t pin)
{
  uint32_t time;
  uint8_t flag;
  int chr[33] = {0};
  int add1,add2;
  int data1,data2;
  static int data;

  if(getDigitalPin(pin) == LOW)   //NEC协议
  {
    while(getDigitalPin(pin) == LOW);//9ms
    time = getSystemTime_us();
    while(getDigitalPin(pin) == HIGH)//4.5ms
    {
        if ((getSystemTime_us() - time > 3000))
            flag=1;//标准码
        else
            flag=0;//重复码 
    }
    
    switch(flag)
    {
        case 0:
                while(getDigitalPin(pin) == LOW); //0.5ms             
                while(getDigitalPin(pin) == HIGH)//>96.6ms
                {
                    break; 
                } 
                return  data;
                break;                    
        case 1:
                for(int i=0; i<33; i++)
                {
                    while(getDigitalPin(pin) == LOW);//0.5ms
                    time = getSystemTime_us();
                    while(getDigitalPin(pin) == HIGH)//1.6ms
                    {
                        if (getSystemTime_us() - time > 2000)//停止位40ms
                            break; 
                    } 
                    if (getSystemTime_us() - time > 1600)
                        chr[i] = 1;
                    else
                        chr[i] = 0;               
                }
                add1 =chr[0] +chr[1]*2 +chr[2]*4 +chr[3]*8 +chr[4]*16 +chr[5]*32 +chr[6]*64 +chr[7]*128 ;
                add2 =chr[8] +chr[9]*2 +chr[10]*4+chr[11]*8+chr[12]*16+chr[13]*32+chr[14]*64+chr[15]*128;
                data1=chr[16]+chr[17]*2+chr[18]*4+chr[19]*8+chr[20]*16+chr[21]*32+chr[22]*64+chr[23]*128;
                data2=chr[24]+chr[25]*2+chr[26]*4+chr[27]*8+chr[28]*16+chr[29]*32+chr[30]*64+chr[31]*128;
                
                data = data1;//选择输出
                return data;
                break;
           default:break;
    }
  }
  else 
    return 999;
}

//模拟
/****按键****/
/*
 * pin：ANALOG端口
 */
int getKeyPin(uint8_t pin)				
{
    uint16_t value;
    value = getAnalogPin(pin);
	
    if(value<100)
	{
        return 1;
    }
    else if(value>100 && value<200)
	{
        return 2;
    }
    else if(value>200 && value<330)
	{
        return 3;
    }
    else if(value>330 && value<400)
	{
        return 4;
    }
    else
	{	
        return 0;
	}
}

/****红外测距****/
/*
 * pin：ANALOG端口
 */
int getIRDistancePin(uint8_t pin)
{
    return getAnalogPin(pin);  
}

/****光电****/
/*
 * pin：ANALOG端口
 */
int getLightSensorPin(uint8_t pin)
{
    return getAnalogPin(pin);  
}

/****光敏****/
/*
 * pin：ANALOG端口
 */
int getPhotosensitivePin(uint8_t pin)
{
    return getAnalogPin(pin);  
}

/****声音****/
/*
 * pin：ANALOG端口
 */
int getSound()			//内置
{
    return getAnalogPin(A0);  
}
int getSoundPin(uint8_t pin)//外置
{
    return getAnalogPin(pin);  
}

/****火焰****/
/*
 * pin：ANALOG端口
 */
int getFlamePin(uint8_t pin)
{
	return getAnalogPin(pin); 	
}

/****电位器****/
/*
 * pin：ANALOG端口
 */
int getPotentiometerPin(uint8_t pin)
{
	return getAnalogPin(pin); 	
}

/****滑杆****/
/*
 * pin：ANALOG端口
 */
int getSliderPin(uint8_t pin)
{
	return getAnalogPin(pin); 	
}




//PWM
/****舵机****/
/*
 *	pin：	4-8/44-46/13
 *	angle:	0-180°
 */
Servo _servo[9];
uint8_t ServoAngleFlag[9] = {0,0,0,0,0,0,0,0,0};				//舵机初始化标志位
int ServoAngleRecord[9] = {360,360,360,360,360,360,360,360,360};	//角度记录
void setServoPin(uint8_t pin, uint8_t angle)		
{
    if(pin>3 && pin<9)					//舵机PWM输出4-8			
	{
		if(ServoAngleFlag[pin-4]==0)	//初始化
		{
			ServoAngleFlag[pin-4] = 1;
			_servo[pin-4].attach(pin); 
		}
		if(angle>180) angle = 180;
		else if(angle<0) angle = 0;
		if(ServoAngleRecord[pin-4]==angle) return ;
		_servo[pin-4].write(angle);		//0-180°
		ServoAngleRecord[pin-4] = angle ;
	}
	else if(pin>43 && pin<47)				//舵机PWM输出44-46	
	{
		if(ServoAngleFlag[pin-39]==0)	//初始化
		{
			ServoAngleFlag[pin-39] = 1;
			_servo[pin-39].attach(pin); 
		}
		if(angle>180) angle = 180;
		else if(angle<0) angle = 0;
		if(ServoAngleRecord[pin-39] == angle) return ;
		_servo[pin-39].write(angle);	//0-180°
		ServoAngleRecord[pin-39] = angle ;			
	}
	else if(pin == 13)				//舵机PWM输出13
	{
		if(ServoAngleFlag[pin-5]==0)	//初始化
		{
			ServoAngleFlag[pin-5] = 1;
			_servo[pin-5].attach(pin); 
		}
		if(angle>180) angle = 180;
		else if(angle<0) angle = 0;
		if(ServoAngleRecord[pin-5] == angle) return ;
		_servo[pin-5].write(angle);	//0-180°
		ServoAngleRecord[pin-5] = angle ;			
	}
	else return;
}






/****音乐****/
int tonePin = 9;	

/*
 *	数字下一根下划线，1/2拍，2根下划线1/4拍，依此类推
 *	规定一拍音符的时间为1s；半拍为0.5s；1/4拍为0.25s；1/8拍为0.125s，依次类推
 *	数字3?带个点，点的意思是去3的拍子的一半，即3?的拍子是1+0.5
 *	规律就是时间上单个音符没有下划线，就是一拍（1），有下划线是半拍（0.5），两个下划线是四分之一拍（0.25），
 *	有“—”=前面音符的拍子+1；频率上就是按照音符是否带点，点在上还是在下到表中查找就可以了。
 */
//葫芦娃音乐
int tune[] =		
{
	TONE_DH1,TONE_D6,TONE_D5,TONE_D6,TONE_0,  TONE_DH1,TONE_D6, TONE_D5, TONE_DH1,TONE_D6, TONE_0, TONE_D6,
	TONE_D6, TONE_D6,TONE_D5,TONE_D6,TONE_0,  TONE_D6, TONE_DH1,TONE_D6, TONE_D5, TONE_DH1,TONE_D6,TONE_0, 
	TONE_D1, TONE_D1,TONE_D3,TONE_D1,TONE_D1, TONE_D3, TONE_0,  TONE_D6, TONE_D6, TONE_D6, TONE_D5,TONE_D6,
	TONE_D5, TONE_D1,TONE_D3,TONE_0, TONE_DH1,TONE_D6, TONE_D6, TONE_D5, TONE_D6, TONE_D5, TONE_D1,TONE_D2,
	TONE_0,  TONE_D7,TONE_D7,TONE_D5,TONE_D3, TONE_D5, TONE_DH1,TONE_0,  TONE_D6, TONE_D6, TONE_D5,TONE_D5,
	TONE_D6, TONE_D6,TONE_0, TONE_D5,TONE_D1, TONE_D3, TONE_0,  TONE_DH1,TONE_0,  TONE_D6, TONE_D6,TONE_D5,
	TONE_D5, TONE_D6,TONE_D6,TONE_0, TONE_D5, TONE_D1, TONE_D2, TONE_0,  TONE_D3, TONE_D3, TONE_D1,TONE_DL6,
	TONE_D1, TONE_D3,TONE_D5,TONE_D6,TONE_D6, TONE_D3, TONE_D5, TONE_D6, TONE_D6, TONE_DH1,TONE_0, TONE_D7, 
	TONE_D5, TONE_D6	  
};//这部分就是整首曲子的音符部分

float duration[] =
{
	1,1,0.5,0.5,1,0.5,0.5,0.5,0.5,1,0.5,0.5,
	0.5,1,0.5,1,0.5,0.5,0.5,0.5,0.5,0.5,1,1,  
	1,1,1+1,0.5,1,1+0.5,1,1,1,0.5,0.5,1,
	0.5,1,1+0.5,1,0.5,0.5,0.5,0.5,1+1,
	0.5,1,1+0.5,1,1+1,0.5,0.5,1,1+1+1+1,
	0.5,0.5,0.5+0.25,0.25,0.5+0.25,0.25,0.5+0.25,0.25,
	0.5,1,0.5,1,1,
	0.5,0.5,0.5+0.25,0.25,0.5+0.25,0.25,0.5+0.25,0.25,
	0.5,1,0.5,1,1,1+1,0.5,0.5,1,1+1+1+1,
	0.5,1,0.5,1+1,0.5,1,0.5,1+1,1+1,0.5,0.5,1,
	1+1+1+1
};//这部分是整首曲子的节拍部分

int length;//这里定义一个变量，后面用来表示共有多少个音符		

/****默认音乐输出****/
void setMusic()
{

	pinMode(tonePin,OUTPUT);//设置蜂鸣器的pin为输出模式
	
	length = sizeof(tune)/sizeof(tune[0]);//这里用sizeof，可以查出tone序列里有多少个音符	
	for(int x=0; x<length; x++)//循环音符的次数
	{
		tone(tonePin,tune[x]);//此函数依次播放tune序列里的数组，即每个音符
		delay(400*duration[x]);//每个音符持续的时间，即节拍duration，400是调整时间的越大，曲子速度越慢，越小曲子速度越快，自己掌握吧
		noTone(tonePin);//停止当前音符，进入下一音符
	}
	
}

/****自定义音乐输出****/
/*
 *_tune[]:曲调
 *_duration[]: 节拍
 */
void setMusicOUT(int _tune[], float _duration[])
{
	pinMode(tonePin,OUTPUT);//设置蜂鸣器的pin为输出模式

	length = sizeof(_tune)/sizeof(_tune[0]);//这里用sizeof，可以查出tone序列里有多少个音符
	for(int x=0; x<length; x++)//循环音符的次数
	{
		tone(tonePin,_tune[x]);//此函数依次播放tune序列里的数组，即每个音符
		delay(400*_duration[x]);//每个音符持续的时间，即节拍duration，400是调整时间的越大，曲子速度越慢，越小曲子速度越快，自己掌握吧
		noTone(tonePin);//停止当前音符，进入下一音符
	}	
}
/*
 *_tune:曲调
 *_duration: 节拍
 */
void setMusicOUT(int _tune, float _duration)
{
	pinMode(tonePin,OUTPUT);//设置蜂鸣器的pin为输出模式

	tone(tonePin,_tune);//此函数依次播放tune序列里的数组，即每个音符
	delay(400*_duration);//每个音符持续的时间，即节拍duration，400是调整时间的越大，曲子速度越慢，越小曲子速度越快，自己掌握吧
	noTone(tonePin);//停止当前音符，进入下一音符
}


/****Beep蜂鸣器****/
/*
 * turn：ON/OFF
 */
uint8_t BeepFlag = 0;	//内置蜂鸣器初始化标志位
void setBeep(uint8_t turn)//内置
{
	if(BeepFlag==0)
	{
		BeepFlag = 1;
		pinMode(9, OUTPUT);		

	}
	if(turn==ON)
	{
		analogWrite(9,100);
	}
	else if(turn==OFF)
	{
		analogWrite(9,0);		
	}
}

/*
 * pin：PWM端口
 * turn：ON/OFF
 */
uint8_t BeepPinFlag = 0;	//外置蜂鸣器初始化标志位
void setBeepPin(uint8_t pin ,uint8_t turn)//外置
{
	if(BeepPinFlag==0)
	{
		BeepPinFlag = 1;		
		pinMode(pin, OUTPUT);		
	}
	if(turn==ON)
	{
		analogWrite(pin,100);
	}
	else if(turn==OFF)
	{
		analogWrite(pin,0);		
	}
}

//软IIC
/****超声波****/
/*
 * pin: I1-I3
 */
uint16_t getUltrasoundPin(uint8_t pin)			//输入
{
	set_I2C_Init(pin);

	uint16_t data;
	uint8_t  datah=0, datal=0, cmd=0, s=0, SUM=0, i;

	for(i=0;i<3;i++)
	{
	Soft_I2C_Start();
    Soft_I2C_Write(_ULTRASOUND_ADDR_);
    Soft_I2C_Write(0xaa);
    Soft_I2C_Stop();

    Soft_I2C_Start();
    Soft_I2C_Write(_ULTRASOUND_ADDR_+1);
    cmd  = Soft_I2C_Read(0);	//数据开头 
	if(cmd!=0xAA) cmd  = Soft_I2C_Read(0);
    datah= Soft_I2C_Read(0);	//数据高八位
    datal= Soft_I2C_Read(0);	//数据低八位 
    s = Soft_I2C_Read(1);  	//检验和
    Soft_I2C_Stop();

    SUM = (uint8_t)(cmd + datah + datal);//校验
	if(SUM==s)i=3;		//如果一次读取成功则读一次，不成功读三次
	}
    if(SUM!=s)
    {
		data = 999;
    }
    else
    {
		data = (uint16_t)(((uint16_t)datah<<8)|datal);
    }
    return (uint16_t)data;
}
/****颜色传感器****/
/* 参数说明
 * pin: I1-I3
 * cmd:	20~28 设置灯的状态
 *		29~30保留
 *		31 自动白平衡处理
 *		32 重设白平衡参考值
 *		33 恢复出厂设置
 *		34~36 设白平衡系数
 *		37 设最大颜色值
 *		38 调制模式
 *		39 非调制模式
 *		42 颜色学习
 *		43 停止颜色学习
 *		44 颜色扩展
 *		45 关闭颜色扩展
 *		46-51 设置颜色扩展参考值
 *		52-53 设置黑白参考值
 *		54 黑白阈值模式
 *		55 关闭黑白阈值模式
 * 	parameter: 设置颜色传感器参数，命令34-37，39，46-53才会用到
 */
void setColorSensorPin(uint8_t pin, uint8_t cmd, int parameter)//输出
{
	set_I2C_Init(pin);

	uint8_t sum  = 0;
	uint8_t addr = 0x50;
	uint8_t m_addrh, m_addrl;
	uint8_t first, m_id,end;
	uint16_t m_val;
	uint8_t id =1 ;

	if(cmd==40 || cmd==41) return ;
	//从机地址为10Bite 0xf6xx:高2位固定为11,低八位为0x50~0x57,即实际使用的地址为0x350~0x357(0xf650~0xf657)

	Soft_I2C_Start();
	Soft_I2C_Write(0xf6);//写临时地址
	Soft_I2C_Write(addr);
	Soft_I2C_Write(0xEA);
	Soft_I2C_Write(cmd);
	Soft_I2C_Write(parameter>>8);
	Soft_I2C_Write(parameter&0xff);
	Soft_I2C_Stop();

	if(((cmd>=31) && (cmd<=32)) || (cmd>=34 && cmd<=37) || (cmd>=44 && cmd<=55)) delay(25);
	else if(cmd>=33) delay(200);
	else if(cmd>=43) delay(1500);
}

/* 参数说明
 * pin: I1-I3
 * cmd:	1~3 读RGB三原色值，
 * 		4 读颜色识别结果，识别结果与颜色关系：1-红色，2-绿色，3-蓝色，4-黄色，5-黑色，6-白色
 * 		5 读RGB888数据
 *		6~8 读原始RGB数据
 * 		9~11 RGB模拟光值
 *		12-14 白平衡比例值
 *		15 色调
 *		16 饱和度
 *		17 亮度
 *		18 保留
 *		19 最大输出值
 *		20-60 设置命令
 *		61-66 颜色扩展参考值
 *		67 白色参考值
 *		68 黑色参考值
 */
uint32_t getColorSensorPin(uint8_t pin, uint8_t cmd)//输入
{
	set_I2C_Init(pin);

	uint32_t m_val = 0;
	uint8_t m_val1 = 0, m_val2 = 0, m_val3 = 0;
	uint8_t m_bit1 = 0, m_bit2 = 0;
	uint8_t first  = 0, end    = 0;
	uint8_t m_id   = 0;
	uint8_t id     = 1;
	uint8_t sum    = 0, m_para1= 0, m_para2 = 0;
	uint8_t addrh, addrl;
	uint8_t addr;

	addr = id + 0x50 - 1;//0xf650~0xf657

	Soft_I2C_Start();
	Soft_I2C_Write(0xf6);//SlaveAddrH
	Soft_I2C_Write(addr);//SlaveAddrL
	Soft_I2C_Write(0xFA);
	Soft_I2C_Write(cmd);

	if((cmd<20&&cmd>0) || (cmd>60 && cmd<=74) || ((cmd>=254) && (cmd<=255)))
	{
		Soft_I2C_Start();		//重复起始位
		Soft_I2C_Write(0xf7);	//SlaveAddr|0x01
		first  = Soft_I2C_Read(0);
		m_id   = Soft_I2C_Read(0);
		m_val1 = Soft_I2C_Read(0);
		if((cmd>=5 && cmd<=18) || (cmd>60 && cmd<=74) || (cmd==254))
		{
			m_val2 = Soft_I2C_Read(0);
			m_bit1 = 8; 
			m_bit2 = 0;
		}
		if(cmd==5)
		{
			m_val3 = Soft_I2C_Read(0);
			m_bit1 = 16;
			m_bit2 = 8;
		}
    end = Soft_I2C_Read(1);
	}
	
	Soft_I2C_Stop();//停止位
	sum=m_id+m_val1+m_val2+m_val3;	//校验和计算

	if(first==0xEA && sum==end && m_id==addr)
	{
		m_val = (uint32_t)(m_val1<<m_bit1)|(m_val2<<m_bit2)|(m_val3);
	}
	else
	{
		m_val = 999;
	}
	return m_val;
}

/****姿态传感器****/
/*
 * pin：I1-I3
 * command：1-校正水平姿态方位；2-恢复出厂设置
 */
void setAHRSPin(uint8_t pin, uint8_t command)
{
	set_I2C_Init(pin);
	
	Soft_I2C_Start();
	Soft_I2C_Write(0x3a);//addr
	Soft_I2C_Write(0xaa);
	Soft_I2C_Write(0x55);
	Soft_I2C_Write(command);
	Soft_I2C_Write(0);
	Soft_I2C_Write(0);
	Soft_I2C_Write(0);
	Soft_I2C_Write(0xcf);
	Soft_I2C_Stop();

	setDelay_ms(1000);
}
  
short int g_AhrsRecBuf[8][9];
uint8_t g_AhrsRecBufCnt[8];
uint8_t g_AhrsFistRxFlag[8];
uint8_t g_LastCmdList[8][9];
  
short int getAHRSRecBuf(uint8_t pin, uint8_t cnt)//数据已经读出则返回8888
{
	short int buf;
	if(cnt>8) return 9999;//cnt=9-1
	buf = g_AhrsRecBuf[pin][cnt];
	g_AhrsRecBuf[pin][cnt] = 8888;
	if(buf != 8888)
	{
		g_AhrsRecBufCnt[pin]++;
		g_LastCmdList[pin][cnt] = 1;
	}
	if(g_AhrsRecBufCnt[pin]>8)//已经将缓存全部读出
	{
		g_AhrsRecBufCnt[pin] = 0;
		g_AhrsFistRxFlag[pin]= 0;
	}
	return buf;
}

/*
 * pin：I1-I3
 *command1: 1-横滚角；2-俯仰角；3-偏航角；4-X轴加速度;5-Y轴加速度；6-Z轴加速度；7-X轴陀螺仪；8-Y轴陀螺仪；9-Z轴陀螺仪
 *command2：0-读取当前数据；1-重新读取数据
 */
short int getAHRSPin(uint8_t pin, uint8_t command1, uint8_t command2)
{
	uint8_t i,j;
	uint8_t ACK = 0;
	short int m_data;
	uint8_t m_AhrsRecBufList[22];
	uint8_t m_Rxflag = 0;//接收一次命令
	uint8_t whichx = pin-1;
	
	set_I2C_Init(pin);

	if(g_AhrsFistRxFlag[whichx]==0 || command2 || g_LastCmdList[whichx][command1-1])
	{
		m_Rxflag = 1;//接收一次缓存
		if(command2==0) g_AhrsFistRxFlag[whichx] = 1;
		else g_AhrsFistRxFlag[whichx] = 0;
		for(i=0; i<9; i++) 
		{
			g_LastCmdList[whichx][i] = 0;//清零
		}
		
		Soft_I2C_Start();
		Soft_I2C_Write(0x3b);//addr
		
		for(i=0; i<22; i++)
		{
			if(i==21) ACK = 1;
			else  ACK = 0;
			m_AhrsRecBufList[i] = Soft_I2C_Read(ACK);  
		} 
		
		Soft_I2C_Stop();
	}
	//OS_EXIT_CRITICAL();
	if(m_Rxflag)//接收一次数据
    {
		g_AhrsRecBufCnt[whichx]=0;
		if(m_AhrsRecBufList[0]==0xaa && m_AhrsRecBufList[1]==0x55 && m_AhrsRecBufList[21]==0xcf)//正确数据包
		{
			g_AhrsFistRxFlag[whichx]=2;
			g_AhrsRecBuf[whichx][0]=(short int)(m_AhrsRecBufList[2]) |((char)m_AhrsRecBufList[3]<<8);//roll//横滚角 degree（放大10倍）
			g_AhrsRecBuf[whichx][1]=(short int)(m_AhrsRecBufList[4]) |((char)m_AhrsRecBufList[5]<<8);//pitch//仰俯角 degree（放大10倍）
			g_AhrsRecBuf[whichx][2]=(short int)(m_AhrsRecBufList[6]) |((char)m_AhrsRecBufList[7]<<8);//yaw//偏航角 degree（放大10倍）
			g_AhrsRecBuf[whichx][3]=(short int)(m_AhrsRecBufList[8]) |((char)m_AhrsRecBufList[9]<<8);//ax//加速度X g（放大100倍）
			g_AhrsRecBuf[whichx][4]=(short int)(m_AhrsRecBufList[10])|((char)m_AhrsRecBufList[11]<<8);//ay//加速度Y g（放大100倍）
			g_AhrsRecBuf[whichx][5]=(short int)(m_AhrsRecBufList[12])|((char)m_AhrsRecBufList[13]<<8);//az//加速度Z g（放大100倍）
			g_AhrsRecBuf[whichx][6]=(short int)(m_AhrsRecBufList[14])|((char)m_AhrsRecBufList[15]<<8);//gx//陀螺仪X  dps（放大100倍）
			g_AhrsRecBuf[whichx][7]=(short int)(m_AhrsRecBufList[16])|((char)m_AhrsRecBufList[17]<<8);//gy//陀螺仪Y  dps（放大100倍）
			g_AhrsRecBuf[whichx][8]=(short int)(m_AhrsRecBufList[18])|((char)m_AhrsRecBufList[19]<<8);//gz//陀螺仪Z  dps（放大100倍）
		}
		else 
		{
			g_AhrsFistRxFlag[whichx] = 0;
			return 9999;//接收到非数据包
		}
		m_AhrsRecBufList[0]=0;		
		m_AhrsRecBufList[1]=0; 		
		m_AhrsRecBufList[21]=0;
   }
   if(g_AhrsFistRxFlag[whichx]==2)  m_data = getAHRSRecBuf(whichx,command1-1);//command1=1~9
   else m_data=9999;

   return (short int)m_data;
}

/****Joystick摇杆输入****/
/*
 * pin：I1-I3
 *command: ROCKER_X/Rocker_Y/BUTTON--x/y/按键
 */
int getJoystickCmdPin(uint8_t pin, uint8_t command)
{
    set_I2C_Init(pin);  
  
    int m_temp;
    int m_fix;
   
    Soft_I2C_Start();
    Soft_I2C_Write(_COMPOI_ADDR_);
    Soft_I2C_Write(command);
    Soft_I2C_Stop();
    
    Soft_I2C_Start();
    Soft_I2C_Write(_COMPOI_ADDR_+1);
    m_fix = Soft_I2C_Read(0);
    m_temp= Soft_I2C_Read(1);
    Soft_I2C_Stop(); 
      
	if( m_fix==0) //X轴
    { 
		if (m_temp<=92) return m_temp-100;//左
		else if(m_temp>=108)  return ~(100-m_temp)+1;
		else return 0;

    }
	else if( m_fix==1) //Y轴
    { 
		if (m_temp<=92) return 100-m_temp;//上
		else if(m_temp>=108)  return ~(m_temp-100)+1;
		else return 0;
    }
    else if( m_fix ==2)
    {
		if (m_temp==0xAB)  return 1;//按键
		else return 0;
    }
    else
    {
		return 999;
    }     
}
/****2.4G_遥控****/
//通讯密码设置
/*
 * pin：I1-I3
 * password：123456
 */
void setRFPassWordPin(uint8_t pin, uint32_t password)
{
	set_I2C_Init(pin);

    uint8_t pw[6] = {0x00};
    uint8_t check_sum = 0;	
    
	pw[0] = (uint8_t)( password/1000000);  
	pw[1] = (uint8_t)((password/100000)%10);
	pw[2] = (uint8_t)((password/10000)%10);
	pw[3] = (uint8_t)((password/1000)%10);
	pw[4] = (uint8_t)((password/100)%10);
	pw[5] = (uint8_t)( password%100);

    Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x01);
    check_sum += 0x01;
    Soft_I2C_Write(0x09); 
    check_sum += 0x09;
    Soft_I2C_Write(pw[0]); 
    check_sum += pw[0];
    Soft_I2C_Write(pw[1]);
    check_sum += pw[1];
    Soft_I2C_Write(pw[2]);  
    check_sum += pw[2];   
    Soft_I2C_Write(pw[3]); 
    check_sum += pw[3];
    Soft_I2C_Write(pw[4]);
    check_sum += pw[4];
    Soft_I2C_Write(pw[5]);  
    check_sum += pw[5];
    Soft_I2C_Write(check_sum);  
    Soft_I2C_Stop();    
 }

//通讯接收数据
/*
 * pin：I1-I3
 */
uint16_t getRFModuleBytePin(uint8_t pin)
{
	set_I2C_Init(pin);
	
	uint16_t data = 999;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint8_t n;

	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++)	//一次要读取完所有缓冲数据  
    {
         rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x11 && rf_data[15]==0xCF)//检验命令包
    {	
       data=rf_data[3];
    }
    
   return data;
}

//通讯发送数据
/*
 * pin：I1-I3
 * data：变量/字符
 */
uint8_t setRFModuleBytePin(uint8_t pin,uint8_t _data)
{
	uint8_t check_sum = 0;
	uint8_t ok_flag = 0x02;
	
	set_I2C_Init(pin);
	
	//发射指令
	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x02);//命令,发射数据
    check_sum += 0x02;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(_data); 
    check_sum += _data;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	//设置查询的内容是哪种
	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x04);//命令
    check_sum += 0x04;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	//读取具体数据
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    do{
		ok_flag = Soft_I2C_Read(0);
	}while(ok_flag==0x02); //不断查询，等待发射完毕
    ok_flag = Soft_I2C_Read(1); 
    Soft_I2C_Stop();
    
   return(ok_flag);
}

//接收手柄键值
/*
 * pin：I1-I3
 */
uint16_t getRFModuleRemoteButtonPin(uint8_t pin)
{
	uint16_t data = 999;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint32_t code = 0;
	uint32_t k = 1;
	uint8_t n;

	set_I2C_Init(pin);
	
	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++) //一次要读取完所有缓冲数据
    {
        rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    
    // data=rf_data[3];
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x05 && rf_data[15]==0xCF) //检验命令包
    {
    	code = (uint32_t)rf_data[3]*0x1000000+(uint32_t)rf_data[4]*0x10000+(uint32_t)rf_data[5]*0x100+(uint32_t)rf_data[6];
    	for(n=0; n<20; n++)
		{
			if(code & (k<<n))
			{
				data = n;
				if(code & (k<<26))
				{
					data += 20;
				}
				break;
			}
		}
		return data;
	}
	else 
		return 999;
}

//接收手柄键码
/*
 * pin：I1-I3
 */
uint32_t getRFModuleRemoteCodePin(uint8_t pin)
{	
	uint32_t code = 0;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint32_t k = 1;
	uint8_t n;
	
	set_I2C_Init(pin);

	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++) //一次要读取完所有缓冲数据
    {
         rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    
    // data=rf_data[3];
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x05 && rf_data[15]==0xCF) //检验命令包
    {
    	code = (uint32_t)rf_data[3]*0x1000000+(uint32_t)rf_data[4]*0x10000+(uint32_t)rf_data[5]*0x100+(uint32_t)rf_data[6];
    }

   return code;
}

//接收手柄重力值
/*
 * pin：I1-I3
 * axis: GravityX/GravityY/GravityZ
 */
short int getRFModuleRemoteGravityPin(uint8_t pin, uint8_t axis)
{
	short int data = 999;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint8_t n;
	
	set_I2C_Init(pin);

	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++) //一次要读取完所有缓冲数据
    {
         rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    
    // data=rf_data[3];
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x05 && rf_data[15]==0xCF) //检验命令包
    {
        switch(axis)
		{
        	case 0x01: {data = (char)rf_data[11];} break; //X
			case 0x02: {data = (char)rf_data[12];} break; //Y
			case 0x03: {data = (char)rf_data[13];} break; //Z
			default:   {data =  999;}
		} 
    }

	return data;
}

//接收手柄摇杆值
/*
 * pin：I1-I3
 * direction:ROCKER_LEFT/ROCKER_RIGHT
 * axis: ROCKER_X/Rocker_Y
 */
short int getRFModuleRemoteRockerPin(uint8_t pin, uint8_t direction, uint8_t axis)
{
	short int data = 999;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint8_t n;
	
	set_I2C_Init(pin);
	
	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
	
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++) //一次要读取完所有缓冲数据
    {
         rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    
    // data=rf_data[3];
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x05 && rf_data[15]==0xCF) //检验命令包
    {
    	 if(direction==0)
    	 {
            switch(axis)
			{
				case 0: {data = (char)rf_data[7];} break; //X
				case 1: {data = (char)rf_data[8];} break; //Y
				default:{data =  999;}
			}         
        }
        else
        {
			switch(axis)
			{
				case 0:	{data = (char)rf_data[9];} break;  //X
				case 1:	{data = (char)rf_data[10];}break;  //Y
				default:{data =  999;}
			} 
        }
    }
	return data;
}

//串口(保留该程序-解码BC7210A)
/****红外接收****/
/*
 * _uart：串口0-3
 */
/*
uint8_t uartData   = 0;
uint8_t IRFlag[4] = {0,0,0,0};//串口初始化标志位
uint8_t getIRPin(uint8_t _uart)
{	
	if(IRFlag[_uart]==0)	//初始化
	{
		IRFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
						Serial.begin(9600);	//开串口接收
						UCSR0B &= 0xF3;     //禁止TX0
						pinMode(1,INPUT_PULLUP);//设置IO输入且上拉电阻
						break;
						
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
						Serial1.begin(9600);//开串口接收
						UCSR1B &= 0xF3;     //禁止TX1
						pinMode(41,INPUT_PULLUP);//设置IO输入且上拉电阻
						break;
						
			case uart2 ://RX-17 TX-16
						Serial2.begin(9600);//开串口接收
						UCSR2B &= 0xF3;     //禁止TX2
						pinMode(16,INPUT_PULLUP);//设置IO输入且上拉电阻
						break;
						
			case uart3 ://RX-15 TX-14
						Serial3.begin(9600);//开串口接收
						UCSR3B &= 0xF3;     //禁止TX3
						pinMode(14,INPUT_PULLUP);//设置IO输入且上拉电阻
						break;
						
			default : break;
		}
	}	
	
	switch(_uart)//接收数据
	{
		case uart0 :
					if(Serial.available()>0)//检测接收数据
					{             
						uartData = int(Serial.read());//存储缓冲数值
						setDelay_ms(2);
					}
					break;

		case uart1 :
					if(Serial1.available()>0)//检测接收数据
					{             
						uartData = int(Serial1.read());//存储缓冲数值
						setDelay_ms(2);
					}
					break;

		case uart2 :
					if(Serial2.available()>0)//检测接收数据
					{             
						uartData = int(Serial2.read());//存储缓冲数值
						setDelay_ms(2);
					}
					break;

		case uart3 :
					if(Serial3.available()>0)//检测接收数据
					{             
						uartData = int(Serial3.read());//存储缓冲数值
						setDelay_ms(2);						
					}
					break;

		default : break;		
	}
	switch(_uart)//不连续信号
	{
		case uart0 :
					if(getDigitalPin(1)==1)//检测不接收数据
					{             
						return uartData=0;
					}
					break;

		case uart1 :
					if(getDigitalPin(41)==1)//检测不接收数据
					{             
						return uartData=0;
					}
					break;

		case uart2 :
					if(getDigitalPin(16)==1)//检测不接收数据
					{             
						return uartData=0;
					}
					break;

		case uart3 :
					if(getDigitalPin(14)==1)//检测不接收数据
					{             
						return uartData=0;
					}
					break;

		default : break;	
	}
	if(uartData!=0 && uartData!=255)//过滤引导码
	{
		return uartData;//返回遥控器值
	}
}
*/

/****蓝牙接收***/
/*
 * _uart：串口0-3
 * mode ：键码/键值(DOUBLE_CODE/SINGLE_CODE)
 */
uint8_t j=0;
uint8_t jj=0;
uint8_t bluetoothFlag[4] = {0,0,0,0};//串口初始化标志位
uint16_t getBluetoothPin(uint8_t _uart,uint8_t mode)
{ 
	uint8_t dataByte[RF_NUM]={0x00};//数据缓冲区
	uint16_t dataInt=999;   //返回数据
	uint32_t code = 0;	//键码
	
	if(bluetoothFlag[_uart]==0) //初始化
	{
		bluetoothFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
				Serial.begin(115200); //开串口接收
				break;
            
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
				Serial1.begin(115200);//开串口接收
				break;
            
			case uart2 ://RX-17 TX-16
				Serial2.begin(115200);//开串口接收
				break;
            
			case uart3 ://RX-15 TX-14
				Serial3.begin(115200);//开串口接收
				break;
            
			default : break;
		}
	}
  
	switch(_uart)//接收数据
	{
		case uart0 :
			if(Serial.available()>0)//检测接收数据
			{              
				Serial.readBytes(dataByte,RF_NUM);//存储数据     
			}
			break;

		case uart1 :
			if(Serial1.available()>0)//检测接收数据
			{              
				Serial1.readBytes(dataByte,RF_NUM);//存储数据       
			}
			break;

		case uart2 :
			if(Serial2.available()>0)//检测接收数据
			{              
				Serial2.readBytes(dataByte,RF_NUM);//存储数据;         
			}
			break;

		case uart3 :
			if(Serial3.available()>0)//检测接收数据
			{
				Serial3.readBytes(dataByte,RF_NUM);//存储数据
			}
			break;

		default : break;    
	}
	
	if(dataByte[0]==0x56 && dataByte[1]==0xAB && dataByte[2]==0x14 && dataByte[3]==0xE8 && dataByte[4]==0x10)//检查包头            
    {  
		if(mode == 0)     //组合键码
		{       
			code = (uint32_t)dataByte[5]*0x1000000+(uint32_t)dataByte[6]*0x10000+(uint32_t)dataByte[7]*0x100+(uint32_t)dataByte[8]; 
			dataInt = (uint16_t)code;//2个字节
		}
		else if(mode == 1)//单一键值--低位优先
		{      
			code = (uint32_t)dataByte[5]*0x1000000+(uint32_t)dataByte[6]*0x10000+(uint32_t)dataByte[7]*0x100+(uint32_t)dataByte[8];          
			for(int n=0; n<32; n++)//32个按键空间
			{
				if(code & (1<<n))
				{
					dataInt = n;
					break;
				}
			}
		}
		
		j=0;
		return dataInt;        
    }
    else if(1)      //清缓存
    {
		if(j <= 300)
        {
			j++;
			setDelay_ms(1);
			if(jj==0)	
			{
				jj=1;
				return dataInt;	
			}			
        }
        else
        {
			j=0;
			return 999;
        }
    } 		
}

//硬IIC
/****LCD1602_IIC显示ASCII码****/
/*
 * addr: 地址(0x3F/0x27)
 * x:行(0-1)
 * y:列(0-15)
 * str: 字符/字符串格式
 */
uint8_t LCDFlag = 0;	//LCD初始化标志位
void setLCDWriteStr(uint8_t addr, uint8_t x,uint8_t y,char *str)
{
	LiquidCrystal_I2C lcd(addr,16,4);//设备地址
	
	if(LCDFlag==0)
	{		
		LCDFlag = 1;
		lcd.init();		//初始化
	}
	lcd.backlight();	//亮屏
	lcd.setCursor(y,x);	//设置显示XY坐标
	lcd.print(str);		//显示字符
}
void setLCDWriteStr(uint8_t addr, uint8_t x,uint8_t y,String str)
{
	LiquidCrystal_I2C lcd(addr,16,4);//设备地址  
	if(LCDFlag==0)
	{   
		LCDFlag = 1;
		lcd.init();   //初始化
	}
	lcd.backlight();  //亮屏
	lcd.setCursor(y,x); //设置显示XY坐标
	lcd.print(str);   //显示字符
}

/*
 * addr: 地址(0x3F/0x27)
 * x:行(0-1)
 * y:列(0-15)
 * longNum: 整数
 */
void setLCDWriteInt(uint8_t addr, uint8_t x,uint8_t y,long longNum)
{
	uint32_t _longNum;
	uint8_t  _charNum[4], zf;	//4位有效数字
	
	LiquidCrystal_I2C lcd(addr,16,4);//设备地址
	
	if(LCDFlag==0)
	{		
		LCDFlag = 1;
		lcd.init();	//初始化
	}
	lcd.backlight();//亮屏
	
	if(longNum>=0)		//输入正数
	{
		zf = -1;
	}
	else 					//输入负数
	{
		lcd.setCursor(y,x);
		lcd.write(0x2D);	//-
		zf = 0;
	}
	
	_longNum = abs(longNum);//取正整数
	
	_charNum[3] = (_longNum/1000);		//千位
	_charNum[2] = (_longNum%1000)/100;	//百位
	_charNum[1] = (_longNum%100)/10;	//十位
	_charNum[0] = (_longNum%10)/1;		//个位
	
	for(int i=0; i<4; i++)	//十进制改十六进制
	{
		_charNum[i]=_charNum[i]+48;//ASCII码表	
	}
	
	for(int i=1; i<5; i++)		
	{
		lcd.setCursor(y+i+zf,x);
		lcd.write(_charNum[4-i]);		
	}
}

/*
 * addr: 地址(0x3F/0x27)
 * x:行(0-1)
 * y:列(0-15)
 * floatNum: 6位有效数字+2位精度小数的浮点数
 */
void setLCDWriteFloat(uint8_t addr, uint8_t x,uint8_t y,float floatNum)
{
	uint32_t _longNum;
	uint8_t  _charNum[7], zf;	//6位有效数字+小数点
	
	LiquidCrystal_I2C lcd(addr,16,4);//设备地址

	if(LCDFlag==0)
	{		
		LCDFlag = 1;
		lcd.init();	//初始化
	}
	lcd.backlight();//亮屏
	
	if(floatNum>=0)		//输入正数
	{
		zf = -1;		
	}
	else 					//输入负数
	{
		lcd.setCursor(y,x);
		// lcd.print("-");
		lcd.write(0x2D);	//-
		zf = 0;
	}

	_longNum = (uint32_t)(fabs(floatNum)*100);	//取两位小数的正整数
	
	_charNum[6] = (_longNum/100000);		//千位
	_charNum[5] = (_longNum%100000)/10000;	//百位
	_charNum[4] = (_longNum%10000)/1000;	//十位
	_charNum[3] = (_longNum%1000)/100;		//个位
	_charNum[2] =  0x2E; 					//小数点'.'			
	_charNum[1] = (_longNum%100)/10;		//十分位
	_charNum[0] = (_longNum%10)/1;			//百分位
	
	for(int i=0; i<7; i++)	//十进制改十六进制
	{
		switch(_charNum[i])
		{
			case 0: _charNum[i]=0x30;break;//0-9
			case 1: _charNum[i]=0x31;break;
			case 2: _charNum[i]=0x32;break;
			case 3: _charNum[i]=0x33;break;
			case 4: _charNum[i]=0x34;break;
			case 5: _charNum[i]=0x35;break;
			case 6: _charNum[i]=0x36;break;
			case 7: _charNum[i]=0x37;break;
			case 8: _charNum[i]=0x38;break;
			case 9: _charNum[i]=0x39;break;
			default :break;			
		}		
	}
	
	for(int i=1; i<8; i++)		
	{
		lcd.setCursor(y+i+zf,x);
		lcd.write(_charNum[7-i]);		
	}	
	// setLCDClear(addr);//清屏
}

void  setLCDClear(uint8_t addr)
{
	LiquidCrystal_I2C lcd(addr,16,4);//设备地址
	if(LCDFlag==0)
	{		
		LCDFlag = 1;
		lcd.init();	//初始化
	}	
	lcd.clear();	//清屏
}

short int getRFModuleRemoteRockerPin_Balance(uint8_t pin,uint8_t flag)
{
	INT_TO_BYTE data;
	uint8_t check_sum = 0;
	uint8_t rf_data[16] = {0x00};  
	uint8_t n;  
  
	set_I2C_Init(pin);
  
	Soft_I2C_Start();
    Soft_I2C_Write(0XA0);
    Soft_I2C_Write(0xaa);    
    Soft_I2C_Write(0x55);    
    Soft_I2C_Write(0x05);//命令,查询接收缓冲
    check_sum += 0x05;
    Soft_I2C_Write(0x04); 
    check_sum += 0x04;
    Soft_I2C_Write(0x00); 
    check_sum += 0x00;
	Soft_I2C_Write(check_sum); 
	Soft_I2C_Stop();
  
    Soft_I2C_Start();
    Soft_I2C_Write(0XA0+1);
    for(n=0; n<15; n++) //一次要读取完所有缓冲数据
    {
        rf_data[n] = Soft_I2C_Read(0);
    }
    rf_data[15] = Soft_I2C_Read(1);
    Soft_I2C_Stop();
    
    if(rf_data[0]==0x56 && rf_data[1]==0xAB && rf_data[2]==0x05 && rf_data[15]==0xCF) //检验命令包
    {
        if(flag == 1)
        {      
            data.buf[0] = (char)rf_data[7];
            data.buf[1] = (char)rf_data[10];     
        }
        return data.j;
    }
    return 0;
}
/****EEPROM****/
//***************函数说明*********************
//GetData：开放地址：1-156（与E6保持一致）
//		   返回数值：0-4095（与E6保持一致,方便存阈值）
//SetData：开放地址：1-156（与E6保持一致）
//		   输入数值：0-4095（与E6保持一致,方便存阈值）
//**********************************************
int GetData(int address)
{
	if((address>0)&&(address<=156))
	//return EEPROM.read(address);
	return EEPROM.read(address*2)+EEPROM.read(address*2-1)*0x100;//低8位+高8位*0x100
	else
	return 888;
}

void SetData(int addr,int val)
{
	if(val>4095)val=4095;
	if((addr>0)&&(addr<=156))	
	EEPROM.write(addr*2-1, val/0x100);//高8位
	EEPROM.write(addr*2, val%0x100);//低8位
	//EEPROM.write(addr, val);
}
/****AI摄像头模块****/
//***************函数说明*********************
//_uart:端口选择 
//uart0 :RX-0 TX-1 会和下载串口冲突，使用时需要拔开摄像头后才能下载
//uart1 :RX-19 TX-18 分别对应 编码m4 m3
//uart2 :RX-17 TX-16
//uart3 :RX-15 TX-14
//设置AI视觉模块模式
//cmd0:模式选择
//1、无；2、颜色识别；3、二维码识别；4、小球识别；5、道路识别；6、人脸识别；7、图像分类；8、标签码识别；
//9、颜色识别学习；10、人脸识别学习；11、手势识别；12、图像分类学习；13、交通标志识别；
//cmd1:小球颜色选择（小球识别功能专用）
//0、红；1、绿；2、蓝；3、黄；
//cmdLED:补光灯开关
//0、关；1、开；
//**********************************************

//读取AI视觉模块数据
String comdata = "";//接收数组存储
uint8_t AIdata=0;//返回数据用变量
uint8_t ummo=0;//寄存器归零用
uint8_t remo=0;//返回255用
uint8_t GetAICamData(uint8_t _uart,uint8_t cmd)
{//读取AI视觉模块数据
	if(bluetoothFlag[_uart]==0) //初始化
	{
		bluetoothFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
				Serial.begin(115200); //开串口接收
				break;
            
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
				Serial1.begin(115200);//开串口接收
				break;
            
			case uart2 ://RX-17 TX-16
				Serial2.begin(115200);//开串口接收
				break;
            
			case uart3 ://RX-15 TX-14
				Serial3.begin(115200);//开串口接收
				break;
            
			default : break;
		}
	}
	switch(_uart)//接收数据
	{
		case uart0 :
			while (Serial.available() > 0)  
			{
				comdata += char(Serial.read());
				delay(2);
			}
			break;

		case uart1 :
			while (Serial1.available() > 0)  
			{
				comdata += char(Serial1.read());
				delay(2);
			}
			break;

		case uart2 :
			while (Serial2.available() > 0)  
			{
				comdata += char(Serial2.read());
				delay(2);
			}
			break;

		case uart3 :
			while (Serial3.available() > 0)  
			{
				comdata += char(Serial3.read());
				delay(2);
			}
			break;

		default : break;    
	}	
	if (comdata.length() > 0)
	{
		AIdata=comdata[cmd+4];
	}
	ummo++;
	if(ummo>1){comdata = "";ummo=0;}//归零
	delay(10);
	if(AIdata<0xFF) return AIdata;
	else if(AIdata==0xFF&&remo>5) {remo=0;return 255;}
	else if(AIdata==0xFF) remo++;
	else return 0;
}

String Setdata = "";//发送数组存储
uint8_t LED=0;
void SetAICamData(uint8_t _uart,uint8_t cmd0,uint8_t cmd1)
{//设置AI视觉模块模式
	Setdata+=char(0x86);//0
	Setdata+=char(0xAB);//1
	Setdata+=char(0x14);//2
	Setdata+=char(0xE8);//3
	Setdata+=char(0x13);//4
	Setdata+=char(cmd0);//5
	Setdata+=char(cmd1);//6
	
	Setdata+=char(0);//7
	Setdata+=char(0);//8
	Setdata+=char(0);//9
	Setdata+=char(0);//10
	Setdata+=char(0);//11
	Setdata+=char(0);//12
	Setdata+=char(0);//13
	Setdata+=char(0);//14
	Setdata+=char(0);//15
	Setdata+=char(0);//16
	Setdata+=char(0);//17
	
	Setdata+=char(LED);//18
	Setdata+=char(0xCF);//19
	
	if(bluetoothFlag[_uart]==0) //初始化
	{
		bluetoothFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
				Serial.begin(115200); //开串口接收
				break;
            
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
				Serial1.begin(115200);//开串口接收
				break;
            
			case uart2 ://RX-17 TX-16
				Serial2.begin(115200);//开串口接收
				break;
            
			case uart3 ://RX-15 TX-14
				Serial3.begin(115200);//开串口接收
				break;
            
			default : break;
		}
	}
	switch(_uart)//发送数据
	{
		case uart0 :
			if (Setdata.length() > 0)
			{
				Serial.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart1 :
			if (Setdata.length() > 0)
			{
				Serial1.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart2 :
			if (Setdata.length() > 0)
			{
				Serial2.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart3 :
			if (Setdata.length() > 0)
			{
				Serial3.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		default : break;    
	}	
}

void SetWaitForAICamData(uint8_t _uart,uint8_t cmd0,uint8_t cmd1)
{//等待AI视觉模块模式
	uint16_t time=0;
	uint8_t time2=0;
	uint8_t KEY=0;
	Setdata+=char(0x56);//0
	Setdata+=char(0xAB);//1
	Setdata+=char(0x14);//2
	Setdata+=char(0xE8);//3
	Setdata+=char(0x13);//4
	Setdata+=char(0xEE);//5
	Setdata+=char(0xFF);//6
	
	Setdata+=char(0);//7
	Setdata+=char(0);//8
	Setdata+=char(0);//9
	Setdata+=char(0);//10
	Setdata+=char(0);//11
	Setdata+=char(0);//12
	Setdata+=char(0);//13
	Setdata+=char(0);//14
	Setdata+=char(0);//15
	Setdata+=char(0);//16
	Setdata+=char(0);//17
	
	Setdata+=char(LED);//18
	Setdata+=char(0xCF);//19
	
	if(bluetoothFlag[_uart]==0) //初始化
	{
		bluetoothFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
				Serial.begin(115200); //开串口接收
				break;
            
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
				Serial1.begin(115200);//开串口接收
				break;
            
			case uart2 ://RX-17 TX-16
				Serial2.begin(115200);//开串口接收
				break;
            
			case uart3 ://RX-15 TX-14
				Serial3.begin(115200);//开串口接收
				break;
            
			default : break;
		}
	}
	
	while(1)
    {
		switch(_uart)//接收数据
		{
			case uart0 :
				Serial.println(Setdata);//发送数据
				delay(100);
				while (Serial.available() > 0 && time2<19)  
				{//接收数据包
					if(KEY==0){
						if(char(Serial.read())==char(0x86)) KEY=1;
					}
					if(KEY==1){
						comdata += char(Serial.read());
						delay(2);time2++;
					}delay(2);
				}
			break;

			case uart1 :
				Serial1.println(Setdata);//发送数据
				delay(100);
				while (Serial1.available() > 0 && time2<19)  
				{//接收数据包
					if(KEY==0){
						if(char(Serial1.read())==char(0x86)) KEY=1;
					}
					if(KEY==1){
						comdata += char(Serial1.read());
						delay(2);time2++;
					}delay(2);
				}
			break;

			case uart2 :
				Serial1.println(Setdata);//发送数据
				delay(100);
				while (Serial1.available() > 0 && time2<19)  
				{//接收数据包
					if(KEY==0){
						if(char(Serial1.read())==char(0x86)) KEY=1;
					}
					if(KEY==1){
						comdata += char(Serial1.read());
						delay(2);time2++;
					}delay(2);
				}
			break;

			case uart3 :
				Serial1.println(Setdata);//发送数据
				delay(100);
				while (Serial1.available() > 0 && time2<19)  
				{//接收数据包
					if(KEY==0){
						if(char(Serial1.read())==char(0x86)) KEY=1;
					}
					if(KEY==1){
						comdata += char(Serial1.read());
						delay(2);time2++;
					}delay(2);
				}
			break;

			default : break;    
		}
		if (comdata.length() > 0)
		{//接收成功
			if(comdata[4]==char(0xF0)&&comdata[5]==char(0xF1))break;
		}
		if(time>127)break;
		time++;
	}	
	delay(10);
	Setdata[5]=char(cmd0);//5
	Setdata[6]=char(cmd1);//6
	
	switch(_uart)//发送数据
	{
		case uart0 :
			if (Setdata.length() > 0)
			{
				Serial.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart1 :
			if (Setdata.length() > 0)
			{
				Serial1.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart2 :
			if (Setdata.length() > 0)
			{
				Serial2.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart3 :
			if (Setdata.length() > 0)
			{
				Serial3.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		default : break;    
	}	
	delay(10);
}

void SetAICamLED(uint8_t _uart,uint8_t cmdLED)
{//设置补光灯 开/关
	LED=cmdLED;
	Setdata+=char(0x86);//0
	Setdata+=char(0xAB);//1
	Setdata+=char(0x14);//2
	Setdata+=char(0xE8);//3
	Setdata+=char(0x13);//4
	Setdata+=char(1);//5
	Setdata+=char(0);//6
	
	Setdata+=char(0);//7
	Setdata+=char(0);//8
	Setdata+=char(0);//9
	Setdata+=char(0);//10
	Setdata+=char(0);//11
	Setdata+=char(0);//12
	Setdata+=char(0);//13
	Setdata+=char(1);//14
	Setdata+=char(0);//15
	Setdata+=char(0);//16
	Setdata+=char(0);//17
	
	Setdata+=char(LED);//18
	Setdata+=char(0xCF);//19
	
	if(bluetoothFlag[_uart]==0) //初始化
	{
		bluetoothFlag[_uart]=1;
		switch(_uart)
		{
			case uart0 ://RX-0 TX-1
				Serial.begin(115200); //开串口接收
				break;
            
			case uart1 ://RX-19 TX-18 分别对应 编码m4 m3
				Serial1.begin(115200);//开串口接收
				break;
            
			case uart2 ://RX-17 TX-16
				Serial2.begin(115200);//开串口接收
				break;
            
			case uart3 ://RX-15 TX-14
				Serial3.begin(115200);//开串口接收
				break;
            
			default : break;
		}
	}
	switch(_uart)//发送数据
	{
		case uart0 :
			if (Setdata.length() > 0)
			{
				Serial.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart1 :
			if (Setdata.length() > 0)
			{
				Serial1.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart2 :
			if (Setdata.length() > 0)
			{
				Serial2.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		case uart3 :
			if (Setdata.length() > 0)
			{
				Serial3.println(Setdata);
				Setdata = "";
			}
			delay(10);break;

		default : break;    
	}		
}








