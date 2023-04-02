#include <JMDlib.h>

#define SPEED_X 2.1
#define position_max 400   //编码累积最大值 
#define filter 0.3      //速度互补滤波系数

//PID参数*******************
float Kap = 50*0.01;//默认平衡参数 
float Kad = 35*0.1;
float Ksp = 85*0.1;
float Ksi = 75*0.001;

int angle=0;
int w_angle=0;
int angle_last=0;
int count_l=0;
int count_r=0;
int xerr=0;

float speed_add=0;
float speeds=0;
float _position=0;

int looptime=20;
int Tyao=0;
int positionup=0;
int turn=0;

void setBlance()//设置平衡点
{
    setRGB(BLUE);//亮蓝灯    
    setBeep(ON);
    setDelay_ms(500);
    setBeep(OFF);
    setDelay_ms(500);
    setAHRSPin(I1, 1);//校准中
    setRGB(GREEN);//亮绿灯
    setBeep(ON);
    setDelay_ms(500);
    setBeep(OFF);
    setDelay_ms(100);
    setBeep(ON);
    setDelay_ms(100);
    setBeep(OFF);
    setRGB(CLOSE);
       
    speed_add=0;
    speeds=0;
    _position=0; 
}

void motorPID()//PID平衡函数
{  
    float Pwm_out=0;  
    
    count_l=getCodePin(m1);//计算左右马达编码
    count_r=-getCodePin(m4);
    speeds=(count_l+count_r)*0.5;//计算平均速度 
    
    setClearCodePin(m1);//清零编码盘 使每个周期的马达重新计数
    setClearCodePin(m4);
    count_l=getCodePin(m1);//清零左右马达编码
    count_r=getCodePin(m4);
  
    speed_add=(1-filter)*speed_add+filter*speeds;//互补滤波 计算出当前速度
    _position=_position+speed_add;//通过速度积分 计算出位置累积量
    _position=_position+positionup;

    if(_position>position_max) _position=position_max;  //限定最大最小值
    if(_position<-position_max) _position=-position_max;
  
    angle=getAHRSPin(I1, 1, 0);  //获取传感器参数
    angle=angle+xerr;         //自动修正零点
    w_angle=angle-angle_last; //角速度
    angle_last=angle;

    Pwm_out = (Kap*angle+Kad*w_angle) ;       //角度环
    Pwm_out+= (Ksp*speed_add + Ksi*_position);//速度环
  
    setMotorPin(M1,Pwm_out*SPEED_X+turn);
    setMotorPin(M4,-Pwm_out*SPEED_X+turn);

    if(getAHRSPin(I1, 1, 0)>9000);
    
//    if(angle<10 && angle>-10)setRGB(CLOSE);
//    else setRGB(CLOSE);

    while(angle>450||angle<-450)
    {   
        setMotorPin(M1,0);
        setMotorPin(M4,0); 
        setDelay_ms(50);
        angle=getAHRSPin(I1, 1, 1);//重读传感器
    }  
}

void yaokong()//遥控函数
{
    if(getSystemTime_ms()-Tyao<1500)return ;
    
    turn=getRFModuleRemoteRockerPin(I2, 0, 0); 
    positionup=getRFModuleRemoteRockerPin(I2, 1, 1); 
    
    if(turn<101&&turn>-101)
    {
        turn=-turn*0.8;//转向速度
    }
    else
    {
        turn=0; 
    }
    if(positionup<101&&positionup>-101)
    {
        positionup=positionup*8;//前进速度
    }
    else
    {
        positionup=0;
    }       
}
void setup() {  
    long TT=0;
    Tyao=getSystemTime_ms();  //开始计数
    if(getKey()==0)  //检测是否初始平衡点--0：初始化/1：跳过
    {
        setBlance();
    }
    while(1)
    {
        motorPID();
        
        if(getSystemTime_ms()-TT>looptime)
        {
          yaokong();
          TT=getSystemTime_ms();
        }                            
    }
}

void loop() {
     
}
