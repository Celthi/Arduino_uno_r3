/**************************************************
*****浙江大学暑期实习——单片机项目代码************
*****负责人：卢宪廷，张敏*************************
*****单片机平台Arduino UNO R3*********************
*****传感器模块：US-100超声波模块
                  MMA7361加速度模块
                  TN901温度传感器模块
     蓝牙模块：  HC-06
具体电路连接请看电路图Lu_min
****************************************************/

//字符指令对应表
/***************************
*T代表测量温度temperature
*O代表判断方向orientation
*D代表测量距离distance
*R代表红灯亮red
*G代表绿灯亮green
*B代表蓝灯亮blue
*q代表红灯灭
*w代表绿灯灭
*e代表蓝灯灭


***************************/
char val;    //字符指令存储变量



//加速度传感器变量定义
int x,y,z;
#define YUEZHI 338/*0g对应的模拟输入端口读取的数值
                  * 量程选择1.5g，对应电压为1.65v
                  * 具体的知识详见MMA7361手册*/

//LED引脚定义
#define LEDGREEN 2 //绿灯对应的继电器所插的引脚
#define LEDRED   3 //红灯对应的继电器所插的引脚
#define LEDBLUE  4 //蓝灯对应的继电器所插的引脚


//超声波引脚和所用到变量定义
#define EchoPin  5  //超声波Echo/Rx所对应的引脚
#define TrigPin  6  //超声波Trig/Tx所对应的引脚
unsigned long Time_Echo_us = 0;   //脉冲从发出到返回的时间
unsigned long Len_mm       = 0;   //测得的距离变量，单位为毫米


//TN901引脚定义
#define   TestPin1    7    //TN_A
#define   ClockPin1   8   //TN_C
#define   DataPin1     9   //TN_D
int data_buf[5] ;         //TN901数据暂存数组
int tempData = 0;         //临时变量存储
int i,j,PinState;        // 数据引脚D高低指示变量
float realTemp=0;        //测量温度值



/*********************
*三个传感器函数定义
*meaDistance()    距离测量函数，measure distance
*meaTemperature() 温度测量函数，measure temperature
*meaOrientation() 方向判断函数，measure orientation
*********************/
void meaDistance();
void meaTemperature();
void meaOrientationX();

/*********************
*led灯控制函数
*********************/
void ledOn(char color); //点亮LED
void ledOff(char color);//熄灭LED

/*********************
*函数初始化
*********************/
void setup()
{
  Serial.begin(9600);   //设置波特率9600baud/s

  /*******************
  **各引脚输出输入设定
  *********************/
 //LED灯输出设定
  pinMode(LEDGREEN,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDBLUE,OUTPUT);
 //超声波输入输出引脚设定
  pinMode(EchoPin, INPUT);//
  pinMode(TrigPin, OUTPUT);
 //TN901温度传感器引脚设定
  pinMode(DataPin1,INPUT);      //TN_D
  pinMode(ClockPin1,INPUT);     //TN_C
  pinMode(TestPin1,OUTPUT);     //TN_A
  digitalWrite(TestPin1, HIGH); //TN_A=1置1
}
/**************************************
*主函数，蓝牙模块等待指令的接收******
*接收到指令后执行相应的动作（函数）**
***************************************/
void loop()
{

  while(Serial.available()>0)// 判断蓝牙是否接收到指令
  {
      val=Serial.read();     //接收到指令
      switch (val)           //执行指令
          {
              //详细指令对照表请看程序开头
              case 'T':{meaTemperature();break;}
              case 'O':{meaOrientationX();break;}
              case 'D':{meaDistance();break;}

              case 'R':{ledOn(LEDRED);break;}
              case 'G':{ledOn(LEDGREEN);break;}
              case 'B':{ledOn(LEDBLUE);break;}

              case 'q':{ledOff(LEDRED);break;}
              case 'w':{ledOff(LEDGREEN);break;}
              case 'e':{ledOff(LEDBLUE);break;}

              default:NULL;
          }
      val=0;   //指令清零，并在下一步进入等待指令状态
  }

}
/*******************************
*meaDistance()
*距离测量函数，measure distance
*******************************/
void meaDistance()
{

//发送一个高脉冲，宽度为50us
digitalWrite(TrigPin, HIGH); //
delayMicroseconds(50);       //
digitalWrite(TrigPin, LOW);  //

//脉冲返回时间
Time_Echo_us = pulseIn(EchoPin, HIGH); //
if((Time_Echo_us < 60000) && (Time_Echo_us > 1))//
{
    Len_mm = (Time_Echo_us*34/100)/2;   //计算测量得到的距离，

    //手机显示测量得到的距离
    Serial.print("Present Distance is: ");
    Serial.print(Len_mm, DEC);
    Serial.println("mm");
 }

}
/***************************************
*meaTemperature() measure temperature
*物体温度测量函数， TN901模块
***************************************/
void meaTemperature()
{
    digitalWrite(TestPin1, LOW);  //spi总线低选通
      //模拟SPI时序读取数据，
      //如果感觉难以理解，可以借助TN901的时钟序列来理解程序
    for(i=0;i<5;i++)         //一共读取5个字节
    {
        for(j=0;j<8;j++)          //每个字节8位
        {
            do{
            PinState = digitalRead(ClockPin1);    //读取时钟指示
        }while(PinState);                         //高电平等待
        delayMicroseconds(100);                     //延长100us
        PinState = digitalRead(DataPin1);           //
        if(1 == PinState)                          //读到1电平
             tempData = (tempData<<1 & 0xfe)+1;    //
        else
             tempData = (tempData<<1 & 0xfe);      // 读到 0电平
        do{
            PinState = digitalRead(ClockPin1);      //等待时钟低电平变高电平
          }while(PinState != 1);
        }
        data_buf[i] = tempData;      //存储读到的数据
    }

    digitalWrite(TestPin1, HIGH);   //置高，关闭使能端
    //抽取第一、第二字节，计算得到物体实际温度
    //本来乘以256等价于左移8位，但是程序调试时左移八位
    //没有得到正确的结果
    realTemp = (data_buf[1]*256  + data_buf[2])/16.0 -273.15;
    Serial.print("Temp=");         // 手机显示测得的温度
    Serial.println(realTemp);      //

}
/******************************************
*meaOrientation()
*方向判断函数，measure orientation
********************************************/
void meaOrientationX()
{
  x=analogRead(0);//读取x轴方向电压AD数值
  y=analogRead(1);
  z=analogRead(2);

  if(x<YUEZHI)   //判断x轴的倾倒方向，阈值的大小请参看MMA7361数据手册
    Serial.print("You are going X positive position.\n");
    else Serial.print("You are going X negative position.\n");
    //以下注释程序的保留是为了得到y、z方向的感应而保留
    //如果有需要，直接把注释去掉即可。
/*  if(y<YUEZHI)
    Serial.print("You are going Y positive position.\n");
    else Serial.print("you are going Y negative position.\n");
  if(z<YUEZHI)
    Serial.print("You are going Z positive position.\n");
    else Serial.print("You are going Z negative position.\n");
        */

}
/*************************
*LED灯点亮函数
*传入参数代表LED灯颜色的选择
*  2代表绿色
*  3代表红色
*  4代表蓝色
***************************/
void ledOn(char color)
{
    digitalWrite(color,HIGH);
    switch (color)
        {
            case LEDGREEN:{Serial.println("GREEN LED ON!");break;}
            case LEDRED  :{Serial.println("RED LED ON!");  break;}
            case LEDBLUE :{Serial.println("BLUE LED ON!"); break;}
        }


}
/*********************************
*熄灭LED函数
*传入参数代表LED灯颜色的选择
*  2代表绿色
*  3代表红色
*  4代表蓝色
*********************************/
void ledOff(char color)
{
    digitalWrite(color,LOW);
    switch (color)
        {
            case LEDGREEN:{Serial.println("GREEN LED OFF!");break;}
            case LEDRED  :{Serial.println("RED LED OFF!");  break;}
            case LEDBLUE :{Serial.println("BLUE LED OFF!"); break;}
        }


}
