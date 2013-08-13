/**************************************************
*****�㽭��ѧ����ʵϰ������Ƭ����Ŀ����************
*****�����ˣ�¬��͢������*************************
*****��Ƭ��ƽ̨Arduino UNO R3*********************
*****������ģ�飺US-100������ģ��
                  MMA7361���ٶ�ģ��
                  TN901�¶ȴ�����ģ��
     ����ģ�飺  HC-06
�����·�����뿴��·ͼLu_min
****************************************************/

//�ַ�ָ���Ӧ��
/***************************
*T��������¶�temperature
*O�����жϷ���orientation
*D�����������distance
*R��������red
*G�����̵���green
*B����������blue
*q��������
*w�����̵���
*e����������


***************************/
char val;    //�ַ�ָ��洢����



//���ٶȴ�������������
int x,y,z;
#define YUEZHI 338/*0g��Ӧ��ģ������˿ڶ�ȡ����ֵ
                  * ����ѡ��1.5g����Ӧ��ѹΪ1.65v
                  * �����֪ʶ���MMA7361�ֲ�*/

//LED���Ŷ���
#define LEDGREEN 2 //�̵ƶ�Ӧ�ļ̵������������
#define LEDRED   3 //��ƶ�Ӧ�ļ̵������������
#define LEDBLUE  4 //���ƶ�Ӧ�ļ̵������������


//���������ź����õ���������
#define EchoPin  5  //������Echo/Rx����Ӧ������
#define TrigPin  6  //������Trig/Tx����Ӧ������
unsigned long Time_Echo_us = 0;   //����ӷ��������ص�ʱ��
unsigned long Len_mm       = 0;   //��õľ����������λΪ����


//TN901���Ŷ���
#define   TestPin1    7    //TN_A
#define   ClockPin1   8   //TN_C
#define   DataPin1     9   //TN_D
int data_buf[5] ;         //TN901�����ݴ�����
int tempData = 0;         //��ʱ�����洢
int i,j,PinState;        // ��������D�ߵ�ָʾ����
float realTemp=0;        //�����¶�ֵ



/*********************
*������������������
*meaDistance()    �������������measure distance
*meaTemperature() �¶Ȳ���������measure temperature
*meaOrientation() �����жϺ�����measure orientation
*********************/
void meaDistance();
void meaTemperature();
void meaOrientationX();

/*********************
*led�ƿ��ƺ���
*********************/
void ledOn(char color); //����LED
void ledOff(char color);//Ϩ��LED

/*********************
*������ʼ��
*********************/
void setup()
{
  Serial.begin(9600);   //���ò�����9600baud/s

  /*******************
  **��������������趨
  *********************/
 //LED������趨
  pinMode(LEDGREEN,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDBLUE,OUTPUT);
 //������������������趨
  pinMode(EchoPin, INPUT);//
  pinMode(TrigPin, OUTPUT);
 //TN901�¶ȴ����������趨
  pinMode(DataPin1,INPUT);      //TN_D
  pinMode(ClockPin1,INPUT);     //TN_C
  pinMode(TestPin1,OUTPUT);     //TN_A
  digitalWrite(TestPin1, HIGH); //TN_A=1��1
}
/**************************************
*������������ģ��ȴ�ָ��Ľ���******
*���յ�ָ���ִ����Ӧ�Ķ�����������**
***************************************/
void loop()
{

  while(Serial.available()>0)// �ж������Ƿ���յ�ָ��
  {
      val=Serial.read();     //���յ�ָ��
      switch (val)           //ִ��ָ��
          {
              //��ϸָ����ձ��뿴����ͷ
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
      val=0;   //ָ�����㣬������һ������ȴ�ָ��״̬
  }

}
/*******************************
*meaDistance()
*�������������measure distance
*******************************/
void meaDistance()
{

//����һ�������壬���Ϊ50us
digitalWrite(TrigPin, HIGH); //
delayMicroseconds(50);       //
digitalWrite(TrigPin, LOW);  //

//���巵��ʱ��
Time_Echo_us = pulseIn(EchoPin, HIGH); //
if((Time_Echo_us < 60000) && (Time_Echo_us > 1))//
{
    Len_mm = (Time_Echo_us*34/100)/2;   //��������õ��ľ��룬

    //�ֻ���ʾ�����õ��ľ���
    Serial.print("Present Distance is: ");
    Serial.print(Len_mm, DEC);
    Serial.println("mm");
 }

}
/***************************************
*meaTemperature() measure temperature
*�����¶Ȳ��������� TN901ģ��
***************************************/
void meaTemperature()
{
    digitalWrite(TestPin1, LOW);  //spi���ߵ�ѡͨ
      //ģ��SPIʱ���ȡ���ݣ�
      //����о�������⣬���Խ���TN901��ʱ��������������
    for(i=0;i<5;i++)         //һ����ȡ5���ֽ�
    {
        for(j=0;j<8;j++)          //ÿ���ֽ�8λ
        {
            do{
            PinState = digitalRead(ClockPin1);    //��ȡʱ��ָʾ
        }while(PinState);                         //�ߵ�ƽ�ȴ�
        delayMicroseconds(100);                     //�ӳ�100us
        PinState = digitalRead(DataPin1);           //
        if(1 == PinState)                          //����1��ƽ
             tempData = (tempData<<1 & 0xfe)+1;    //
        else
             tempData = (tempData<<1 & 0xfe);      // ���� 0��ƽ
        do{
            PinState = digitalRead(ClockPin1);      //�ȴ�ʱ�ӵ͵�ƽ��ߵ�ƽ
          }while(PinState != 1);
        }
        data_buf[i] = tempData;      //�洢����������
    }

    digitalWrite(TestPin1, HIGH);   //�øߣ��ر�ʹ�ܶ�
    //��ȡ��һ���ڶ��ֽڣ�����õ�����ʵ���¶�
    //��������256�ȼ�������8λ�����ǳ������ʱ���ư�λ
    //û�еõ���ȷ�Ľ��
    realTemp = (data_buf[1]*256  + data_buf[2])/16.0 -273.15;
    Serial.print("Temp=");         // �ֻ���ʾ��õ��¶�
    Serial.println(realTemp);      //

}
/******************************************
*meaOrientation()
*�����жϺ�����measure orientation
********************************************/
void meaOrientationX()
{
  x=analogRead(0);//��ȡx�᷽���ѹAD��ֵ
  y=analogRead(1);
  z=analogRead(2);

  if(x<YUEZHI)   //�ж�x����㵹������ֵ�Ĵ�С��ο�MMA7361�����ֲ�
    Serial.print("You are going X positive position.\n");
    else Serial.print("You are going X negative position.\n");
    //����ע�ͳ���ı�����Ϊ�˵õ�y��z����ĸ�Ӧ������
    //�������Ҫ��ֱ�Ӱ�ע��ȥ�����ɡ�
/*  if(y<YUEZHI)
    Serial.print("You are going Y positive position.\n");
    else Serial.print("you are going Y negative position.\n");
  if(z<YUEZHI)
    Serial.print("You are going Z positive position.\n");
    else Serial.print("You are going Z negative position.\n");
        */

}
/*************************
*LED�Ƶ�������
*�����������LED����ɫ��ѡ��
*  2������ɫ
*  3�����ɫ
*  4������ɫ
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
*Ϩ��LED����
*�����������LED����ɫ��ѡ��
*  2������ɫ
*  3�����ɫ
*  4������ɫ
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
