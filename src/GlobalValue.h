#ifndef GLOBAL_H
#define GLOBAL_H
#include "MyDataType.h"

/*motor ����*/
#define FOREWARD 0 ///<������
#define BACKWARD 1 ///<������

/*Time Interrupt*/
extern TIME_FLAG  flagDot1ms;
#define flagDot1msW flagDot1ms.word

extern TIME_FLAG  flag1ms;
#define flag1msW flag1ms.word
#define currentLoopSample flag1ms.bit.bit0 ///<������
#define speedLoopSample flag1ms.bit.bit1   ///<�ٶȻ�

extern TIME_FLAG  flag10ms;
#define flag10msW flag10ms.word
#define testSendSample flag10ms.bit.bit0  ///<��ʱ���͵�����Ϣ

extern TIME_FLAG  flag100ms;
#define flag100msW flag100ms.word

extern TIME_FLAG  flag500ms;
#define flag500msW flag500ms.word

extern TIME_FLAG  flag1000ms;
#define flag1000msW flag1000ms.word

/*PID*/
extern PID speedPID;       ///<speed_controller
extern PID currentPID;     ///<current_controller

/*Sensor*/
extern BACK_DATA backData; ///<sensor_message

/*Port*/
extern COMMAND upperCommand; ///<��λ��ָ��
extern Uint16 reciveBuf[];   ///<���ջ���
extern Uint16 reciveFlag;    ///<����flag
extern Uint16 moveCnt;       ///<�˶���ʱ
extern Uint16 duty;          ///<ռ�ձ�
#endif

