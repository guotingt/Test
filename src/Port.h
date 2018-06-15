#ifndef PORT_H
#define PORT_H
#include "MyDataType.h"

/*working status*/
#define STOP_STA     0x01 ///<ֹͣ״̬
#define FOREWARD_STA 0x02 ///<�������߿���״̬
#define BACKWARD_STA 0x03 ///<�½����߹���״̬
#define CHECK_STA    0x04 ///<Ѱ��״̬

/*motion commmand*/
#define DO_STOP     0x01 ///<ֹͣ
#define DO_FOREWARD 0x02 ///<�������߿���
#define DO_BACKWARD 0x03 ///<�½����߹���
#define DO_CHECK    0x04 ///<Ѱ��

/*speed mode*/
#define LOW_SPEED  0x01 ///<����
#define HIGH_SPEED 0x02 ///<����

/*pos flag*/
#define UP_POS 2       ///<����λ
#define DOWN_POS 1     ///<����λ
#define UNKONWN_POS 0  ///<�м�λ��

#define CLEARFAULTTIME 4000 ///<���������־���ʱ��

/**
 * @brief �������־
 */
extern void readSensor();
/**
 * @brief ������Ϣ
 */
extern void sendMsg();
/***
 * @brief ���Ͳ�����Ϣ
 */
extern void sendTest();
/***
 * @brief ��λ����Ϣ����
 */
extern void unPackMsg();
/***
 * @brief ������Ϣ����
 */
extern void unPackMsg2();
/**
 * @brief �����Ϣ
 */
static Uint16 packMsg();

#endif
