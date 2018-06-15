#ifndef PORT_H
#define PORT_H
#include "MyDataType.h"

/*working status*/
#define STOP_STA     0x01 ///<停止状态
#define FOREWARD_STA 0x02 ///<上升或者开门状态
#define BACKWARD_STA 0x03 ///<下降或者关门状态
#define CHECK_STA    0x04 ///<寻零状态

/*motion commmand*/
#define DO_STOP     0x01 ///<停止
#define DO_FOREWARD 0x02 ///<上升或者开门
#define DO_BACKWARD 0x03 ///<下降或者关门
#define DO_CHECK    0x04 ///<寻零

/*speed mode*/
#define LOW_SPEED  0x01 ///<低速
#define HIGH_SPEED 0x02 ///<高速

/*pos flag*/
#define UP_POS 2       ///<上限位
#define DOWN_POS 1     ///<下限位
#define UNKONWN_POS 0  ///<中间位置

#define CLEARFAULTTIME 4000 ///<电流错误标志清除时间

/**
 * @brief 检查错误标志
 */
extern void readSensor();
/**
 * @brief 发送信息
 */
extern void sendMsg();
/***
 * @brief 发送测试信息
 */
extern void sendTest();
/***
 * @brief 上位机信息解码
 */
extern void unPackMsg();
/***
 * @brief 测试信息解码
 */
extern void unPackMsg2();
/**
 * @brief 打包信息
 */
static Uint16 packMsg();

#endif
