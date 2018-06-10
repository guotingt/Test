#ifndef PORT_H
#define PORT_H
#include "MyDataType.h"

/*working status*/
#define STOP_STA     0x01
#define FOREWARD_STA 0x02
#define BACKWARD_STA 0x03
#define CHECK_STA    0x04
#define MANUAL_STA   0x05

/*motion commmand*/
#define DO_STOP     0x01
#define DO_FOREWARD 0x02
#define DO_BACKWARD 0x03
#define DO_CHECK    0x04

/*speed mode*/
#define LOW_SPEED  0x01
#define HIGH_SPEED 0x02

/*pos flag*/
#define UP_POS 2
#define DOWN_POS 1
#define UNKONWN_POS 0

/*current threshold*/
#define CURRENT_THRESHOLD_1 1690  //75A
#define CURRENT_THRESHOLD_2 2027  //90A

#define CURRENTOVER_TIME 100
#define CURRENTOVERH_TIME 40
#define CLEARFAULTTIME 40000

extern void readSensor();

extern void sendMsg();

extern void sendTest();

extern void unPackMsg();

extern void unPackMsg2();

static Uint16 packMsg();

#endif
