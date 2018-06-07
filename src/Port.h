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

/*current threshold*/
#define CURRENT_THRESHOLD_1 676 //30A
#define CURRENT_THRESHOLD_2 901 //40A
#define CURRENT_THRESHOLD_3 1500  //66.77A
#define CURRENT_THRESHOLD_4 2100  //93.21A

#define CURRENTOVER_TIME 10
#define CURRENTOVERH_TIME 4
#define CLEARFAULTTIME 4000

extern void readSensor();

extern void sendMsg();

extern void sendTest();

extern void unPackMsg();

extern void unPackMsg2();

static Uint16 packMsg();

#endif
