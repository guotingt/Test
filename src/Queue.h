#ifndef QUEUE_H
#define QUEUE_H
#include "MyDataType.h"

static void que_push(QUE_def* SQue,Uint16 d,Uint16 size);
static void que_pop(QUE_def* SQue,Uint16 size);
static Uint16 que_fil(QUE_def* SQue,Uint16 size);

extern void FilterPara_ini(QUE_def* SQue);
extern Uint16 FILTERcon(QUE_def* SQue,Uint16 d,Uint16 size);

#endif
