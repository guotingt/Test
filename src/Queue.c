#include "Queue.h"

void que_push(QUE_def* SQue,Uint16 d,Uint16 size)
{
	if(SQue->s_size == size) return;
	SQue->s_data[(SQue->s_front+SQue->s_size) % size] = d;
	SQue->s_size++;
}

void que_pop(QUE_def* SQue,Uint16 size)
{
	if(SQue->s_size == 0) return;
	if(SQue->s_front >= size) SQue->s_front %= size;
	SQue->s_front++;
	SQue->s_size--;
}
Uint16 que_fil(QUE_def* SQue,Uint16 size)
{
	Uint16 i;
	Uint32 temp = 0;
	for(i=0; i<SQue->s_size; i++)
	{
		temp += SQue->s_data[(SQue->s_front + i) % size];
	}
	temp /= SQue->s_size;
	return (Uint16)temp;
}

void FilterPara_ini(QUE_def* SQue)
{
	SQue->s_size = 0;
	SQue->s_front = 0;
	SQue->s_mean	= 0;
}

Uint16 FILTERcon(QUE_def* SQue,Uint16 d,Uint16 size)
{
	que_push(SQue,d,size);
	if(SQue->s_size >= size)
	{
		que_pop(SQue,size);
	}
	SQue->s_mean = que_fil(SQue,size);
	return SQue->s_mean;
}


