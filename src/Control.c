#include "Control.h"

void pidCalc(PID *pPID)
{
	pPID->err = pPID->setPoint - pPID->input;//�����������
	pPID->pOut = (pPID->kp * pPID->err);//��������������ֵ
	pPID->iOut += (pPID->ki * pPID->err);//������ֻ���ֵ
	if(pPID->pOut >= pPID->outMax)
	{
		pPID->pOut = pPID->outMax; //���ñ�����������
	}
	if(pPID->pOut <= pPID->outMin)
	{
		pPID->pOut = pPID->outMin; //���ñ�����������
	}
	if(pPID->iOut >= pPID->outMax)
	{
		pPID->iOut = pPID->outMax; //���û��ֻ�������
	}
	if(pPID->iOut <= pPID->outMin)
	{
		pPID->iOut = pPID->outMin; //���û��ֻ�������
	}
	pPID->sumOut = pPID->pOut + pPID->iOut;//����PID���

	if(pPID->sumOut >= pPID->outMax)
	{
		pPID->sumOut = pPID->outMax; //����PID�������
	}
	if(pPID->sumOut <= pPID->outMin)
	{
		pPID->sumOut = pPID->outMin; //����PID�������
	}
	pPID->sumOut >>= 16; //�������65536
}

void pidReset(PID *pPID)
{
	pPID->setPoint = 0;//�趨ֵ����
	pPID->err = 0;//�����������
	pPID->pOut = 0;//���������������
	pPID->iOut = 0;//���ֻ����������
	pPID->sumOut = 0;//PID�������
}
