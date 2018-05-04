#include "Control.h"

void pidCalc(PID *pPID)
{
	if(MANUAL == pPID->mode)
	{
		return;
	}
	else
	{
		pPID->err = pPID->setPoint - pPID->input;
		pPID->pOut = pPID->kp * pPID->err;//����Upֵ
		pPID->iOut +=  pPID->ki * pPID->err;//����Uiֵ
		if(pPID->pOut >= pPID->outMax)
		{
			pPID->pOut = pPID->outMax; //����Up����
		}
		if(pPID->pOut <= pPID->outMin)
		{
			pPID->pOut = pPID->outMin; //����Up����
		}
		if(pPID->iOut >= pPID->outMax)
		{
			pPID->iOut = pPID->outMax; //����Ui����
		}
		if(pPID->iOut <= pPID->outMin)
		{
			pPID->iOut = pPID->outMin; //����Ui����
		}
		pPID->sumOut = pPID->pOut + pPID->iOut;//����PID���

		if(pPID->sumOut >= pPID->outMax)
		{
			pPID->sumOut = pPID->outMax; //�����������
		}
		if(pPID->sumOut <= pPID->outMin)
		{
			pPID->sumOut = pPID->outMin; //�����������
		}
		pPID->sumOut >>= 16; //�������65536
	}
}

