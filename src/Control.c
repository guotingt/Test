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
		pPID->pOut = pPID->kp * pPID->err;//计算Up值
		pPID->iOut +=  pPID->ki * pPID->err;//计算Ui值
		if(pPID->pOut >= pPID->outMax)
		{
			pPID->pOut = pPID->outMax; //设置Up上限
		}
		if(pPID->pOut <= pPID->outMin)
		{
			pPID->pOut = pPID->outMin; //设置Up下限
		}
		if(pPID->iOut >= pPID->outMax)
		{
			pPID->iOut = pPID->outMax; //设置Ui上限
		}
		if(pPID->iOut <= pPID->outMin)
		{
			pPID->iOut = pPID->outMin; //设置Ui下限
		}
		pPID->sumOut = pPID->pOut + pPID->iOut;//计算PID输出

		if(pPID->sumOut >= pPID->outMax)
		{
			pPID->sumOut = pPID->outMax; //设置输出上限
		}
		if(pPID->sumOut <= pPID->outMin)
		{
			pPID->sumOut = pPID->outMin; //设置输出下限
		}
		pPID->sumOut >>= 16; //结果除以65536
	}
}

