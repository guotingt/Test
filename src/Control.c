#include "Control.h"

void pidCalc(PID *pPID)
{
	pPID->err = pPID->setPoint - pPID->input;//计算最新误差
	pPID->pOut = (pPID->kp * pPID->err);//计算比例环节输出值
	pPID->iOut += (pPID->ki * pPID->err);//计算积分环节值
	if(pPID->pOut >= pPID->outMax)
	{
		pPID->pOut = pPID->outMax; //设置比例环节上限
	}
	if(pPID->pOut <= pPID->outMin)
	{
		pPID->pOut = pPID->outMin; //设置比例环节下限
	}
	if(pPID->iOut >= pPID->outMax)
	{
		pPID->iOut = pPID->outMax; //设置积分环节上限
	}
	if(pPID->iOut <= pPID->outMin)
	{
		pPID->iOut = pPID->outMin; //设置积分环节下限
	}
	pPID->sumOut = pPID->pOut + pPID->iOut;//计算PID输出

	if(pPID->sumOut >= pPID->outMax)
	{
		pPID->sumOut = pPID->outMax; //设置PID输出上限
	}
	if(pPID->sumOut <= pPID->outMin)
	{
		pPID->sumOut = pPID->outMin; //设置PID输出下限
	}
	pPID->sumOut >>= 16; //结果除以65536
}

void pidReset(PID *pPID)
{
	pPID->setPoint = 0;//设定值清零
	pPID->err = 0;//最新误差清零
	pPID->pOut = 0;//比例环节输出清零
	pPID->iOut = 0;//积分环节输出清零
	pPID->sumOut = 0;//PID输出清零
}
