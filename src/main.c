#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"

#define OPENLOOP 1
#define CURRENT 0
#define SPEED 0

#define PWM_CHECK 6750

/*golbal value def*/
QUE_def uiQueue;///<Ui
QUE_def viQueue;///<Vi
QUE_def wiQueue;///<Wi
PID speedPID;
PID currentPID;
COMMAND upperCommand;

Uint16 reciveFlag = 0;
Uint16 posFlag = 0;
Uint16 keepCnt = 0;

void initPID()
{
	speedPID.outMax = 500*65536;
	speedPID.outMin = 0;
	speedPID.kp = 10000;//42000  39000  38000
	speedPID.ki = 20;//675      300     250

	currentPID.outMax = 100*65536;//设置上限电流为1.5A  214748364U
	currentPID.outMin = 0;//设置下限电流为0
	currentPID.kp = 10000;
	currentPID.ki = 10;
}
Uint16 speedRamp(Uint16 speed,Uint16 stepT,Uint16 stepL,Uint16* pKeepCnt)
{
	Uint16 rampSpeed = 0;
	if(speed > backData.speed)
	{
		if(0 == (*pKeepCnt)%stepT)
		{
			rampSpeed += stepL;
		}
		if(rampSpeed >= speed)
		{
			rampSpeed = speed;
			*pKeepCnt = 0;
		}
		else
		{
			(*pKeepCnt)++;
		}
	}
	else
	{
		if(0 == (*pKeepCnt)%stepT)
		{
			rampSpeed -= stepL;
		}
		if(rampSpeed <= speed)
		{
			rampSpeed = speed;
			*pKeepCnt = 0;
		}
		else
		{
			(*pKeepCnt)++;
		}
	}
	return rampSpeed;
}
/*selfCheck*/
void selfCheck()
{
    if (MANUAL_STA == backData.status)
    {
        return;
    }
    else
    {
        SET_PWM(PWM_CHECK);
        readSensor();
    }
}
int main()
{

    /*initialize DSP */
	dsp28335Init();

    /*initialize Controller*/
	initPID();

//    /*read sensor data*/
//    readSensor();
//   /*self check*/
//    while(2 != posFlag)
//    {
//    	backData.status = CHECK_STA;
//    	motorDir = 0;
//    	readSensor();
//    	selfCheck();
//    	if(0 != pwmUpdateSample)
//		{
//			pwmUpdateSample = 0;
//			if(STOP_STA != backData.status)
//			{
//				pwmUpdate();
//			}
//		}
//    }
//    /*send point 1*/
//    sendMsg();
    while (1) 
    {
    	/*read sensor data*/
    	if(0 != sensorReadSample)
    	{
    		sensorReadSample = 0;
    		readSensor();
    		currentPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    		speedPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    	}
        /*check faultCode*/
        if (0x00 != (backData.faultCode&0x3F))
        {
        	PWM_DISABLE;
            //sendMsg(sendBuf,&backData);
        }
        else
        {
        	/*换相*/
        	if(0 != pwmUpdateSample)
			{
				pwmUpdateSample = 0;
				if((STOP_STA != backData.status) && (MANUAL_STA != backData.status))
				{
					pwmUpdate();
				}
			}
#if OPENLOOP
        	SET_PWM(speedPID.sumOut);
#endif
#if SPEED
            /*speed pid */
            if(0 != speedLoopSample)
            {
            	speedLoopSample = 0;
            	speedPID.setPoint = speedRamp(500,10,50,&keepCnt);
            	speedPID.input = backData.speed;
            	pidCalc(&speedPID);
            	SET_PWM(speedPID.sumOut);
            }
#endif
#if CURRENT
            /*current pid */
            if(0 != currentLoopSample)
            {
                currentPID.setPoint = speedPID.sumOut;
                currentPID.input = backData.current;
            	currentLoopSample = 0;
            	pidCalc(&currentPID);
            	//SET_PWM(currentPID.sumOut);
            }
#endif
        }
        if (1 == reciveFlag)
        {
            //sendMsg();
        	sendTest();
            reciveFlag = 0;
        }
        if(0 != testSendSample)
        {
        	sendTest();
        	testSendSample = 0;
        }
    }
}
