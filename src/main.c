#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"

/*out max*/
#define OUT_MIN_CURRENT -100
#define OUT_MAX_CURRENT 100
#define OUT_MIN_SPEED -100
#define OUT_MAX_SPEED 100
#define PWM_CHECK 10
/*sample time*/
#define CURRENT_SAMPLE 10
#define SPEED_SAMPLE 100

/*golbal value def*/
PID speedPID;
PID currentPID;
COMMAND upperCommand;
Uint16 reciveFlag = 0;

QUE_def uiQueue;///<Ui
QUE_def viQueue;///<Vi
QUE_def wiQueue;///<Wi

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
///*selfCheck*/
//void selfCheck()
//{
//    if (MANUAL_STA == backData.status)
//    {
//        return;
//    }
//    else
//    {
//        SET_PWM(PWM_CHECK);
//        readSensor();
//    }
//}
int main()
{

    /*initialize DSP */
	dsp28335Init();

    /*initialize Controller*/
	initPID();

    /*read sensor data*/
    readSensor();

//    /*self check*/
//    selfCheck();

    /*send point 1*/
    sendMsg();

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
//        if (0x00 != (backData.faultCode&0x3F))
//        {
//        	PWM_DISABLE;
//            sendMsg(sendBuf,&backData);
//        }
        else
        {
        	/*换相*/
        	if(0 != pwmUpdateSample)
			{
				pwmUpdateSample = 0;
				//if((STOP_STA != backData.status) && (MANUAL_STA != backData.status))
				{
					pwmUpdate();
				}
			}
            /*speed pid */
            if(0 != speedLoopSample)
            {
            	speedLoopSample = 0;
            	speedPID.setPoint = speedRamp(500,10,50,&keepCnt);
            	speedPID.input = backData.speed;
            	pidCalc(&speedPID);
            }
            /*current pid */
            if(0 != currentLoopSample)
            {
                currentPID.setPoint = speedPID.sumOut;
                currentPID.input = backData.current;
            	currentLoopSample = 0;
            	pidCalc(&currentPID);
            	//SET_PWM(currentPID.sumOut);
            }
        }
        if (1 == reciveFlag)
        {
            sendMsg(sendBuf,&backData);
            reciveFlag = 0;
        }
        if(0 != testSendSample)
        {
        	sendTest();
        	testSendSample = 0;
        }
    }
}
