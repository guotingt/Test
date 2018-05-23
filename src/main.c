#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"
#include "math.h"

#define CURRENT 0
#define SPEED 1
#define SPEED_CURVE 0

/*golbal value def*/
QUE_def uiQueue;///<Ui
QUE_def viQueue;///<Vi
QUE_def wiQueue;///<Wi
PID speedPID;
PID currentPID;
COMMAND upperCommand;

Uint16 reciveFlag = 0;
Uint16 keepCnt = 0;
Uint16 moveCnt = 0;
Uint16 timeOutFlag = 0;
Uint16 duty = 1;

void initPID()
{
	speedPID.outMax = 196608000;//%80
	speedPID.outMin = 327675;//%5
	speedPID.kp = 262144;//4
	speedPID.ki = 13107;//0.2

//	currentPID.outMax = 100*65536;//设置上限电流为1.5A  214748364U
//	currentPID.outMin = 0;//设置下限电流为0
//	currentPID.kp = 10000;
//	currentPID.ki = 10;
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
        SET_PWM_PERCENT(10);
        readSensor();
    }
}
int main()
{

    /*initialize DSP */
	dsp28335Init();

    /*initialize Controller*/
	initPID();

    readHall();

    SET_PWM_PERCENT(duty);

    //pwmUpdate();

    while (1) 
    {
    	/*read sensor data*/
    	if(0 != sensorReadSample)
    	{
    		sensorReadSample = 0;
    		//readSensor();
    		/*pid calc mode set*/
    		currentPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    		speedPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    	 }

#if SPEED
    	if(0 != speedLoopSample)
    	{
    		speedLoopSample = 0;
    		//speedPID.setPoint = 100;
    		speedPID.input = backData.speedCapture;
    		pidCalc(&speedPID);
    		duty = (Uint16)(speedPID.sumOut * 100/3750) ;
    		SET_PWM(3750-speedPID.sumOut);
    	}
#endif

#if SPEED_CURVE
        /*speed pid */
		if(0 != speedLoopSample)
		{
			speedLoopSample = 0;
			if(0 == backData.loadType)
			{
				if((FOREWARD_STA == backData.status)||(BACKWARD_STA == backData.status))
				{
					moveCnt++;
					if(0 == backData.motorDir)
					{
						if(moveCnt <= T8_T1)
						{
							if(KR_UP_MS * moveCnt < 100)
							{
								speedPID.setPoint = 100;
							}
							else
							{
								speedPID.setPoint =(Uint16)(KR_UP_MS * moveCnt);
							}
						}
						else if(moveCnt <= T8_T2)
						{

							speedPID.setPoint = NOMAL_RATE_UP;
						}
						else if(moveCnt <= T8)
						{
							if((NOMAL_RATE_UP - (Uint16)(KR_UP_MS * (moveCnt - T8_T2))) < 100)
							{
								speedPID.setPoint = 100;
							}
							else
							{
								speedPID.setPoint = NOMAL_RATE_UP - (Uint16)(KR_UP_MS * (moveCnt - T8_T2));
							}
						}
						else if(moveCnt == T8)
						{
							backData.status = STOP_STA;
						}
					}
					else
					{
						if(moveCnt <= T7_T1)
						{
							if(KR_DOWN_MS * moveCnt < 100)
							{
								speedPID.setPoint = 100;
							}
							else
							{
								speedPID.setPoint =(Uint16)(KR_DOWN_MS * moveCnt);
							}
						}
						else if(moveCnt <= T7_T2)
						{

							speedPID.setPoint = NOMAL_RATE_DOWN;
						}
						else if(moveCnt <= T7)
						{
							if((NOMAL_RATE_DOWN - (Uint16)(KR_DOWN_MS * (moveCnt - T7_T2))) < 100)
							{
								speedPID.setPoint = 100;
							}
							else
							{
								speedPID.setPoint = NOMAL_RATE_DOWN - (Uint16)(KR_DOWN_MS * (moveCnt - T7_T2));
							}
						}
						else
						{
							backData.status = STOP_STA;
						}

					}
					speedPID.input = backData.speedCapture;
					pidCalc(&speedPID);
					SET_PWM(3750 - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/3750);
				}
				else if(CHECK_STA == backData.status)
				{
					speedPID.setPoint = 100;
					speedPID.input = backData.speedCapture;
					pidCalc(&speedPID);
					SET_PWM(3750 - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/3750);
				}
			}
			else
			{
				speedPID.setPoint = 100;
				speedPID.input = backData.speedCapture;
				pidCalc(&speedPID);
				SET_PWM(3750 - speedPID.sumOut);
				duty = (Uint16)(speedPID.sumOut * 100/3750);
			}
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

        if (1 == reciveFlag)
        {
            //sendMsg();
        	//sendTest();
            reciveFlag = 0;
        }
        if(0 != testSendSample)
        {
        	sendTest();
        	testSendSample = 0;
        }
    }
}
