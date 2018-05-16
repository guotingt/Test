#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"

#define OPENLOOP 1
#define CURRENT 0
#define SPEED 0

#define PWM_CHECK_PERC 10
#define NOMAL_RATE_DOWN 30000
#define NOMAL_RATE_UP 24000
#define T8 8000
#define T8_T1 1700
#define T8_T2 6300
#define T7 7000
#define T7_T1  700
#define T7_T2  6300
#define T6 6000
#define KR_UP_MS 17.64
#define KR_DOWN_MS 42.84
#define LOAD_TEST_V 30000/8
#define BIG_LOAD_I 40

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
    		//readSensor();
    		//readHall();
    		/*pid calc mode set*/
    		currentPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    		speedPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    		 /*check faultCode*/
//    		if(STOP_STA == backData.status || 0x0000 != (backData.faultCode&0x003F))
//    		{
//    			PWM_DISABLE;
//    		}
    		/*load test*/
//    		if((backData.speedCapture - LOAD_TEST_V) <= LOAD_TEST_V/10)
//    		{
//    			if((abs(backData.current) - BIG_LOAD_I) < BIG_LOAD_I/10)
//				{
//    				backData.loadType = 1;
//				}
//    		}
    	 }

//        /*check faultCode*/
//        if (0x0000 != (backData.faultCode&0x003F))
//        {
//        	PWM_DISABLE;
//            sendMsg(sendBuf,&backData);
//        }
//        else
//        {
//        	/*换相*/
//        	if(0 != pwmUpdateSample)
//			{
//				pwmUpdateSample = 0;
//				if((STOP_STA != backData.status) && (MANUAL_STA != backData.status))
//				{
//					pwmUpdate();
//				}
//			}

#if OPENLOOP
    	SET_PWM_PERCENT(10);
#endif

#if SPEED
    	speedPID.setPoint = (Uint16)(NOMAL_RATE_UP / 8);
    	speedPID.input = backData.speedCapture;
		pidCalc(&speedPID);
		SET_PWM(speedPID.sumOut);
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
					if(1 == backData.motorDir)
					{
						if(moveCnt <= T8_T1)
						{
							speedPID.setPoint =(Uint16)(KR_UP_MS * moveCnt);
						}
						else if(moveCnt <= T8_T2)
						{

							speedPID.setPoint = NOMAL_RATE_UP;
						}
						else if(moveCnt <= T8)
						{
							speedPID.setPoint = NOMAL_RATE_UP - (Uint16)(KR_UP_MS * (moveCnt - T8_T2));
						}
						else
						{
							speedPID.setPoint = (Uint16)(NOMAL_RATE_UP / 8);
							timeOutFlag |= 0x0001;
						}
					}
					else
					{
						if(moveCnt <= T7_T1)
						{
							speedPID.setPoint =(Uint16)(KR_DOWN_MS * moveCnt);
						}
						else if(moveCnt <= T7_T2)
						{

							speedPID.setPoint = NOMAL_RATE_DOWN;
						}
						else if(moveCnt <= T7)
						{
							speedPID.setPoint = NOMAL_RATE_DOWN - (Uint16)(KR_DOWN_MS * (moveCnt - T7_T2));
						}
						else
						{
							speedPID.setPoint = (Uint16)(NOMAL_RATE_DOWN / 8);
							timeOutFlag |= 0x0001;
						}

					}
					//speedPID.setPoint = speedRamp(500,10,50,&keepCnt);
					speedPID.input = backData.speedCapture;
					pidCalc(&speedPID);
					SET_PWM(speedPID.sumOut);
				}
				else if(CHECK_STA == backData.status)
				{
					speedPID.setPoint = (Uint16)(NOMAL_RATE_UP / 8);
					speedPID.input = backData.speedCapture;
					pidCalc(&speedPID);
					SET_PWM(speedPID.sumOut);
				}
			}
			else
			{
				speedPID.setPoint = (Uint16)(NOMAL_RATE_UP / 8);
				speedPID.input = backData.speedCapture;
				pidCalc(&speedPID);
				SET_PWM(speedPID.sumOut);
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
