#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"
#include "math.h"
#include "string.h"

#define CURRENT 0
#define SPEED 1

/*golbal value def*/
PID speedPID;
PID currentPID;
COMMAND upperCommand;
Uint16 reciveFlag = 0;

Uint16 moveCnt = 0;
Uint16 duty = 1;

Uint16 keepCnt = 0;
void initPID()
{
	memset(&speedPID,0x00,sizeof(PID));
	speedPID.outMax = 196608000;//%80
	speedPID.outMin = 327675;//%5
	speedPID.kp = 262144;//4
	speedPID.ki = 13107;//0.2

//	currentPID.outMax = 100*65536;
//	currentPID.outMin = 0;
//	currentPID.kp = 10000;
//	currentPID.ki = 10;
}

int main()
{

    /*initialize DSP */
	dsp28335Init();

    /*initialize Controller*/
	initPID();

    readHall();

    SET_PWM_PERCENT(duty);

    while (1) 
    {
    	/*read sensor data*/
    	if(0 != sensorReadSample)
    	{
    		sensorReadSample = 0;
    		/*pid calc mode set*/
    		currentPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    		speedPID.mode = (MANUAL_STA == backData.status) ? MANUAL : AUTOMATIC;
    	 }

#if SPEED
    	/*speed pid */
    	if(0 != speedLoopSample)
    	{
    		speedLoopSample = 0;
    		speedPID.input = backData.speedCapture;
    		pidCalc(&speedPID);
    		duty = (Uint16)(speedPID.sumOut * 100/3750) ;
    		SET_PWM(3750-speedPID.sumOut);
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
			SET_PWM(3750 - currentPID.sumOut);
		}
#endif

        if (1 == reciveFlag)
        {
            //sendMsg();
            reciveFlag = 0;
        }
        if(0 != testSendSample)
        {
        	sendTest();
        	testSendSample = 0;
        }
    }
}
