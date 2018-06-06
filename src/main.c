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
Uint16 duty = 0;

int main()
{

    /*initialize DSP */
	dsp28335Init();

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
    	/* speed pid */
    	if(0 != speedLoopSample)
    	{
    		speedLoopSample = 0;
 	        if(FOREWARD_STA == backData.status || BACKWARD_STA == backData.status )
		    {
		    	speedPID.input = backData.speedCapture;
		    	pidCalc(&speedPID);
		    	duty = (Uint16)(speedPID.sumOut * 100/3750);
		    	SET_PWM(3750-speedPID.sumOut);
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
