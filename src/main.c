#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"
#include "math.h"
#include "string.h"

#define CURRENT 0
#define SPEED 0

/*golbal value def*/
PID speedPID;
PID currentPID;
COMMAND upperCommand;
Uint16 reciveFlag = 0;
Uint16 moveCnt = 0;
Uint16 duty = 10;

int main()
{

    /*initialize DSP */
	dsp28335Init();

	readHall();

    SET_PWM_PERCENT(duty);

    pwmUpdate();

    while (1) 
    {
#if SPEED
    	/* speed PID */
    	if(0 != speedLoopSample)
    	{
    		speedLoopSample = 0;
 	        if(FOREWARD_STA == backData.status || BACKWARD_STA == backData.status )
		    {
		    	speedPID.input = backData.speedCapture;
		    	pidCalc(&speedPID);
		    }
    	}
    	/*current PID*/
    	if(0 != currentLoopSample)
    	{
    		currentLoopSample = 0;
 	        if(FOREWARD_STA == backData.status || BACKWARD_STA == backData.status )
		    {
		    	currentPID.setPoint = speedPID.sumOut;
		    	currentPID.input = abs(backData.current);
		    	pidCalc(&currentPID);
		    	duty = (Uint16)(currentPID.sumOut * 100/PWM_PERIOD);
		    	SET_PWM(PWM_PERIOD-currentPID.sumOut);
		    }
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
