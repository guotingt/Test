#include "MyDataType.h"
#include "Control.h"
#include "Port.h"
#include "GlobalValue.h"
#include "DSPInit.h"
#include "math.h"
#include "string.h"

/*golbal value def*/
PID speedPID;          ///<速度PID参数
PID currentPID;        ///<电流PID参数
COMMAND upperCommand;  ///<上位机指令
Uint16 reciveFlag = 0; ///<接收标志
Uint16 moveCnt = 0;    ///<运动计数
Uint16 duty = 0;       ///<占空比

int main()
{

    /*initialize DSP */
	dsp28335Init();

	/*读取霍尔状态*/
	readHall();

//	pwmUpdate();//test

    while (1) 
    {

#if SPEED
    	if(0 != speedLoopSample)
		{
			speedLoopSample = 0;
			if(FOREWARD_STA == backData.status || BACKWARD_STA == backData.status )
			{
//				speedPID.setPoint = 50;//test
				speedPID.input = backData.speedCapture;
				pidCalc(&speedPID);
		    	duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
		    	SET_PWM(PWM_PERIOD-speedPID.sumOut);
			}
		}
#endif

#if CURRENT
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

#if CURRENT2
    	if(0 != currentLoopSample)
    	{
    		currentLoopSample = 0;
 	        if(FOREWARD_STA == backData.status || BACKWARD_STA == backData.status )
		    {
		    	currentPID.setPoint = speedPID.setPoint;
		    	currentPID.input = abs(backData.current);
		    	pidCalc(&currentPID);
		    	duty = (Uint16)(currentPID.sumOut * 100/PWM_PERIOD);
		    	SET_PWM(PWM_PERIOD-currentPID.sumOut);
		    }
    	}
#endif

    	/*接收到系统信息后返回状态*/
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
