#include"Port.h"
#include"GlobalValue.h"
#include"string.h"
#include"DspInit.h"
#include"Control.h"

Uint16 sendBuf[10] = {0};
Uint16 testBuf[80] = {0};
Uint16 reciveBuf[38] = {0};
BACK_DATA backData;

Uint16 currentOver = 0;
Uint16 currentOverH = 0;
Uint16 currentOverFlag = 0;
Uint16 cleraFault = 0;

Uint16 timeDelayDown = 0;
Uint16 timeDelayUp = 0;
Uint16 cycleMoveCnt = 0;

Uint16 iThL = 1690;//75A
Uint16 iThH = 2027;//90A
Uint16 tL = 10;//10ms
Uint16 tH = 4;//4ms

void readSensor()
{
	currentRead();//Get Current;

	/*手动检测*/
	if(!GpioDataRegs.GPADAT.bit.GPIO21)
	{
		backData.faultCode |= (0x0001<<6);
	}
	else
	{
		backData.faultCode &= (~(0x0001<<6));
	}

	/*下限位检测*/
	if(GpioDataRegs.GPADAT.bit.GPIO22)
	{
		backData.lowerOver = 0;
		backData.faultCode &= (~(0x0001<<5));
	}
	else
	{
		timeDelayDown++;
		backData.lowerOver = 1;
		backData.faultCode |= (0x0001<<5);//bit5 lower over
	}
	if(6000 == timeDelayDown)
	{
		timeDelayDown = 0;
		if((STOP_STA == backData.status) && (backData.posFlag != 0))
		{
			cycleMoveCnt++;
//			backData.status = FOREWARD_STA;
//			backData.motorDir = FOREWARD;
//			moveCnt = 0;
//			readHall();
//			pwmUpdate();
		}
	}
	/*上限位检测*/
	if(GpioDataRegs.GPADAT.bit.GPIO23)
	{
		backData.upperOver = 0;
		backData.faultCode &= (~(0x0001<<4));
	}
	else
	{
		timeDelayUp++;
		backData.upperOver = 1;
		backData.faultCode |= (0x0001<<4);//bit4 upper over
	}
	if(6000 == timeDelayUp)
	{
		timeDelayUp = 0;
		if((STOP_STA == backData.status) && (backData.posFlag != 0))
		{
			cycleMoveCnt++;
//			backData.status = BACKWARD_STA;
//			backData.motorDir = BACKWARD;
//			moveCnt = 0;
//			readHall();
//			pwmUpdate();
		}
	}
	if (abs(backData.current) >= iThL)
	{
		currentOver++;
		if (currentOver > tL)
		{
			backData.faultCode |= 0x0001;
			PWM_OFF;
			backData.status = STOP_STA;
			backData.posFlag = UNKONWN_POS;//异常位置
			pidReset(&speedPID);
			pidReset(&currentPID);
			SET_PWM(PWM_PERIOD - speedPID.sumOut);
			duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
			currentOverFlag = 1;
		}
	}
	if (abs(backData.current) >= iThH)
	{
		currentOverH++;
		if (currentOverH > tH)
		{
			backData.faultCode |= 0x0001;
			PWM_OFF;
			backData.status = STOP_STA;
			backData.posFlag = UNKONWN_POS;//异常位置
			pidReset(&speedPID);
			pidReset(&currentPID);
			SET_PWM(PWM_PERIOD - speedPID.sumOut);
			duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
			currentOverFlag = 1;
		}
	}
	if(abs(backData.current) < iThL)
	{
		currentOver = 0;
		currentOverH = 0;
	}
	if (1 == currentOverFlag)
	{
		cleraFault ++;
		if(CLEARFAULTTIME == cleraFault)
		{
			currentOverFlag = 0;
			cleraFault = 0;
			backData.faultCode &= ~0x0001; //清电流溢出错误标志
		}
	}

	/*hall状态异常检测*/
	if ((0x0007 == (backData.hallPos&0x0007)) || (0x0000 == backData.hallPos))
	{
		backData.faultCode |= (0x0001<<2);//bit2 hall error
		PWM_OFF;
		backData.status = STOP_STA;
		backData.posFlag = UNKONWN_POS;//异常位置
		pidReset(&speedPID);
		pidReset(&currentPID);
		SET_PWM(PWM_PERIOD - speedPID.sumOut);
		duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
	}
	else
	{
		backData.faultCode &= (~(0x0001<<2));
	}
}

void sendMsg()
{
	Uint16 i,len;
	len = packMsg();
	for(i = 0; i < len; i++)
	{
		scia_xmit(sendBuf[i]);
	}
}

//void unPackMsg()
//{
//	Uint16 offset = 2;
//    upperCommand.motionCmd = *(reciveBuf + offset);
//    offset++;
//    upperCommand.speedMode = *(reciveBuf + offset);
//    offset++;
//    /*Process*/
//    if(MANUAL_STA != backData.status)
//    {
//		switch (upperCommand.motionCmd)
//		{
//		case DO_STOP:
//			if((BACKWARD_STA == backData.status) || (CHECK_STA == backData.status))
//			{
//				backData.posFlag = 1;
//				backData.posCnt = 0;//correct
//			}
//			else if(FOREWARD_STA == backData.status)
//			{
//				backData.posFlag = 2;
//				backData.posCnt = MAX_CNT;//correct
//			}
//			else
//			{
//				backData.posFlag = 0;
//			}
//			backData.status = STOP_STA;
//			PWM_OFF;
//			break;
//		case DO_BACKWARD:
//			if((STOP_STA  == backData.status) || (BACKWARD_STA  == backData.status))
//			{
//				backData.status = BACKWARD_STA;
//				backData.motorDir = 1;
//				readHall();
//				pwmUpdate();
//			}
//			else
//			{
//				backData.status = STOP_STA;
//				PWM_OFF;
//			}
//			break;
//		case DO_FOREWARD:
//			if((STOP_STA  == backData.status) || (FOREWARD_STA  == backData.status))
//			{
//				backData.status = FOREWARD_STA;
//				readHall();
//				pwmUpdate();
//			}
//			else
//			{
//				backData.status = STOP_STA;
//				PWM_OFF;
//			}
//			break;
//		case DO_CHECK:
//			if((STOP_STA  == backData.status) || (CHECK_STA  == backData.status))
//			{
//				backData.status = CHECK_STA;
//				backData.motorDir = 1;
//				readHall();
//				pwmUpdate();
//			}
//			else
//			{
//				backData.status = STOP_STA;
//				PWM_OFF;
//			}
//			break;
//		default:
//			break;
//		}
//    }
//}

void unPackMsg2()
{
	Uint16 stmpL = 0;
	Uint16 stmpH = 0;
	Uint32 ltmpL = 0;
	Uint32 ltmpH = 0;
	int16 offset;
	if(0 == reciveBuf[4])
	{
		offset = 2;
		upperCommand.motionCmd = *(reciveBuf + offset);offset++;//2
		upperCommand.speedMode = *(reciveBuf + offset);offset++;//3
		stmpL = reciveBuf[35];
		stmpH = reciveBuf[36];
		speedPID.setPoint = stmpL + (stmpH<<8);
		if(MANUAL_STA != backData.status)
		{
			/*Process*/
			switch (upperCommand.motionCmd)
			{
			case DO_STOP:
				if((BACKWARD_STA == backData.status) || (CHECK_STA == backData.status))
				{
					backData.posFlag = DOWN_POS;
				}
				else if(FOREWARD_STA == backData.status)
				{
					backData.posFlag = UP_POS;
				}
				PWM_OFF;
				backData.status = STOP_STA;
				pidReset(&speedPID);
				pidReset(&currentPID);
				SET_PWM(PWM_PERIOD - speedPID.sumOut);
				duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
				break;
			case DO_BACKWARD:
				if(STOP_STA  == backData.status)
				{
					backData.status = BACKWARD_STA;
					backData.motorDir = BACKWARD;
					moveCnt = 0;
					readHall();
					pwmUpdate();
				}
				else if((CHECK_STA == backData.status)||(FOREWARD_STA == backData.status))
				{
					backData.status = STOP_STA;
					backData.posFlag = UNKONWN_POS;//异常位置
					pidReset(&speedPID);
					pidReset(&currentPID);
					SET_PWM(PWM_PERIOD - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
				}
				break;
			case DO_FOREWARD:
				if(STOP_STA  == backData.status)
				{
					backData.status = FOREWARD_STA;
					backData.motorDir = FOREWARD;
					moveCnt = 0;
					readHall();
					pwmUpdate();
				}
				else if((CHECK_STA == backData.status)||(BACKWARD_STA == backData.status))
				{
					PWM_OFF;
					backData.status = STOP_STA;
					backData.posFlag = UNKONWN_POS;//异常位置
					pidReset(&speedPID);
					pidReset(&currentPID);
					SET_PWM(PWM_PERIOD - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
				}
				break;
			case DO_CHECK:
				if(STOP_STA  == backData.status)
				{
					backData.status = CHECK_STA;
					backData.motorDir = BACKWARD;
					readHall();
					pwmUpdate();
				}
				else if((FOREWARD_STA == backData.status)||(BACKWARD_STA == backData.status))
				{
					PWM_OFF;
					backData.status = STOP_STA;
					backData.posFlag = UNKONWN_POS;//异常位置
					pidReset(&speedPID);
					pidReset(&currentPID);
					SET_PWM(PWM_PERIOD - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/PWM_PERIOD);
				}
				break;
			default:
				break;
			}
		}
	}
	else
	{
		offset = 5;
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		speedPID.kp = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		speedPID.ki = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		speedPID.outMax = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		currentPID.kp = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		currentPID.ki = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpL = stmpL + (stmpH<<8);
		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		ltmpH = stmpL + (stmpH<<8);
		currentPID.outMax = ltmpL + (ltmpH<<16);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		iThL = stmpL + (stmpH<<8);

		stmpL = reciveBuf[offset]; offset++;
		stmpH = reciveBuf[offset]; offset++;
		iThH = stmpL + (stmpH<<8);

		tL = reciveBuf[offset]; offset++;
		tH = reciveBuf[offset]; offset++;
	}
}
Uint16 packMsg()
{
	Uint16 offset = 0;
	Uint16 xors = 0;
	Uint16 i = 0;
    *(sendBuf + offset) = 0xAA; offset++;
    *(sendBuf + offset) = 0x55; offset++;
    *(sendBuf + offset) = (Uint16)backData.status; offset++;
    *(sendBuf + offset) = (Uint16)backData.faultCode; offset++;
    memset(sendBuf,0x00,2);
    for (i = 2; i < 5;i++)
    {
        xors ^= sendBuf[i];
    }
    *(sendBuf + offset) = xors; offset++;
    *(sendBuf + offset) = 0xBF; offset++;
    return offset;
}
void sendTest()
{
	Uint16 offset = 0;
	Uint16 xors = 0;
	Uint16 i = 0;
	readHall1();
	*(testBuf + offset) = 0xAA; offset++;
	*(testBuf + offset) = 0x55; offset++;
	*(testBuf + offset) = backData.status; offset++;
	*(testBuf + offset) = backData.faultCode; offset++;
	*(testBuf + offset) = backData.speedCapture; offset++;
	*(testBuf + offset) = backData.speedCapture>>8;offset++;
	*(testBuf + offset) = backData.current; offset++;
	*(testBuf + offset) = backData.current>>8; offset++;
	*(testBuf + offset) = duty;offset++;
	*(testBuf + offset) = duty>>8;offset++;
	*(testBuf + offset) = speedPID.sumOut;offset++;
	*(testBuf + offset) = speedPID.sumOut>>8;offset++;
	*(testBuf + offset) = speedPID.sumOut>>16;offset++;
	*(testBuf + offset) = speedPID.sumOut>>24;offset++;
	*(testBuf + offset) = (Uint16)(speedPID.kp);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.kp>>8);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.kp>>16);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.kp>>24);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.ki);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.ki>>8);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.ki>>16);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.ki>>24);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.outMax);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.outMax>>8);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.outMax>>16);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.outMax>>24);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.kp);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.kp>>8);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.kp>>16);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.kp>>24);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.ki);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.ki>>8);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.ki>>16);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.ki>>24);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.outMax);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.outMax>>8);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.outMax>>16);offset++;
	*(testBuf + offset) = (Uint16)(currentPID.outMax>>24);offset++;
	*(testBuf + offset) = (Uint16)(backData.hallPos);offset++;
	*(testBuf + offset) = (Uint16)(backData.hallPos1);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.setPoint);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.setPoint>>8);offset++;
	*(testBuf + offset) = (Uint16)(iThL);offset++;
	*(testBuf + offset) = (Uint16)(iThL>>8);offset++;
	*(testBuf + offset) = (Uint16)(iThH);offset++;
	*(testBuf + offset) = (Uint16)(iThH>>8);offset++;
	*(testBuf + offset) = (Uint16)(tL);offset++;
	*(testBuf + offset) = (Uint16)(tH);offset++;
	for (i = 2; i < offset;i++)
	{
		xors ^= testBuf[i];
	}
	*(testBuf + offset) = xors; offset++;
	*(testBuf + offset) = 0xBF; offset++;
	for(i = 0; i < offset; i++)
	{
		scia_xmit(testBuf[i]);
	}
}
