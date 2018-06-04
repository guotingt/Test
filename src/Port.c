#include"Port.h"
#include"GlobalValue.h"
#include"string.h"
#include"DspInit.h"
#include"Control.h"


Uint16 sendBuf[10] = {0};
Uint16 testBuf[80] = {0};
Uint16 reciveBuf[20] = {0};
BACK_DATA backData;
Uint16 currentOver;
Uint16 currentOverH;
Uint16 currentOverFlag = 0;
Uint16 cleraFault;

void readSensor()
{
	currentRead();//Get Current;

	/*Process*/
    /*mode is manual or automatic*/
	if(GpioDataRegs.GPADAT.bit.GPIO21)
	{
		//backData.status = MANUAL_STA;
	}
	/*下限位检测*/
	if(GpioDataRegs.GPADAT.bit.GPIO22)
	{
		backData.lowerOver = 0;
		backData.faultCode &= (~(0x0001<<5));
	}
	else
	{
		backData.lowerOver = 1;
		backData.faultCode |= (0x0001<<5);//bit5 lower over

	}
	/*上限位检测*/
	if(GpioDataRegs.GPADAT.bit.GPIO23)
	{
		backData.upperOver = 0;
		backData.faultCode &= (~(0x0001<<4));
	}
	else
	{
		backData.upperOver = 1;
		backData.faultCode |= (0x0001<<4);//bit4 upper over
	}

	if (abs(backData.current) >= CURRENT_THRESHOLD_3)
	{
		currentOver++;
		if (currentOver > CURRENTOVER_TIME)
		{
			backData.faultCode |= (0x0001);
			PWM_OFF;
			backData.status = STOP_STA;
			backData.posFlag = 0;//异常位置
			pidReset(&speedPID);
			SET_PWM(3750 - speedPID.sumOut);
			duty = (Uint16)(speedPID.sumOut * 100/3750);
			currentOverFlag = 1;
		}
	}
	if (abs(backData.current) >= CURRENT_THRESHOLD_4) //93A
	{
		currentOverH++;
		if (currentOverH > CURRENTOVERH_TIME)
		{
			backData.faultCode |= (0x0001);
			PWM_OFF;
			backData.status = STOP_STA;
			backData.posFlag = 0;//异常位置
			pidReset(&speedPID);
			SET_PWM(3750 - speedPID.sumOut);
			duty = (Uint16)(speedPID.sumOut * 100/3750);
			currentOverFlag = 1;
		}
	}
	if(abs(backData.current) < CURRENT_THRESHOLD_3)
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
		backData.posFlag = 0;//异常位置
		pidReset(&speedPID);
		SET_PWM(3750 - speedPID.sumOut);
		duty = (Uint16)(speedPID.sumOut * 100/3750);
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
		if(MANUAL_STA != backData.status)
		{
			/*Process*/
			switch (upperCommand.motionCmd)
			{
			case DO_STOP:
				if((BACKWARD_STA == backData.status) || (CHECK_STA == backData.status))
				{
					backData.posFlag = 1;
					backData.posCntDown = 0;
				}
				else if(FOREWARD_STA == backData.status)
				{
					backData.posFlag = 2;
					backData.posCntUp = 0;
				}
				PWM_OFF;
				backData.status = STOP_STA;
				pidReset(&speedPID);
				SET_PWM(3750 - speedPID.sumOut);
				duty = (Uint16)(speedPID.sumOut * 100/3750);
				break;
			case DO_BACKWARD:
				if((STOP_STA  == backData.status) || (BACKWARD_STA  == backData.status))//todo
				{
					backData.status = BACKWARD_STA;
					backData.motorDir = BACKWARD;
					moveCnt = 0;
					readHall();
					pwmUpdate();
				}
				else
				{
					backData.status = STOP_STA;
					backData.posFlag = 0;//异常位置
					pidReset(&speedPID);
					SET_PWM(3750 - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/3750);
				}
				break;
			case DO_FOREWARD:
				if((STOP_STA  == backData.status) || (FOREWARD_STA  == backData.status))
				{
					backData.status = FOREWARD_STA;
					backData.motorDir = FOREWARD;
					moveCnt = 0;
					readHall();
					pwmUpdate();
				}
				else
				{
					PWM_OFF;
					backData.status = STOP_STA;
					backData.posFlag = 0;//异常位置
					pidReset(&speedPID);
					SET_PWM(3750 - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/3750);
				}
				break;
			case DO_CHECK:
				if((STOP_STA  == backData.status) || (CHECK_STA  == backData.status))
				{
					backData.status = CHECK_STA;
					backData.motorDir = BACKWARD;
					readHall();
					pwmUpdate();
				}
				else
				{
					PWM_OFF;
					backData.status = STOP_STA;
					backData.posFlag = 0;//异常位置
					pidReset(&speedPID);
					SET_PWM(3750 - speedPID.sumOut);
					duty = (Uint16)(speedPID.sumOut * 100/3750);
				}
				break;
			default:
				break;
			}
			stmpL = reciveBuf[17];
			stmpH = reciveBuf[18];
			speedPID.setPoint = stmpL + (stmpH<<8);
		//	duty = stmpL + (stmpH<<8);
		//	SET_PWM_PERCENT(duty);

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
//	*(testBuf + offset) = (Uint16)(speedPID.setPoint);offset++;  //
//	*(testBuf + offset) = (Uint16)(speedPID.setPoint>>8);offset++;
	*(testBuf + offset) = backData.speedCapture; offset++;
	*(testBuf + offset) = backData.speedCapture>>8;offset++;
	*(testBuf + offset) = backData.current; offset++;
	*(testBuf + offset) = backData.current>>8; offset++;
	*(testBuf + offset) = (Uint16)(duty);offset++;
	*(testBuf + offset) = (Uint16)(duty>>8);offset++;
	*(testBuf + offset) = backData.posCnt;offset++;
	*(testBuf + offset) = backData.posCnt>>8;offset++;
	*(testBuf + offset) = backData.posCnt>>16;offset++;
	*(testBuf + offset) = backData.posCnt>>24;offset++;
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
	*(testBuf + offset) = (Uint16)(backData.hallPos);offset++;
	*(testBuf + offset) = (Uint16)(backData.hallPos1);offset++;
	*(testBuf + offset) = (Uint16)(speedPID.setPoint);offset++;  //
	*(testBuf + offset) = (Uint16)(speedPID.setPoint>>8);offset++;
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
