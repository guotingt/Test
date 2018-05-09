#include"Port.h"
#include"GlobalValue.h"
#include"string.h"
#include"DspInit.h"

Uint16 sendBuf[10] = {0};
Uint16 testBuf[80] = {0};
Uint16 reciveBuf[10] = {0};
BACK_DATA backData;

void readSensor()
{
	currentRead();//Get Current;

	/*Process*/
    /*mode is manual or automatic*/
	if(GpioDataRegs.GPADAT.bit.GPIO21)
	{
		backData.status = MANUAL_STA;
	}
	if(GpioDataRegs.GPADAT.bit.GPIO22)
	{
		backData.lowerOver = 1;
	}
	if(GpioDataRegs.GPADAT.bit.GPIO23)
	{
		backData.upperOver = 1;
	}
	/*Read GPIOX to define status*/
	if (backData.current >= CURRENT_THRESHOLD_1)
	{
		backData.faultCode |= (0x01<<0);//bit0 over current
	}
	else if (backData.current >= CURRENT_THRESHOLD_2)
	{
		backData.faultCode |= (0x01<<3);//bit3 over load
	}
	if ((0x07 == (backData.hallPos&0x07)) || (0x00 == (backData.hallPos&0x00)) )
	{
		backData.faultCode |= (0x01<<2);//bit2 hall error
	}
	if (1 == backData.upperOver)
	{
		backData.faultCode |= (0x01<<4);//bit4 upper over
	}
	if (1 == backData.lowerOver)
	{
		backData.faultCode |= (0x01<<5);//bit5 lower over
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

void unPackMsg()
{
	Uint16 offset = 2;
    upperCommand.motionCmd = *(reciveBuf + offset);
    offset++;
    upperCommand.speedMode = *(reciveBuf + offset);
    offset++;
    /*Process*/
    switch (upperCommand.motionCmd)
	{
	case DO_STOP:
		if((BACKWARD_STA == backData.status) || (CHECK_STA == backData.status))
		{
			posFlag = 1;
		}
		else if(FOREWARD_STA == backData.status)
		{
			posFlag = 2;
		}
		else
		{
			posFlag = 0;
		}
	    backData.status = STOP_STA;
	    posCnt = 0;
	    break;
	case DO_BACKWARD:
		switch (upperCommand.speedMode)
		{
		case LOW_SPEED:
			backData.status = CHECK_STA;
			motorDir = 0;//
			break;
		case HIGH_SPEED:
			backData.status = BACKWARD_STA;
			motorDir = 0;
			break;
		default:
			break;
		}
		break;
	case DO_FOREWARD:
		backData.status = FOREWARD_STA;
		motorDir = 1;
		break;
	case DO_CHECK:
		backData.status = CHECK_STA;
		motorDir = 0;
		break;
	default:
		break;
	}
}
void unPackMsg2()
{
	int16 stmpL = 0;
	int16 stmpH = 0;
	Uint16 offset = 2;
	stmpL = reciveBuf[offset]; offset++;
	stmpH = reciveBuf[offset]; offset++;
	speedPID.kp = stmpL + stmpH<<8;
	stmpL = reciveBuf[offset]; offset++;
	stmpH = reciveBuf[offset]; offset++;
	speedPID.ki = stmpL + stmpH<<8;
	stmpL = reciveBuf[offset]; offset++;
	stmpH = reciveBuf[offset]; offset++;
	speedPID.outMax = stmpL + stmpH<<8;
	speedPID.outMin = -speedPID.outMax;
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
	*(testBuf + offset) = 0xAA; offset++;
	*(testBuf + offset) = 0x55; offset++;
	*(testBuf + offset) = backData.status; offset++;
	*(testBuf + offset) = backData.faultCode; offset++;
	*(testBuf + offset) = backData.current; offset++;
	*(testBuf + offset) = backData.current>>8; offset++;
	*(testBuf + offset) = backData.speed; offset++;
	*(testBuf + offset) = backData.speed>>8;offset++;
	*(testBuf + offset) = posCnt;offset++;
	*(testBuf + offset) = posCnt>>8;offset++;
	*(testBuf + offset) = posCnt>>16;offset++;
	*(testBuf + offset) = posCnt>>24;offset++;
 	//memset(testBuf,0x00,2);
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
	//return offset;
}
