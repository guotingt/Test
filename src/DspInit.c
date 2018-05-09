#include "DspInit.h"
#include "GlobalValue.h"
#include "Port.h"
#include "Queue.h"
#include "string.h"

TIME_FLAG  flagDot1ms;
TIME_FLAG  flag1ms;
TIME_FLAG  flag10ms;
TIME_FLAG  flag100ms;
TIME_FLAG  flag500ms;
TIME_FLAG  flag1000ms;

volatile Uint16 DMABuf1[20] = {0};///<DMA data
volatile Uint16 *DMADest;         ///<DMA DST_Addr
volatile Uint16 *DMASource;       ///<DMA SRC_Addr

volatile Uint16 msCnt1 = 0;   ///<1ms
volatile Uint16 msCnt10 = 0;  ///<10ms
volatile Uint16 msCnt100 = 0; ///<100ms
volatile Uint16 msCnt500 = 0; ///<0.5s
volatile Uint16 msCnt1000 = 0;///<1s

Uint32 interruptCnt = 0;
volatile Uint16 currentBaseW = 0;
volatile Uint16 currentBaseU = 0;
volatile Uint16 currentBaseV = 0;

Uint16 motorRuning = 0;
Uint16 motorDir = 0;
Uint16 LastHallGpio = 0;
Uint16 NewHallGpio = 0;
Uint32 VirtualTimer = 0;
Uint32 SpeedNewTimer = 0;
Uint32 SpeedLastTimer = 0;
Uint16 modcnt = 0;
Uint32 posCnt = 0;

void EPwm1Setup(Uint16 period,Uint16 duty)
{
	EPwm1Regs.TBPRD = period;                      // Period = 1600 TBCLK counts,up-down mode
	EPwm1Regs.TBPHS.half.TBPHS = 0;                // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;         //TBPRD is loaded from its shadow register
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;    // Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;  // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR ;            // set actions for EPWM1B
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm1Regs.CMPA.half.CMPA = duty;
	EPwm1Regs.CMPB = duty;
}
void EPwm2Setup(Uint16 period,Uint16 duty)
{
	EPwm2Regs.TBPRD = period;                         // Period =1600 TBCLK counts  up-down mode
	EPwm2Regs.TBPHS.half.TBPHS = 0;                   // Set Phase register to zero
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    // Symmetrical mode
	EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;            // Slave module
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        // sync flow-through
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // load on CTR=Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // load on CTR=Zero
	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;                // set actions for EPWM2A
	EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;                // set actions for EPWM2B
	EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm2Regs.CMPA.half.CMPA = duty;
	EPwm2Regs.CMPB = duty;
}
void EPwm3Setup(Uint16 period,Uint16 duty)
{
	EPwm3Regs.TBPRD = period;                         // Period = 1600 TBCLK counts,up-down mode
	EPwm3Regs.TBPHS.half.TBPHS = 0;                   // Set Phase register to 50000
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;    // Symmetrical mode
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;            // Slave module
	EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        // sync flow-through
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;     // load on CTR=Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;     // load on CTR=Zero
	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;                // set actions for EPWM3A
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;                // set actions for EPWM3B
	EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;
	EPwm3Regs.CMPA.half.CMPA = duty;
	EPwm3Regs.CMPB = duty;
}

void ECap1Setup()
{
	ECap1Regs.ECCTL1.bit.CAP1POL = EC_FALLING;           //CAP1: falling
	ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;           //CAP2: falling
	ECap1Regs.ECCTL1.bit.CAP3POL = EC_FALLING;           //CAP3: falling
	ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;           //CAP4: falling
	ECap1Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;          //absolute time stamp operation
	ECap1Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;          //absolute time stamp operation
	ECap1Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;          //absolute time stamp operation
	ECap1Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;          //absolute time stamp operation
	ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;            //Enable Loading of CAP1-4 registers on a capture even
	ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;             //no prescale, by-pass the prescaler
	ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;         //capture operating mode select
	ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;    //continuous capture
	ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;       //select sync-in event to be the sync-out signal (pass through)
	ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;          //Enable TSCTR to be loaded from CTRPHS register upon either a SYNCI signal or a S/W force event;
	ECap1Regs.TSCTR  = 0;
	ECap1Regs.CTRPHS = 0;
	ECap1Regs.ECEINT.all = 0x0000;                       //stop all interrupt
	ECap1Regs.ECCLR.all = 0xFFFF;                        //clear all flag
	ECap1Regs.ECEINT.bit.CEVT4=1;  	                     // Enable cevt4 interrupt
	ECap1Regs.ECEINT.bit.CTROVF=1;                       //Enable CTROVF interrupt
	ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;             // start to run ECap1
}
void ECap2Setup()
{
	ECap2Regs.ECCTL1.bit.CAP1POL = EC_FALLING;           //CAP1: rising
	ECap2Regs.ECCTL1.bit.CAP2POL = EC_FALLING;           //CAP2: falling
	ECap2Regs.ECCTL1.bit.CAP3POL = EC_FALLING;           //CAP3: rising
	ECap2Regs.ECCTL1.bit.CAP4POL = EC_FALLING;           //CAP4: falling
	ECap2Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;          //absolute time stamp operation
	ECap2Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;          //absolute time stamp operation
	ECap2Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;          //absolute time stamp operation
	ECap2Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;          //absolute time stamp operation
	ECap2Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;            //Enable Loading of CAP1-4 registers on a capture even
	ECap2Regs.ECCTL1.bit.PRESCALE = EC_DIV1;             //no prescale, by-pass the prescaler
	ECap2Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;         //capture operating mode select
	ECap2Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;    //continuous capture
	ECap2Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;       //select sync-in event to be the sync-out signal (pass through)
	ECap2Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;          //Enable TSCTR to be loaded from CTRPHS register upon either a SYNCI signal or a S/W force event;
	ECap2Regs.TSCTR = 0;
	ECap2Regs.CTRPHS = 0;
	ECap2Regs.ECEINT.all = 0x0000;						 //stop all interrupt
	ECap2Regs.ECCLR.all = 0xFFFF;						 //clear all flag
	ECap2Regs.ECEINT.bit.CEVT4 = 1;						 //Enable cevt4 interrupt
	ECap2Regs.ECEINT.bit.CTROVF = 1;					 //Enable CTROVF interrupt
	ECap2Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;		     //start to run ECap2
}
void ECap3Setup()
{
	ECap3Regs.ECCTL1.bit.CAP1POL = EC_FALLING;           //CAP1: rising
	ECap3Regs.ECCTL1.bit.CAP2POL = EC_FALLING;           //CAP2: falling
	ECap3Regs.ECCTL1.bit.CAP3POL = EC_FALLING;           //CAP3: rising
	ECap3Regs.ECCTL1.bit.CAP4POL = EC_FALLING;           //CAP4: falling
	ECap3Regs.ECCTL1.bit.CTRRST1 = EC_ABS_MODE;          //absolute time stamp operation
	ECap3Regs.ECCTL1.bit.CTRRST2 = EC_ABS_MODE;          //absolute time stamp operation
	ECap3Regs.ECCTL1.bit.CTRRST3 = EC_ABS_MODE;          //absolute time stamp operation
	ECap3Regs.ECCTL1.bit.CTRRST4 = EC_ABS_MODE;          //absolute time stamp operation
	ECap3Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;            //Enable Loading of CAP1-4 registers on a capture even
	ECap3Regs.ECCTL1.bit.PRESCALE = EC_DIV1;             //no prescale, by-pass the prescaler
	ECap3Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;         //capture operating mode select
	ECap3Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;    //continuous capture
	ECap3Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;       //select sync-in event to be the sync-out signal (pass through)
	ECap3Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;          //Enable TSCTR to be loaded from CTRPHS register upon either a SYNCI	signal or a S/W force event;
	ECap3Regs.TSCTR = 0;
	ECap3Regs.CTRPHS = 0;
	ECap3Regs.ECEINT.all = 0x0000;   					//stop all interrupt
	ECap3Regs.ECCLR.all = 0xFFFF;    					//clear all flag
	ECap3Regs.ECEINT.bit.CEVT4 = 1;  					//Enable cevt4 interrupt
	ECap3Regs.ECEINT.bit.CTROVF = 1; 				    //Enable CTROVF interrupt
	ECap3Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN;            //start to run ECap3
}
void DMASetup()
{
	DMADest=&DMABuf1[0];
	DMASource= &AdcMirror.ADCRESULT0;
	DMACH1AddrConfig(DMADest,DMASource);
	DMACH1BurstConfig(1,1,10);        	//Will set up to use 32-bit datasize, pointers are based on 16-bit words
	DMACH1TransferConfig(9,0,0);      	//so need to increment by 2 to grab the correct location
	DMACH1WrapConfig(0,0,0,1);
	//Peripheral Interrupt Source Select: SEQ1INT
	//Peripheral Interrupt Trigger Enable
	//One Shot Mode Bit:   this bit is set to 1, then subsequent burst transfers occur without additional event triggers after the first event trigger.
	//Continuous Mode Bit: this bit is set to 1, then DMA re-initializes when TRANSFER_COUNT is zero and waits for the next interrupt event trigger.
    //Sync Disable
	//Disable Overflow Interrupt
	//16-bit data transfer size
	//Disable channel interrupt
	DMACH1ModeConfig(DMA_SEQ1INT,PERINT_ENABLE,ONESHOT_ENABLE,CONT_ENABLE,SYNC_DISABLE,SYNC_SRC,OVRFLOW_DISABLE,SIXTEEN_BIT,CHINT_END,CHINT_ENABLE);
	StartDMACH1();
}
void AdcSetup()
{
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        //Cascaded mode. SEQ1 and SEQ2 operate as a single 16-state sequencer (SEQ).
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 0;        //disable override
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1;        //Continuous conversion mode
    AdcRegs.ADCTRL1.bit.CPS = 0;             //ADCCLK = Fclk/1
    AdcRegs.ADCTRL1.bit.ACQ_PS = 0x0F;       //The width of SOC pulse is ADCTRL1[11:8] + 1 times the ADCLK period
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;    //Interrupt request by INT_SEQ1 is enable
    AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1 = 0;    //INT_SEQ1 is set at the end of every SEQ1 sequence
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;        //Immediately reset sequencer to state CONV0
    AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;       //Sequential sampling mode is selected
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 3;        //Core clock divider.12.5Mh;
    AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x1;  //The maximum number of conversions executed in an auto conversion is 2
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;   // A0 selected
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;   // A1 selected
    //AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2;   // A2 selected
    AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;        //Software trigger-Start SEQ1 from currently stopped position (i.e., Idle mode)
}
void SCIASetup()
{
	SciaRegs.SCICCR.all = 0x0007;        // 1 stop bit, No loopback, No parity,8 char bits, idle-line protocol
	SciaRegs.SCICTL1.all = 0x0003;       // enable TX, RX, internal SCICLK
	SciaRegs.SCICTL2.all = 0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA = 0;   // disable TXRDY interrupt
	SciaRegs.SCICTL2.bit.RXBKINTENA =0;  // disable RXRDY/BRKDT interrupt
    SciaRegs.SCIHBAUD = 0x0000;          // (2400baud,7A0h),(4800baud, 3D0h),(9600baud,1E7h),(19200baud,F3h),(38400baud,79h)  @LSPCLK = 37.5MHz.
    SciaRegs.SCILBAUD = 0x00F3;
    SciaRegs.SCICTL1.all =0x0023;  		 // Relinquish SCI from Reset
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x204a;
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;
    SciaRegs.SCIFFCT.all = 0x0;

}
void configureLed(void)
{
   EALLOW;
   GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0; // GPIO87复用为GPIO功能
   GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;  // GPIO1设置为输出
   EDIS;
   GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;
}
void scia_xmit(Uint16 a)
{
	 while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
	 {}
	 SciaRegs.SCITXBUF = a;
}
void currentRead()
{
//    while(1 == AdcRegs.ADCST.bit.SEQ1_BSY)
//    {}
	backData.currentU = FILTERcon(&uiQueue,DMABuf1[0],QUE_MAX);
	backData.currentV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
	backData.currentW = -(backData.currentU+backData.currentV);
    if(0 == motorDir)//Forward
    {
		switch(backData.hallPos)
		{
		case 5://UV
			//backData.currentV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
			backData.current = backData.currentV - currentBaseV;
			break;
		case 1://UW
			//backData.currentW = FILTERcon(&wiQueue,DMABuf1[20],QUE_MAX);
			backData.current = backData.currentW - currentBaseW;
			break;
		case 3://VW
			//backData.currentW = FILTERcon(&wiQueue,DMABuf1[20],QUE_MAX);
			backData.current = backData.currentW - currentBaseW;
			break;
		case 2://VU
			//backData.currentU = FILTERcon(&uiQueue,DMABuf1[0],QUE_MAX);
			backData.current = backData.currentU - currentBaseU;
			break;
		case 6://WU
			//backData.currentU = FILTERcon(&uiQueue,DMABuf1[0],QUE_MAX);
			backData.current = backData.currentU - currentBaseU;
			break;
		case 4://WV
			//backData.currentV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
			backData.current = backData.currentV - currentBaseV;
			break;
		default:
			break;
		}
    }
    else if(1 == motorDir)//Backward
    {
    	switch(backData.hallPos)
		{
		case 2://UV
			//backData.currentV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
			backData.current = backData.currentV - currentBaseV;
			break;
		case 6://UW
			//backData.currentW = FILTERcon(&wiQueue,DMABuf1[20],QUE_MAX);
			backData.current = backData.currentW - currentBaseW;
			break;
		case 4://VW
			//backData.currentW = FILTERcon(&wiQueue,DMABuf1[20],QUE_MAX);
			backData.current = backData.currentW - currentBaseW;
			break;
		case 5://VU
			//backData.currentU = FILTERcon(&uiQueue,DMABuf1[0],QUE_MAX);
			backData.current = backData.currentU - currentBaseU;
			break;
		case 1://WU
			//backData.currentU = FILTERcon(&uiQueue,DMABuf1[0],QUE_MAX);
			backData.current = backData.currentU - currentBaseU;
			break;
		case 3://WV
			//backData.currentV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
			backData.current = backData.currentV - currentBaseV;
			break;
		default:
			break;
		}
    }
}
void dsp28335Init()
{

	InitSysCtrl();								   //设置PLL, WatchDog, 使能外设时钟

	InitEPwm1Gpio();                               //Configure GPIO0 as EPWM1A,  GPIO1 as EPWM1B
	InitEPwm2Gpio();                               //Configure GPIO2 as EPWM2A,  GPIO3 as EPWM2B
	InitEPwm3Gpio();                               //Configure GPIO4 as EPWM3A,  GPIO5 as EPWM3B
	InitECap1Gpio();							   //Configure GPIO24 as ECAP1
	InitECap2Gpio();							   //Configure GPIO7 as ECAP2
	InitECap3Gpio();							   //Configure GPIO9 as ECAP3
	InitSciaGpio();								   //Configure GPIO28 as RX, GPIO29 as TX
										   //ADC复位
	configureLed();
	DMAInitialize();                               //DMA复位

	DINT;										   //禁止CPU全局中断
	InitPieCtrl();								   //初始化PIE中断向量表，并使其指向中断服务子程序（ISR）
	/*禁止CPU中断和清除所有CPU中断标志*/
	IER = 0x0000;
	IFR = 0x0000;
	InitPieVectTable();                           //PIE 向量表指针指向中断服务程(ISR)完成其初始化.

//	MemCopy(&RamfuncsLoadStart,&RamfuncsLoadEnd,&RamfuncsRunStart);
//	InitFlash();
	InitAdc();
	EALLOW;                                       //This is needed to write to EALLOW protected registers
	PieVectTable.ECAP1_INT = &ISRCap1;            //将CAP1中断添加都中断向量表里
	PieVectTable.ECAP2_INT = &ISRCap2;            //将CAP2中断添加都中断向量表里
    PieVectTable.ECAP3_INT = &ISRCap3;   	      //将CAP3中断添加都中断向量表里
    PieVectTable.TINT0 = &ISRTimer0;              //将定时器0中断添加都中断向量表里
    PieVectTable.SCIRXINTA =&ISRSCIARX;           //将串口接收中断添加都中断向量表里
    PieVectTable.DINTCH1= &local_DINTCH1_ISR;
    EDIS;                                         //This is needed to disable write to EALLOW protected register

    InitCpuTimers();                              //定时器初始化
    ConfigCpuTimer(&CpuTimer0, 150, 100);       //定时器0初始化/10KHz
    StartCpuTimer0();                             //开启定时器0

    EPwm1Setup(PWM_PERIOD,PWM_DUTY);    		  //EPWM1配置
    EPwm2Setup(PWM_PERIOD,PWM_DUTY);			  //EPWM2配置
    EPwm3Setup(PWM_PERIOD,PWM_DUTY);			  //EPWM3配置
    //PWM_DISABLE;

    ECap1Setup();    							  //ECAP1配置
    ECap2Setup();    							  //ECAP2配置
    ECap3Setup();    							  //ECAP3配置

    DMASetup();     							  //DMA初始化配置
    AdcSetup();     							  //ADC初始化配置
    SCIASetup();     							  //SCIA初始化配置

    IER |= M_INT1;   							  //使能第一组中断
    IER |= M_INT4;   							  //使能第四组中断
    IER |= M_INT9;   							  //使能第九组中断
    IER |= M_INT13; 						      //使能中断13
    IER |= M_INT7; 						          //使能第七组中断


    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;            //使能PIE总中断
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;            //使能第一组中断里的第七个中断--定时器0中断
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;            //使能第四组中断里的第一个中断--CAP1中断
    PieCtrlRegs.PIEIER4.bit.INTx2 = 1;            //使能第四组中断里的第二个中断--CAP2中断
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;            //使能第四组中断里的第三个中断--CAP3中断
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;            //使能第九组中断里的第一个中断--SCIARX接收中断
    /*DMA通道中断在配置中使能*/
    dataInit();
    EINT;                                         //中断使能
    ERTM;                                         //使能总实时中断
}

interrupt void ISRSCIARX(void)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;        //Acknowledge this interrupt to receive more interrupts from group 1
	Uint16 tmpChar;
	static Uint16 ptr = 0;
	static Uint16 xors = 0;
	while(SciaRegs.SCIFFRX.bit.RXFFST !=0)
	{
		tmpChar = SciaRegs.SCIRXBUF.all;
		if(ptr < 2)
		{
		  switch(ptr)
		  {
		  case 0:
			  reciveBuf[ptr] = tmpChar;
			  ptr =((tmpChar == 0xAA)?1:0);
			  break;
		  case 1:
			  reciveBuf[ptr] = tmpChar;
			  ptr =((tmpChar == 0x55)?2:0);
			  break;
		  }
		}
		else if(ptr < 6)
		{
		  reciveBuf[ptr++] = tmpChar;
		  xors ^= tmpChar;
		}
		else if(ptr < 7)
		{
		  reciveBuf[ptr] = tmpChar;
		  if(0 == xors && 0xBF == reciveBuf[ptr])
		  {
			  unPackMsg(reciveBuf,&upperCommand);
		  }
		  else
		  {
			  ptr = 0;
			  xors = 0;
		  }
		}
		else
		{
		  ptr = 0;
		  xors = 0;
		}
	}
	SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
}


interrupt void ISRTimer0(void)
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  //Acknowledge this interrupt to receive more interrupts from group 1
    CpuTimer0Regs.TCR.bit.TIF = 1;           // 定时到了指定时间，标志位置位，清除标志
    CpuTimer0Regs.TCR.bit.TRB = 1;           // 重载Timer0的定时数据
//    interruptCnt++;
    speedRead();//0.1ms 10000
    flagDot1msW = 0xffff; //100us时间到
    msCnt1++;
	if(msCnt1 >= 10)
	{
	  msCnt1 = 0;
	  flag1msW = 0xffff;  //1ms时间到
	  msCnt10++;
	  if(msCnt10 >= 10)
	  {
		msCnt10 = 0;
		flag10msW = 0xffff; //10ms时间到
		msCnt100++;
		if(msCnt100 >= 10)
		{
		  msCnt100 = 0;
		  flag100msW = 0xffff; //100ms时间到
		  msCnt500++;
		  if(msCnt500 >= 5)
		  {
			msCnt500 = 0;
			flag500msW = 0xffff;//0.5s
			msCnt1000++;
			if(msCnt1000 >= 2)
			{
			  msCnt1000 = 0;
			  LED_TOGGLE;
			  flag1000msW = 0xffff;//1s
			}
		  }
		}
	  }
	}
}
interrupt void ISRCap1(void)
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;  //Acknowledge this interrupt to receive more interrupts from group 4
}

interrupt void ISRCap2(void)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;  //Acknowledge this interrupt to receive more interrupts from group 4
}

interrupt void ISRCap3(void)
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;  //Acknowledge this interrupt to receive more interrupts from group 4
}

interrupt void local_DINTCH1_ISR(void)
{
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;  //Acknowledge this interrupt to receive more interrupts from group 7
}
void dataInit()
{
	//Uint16 i;
	memset(&backData,0x00,sizeof(BACK_DATA));
	memset(&upperCommand,0x00,sizeof(BACK_DATA));
	motorRuning = 0;
	motorDir = 0;
	upperCommand.motionCmd = DO_STOP;
	backData.status = STOP_STA;
//	/*Current_Base*/
//	for(i = 0;i < QUE_MAX;i++)
//	{
//		currentBaseW = FILTERcon(&wiQueue,DMABuf1[0],QUE_MAX);
//		currentBaseV = FILTERcon(&viQueue,DMABuf1[10],QUE_MAX);
//		currentBaseU = FILTERcon(&uiQueue,DMABuf1[20],QUE_MAX);
//	}
	/*HallPos*/
	if(GpioDataRegs.GPADAT.bit.GPIO24)
	{
		backData.hallPos |= (0x0001<<0);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<0);
	}
	if(GpioDataRegs.GPADAT.bit.GPIO7)
	{
		backData.hallPos |= (0x0001<<1);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<1);
	}
	if(GpioDataRegs.GPADAT.bit.GPIO9)
	{
		backData.hallPos |= (0x0001<<2);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<2);
	}
}
void pwmUpdate()
{
	if(0 == motorDir)
	{
		switch(backData.hallPos)
		{
		case 5://UV
			PWM_DISABLE;
			PWM_U1_ENABLE;
			PWM_V2_ENABLE;
			break;
		case 1://UW
			PWM_DISABLE;
			PWM_U1_ENABLE;
			PWM_W2_ENABLE;
			break;
		case 3://VW
			PWM_DISABLE;
			PWM_V1_ENABLE;
			PWM_W2_ENABLE;
			break;
		case 2://VU
			PWM_DISABLE;
			PWM_V1_ENABLE;
			PWM_U2_ENABLE;
			break;
		case 6://WU
			PWM_DISABLE;
			PWM_W1_ENABLE;
			PWM_U2_ENABLE;
			break;
		case 4://WV
			PWM_DISABLE;
			PWM_W1_ENABLE;
			PWM_V2_ENABLE;
			break;
		default:;
		}
	}
	else if(1 == motorDir)
	{
		switch(backData.hallPos)
		{
		case 2://UV
			PWM_DISABLE;
			PWM_U1_ENABLE;
			PWM_V2_ENABLE;
			break;
		case 6://UW
			PWM_DISABLE;
			PWM_U1_ENABLE;
			PWM_W2_ENABLE;
			break;
		case 4://VW
			PWM_DISABLE;
			PWM_V1_ENABLE;
			PWM_W2_ENABLE;
			break;
		case 5://VU
			PWM_DISABLE;
			PWM_V1_ENABLE;
			PWM_U2_ENABLE;
			break;
		case 1://WU
			PWM_DISABLE;
			PWM_W1_ENABLE;
			PWM_U2_ENABLE;
			break;
		case 3://WV
			PWM_DISABLE;
			PWM_W1_ENABLE;
			PWM_V2_ENABLE;
			break;
		default:;
		}
	}
}
void speedRead()
{
	if(GpioDataRegs.GPADAT.bit.GPIO24)
	{
		backData.hallPos |= (0x0001<<0);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<0);
	}
	if(GpioDataRegs.GPADAT.bit.GPIO7)
	{
		backData.hallPos |= (0x0001<<1);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<1);
	}
	if(GpioDataRegs.GPADAT.bit.GPIO9)
	{
		backData.hallPos |= (0x0001<<2);
	}
	else
	{
		backData.hallPos &= ~(0x0001<<2);
	}
	LastHallGpio = backData.hallPos;
	if(VirtualTimer==0)
	{
		NewHallGpio = backData.hallPos;
	}
	if(VirtualTimer!=0)
	{
		if(LastHallGpio != NewHallGpio)
		{
			NewHallGpio = LastHallGpio;
			modcnt++;
		}
	}
	if(modcnt == 7)
	{
		SpeedNewTimer = VirtualTimer;
		backData.speed = speed_calc(SpeedNewTimer,SpeedLastTimer);
		motorRuning = 1;
		modcnt = 1;
		posCnt++;
	}
	if(modcnt == 1)
	{
		SpeedLastTimer = VirtualTimer;
	}
	VirtualTimer++;
	VirtualTimer &= 0x00007FFF;
	if (VirtualTimer == 0x7FFF)
	{
		VirtualTimer = 1;
	}
}
Uint16 speed_calc(Uint32 Timer1,Uint32 Timer2)
{
	Uint32 TimerDelay;
	Uint16 ret;
	if(Timer1<Timer2)
	{
		TimerDelay=Timer1-Timer2+32767;
	}
	else
	{
		TimerDelay=Timer1-Timer2;
	}
	ret = (Uint16)(10000/TimerDelay);
	return ret;
}
