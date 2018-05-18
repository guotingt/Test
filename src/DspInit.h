#ifndef DSP_INIT_H
#define DSP_INIT_H
#include "MyDataType.h"
#include "DSP28x_Project.h"

#define PWM_PERIOD    3750//20K 10K:7500
#define PWM_DUTY      3375//%10 6750

#define PWM_U1_ENABLE   EPwm1Regs.AQCSFRC.bit.CSFA = 0
#define PWM_U2_ENABLE   EPwm1Regs.AQCSFRC.bit.CSFB = 0
#define PWM_U1_ON       EPwm1Regs.AQCSFRC.bit.CSFA = 1
#define PWM_U2_ON       EPwm1Regs.AQCSFRC.bit.CSFB = 1
#define PWM_U1_OFF  	EPwm1Regs.AQCSFRC.bit.CSFA = 2
#define PWM_U2_OFF  	EPwm1Regs.AQCSFRC.bit.CSFB = 2

#define PWM_V1_ENABLE   EPwm2Regs.AQCSFRC.bit.CSFA = 0
#define PWM_V2_ENABLE   EPwm2Regs.AQCSFRC.bit.CSFB = 0
#define PWM_V1_ON       EPwm2Regs.AQCSFRC.bit.CSFA = 1
#define PWM_V2_ON       EPwm2Regs.AQCSFRC.bit.CSFB = 1
#define PWM_V1_OFF  	EPwm2Regs.AQCSFRC.bit.CSFA = 2
#define PWM_V2_OFF  	EPwm2Regs.AQCSFRC.bit.CSFB = 2

#define PWM_W1_ENABLE   EPwm3Regs.AQCSFRC.bit.CSFA = 0
#define PWM_W2_ENABLE   EPwm3Regs.AQCSFRC.bit.CSFB = 0
#define PWM_W1_ON		EPwm3Regs.AQCSFRC.bit.CSFA = 1
#define PWM_W2_ON		EPwm3Regs.AQCSFRC.bit.CSFB = 1
#define PWM_W1_OFF  	EPwm3Regs.AQCSFRC.bit.CSFA = 2
#define PWM_W2_OFF  	EPwm3Regs.AQCSFRC.bit.CSFB = 2

#define PWM_OFF  	  {EPwm1Regs.AQCSFRC.bit.CSFA = 2;\
					   EPwm2Regs.AQCSFRC.bit.CSFA = 2;\
					   EPwm3Regs.AQCSFRC.bit.CSFA = 2;\
					   EPwm1Regs.AQCSFRC.bit.CSFB = 2;\
					   EPwm2Regs.AQCSFRC.bit.CSFB = 2;\
					   EPwm3Regs.AQCSFRC.bit.CSFB = 2;}

#define SET_PWM(n)    {EPwm1Regs.CMPA.half.CMPA  = n;\
					   EPwm1Regs.CMPB = n;\
					   EPwm2Regs.CMPA.half.CMPA  = n;\
					   EPwm2Regs.CMPB = n;\
					   EPwm3Regs.CMPA.half.CMPA  = n;\
					   EPwm3Regs.CMPB = n;}

#define SET_PWM_PERCENT(n) {EPwm1Regs.CMPA.half.CMPA  = 75 * (100 -n)/2;\
					   EPwm1Regs.CMPB = 75 * (100 -n)/2;\
					   EPwm2Regs.CMPA.half.CMPA  = 75 * (100 -n)/2;\
					   EPwm2Regs.CMPB = 75 * (100 -n)/2;\
					   EPwm3Regs.CMPA.half.CMPA  = 75 * (100 -n)/2;\
					   EPwm3Regs.CMPB = 75 * (100 -n)/2;}

#define LED_TOGGLE 	  GpioDataRegs.GPCTOGGLE.bit.GPIO87 = 1;

/**
 * @brief EPWM1 initialize
 * @param period
 * @param duty
 */
static void EPwm1Setup(Uint16 period,Uint16 duty);

/**
 * @brief EPWM2 initialize
 * @param period
 * @param duty
 */
static void EPwm2Setup(Uint16 period,Uint16 duty);

/**
 * @brief EPWM3 initialize
 * @param period
 * @param duty
 */
static void EPwm3Setup(Uint16 period,Uint16 duty);

/**
 * @brief ECAP1 initialize
 */
static void ECap1Setup(void);

/**
 * @brief ECAP2 initialize
 */
static void ECap2Setup(void);

/**
 * @brief ECAP3 initialize
 */
static void ECap3Setup(void);

/**
 * @brief ADC initialize
 */
static void AdcSetup(void);
/**
 * @brief DMA initialize
 */
static void DMASetup(void);
/**
 * @brief SCI initialize
 */
static void SCISetup(void);
/**
 * @brief LED GPIO configure
 */
static void configureLed(void);
/***
 * @brief configure input ISR port
 */
static void congigureSW(void);
/***
 * @breif data initialize
 */
static void dataInit();
/**
 * @brief ISR for CAP1
 */
interrupt void ISRCap1(void);
/**
 * @brief ISR for CAP2
 */
interrupt void ISRCap2(void);
/**
 * @brief ISR for CAP3
 */
interrupt void ISRCap3(void);
/**
 * @brief ISR for Timer0
 */
interrupt void ISRTimer0(void);
/**
 * @brief ISR for SCIA RX
 */
interrupt void ISRSCIARX(void);
/**
 * @brief ISR for DMA ch1
 */
interrupt void local_DINTCH1_ISR(void);
/**
 * @brief ISR for GPIO21 HANDF
 */
interrupt void xintHand_isr(void);
/**
 * @brief ISR for GPIO22 DOWN
 */
interrupt void xintUp_isr(void);
/**
 * @brief ISR for GPIO23 UP
 */
interrupt void xintDown_isr(void);
/**
 * @brief scia send
 */
extern void scia_xmit(Uint16 a);
/**
 * @brief DSP initialize
 */
extern void dsp28335Init(void);
/**
 * @brief get current data
 */
extern void currentRead(void);
/**
 * @brief get speed data
 */
extern void speedRead(void);
/**
 * @brief PWM update
 */
extern void pwmUpdate();
/**
 * @biref read hall state
 */
extern void readHall();
/**
 * @brief calc speed
 */
static Uint16 speed_calc(Uint32 Timer1,Uint32 Timer2);

static Uint16 speedCapture();

static void readPulse();

static Uint16 speedFilter(Uint16 newSpeed);

#endif
