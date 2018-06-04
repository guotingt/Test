#ifndef DSP_INIT_H
#define DSP_INIT_H
#include "MyDataType.h"
#include "DSP28x_Project.h"

/*debug相关*/
#define SPEED_CURVE1 0
/*曲线相关*/
#define LOW_RATE 100

//#define NOMAL_RATE_UP 450  //450
//#define TUP_ALL 800
//#define TUP_T1  100  //70
//#define TUP_T2  700  //700
//#define K_UP_10MS  4.5
//
//#define NOMAL_RATE_DOWN 500 //500
//#define TDOWN_ALL 700
//#define TDOWN_T1 70
//#define TDOWN_T2 630
//#define K_DOWN_10MS 7.14

#define NOMAL_RATE_UP 350  //450
#define TUP_ALL 800
#define TUP_T1  133  //70
#define TUP_T2  667  //700
#define K_UP_10MS  2.625

#define NOMAL_RATE_DOWN 350 //500
#define TDOWN_ALL 800
#define TDOWN_T1 133
#define TDOWN_T2 667
#define K_DOWN_10MS 2.625

#define POS_ALL 3500
#define POS_SHUT 100

/*PWM相关*/
#define PWM_PERIOD    3750//10K 5K:7500
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
 * @brief data initialize
 */
static void dataInit();
/**
 * @brief capture speed
 */
static Uint16 speedCapture();
/**
 * @brief read posCnt
 */
static void readPulse();
/**
 * @brief filter
 */
static int16 filterCurrent(volatile Uint16* pSrc,int16 *pArray);
/**
 * @brief
 */
static void setVCurve(Uint16 t1,Uint16 t2,Uint16 tAll,float32 k,Uint16 maxV,Uint16 lowV);
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
 * @brief ISR for GPIO22 DOWN
 */
interrupt void xintUp_isr(void);
/**
 * @brief ISR for GPIO23 UP
 */
interrupt void xintDown_isr(void);
/**
 * @brief Main ISR
 */
interrupt void ISRTimer01(void);
/**
 * @brief serial port send byte
 * @param a byte
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
extern void currentBaseRead(void);
/**
 * @brief PWM update
 */
extern void pwmUpdate();
/**
 * @brief read hall state in ecap
 */
extern Uint16 readHall();
/***
 * @brief read hall state 10ms
 */
extern void readHall1();

#endif

