#ifndef DSP_INIT_H
#define DSP_INIT_H
#include "MyDataType.h"
#include "DSP28x_Project.h"

/*debug相关*/
#define SPEED_CURVE1 1
#define CURRENT 0
#define SPEED 0
#define CURRENT2 0

/*曲线相关*/
#define LOW_RATE 100

// 450 800 133 733 3.375 6.75 运弹上 7jws20c-v2-1
// 500 700 70  630 7.14  7.14 运弹下

#define NOMAL_RATE_UP 450
#define TUP_ALL 800
#define TUP_T1  133
#define TUP_T2  733
#define K_UP_10MS1  3.375
#define K_UP_10MS2  6.75

#define NOMAL_RATE_DOWN 500
#define TDOWN_ALL 700
#define TDOWN_T1 70
#define TDOWN_T2 630
#define K_DOWN_10MS1 7.14
#define K_DOWN_10MS2  7.14

// 450 600 212 529 2.12 6.35 开关门  7jws20c-v2-2

//#define NOMAL_RATE_UP 450
//#define TUP_ALL 600
//#define TUP_T1  212
//#define TUP_T2  529
//#define K_UP_10MS1  2.12
//#define K_UP_10MS2  6.35
//
//#define NOMAL_RATE_DOWN 450
//#define TDOWN_ALL 600
//#define TDOWN_T1 212
//#define TDOWN_T2 529
//#define K_DOWN_10MS1 2.12
//#define K_DOWN_10MS2  6.35

/*PWM相关*/
#define PWM_PERIOD    1875//25K:1500 20K:1875 10K:3750 5K:7500
#define PWM_DUTY      1875//%0

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

//#define SET_PWM_PERCENT(n) {EPwm1Regs.CMPA.half.CMPA  = (15 * (100 -n));\
//					   EPwm1Regs.CMPB = (215 * (100 -n));\
//					   EPwm2Regs.CMPA.half.CMPA  = (15 * (100 -n));\
//					   EPwm2Regs.CMPB = (15 * (100 -n));\
//					   EPwm3Regs.CMPA.half.CMPA  = (15 * (100 -n));\
//					   EPwm3Regs.CMPB = (15 * (100 -n));}

#define LED_TOGGLE 	  GpioDataRegs.GPCTOGGLE.bit.GPIO87 = 1;

/*电流相关*/
#define FILTER_LEN 6
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
 * @brief 等腰梯形速度曲线设定
 */
//static void setVCurve(Uint16 t1,Uint16 t2,Uint16 tAll,float32 k,Uint16 maxV,Uint16 lowV);
/**
 * @brief 非等腰梯形速度曲线设定
 */
static void setVCurve1(Uint16 t1,Uint16 t2,Uint16 tAll,float32 k1,float32 k2,Uint16 maxV,Uint16 lowV);
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
/**
 * @brief 读取电流基准
 */
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
/***
 * @brief 电流均值滤波
 */
static Uint16 currentFilter(Uint16 *pBuf,Uint16 newValue);

//void setVCurve2(Uint16 t1,float32 k1,Uint16 maxV);
#endif

