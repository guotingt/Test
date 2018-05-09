#ifndef DSP_INIT_H
#define DSP_INIT_H
#include "MyDataType.h"
#include "DSP28x_Project.h"

#define PWM_PERIOD    7500
#define PWM_DUTY      6750
#define PWM_U1_DISABLE  EPwm1Regs.AQCSFRC.bit.CSFA = 2
#define PWM_U2_DISABLE  EPwm1Regs.AQCSFRC.bit.CSFB = 2
#define PWM_U1_ENABLE   EPwm1Regs.AQCSFRC.bit.CSFA = 0
#define PWM_U2_ENABLE   EPwm1Regs.AQCSFRC.bit.CSFB = 0
#define PWM_V1_DISABLE  EPwm2Regs.AQCSFRC.bit.CSFA = 2
#define PWM_V2_DISABLE  EPwm2Regs.AQCSFRC.bit.CSFB = 2
#define PWM_V1_ENABLE   EPwm2Regs.AQCSFRC.bit.CSFA = 0
#define PWM_V2_ENABLE   EPwm2Regs.AQCSFRC.bit.CSFB = 0
#define PWM_W1_DISABLE  EPwm3Regs.AQCSFRC.bit.CSFA = 2
#define PWM_W2_DISABLE  EPwm3Regs.AQCSFRC.bit.CSFB = 2
#define PWM_W1_ENABLE   EPwm3Regs.AQCSFRC.bit.CSFA = 0
#define PWM_W2_ENABLE   EPwm3Regs.AQCSFRC.bit.CSFB = 0
#define PWM_DISABLE   {EPwm1Regs.AQCSFRC.bit.CSFA = 2;\
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
 * @brief
 */
static Uint16 speed_calc(Uint32 Timer1,Uint32 Timer2);
#endif
