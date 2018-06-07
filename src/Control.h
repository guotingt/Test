#ifndef CONTROL_H
#define CONTROL_H
#include "MyDataType.h"

/***
 *@brief PID calculator
 *@param PID struct
 */
extern void pidCalc(PID *pPID);
/***
 * @brief reset PID
 * @param PID struct
 */
extern void pidReset(PID *pPID);

#endif
