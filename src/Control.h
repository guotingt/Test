#ifndef CONTROL_H
#define CONTROL_H
#include "MyDataType.h"

/*macro definition*/
#define MANUAL 0
#define AUTOMATIC 1

/***
 *@brief PID calculator
 *@param PID struct
 */
void pidCalc(PID *pPID);

#endif
