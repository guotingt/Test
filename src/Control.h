#ifndef CONTROL_H
#define CONTROL_H
#include "MyDataType.h"

/*macro definition*/
#define MANUAL 0
#define AUTOMATIC 1
#define POSITIVE 0
#define NEGITIVE 1 

/***
 *@brief PID calculator
 *@param PID
 */
void pidCalc(PID *pPID);

#endif
