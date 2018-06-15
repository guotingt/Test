#ifndef CONTROL_H
#define CONTROL_H

#include "MyDataType.h" //PID结构体定义

/***
 *@brief PID算法
 *@param PID结构体参数，例如speedPID和currentPID
 */
extern void pidCalc(PID *pPID);
/***
 * @brief PID参量重置
 * @param PID结构体参数，例如speedPID和currentPID
 */
extern void pidReset(PID *pPID);

#endif
