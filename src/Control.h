#ifndef CONTROL_H
#define CONTROL_H

#include "MyDataType.h" //PID�ṹ�嶨��

/***
 *@brief PID�㷨
 *@param PID�ṹ�����������speedPID��currentPID
 */
extern void pidCalc(PID *pPID);
/***
 * @brief PID��������
 * @param PID�ṹ�����������speedPID��currentPID
 */
extern void pidReset(PID *pPID);

#endif
