#ifndef __PID_H__
#define __PID_H__

void PIDLoopControl();
int GetAbsPresPIDPercent();
int GetDiffPresPIDPercent();
int GetTempPIDPercent();
int GetRHPIDPercent();

#endif
