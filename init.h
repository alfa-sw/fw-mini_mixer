/**/
/*============================================================================*/
/**
**      @file      INIT.H
**
**      @brief     Header file relativo a INIT.C
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#ifndef INIT_H
#define INIT_H




void InitPeripherals (void);
void InitPWM(void);
void InitADCSwitchOff(void);
void InitTMR(void);
void InitStepper(void);
void InitPI(void);
void InitControlMode(void);
void InitUser(void);
void InitStepperPosition(void);

unsigned char CheckDisplayTestResults(void);
void CheckFlash(void);
#endif
