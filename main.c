/**/
/*============================================================================*/
/**
**      @file      MAIN.C
**
**      @brief     Main
**
**      @version   Alfa Color Tester
**/
/*============================================================================*/
/**/

/*===== INCLUSIONI ========================================================== */
#include "stepper.h"
#include "serialCom.h"
#include "color.h"
#include "macro.h"
#include "INPUT.h"
#include "TIMERMG.h"

/* ===== SETTING CONFIGURATION BITS================================================== */

_FBS (BSS_NO_FLASH & BWRP_WRPROTECT_OFF);
/* no Boot sector and
   write protection disabled */

_FOSCSEL (IESO_OFF & FNOSC_FRC);
/* The chip is started using FRC then switch */

_FWDT (FWDTEN_OFF);
/* Turn off Watchdog Timer */

#ifdef DEBUG_SLAVE
_FGS (GSS_OFF & GCP_OFF & GWRP_OFF);
/* Set Code Protection Off for the General Segment */
#else
_FGS (GSS_STD & GWRP_OFF);
/* Set Code Protection Standard Security for the General Segment */
#endif

_FOSC (FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_OFF & POSCMD_XT);
/* clock switch & monitoring disabled
   remapable I/O enabled
   OSC2 pin is clock O/P
   oscilator is XT
*/

_FPOR (PWMPIN_ON & HPOL_ON & LPOL_ON & FPWRT_PWR128);
/* PWM mode is Port registers
   PWM high & low active high
   FPOR power on reset 128ms
*/
_FICD(JTAGEN_OFF & ICS_PGD3);
/* JTAG Enable Bit: JTAG is disabled
   ICD communication channel select bits: communicate on PGC3/EMUC3 and PGD3/EMUD3 
*/
/* ===== MACRO LOCALI ====================================================== */
				
/* ===== TIPI LOCALI ======================================================= */
/* ===== VARIABILI LOCALI ================================================== */

/* ===== COSTANTI LOCALI =================================================== */

/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */
/*====== DICHIARAZIONE FUNZIONI LOCALI ====================================== */


int main(void)
{

	//Perform a clock switch to 40MIPS (80Mhz)
	PLLFBD = 38;			//M=PLLFBD+2       = 38
	CLKDIVbits.PLLPOST = 0;	//N2=2(PLLPOST+1)  = 2
	CLKDIVbits.PLLPRE = 0;	//N1=PLLPRE+2      = 2
	
	//unlock OSCCON register
	__builtin_write_OSCCONH(0x03);
	__builtin_write_OSCCONL(0x01);
	
    //wait for clock to stabilize	
	while(OSCCONbits.COSC != 0b011);
	//wait for PLL to lock
	while(OSCCONbits.LOCK !=1);
    //clock switch finished
    
  CheckFlash();
    
  //call init functions
	// InitADCSwitchOff();  
	InitPeripherals();
	
	InitTMR();
	initDC_Motor_Braking();
  InitUser();
  // initSerialCom();
  initColor();
  
  
    //set Statemachine initial state
    state = STATE_INIT;
  
    // StartTimer(T_SLAVE_WAIT_LINK_TIMER);
    
    // StartTimer(T_DELAY_SHUTDOWN);
    
    while(1)
    {      

      // StepperStateMachine();
      
      // serialCommManager();
      
      colorManager();
      
      inputManager();
      
      TimerMg();
    }
}	


