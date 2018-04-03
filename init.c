/*============================================================================*/
/**
**      @file      INIT.C
**
**      @brief     Definizione funzioni di INIT
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/


/*===== INCLUSIONI ========================================================== */
#include "typedef.h"
#include "macro.h"
#include "stepper.h"
#include "timermg.h"
#include "color.h"
#include "ram.h"
#include "const.h"
#include "userparams.h"

/*----------------------------------------------------------------------------*/
/* ===== MACRO LOCALI ====================================================== */
/* -- Program memory macros -------------------------------------------------- */
#define __ACT_CODE_BASE (0x0000L)
#define __ACT_CODE_END  (0x57FEL)

#define __ACT_CODE_CRC   (0x57FEL)
/* ===== VARIABILI LOCALI ================================================== */
static unsigned long num_word_flash_appl;
static unsigned short CRCFlashValue;
static unsigned short CRCFlash;
/* ===== COSTANTI LOCALI =================================================== */
const unsigned short __attribute__ ((space(psv), address (__ACT_CODE_CRC)))CRCFLASH = 0;
__psv__ const unsigned short *ptrCRCFlash = (unsigned short *) __ACT_CODE_CRC;


/*====== DICHIARAZIONE FUNZIONI LOCALI ====================================== */
/******************************************************************************
* Function:     InitControlMode()
*
* Output:		None
*
* Overview:		This function sets up the stepper start-up mode
*
* Note:			None
*******************************************************************************/

void InitControlMode(void)
{
    //SET UP THE STARTUP CONTROL MODE
    #if defined(BASE_COLOR_ACTUATOR) 
    //set initial current control mode 
    uGF.currentControlLoop = ON;
    #else
    uGF.currentControlLoop = OFF;
    #endif
    
    //set initial control mode to fixed voltage (fixed current mode disabled)
    uGF.controlMode = FIXED_CURRENT;

    //set initial decay mode to fixed (alternate decay disabled
    uGF.decayMode = FIXED_DECAY;

    //set initial step size
    stepSize = ST_1_16STEP;

    //set initial control mode to speed control (disable postion control)
    uGF.positionControl = SPEED_CONTROL;

    //set initial speed source to pot
    uGF.speedSource = DMCI_REF_SPEED;

    //set decay modes for base and alternate decay
    baseDecay = D_SLOW_L_MOSFET;
    alternateDecay = D_SLOW_L_MOSFET;

    //Set initial full step Mode to Two-phase-on
    fullStepMode = FULLSTEP_WAVE_DRIVE;

    directionFlag =0;
    directionFlagOld=0;
}

/******************************************************************************
* Function:     InitStepper()
*
* Output:		None
*
* Overview:		The function initializes the controller internal variables
*
* Note:			None
*******************************************************************************/

void InitStepper(void)
{
	//reset button debounce counter
	buttonCounter = 0;
	
	//set initial speed controller variables
	speed2Time = 32767;                    //timer2/3 initial value will be loaded with this value; keep the value high
	speedOut = speedRef = 0;
	
	//set acceleration and deceleration rates
	accelerationRate = ACCELERATION_STEPS;
	decelerationRate = DECELERATION_STEPS;
	
	//acceleration and deceleration rates should not be zero
	if(accelerationRate<1) accelerationRate= 1;
	if(decelerationRate<1) decelerationRate= 1;
	
	//set initial RTDM speed value
	dmciSpeed = 826;
	
	//set initial decay to baseDecay
	decay1 = baseDecay;
	decay2 = baseDecay;
	
	//experimental value - value which works best for most of the stepping modes
	//trial and error procedure; Range is 0 to 15
	altDecayScale = 4;
	
	//reset step counter = start from 0 degrees
	stepCount = 0;
	
	//calculate step Counter increment value
	stepSizeCount = (TABLE_SIZE)>>(stepSize);
	//create copy of the stepSize variable for DMCI usage
	stepSizeCopy = stepSize;
	
	//set reference scaling value based on open/closed loop control mode
	if(uGF.currentControlLoop == OFF)
	  maxRefValue = OPEN_LOOP_VOLTAGE;
	else
	  maxRefValue = CLOSED_LOOP_CURRENT;
	
	//initialize position and position reference
	IEC0bits.T3IE = 0;      // Disable Timer2/3 interrupt
	position = 0;                 //motor actual position
	IEC0bits.T3IE = 1;      // Enable Timer2/3 interrupt
	positionRef = 0;              //desired motor position
	
	//syncronize DMCI copies with the actual variables used by the controller
	dmciPositionRefCopy = dmciPositionRef = 0;
	uGF.positionControlCopy = uGF.positionControl;
	uGF.currentControlLoopCopy = uGF.currentControlLoop;
	
	//dmci position scale resolution set to quarter step
	dmicPosRefScale = TABLE_SIZE>>2;
	
	//reset resonance compensation; the flag is set inside the speed controller at the appropiate time
	resonanceCompensation = OFF;
	
	//InitStepperPosition();	
}

/******************************************************************************
* Function:     InitPeripherals()
*
* Output:		None
*
* Overview:		This function configures the unused pins and UART
*
* Note:			None
*******************************************************************************/

void InitPeripherals(void){
  
  //set the following pins as inputs to avoid conflicts;
  //these pins are tied to other functional pins(like PWM, POT, Buttons) and they are used by other PIMs
 
  AD1PCFGL = 0x01f8;		//Port pin multiplexed with AN0-AN2 in Analog mode
  
  STEPPER_DTIVER_STANDBY =1;
  TRISAbits.TRISA10=0;
	
  
  TRISBbits.TRISB8 = 1;			//must be input
	TRISBbits.TRISB9 = 1;			//must be input

	RPOR8bits.RP16R = 0;
	RPOR8bits.RP17R = 0;
	
	//UART cfg
	TRISCbits.TRISC5 = 0;
	TRISBbits.TRISB4 = 1;
	
	//configure UART pins to RP18 and RP21
	RPINR18bits.U1RXR = 4;      //receive pin
	_RP21R = 3;                 //transmit pin
	//end UART cfg
	
	//enable nested interrupts
	INTCON1bits.NSTDIS=0;

  Init_Motor_Driver_Fault();
	Init_Start_Key();
  Init_Door_Pos();   
  Init_Motor_Voltage_Presence();
  
  Init_EV_LED();
  // Init_DC_MOTOR();
  Init_DRIVER_RESET();
  Init_DC_MOTOR();
  Init_Motor_Relais();
	#ifdef MEASURE_CHANGE_PAGE_TIME
  // Imposto il pin RB6 come pin di output
  TRISBbits.TRISB7 = 0;
  #endif
  
}


/******************************************************************************
* Function:     InitPWM()
*
* Output:		None
*
* Overview:		This function initializes the PWM
*
* Note:			None
*******************************************************************************/

void InitPWM(void)
{
  
  TRISBbits.TRISB12 = 1;	//set pwm1 pins 1 and 2 as inputs
	TRISBbits.TRISB13 = 1;
	TRISBbits.TRISB14 = 1;
	TRISBbits.TRISB15 = 1;
	
	TRISCbits.TRISC6 = 1;	//set pwm1 pins 3 and pwm2 as inputs
	TRISCbits.TRISC7 = 1;
	TRISBbits.TRISB10 = 1;
	TRISBbits.TRISB11 = 1;
	
	//clear duty cycle registers
	P1DC1 = 0;
	P1DC2 = 0;
	P1DC3 = 0;
	P2DC1 = 0;
	
	P1TPER = PWM_FCY;       // Setup PWM period
 	P2TPER = PWM_FCY;       // Setup PWM period
 	
 	PWM1CON1 = 0x0077;      // Enable PWM1H1,PWM1L1,PWM1H2,PWM1L2,PWM1H3, PWM1L3 pins for complementary mode
	PWM1CON2 = 0x0002;      // special event postcale 1:1
	// updates to the PDCx registers are sync to the PWM time base
	// Output overrides via the PxOVDCON register are sync to the PWM time base
	
	PWM2CON1 = 0x0011;      // Enable PWM2H1,PWM2L1 pins for complementary mode
	PWM2CON2 = 0x0002;      // special event postcale 1:1
	// updates to the PDCx registers are sync to the PWM time base
	// Output overrides via the PxOVDCON register are sync to the PWM time base
	
	//set dead time
	P1DTCON1bits.DTAPS = 2;  // Clck DeadTime = 4 Tcy
	P1DTCON1bits.DTA = 5;    // Dead time = 5*4*25ns = 500ns
	P2DTCON1bits.DTAPS = 2;  // Clck DeadTime = 4 Tcy
	P2DTCON1bits.DTA = 5;    // Dead time = 5*4*25ns = 500ns
	P1DTCON2 = 0;
	P2DTCON2 = 0;
	
	// P1SECMP: Special Event Compare Count Register
	// Phase of ADC capture set relative to PWM cycle: 0 offset and counting up
	P1SECMP = 1;
	
	//Override all PWM pairs; all off.
	P2OVDCON = 0;
	P1OVDCON = 0;
	
	//PWM with double interrupts and updates
	P1TCON = 0x0003;		//up/down ; continuous ; double updates
	P2TCON = 0x0003;		//up/down ; continuous ; double updates
	
	TRISBbits.TRISB12 = 0;	//set pwm1 pins 1 and 2 as outputs
	TRISBbits.TRISB13 = 0;
	TRISBbits.TRISB14 = 0;
	TRISBbits.TRISB15 = 0;
	
	TRISCbits.TRISC6 = 0;	//set pwm1 pins 3 and pwm2 as outputs
	TRISCbits.TRISC7 = 0;
	TRISBbits.TRISB10 = 0;
	TRISBbits.TRISB11 = 0;
	
	SEVTCMP = 0; 			//set ADC trigger to PWM Active pulse middle
	
	//configure fault		// RP24 - digital PIN
	RPINR12 = 24;			// assign to RP24
	RPINR13 = 24;			// assign to RP24
	TRISCbits.TRISC8 = 1;	// B5(RP24) set as input
	P1FLTACON = 0x000f;		// set fault register
	P2FLTACON = 0x000f;		// set fault register
	
	IFS3bits.FLTA1IF = 0;	//clear FLTA1 interrupt flag
	IFS4bits.FLTA2IF = 0;   //clear FLTA2 interrupt flag
	
	//activate FAULT interrupts
	IEC3bits.FLTA1IE = 1;
	IEC4bits.FLTA2IE = 1;
	//end fault cfg
	
	//interrupt config
	IFS3bits.PWM1IF=0;		//clear flag
	IEC3bits.PWM1IE=0;		//disable interrupt
	IFS4bits.PWM2IF=0;		//clear flag
	IEC4bits.PWM2IE=0;		//disable interrupt
	
	IPC14bits.PWM1IP = 5;
	
	//disable PWM
	P1TCONbits.PTEN = 0;
	P2TCONbits.PTEN = 0;
	
}

/******************************************************************************
* Function:     InitADCStepper()
*
* Output:		None
*
* Overview:		Th function initializes the ADC
*
* Note:			None
*******************************************************************************/

void InitADCStepper(void)
{
  
	AD1PCFGL = 0x01f0;		//Port pin multiplexed with AN0-AN2 in Analog mode
	
	TRISAbits.TRISA0=1;     //set the used ADC pins as inputs
	TRISAbits.TRISA1=1;
	//TRISBbits.TRISB1=1;
	TRISBbits.TRISB0=1;
	
	AD1CON1 = 0x236C;       //ADC is off
	//Stop module operation in Idle mode
	//10-bit, 4-channel ADC operation
	//Data Output Format bits Signed (sddd dddd dd00 0000)
	//Samples CH0, CH1, CH2, CH3 simultaneously when CHPS<1:0> = 1x
	//Sampling begins immediately after last conversion SAMP bit is auto-set.
	//Motor Control PWM interval ends sampling and starts conversion
	
	AD1CHS123 = 0x0000;     //MUX A CH1, CH2, CH3 negative input is VREF-
	//MUX A CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
	//MUX B CH1, CH2, CH3 negative input is VREF-
	//MUX B CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
	
	AD1CHS0 = 0x0003;       //MUX B Channel 0 negative input is VREF-
	//MUX B Channel 0 positive input is AN0
	//MUX A Channel 0 negative input is VREF-
	//MUX A Channel 0 positive input is AN3
	
	AD1CSSL = 0x0000;   	//Skip all ANx channels for input scan
	
	AD1CON3 = 0x0001;   	//ADC Clock derived from system clock
	//Autosample time time bits = 0 TAD since PWM is controlling sampling time
	//TAD = 2*TCY, TAD = 50 nSec
	
	AD1CON2 = 0x0300;   	//ADREF+ = AVDD ADREF- = AVSS
	//Do not scan inputs
	//11 = Converts CH0, CH1, CH2, and CH3
	//ADC is currently filling the first half of the buffer
	//Interrupts at the completion of conversion for every sample/convert sequence
	IPC3bits.AD1IP = 6;
	
	IFS0bits.AD1IF = 0;		// clear ADC interrupt flag
	IEC0bits.AD1IE = 1;		// enable ADC interrupt
	
	
	AD1CON1bits.ADON = 1;	//turn ADC on
}

/******************************************************************************
* Function:     InitADCSwitchOff()
*
* Output:		None
*
* Overview:		Th function initializes the ADC
*
* Note:			None
*******************************************************************************/

void InitADCSwitchOff(void)
{
  
	AD1PCFGL = 0x01f0;		//Port pin multiplexed with AN0-AN2 in Analog mode
	
	TRISAbits.TRISA0=1;     //set the used ADC pins as inputs
	TRISAbits.TRISA1=1;
	//TRISBbits.TRISB1=1;
	TRISBbits.TRISB0=1;
	
  AD1CON1 = 0x00E4;     // A/D Converter is disabled
  // Data Output Format bits: INTEGER
  // SSRC bit = 111 implies internal counter
  //Sampling begins immediately after last conversion SAMP bit is auto-set.
  
  AD1CHS123 = 0x0000;     //MUX A CH1, CH2, CH3 negative input is VREF-
  //MUX A CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
  //MUX B CH1, CH2, CH3 negative input is VREF-
  //MUX B CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
  
  AD1CHS0 = 0x0002; // Connect RB2/AN2 as CH0 input
  
  AD1CSSL = 0x0000;   	//Skip all ANx channels for input scan
  
  AD1CON3 = 0x0001;   	//ADC Clock derived from system clock
  //Autosample time time bits = 0 TAD since PWM is controlling sampling time
  //TAD = 2*TCY, TAD = 50 nSec
  
  AD1CON2 = 0x0300;   	//ADREF+ = AVDD ADREF- = AVSS
  //Do not scan inputs
  //00 = Converts CH0
  //ADC is currently filling the first half of the buffer
  
  
	AD1CON1bits.ADON = 1;	//turn ADC on
	
	StartTimer(T_ADC_SAMPLE);
}

/******************************************************************************
* Function:     InitTMR()
*
* Output:		None
*
* Overview:		The function initializes timer2/3 as a 32 bit timer
*                and timer1 as a 16 bit timer
*
* Note:			None
*******************************************************************************/

void InitTMR(void)
{
  //Timer 1 controls position/speed controller sample time
	TMR1 = 0;				// Resetting TIMER
	PR1 = SPEED_CONTROL_RATE_TIMER; // speed controller rate
	T1CON = 0x0000;			// reset timer configuration
	T1CONbits.TCKPS = 1;	// 1 = 1:8 prescaler	

	IPC0bits.T1IP = 3; 		// Set Timer 1 Interrupt Priority Level
	IFS0bits.T1IF = 0; 		// Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; 		// Enable Timer1 interrupt
	T1CONbits.TON = 1; 		// Enable Timer1
	
// 	T3CONbits.TON = 0;      // Stop any 16-bit Timer3 operation
// 	T2CONbits.TON = 0;      // Stop any 16/32-bit Timer3 operation
// 	T2CONbits.T32 = 1;      // Enable 32-bit Timer mode
// 	T2CONbits.TCS = 0;      // Select internal instruction cycle clock
// 	T2CONbits.TGATE = 0;    // Disable Gated Timer mode
// 	T2CONbits.TCKPS = 0;    // Select 1:1 Prescaler
	
// 	TMR3 = 0x00;            // Clear 32-bit Timer (msw)
// 	TMR2 = 0x00;            // Clear 32-bit Timer (lsw)
// 	PR3 = 0x0001;           // Load 32-bit period value (msw)
// 	PR2 = 0x0000;           // Load 32-bit period value (lsw)
	
// 	IPC2bits.T3IP = 4; 		// Set Timer 1 Interrupt Priority Level
	
// 	IFS0bits.T3IF = 0;      // Clear Timer3 Interrupt Flag
// 	IEC0bits.T3IE = 0;      // Enable Timer3 interrupt
// 	T2CONbits.TON = 0;      // Start 32-bit Timer
}

/******************************************************************************
* Function:     InitPI()
*
* Output:		None
*
* Overview:		The function resets the PI controller internal variables
*
* Note:			None
*******************************************************************************/
void InitPI(void)
{
     PI_err1 =0;                     //error for winding 1
     PI_out1 =0;                     //Controller output (voltage)
     PI_sum1 =0;                     //PI integral accumulator

     PI_err2 =0;                     //error for winding 1
     PI_out2 =0;                     //Controller output (voltage)
     PI_sum2 =0;                     //PI integral accumulator

}


/******************************************************************************
* Function:     InitUser()
*
* Output:		None
*
* Overview:		User initialization
*
* Note:			None
*******************************************************************************/

void InitUser(void)
{
  StartTimer(T_DEL_FILTER_DIG_IN);
  StartTimer(T_DELAY_FILTER_RESERVE_DIG_IN);
  StartTimer(T_DELAY_INIT_DONE);
}

/******************************************************************************
* Function:     InitStepperPosition()
*
* Output:		None
*
* Overview:		Move the stepper to magnetic balance
*
* Note:			None
*******************************************************************************/

void InitStepperPosition(void)
{
  
  InitADCStepper();
  InitPWM();
  
  stepCount = 0;//128;
  
  CalcStep();
  CalcDecay();
  InitPI();                   //reset PI initial state
  
  //Enable PWM
  
  DISICNT = 10;   //disable all  interrupts with priority below 7 for 10 instruction cycles
  P1TMR = 0; //sync PWM1 and PWM2 timers
  P2TMR = 1;
  P1TCONbits.PTEN = 1; //start PWM1
  P2TCONbits.PTEN = 1; //start PWM2
  
  pwmOut1 = voltageOut1;
  pwmOut2 = voltageOut2;
  
  SetPWM();

}

unsigned char CheckDisplayTestResults(void)
/*
*//*=====================================================================*//**
**
**      @brief Verifies CPU self test results from Display unit with
**      expected values.
**
**      @param void
**
**      @retval If one or more mismatch is detected returns FALSE,
**      Returns FALSE otherwise.
**
**
*//*=====================================================================*//**
*/
{
  // #ifdef FORCE_DISPLAY_CPU_FAILURE
  // return FALSE;
  // #endif

  unsigned char res = TRUE;

  // /* Expected results (cfr. MicroChip Self Test library User Guide,
  //    pg. 17), correct values for the p24FJ256GB110 are identical to
  //    the values listed in the column for p24HJ256GP610A, but the
  //    second value which is 0x6822 instead of 0x5F7F. */
  // const int expected_test_results[] = {
  //   0x694D, 0x6822, 0xC2A7, 0x00F8, 0x1BD2, 0xAE40
  // };

  // int i;
  // for (i = 0; i < 6; ++ i)
  // {
  //   if (expected_test_results[i] != TestResults[i])
  //   {
  //     res = FALSE;
  //     break;
  //   }
  // }

  return res;
}

static unsigned short CRCareaFlash(unsigned long address, unsigned long n_word,unsigned short CRCinit)
/**/
/*=======================================================================*/
/**
**      @brief Calcola CRC di una zona di FLASH specificata
**             dai parametri di ingresso                   
**             L'accesso è eseguito a word
**
**      @param : address      Indirizzo iniziale dell'area
**                            da controllare
**               n_word       Numero dei bytes da includere nel calcolo
**               CRCinit      Valore iniziale di CRC ( = 0 se n_char 
**                            copre l'intera zona da verificare, 
**                            = CRCarea della zona precedente se
**                            si sta procedendo a blocchi
**
**      @retval CRCinit
**/
/*=======================================================================*/
/**/
{
  unionDWord_t dwvResult;
  /* La routine proviene dalla dispensa "CRC Fundamentals", pagg. 196, 197. */

  /* Nota sull'algoritmo:
     dato un vettore, se ne calcoli il CRC_16: se si accodano i 2 bytes del
     CRC_16 a tale vettore (low byte first!!!), il CRC_16 calcolato
     sul vettore così ottenuto DEVE valere zero.
     Tale proprietà può essere sfruttata nelle comunicazione seriali
     per verificare che un messaggio ricevuto,
     contenente in coda i 2 bytes del CRC_16 (calcolati dal trasmettitore),
     sia stato inviato correttamente: il CRC_16, calcolato dal ricevente
     sul messaggio complessivo deve valere zero. */

  unsigned long i;
  unsigned char j;
  unsigned short index;
  unsigned char psv_shadow;

  unsigned short wTBLPAGSave;

  /* save the PSVPAG */
  psv_shadow = PSVPAG;
  /* set the PSVPAG for accessing CRC_TABLE[] */
  PSVPAG = __builtin_psvpage (CRC_TABLE);

  for (i = 0; i < n_word; i++)
    {
      /* Reset Watchdog*/
      // ClrWdt();

      wTBLPAGSave = TBLPAG;
      TBLPAG = ((unionDWord_t*)&address)->word[1];

      dwvResult.word[1] = __builtin_tblrdh((unsigned short)address);
      dwvResult.word[0] = __builtin_tblrdl((unsigned short)address);
      TBLPAG = wTBLPAGSave;
      for (j=0; j<4; j++)
        {
          index = ( (CRCinit ^ ( (unsigned short) dwvResult.byte[j] & 0x00FF) ) & 0x00FF);
          CRCinit = ( (CRCinit >> 8) & 0x00FF) ^ CRC_TABLE[index];
        }
      address+=2;
    } /* end for */

  /* restore the PSVPAG for the compiler-managed PSVPAG */
  PSVPAG = psv_shadow;

  return CRCinit;
} /* end CRCareaFlash */



void CheckFlash(void)
/*
*//*=========================================================================*//**
**
**      @brief        : Controllo integrità memoria Flash
**
**      @param        : void
**
**      @retval       : void
**
*//*=========================================================================*//**
*/

{
  num_word_flash_appl = (__ACT_CODE_END - __ACT_CODE_BASE)/2;
  CRCFlash=CRCareaFlash(__ACT_CODE_BASE, num_word_flash_appl, 0);
  CRCFlashValue = (*ptrCRCFlash);

 if (CRCFlash != CRCFlashValue)
   {
     #ifndef DISABLE_CRC_FLASH
     while(1)
     {
       Sleep();
     }
     #endif
   }
}
