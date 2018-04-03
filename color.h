/**/
/*============================================================================*/
/**
**      @file      color.h
**
**      @brief     Header file relativo a color.c
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#ifndef _COLOR_H_
#define _COLOR_H_

/*===== INCLUSIONI ========================================================= */
#include "serialCom.h"

/*===== DICHIARAZIONI LOCALI ================================================*/
/*===== DEFINE GENERICHE ====================================================*/
#define SW_VERSION_COLOR 0x209401

#define DELTA_RIGHT_POS_0 -8
#define NUM_STEP_PER_ML 28678L /*count*/
#define NUM_COUNT_PER_MM 16131L

#define    PHOTOC_DARK  1  //fotocellula occlusa
#define    PHOTOC_LIGHT 0  //fotocellula libera

#define  POS_0  0


//Inputs

  #define    Init_Motor_Driver_Fault()      TRISCbits.TRISC2=1
  #define    MOTOR_DRIVER_FAULT             PORTCbits.RC2
  
  #define    Init_Start_Key()               TRISCbits.TRISC3=1
  #define    START_KEY                      PORTCbits.RC3
  
  #define    Init_Door_Pos()                TRISCbits.TRISC4=1
	#define    DOOR_POS                       PORTCbits.RC4  //S1 button port pin   
	
  #define    Init_Motor_Voltage_Presence()  TRISBbits.TRISB1=1
	#define    MOTOR_VOLTAGE_PRESENCE         PORTBbits.RB1  //S1 button port pin  
	
	#define    Init_Time_Selector()           TRISBbits.TRISB2=1; TRISBbits.TRISB3=1;                                          \
	                                          TRISAbits.TRISA4=1; TRISAbits.TRISA9=1; TRISBbits.TRISB7=1
	#define    TIME_SEL_BIT_0                 PORTBbits.RB7    
	#define    TIME_SEL_BIT_1                 PORTAbits.RA9  	
  #define    TIME_SEL_BIT_2                 PORTAbits.RA4 
  #define    TIME_SEL_BIT_3                 PORTBbits.RB3 
  #define    TIME_SEL_BIT_4                 PORTBbits.RB2 
//Outputs

  #define    Init_EV_LED()                LATCbits.LATC1=0; TRISCbits.TRISC1 = 0;			//must be output 
  // #define    Init_DC_MOTOR()                LATCbits.LATC0=0; TRISCbits.TRISC0 = 0;			//must be output
  #define    Init_DRIVER_RESET()          LATAbits.LATA7=1; TRISAbits.TRISA7 = 0;			//must be output 
  #define    Init_DC_MOTOR()              LATCbits.LATC0=1; TRISCbits.TRISC0 = 0;			//must be output 
  #define    Init_Motor_Relais()          LATBbits.LATB15=0; TRISBbits.TRISB15 = 0;			//must be output 


#define     DC_MOTOR                      LATCbits.LATC0  //S1 button port pin
#define     EV_LED                        LATCbits.LATC1
#define     DRIVER_RESET                  LATAbits.LATA7
#define     MOTOR_RELAIS                  LATBbits.LATB15
  
/*Settaggio registri*/
#define PWM_FREQUENCY_KHZ   20
#define PWM_PERIOD_NS       (unsigned long)(TEN_6_POW/PWM_FREQUENCY_KHZ)
#define PWM_CLOCK_TOSC_PS   25000

enum {
  PWM_STEP_0 = 0,
  PWM_STEP_1,
  PWM_STEP_2,
  PWM_STEP_3,
  PWM_STEP_4,
  PWM_STEP_5,
  PWM_STEP_6,
  PWM_STEP_7,
  PWM_STEP_8,
  PWM_STEP_9,
  PWM_STEP_10 };

#define NUM_STEP_DC_MOTOR           10

#define PERIOD_VALUE_DC_MOTOR         (((PWM_PERIOD_NS*TEN_3_POW)/PWM_CLOCK_TOSC_PS)-1)
#define PERIOD_STEP_VALUE_DC_MOTOR    (PERIOD_VALUE_DC_MOTOR/NUM_STEP_DC_MOTOR)


#define PWM_DC_MOTOR_STEP_0_PCT     (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_0)            
#define PWM_DC_MOTOR_STEP_10_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_1)
#define PWM_DC_MOTOR_STEP_20_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_2)
#define PWM_DC_MOTOR_STEP_30_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_3)
#define PWM_DC_MOTOR_STEP_40_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_4)
#define PWM_DC_MOTOR_STEP_50_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_5)
#define PWM_DC_MOTOR_STEP_60_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_6)
#define PWM_DC_MOTOR_STEP_70_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_7)
#define PWM_DC_MOTOR_STEP_80_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_8)
#define PWM_DC_MOTOR_STEP_90_PCT    (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_9)
#define PWM_DC_MOTOR_STEP_100_PCT   (PERIOD_STEP_VALUE_DC_MOTOR*PWM_STEP_10)

#define DC_VALUE_MOTOR_OFF           0
#define DC_VALUE_MOTOR_ON      PWM_DC_MOTOR_STEP_30_PCT  
  
#define PWM_DC_MOTOR_MAXIMUM_DUTY 100 //% 
#define PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_DW 0 //% 
#define PWM_DC_MOTOR_MAXIMUM_DUTY_RAMP_DW 50 //%0 
#define PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_UP 20 //% 

#define     DC_MOTOR_OFF()                  OC1RS = DC_VALUE_MOTOR_OFF
// #define     DC_MOTOR_ON(x)                  OC1RS = (PERIOD_VALUE_DC_MOTOR*x/100)
#define     HOMING_DC_MOTOR_ON()            OC1RS = DC_VALUE_MOTOR_ON

#define     EV_LED_OFF()                    EV_LED = OFF; LEDStatus = OFF
#define     EV_LED_ON()                     EV_LED = ON;  LEDStatus = ON



// #define     DC_MOTOR_OFF()  DC_MOTOR = ON
// #define     DC_MOTOR_ON()   DC_MOTOR = OFF

extern void initDC_Motor_Braking(void);  


#define IS_MOTOR_DRIVER_FAULT() (!DigInStatus.Bit.StatusType5)
#define IS_START_KEY_PRESSED() (!DigInStatus.Bit.StatusType6)
#define IS_DOOR_OPEN() (DigInStatus.Bit.StatusType7)
#define IS_DOOR_CLOSED() (!DigInStatus.Bit.StatusType7)
#define IS_MOTOR_VOLTAGE_PRESENCE() (DigInStatus.Bit.StatusType8)


#define STIRRING_TIME_25_SEC 25
#define STIRRING_TIME_40_SEC 40
#define STIRRING_TIME_55_SEC 55
#define STIRRING_TIME_85_SEC 85

  // valori inviati dalla SCCBrd per indicare lo stato
  enum
  {
	/*0*/    COLOR_INIT_ST,
	/*1*/    COLOR_STANDBY_DOOR_CLOSED_ST,	
	/*2*/    COLOR_STANDBY_DOOR_OPEN_ST,
	/*3*/    COLOR_STIRRING_RUN_ST,
	/*4*/    COLOR_MOTOR_BRAKING_ST,
	/*5*/    COLOR_SECURITY_TEST_ST,
	/*6*/    COLOR_ERROR_ST,
	};
	// error code
	enum
	{
	/*0*/    NO_ERROR,
	/*1*/    COLOR_SERIAL_TOUT_ERROR
	};

// enum degli step
enum
{
  STEP_0,
  STEP_1,
  STEP_2,
  STEP_3,
  STEP_4,
  STEP_5,
  STEP_6,
  STEP_7,
  STEP_8,
  STEP_9,
  STEP_10,
  STEP_11,
  STEP_12,
  STEP_13,
  STEP_14,
  STEP_15,
  STEP_16,
  STEP_17,
  STEP_18,
  STEP_19
};

	// error code  procedure
	enum
	{
	/*0*/    PROC_RUN,
	/*1*/    PROC_END,
  /*2*/    PROC_ERROR,
	};
	
		// LEDType
	enum
	{
	/*0*/    LED_OFF,
	/*1*/    LED_ON,	
	/*2*/    LED_BLINK_SLOW,
  /*3*/    LED_BLINK_FAST,
	};

/*===== TIPI ================================================================*/
typedef struct
{
	unsigned char typeMessage;
  unsigned short stirringDuration;
 
  union __attribute__ ((packed))
    {
      unsigned char cmd;
      struct
      {
        unsigned char stop    : 1;
        unsigned char homing    : 1;
        unsigned char run : 1;
        unsigned char unused    : 5;
      };     
    } command;

  union __attribute__ ((packed))
    {
      unsigned long allFlags;
      unsigned char byteFlags[4];
    } colorFlags;
} colorAct_t;


/*===== PROTOTIPI FUNZIONI ==================================================*/
extern void MakeColorMessage(uartBuffer_t *txBuffer, unsigned char slave_id);
extern void DecodeColorMessage(uartBuffer_t *rxBuffer,unsigned char slave_id);
extern void  initColor(void);
extern void  colorManager(void);
extern void setLED(unsigned char type);

#define isColorCmdStopping()   (color.command.stop)
#define isSpeedRamp(x, y)   (x || y)


#define isColorCmdHoming()   (color.command.homing)
#define isColorCmdRun()   (color.command.run)


#endif
