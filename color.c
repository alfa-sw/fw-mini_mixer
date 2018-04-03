/**/
/*============================================================================*/
/**
**      @file      color.c
**
**      @brief     Definizione funzioni di gestione scheda base/colorante SCCBRD
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

/*======================== INCLUSIONI ======================================= */

#include "typedef.h"
#include "macro.h"
#include "ram.h"
#include "color.h"
#include "const.h"
#include "serialCom.h"
#include "stepper.h"
#include "input.h"
#include "timermg.h"
#include "init.h"
#include <stdio.h>

/* ===== MACRO LOCALI ====================================================== */
#define N_MAX_TEST_RETRY 1
#define DELAY_LED_BLINK_ON_SLOW  500 //2 sec
#define DELAY_LED_BLINK_OFF_SLOW 500 //1 sec

#define DELAY_LED_BLINK_ON_FAST  250 // 500 ms 
#define DELAY_LED_BLINK_OFF_FAST 250 // 500 ms

/* ===== TIPI LOCALI ======================================================= */
/* ===== VARIABILI LOCALI ================================================== */
/* ===== COSTANTI LOCALI =================================================== */
/* ===== VARIABILI LOCALI ================================================== */
static unsigned long pwm_duty, pwm_duty_max, pwm_duty_ramp;
static unsigned short time;
static unsigned char testRetry = 0;
/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */
static void  StatusManager(void);
static unsigned char DC_MotorBraking(void);
static unsigned char DC_MotorStirring(void);
static unsigned char DC_MotorTesting(void);
static void EV_LedManager(void);
static void DC_MotorOn(unsigned char x);
static unsigned char isBrakingCmd(void);
/* ===== DEFINIZIONE FUNZIONI LOCALI ======================================= */

void MakeColorMessage(uartBuffer_t *txBuffer, unsigned char slave_id)
/*
*//*=====================================================================*//**
**      @brief Create the serial message for  MABRD
**
**      @param txBuffer pointer to the tx buffer
**
**      @param slave_id slave identifier
**
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  unsigned char idx = 0;
  /* initialize tx frame, reserve extra byte for pktlen */
  FRAME_BEGIN(txBuffer, idx, 100 + slave_id); 
  STUFF_BYTE( txBuffer->buffer, idx, color.typeMessage);
  STUFF_BYTE( txBuffer->buffer, idx, Status.level);
  STUFF_BYTE( txBuffer->buffer, idx, Status.errorCode);
  
  /* crc, pktlen taken care of here */
  FRAME_END( txBuffer, idx);
}

void DecodeColorMessage(uartBuffer_t *rxBuffer,unsigned char slave_id)
/*
*//*=====================================================================*//**
**      @brief Decode the serial message received from Display
**
**      @param rxBuffer pointer to the rx buffer
**
**      @param slave_id slave identifier
**
**      @retval void
**
*//*=====================================================================*//**
*/
{  
  unsigned char idx = FRAME_PAYLOAD_START;
  unionWord_t  tmpWord1;
  
  color.typeMessage = rxBuffer->buffer[idx++];
  color.command.cmd = rxBuffer->buffer[idx++];
  switch (color.typeMessage)
  {
  
  default:
    break;
  }
  
}

void  initColor(void)
/*
*//*=====================================================================*//**
**      @brief Initialize color actuators status after Power on
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  Status.level = COLOR_INIT_ST;
  OldStatus.level =  Status.level;
}

void initDC_Motor_Braking(void)
/*
*//*=====================================================================*//**
**      @brief init PWM for DC MOTOR control
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
     //RP16 = OC1
  RPOR8bits.RP16R = 18;
  
  /*Settings for OC1 - PWM DC MOTOR*/
  OC1R = DC_VALUE_MOTOR_OFF;
  OC1RS = DC_VALUE_MOTOR_OFF;
  OC1CON = 0x0006;
 
	
  T2CONbits.TON = 0;      // Stop any 16/32-bit Timer3 operation
	T2CONbits.TCS = 0;      // Select internal instruction cycle clock
	T2CONbits.TGATE = 0;    // Disable Gated Timer mode
	T2CONbits.TCKPS = 0;    // Select 1:1 Prescaler
	
	TMR2 = 0x00;            // Clear 32-bit Timer (lsw)
	PR2 = PERIOD_VALUE_DC_MOTOR;           // Load 16-bit period value
		
	IFS0bits.T2IF = 0;      // Clear Timer2 Interrupt Flag
	IEC0bits.T2IE = 0;      // Enable Timer2 interrupt
	T2CONbits.TON = 1;      // Start 16-bit Timer
}

static unsigned char isBrakingCmd(void)
/*
*//*=====================================================================*//**
**
**      @brief Return TRUE if braking cmd 
**
**      @param void 
** 
**      @retval TRUE/FALSE
**
**
*//*=====================================================================*//**
*/
{
  unsigned char ret = FALSE;
  if ((DC_MotorStirring() == PROC_END) || IS_DOOR_OPEN() || (KeyboardStatus == KEY_START_STOP)||
    (!IS_MOTOR_VOLTAGE_PRESENCE() && (StatusTimer(T_STIRRING_RUN) == T_RUNNING)))
  {
    ret = TRUE;
  }  
  return(ret);
}


static void  StatusManager(void)
/*
*//*=====================================================================*//**
**      @brief Updates container status
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
{  
  unsigned char statusTest,statusBraking;
  
  if (StatusTimer(T_DELAY_INIT_DONE) == T_ELAPSED && IS_MOTOR_DRIVER_FAULT())
  {
    Status.level = COLOR_ERROR_ST; 
    Nop();
    Nop();
  }  
  switch(Status.level)
  {
  case COLOR_INIT_ST:
    setLED(LED_ON);
    if (StatusTimer(T_DELAY_INIT_DONE) == T_ELAPSED)
    {
      if (IS_DOOR_OPEN())
      {
        Status.level = COLOR_STANDBY_DOOR_OPEN_ST;
      }
      else if (IS_DOOR_CLOSED())
      {
        Status.level = COLOR_STANDBY_DOOR_CLOSED_ST;
        MOTOR_RELAIS = ON;
      }  
    }
    break;
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_STANDBY_DOOR_CLOSED_ST:
    if (IS_DOOR_OPEN())
    {
      Status.level = COLOR_SECURITY_TEST_ST;
    }  
    else if (KeyboardStatus == KEY_START_STOP)
    {
      Status.level = COLOR_STIRRING_RUN_ST;
      Durata[T_STIRRING_RUN] = color.stirringDuration*CONV_SEC_COUNT;     
      KeyboardStatus = NO_TASTI;
    }  
    break;
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_STANDBY_DOOR_OPEN_ST:
    if (IS_DOOR_CLOSED())
    {
      Status.level = COLOR_STANDBY_DOOR_CLOSED_ST;
      MOTOR_RELAIS = ON;
    }  
    break;    
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_STIRRING_RUN_ST:    
    setLED(LED_BLINK_SLOW);
    if (isBrakingCmd())
    {
      Status.level = COLOR_MOTOR_BRAKING_ST;
      MOTOR_RELAIS = OFF;
      if (KeyboardStatus == KEY_START_STOP)
      {
        //Braking due to KEY PRESSED
        StartTimer(T_DELAY_ACTIVATE_RELAIS); 
        KeyboardStatus = NO_TASTI; 
      }
      if (StatusTimer(T_STIRRING_RUN) != T_ELAPSED)
      {
        //Braking due to door open
        StartTimer(T_TEST_MOTOR_BRAKING); 
        // EV_LED_OFF();
      }
      else
      {
      //Braking due to time elapsed
        StartTimer(T_DELAY_ACTIVATE_RELAIS); 
      }  
      StopTimer(T_STIRRING_RUN);
    }
    break;
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_MOTOR_BRAKING_ST:
    statusBraking = DC_MotorBraking();
    if (IS_DOOR_OPEN())
    {
      if (statusBraking == PROC_END)
      {
        Status.level = COLOR_SECURITY_TEST_ST;         
      }
      else if (StatusTimer(T_TEST_MOTOR_BRAKING) == T_HALTED)
      {
        StartTimer(T_TEST_MOTOR_BRAKING);  
      }      
    }
    else  if ((IS_DOOR_CLOSED()) && (statusBraking == PROC_END))
    {
      Status.level = COLOR_STANDBY_DOOR_CLOSED_ST;      
      MOTOR_RELAIS = ON;
    }  
    break;
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_SECURITY_TEST_ST:
    setLED(LED_ON);
    statusTest = DC_MotorTesting();
    if (statusTest == PROC_END)
    {
      if (IS_DOOR_CLOSED())
      {
        Status.level = COLOR_STANDBY_DOOR_CLOSED_ST;
      }
      else if (IS_DOOR_OPEN())
      {
        Status.level = COLOR_STANDBY_DOOR_OPEN_ST;
      }   
    }
    else if (statusTest == PROC_ERROR)
    {
      Status.level = COLOR_ERROR_ST; 
      Nop();
      Nop();
    }  
    break;
    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
  case COLOR_ERROR_ST:    
    setLED(LED_BLINK_FAST);
    DC_MOTOR_OFF();
    break;  
  }
  
  /*change status*/
  if (Status.level != OldStatus.level)
  {
    OldStatus.level = Status.level;
    Status.step = STEP_0;
  }  
  
}

static unsigned char DC_MotorStirring(void)
/**/
/*===========================================================================*/
/**
**   @brief Returns result of stirring procedure
**
**   @param void
**
**   @return PROC_RUN/PROC_END/PROC_ERROR
**/
/*===========================================================================*/
/**/
{
  unsigned char ret = PROC_RUN;
  unsigned long ltemp;
  
  switch(Status.step)
  {
  case STEP_0:
    StopTimer(T_TEST_MOTOR_BRAKING);
    StartTimer(T_RAMP_UP);
    pwm_duty = 0;
    Status.step++;
    break;
    
  case STEP_1:    
    if ((StatusTimer(T_RAMP_UP) == T_RUNNING) || (StatusTimer(T_RAMP_UP) == T_ELAPSED))
    {
      time = ReadTimer(T_RAMP_UP);
      if (time >= Durata[T_RAMP_UP])
      {
        pwm_duty = PWM_DC_MOTOR_MAXIMUM_DUTY;
        StartTimer(T_STIRRING_RUN);
        Status.step++;
      }
      else
      {
        ltemp =(unsigned long)(time)*(PWM_DC_MOTOR_MAXIMUM_DUTY - PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_UP)/Durata[T_RAMP_UP];
        pwm_duty = PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_UP + (unsigned char)(ltemp);
      }
    }
    break;
    
  case STEP_2:
    pwm_duty = PWM_DC_MOTOR_MAXIMUM_DUTY;  
    if (StatusTimer(T_STIRRING_RUN) == T_ELAPSED)
    {
      Status.step++;
    }  
    break;
  
  case STEP_3:
    ret = PROC_END;
    break;      
    
  }
  // EV_LED_ON();
  DC_MotorOn(pwm_duty);
  pwm_duty_max = pwm_duty;
  return ret;
}

static unsigned char DC_MotorBraking(void)
/**/
/*===========================================================================*/
/**
**   @brief Returns result of stirring procedure
**
**   @param void
**
**   @return PROC_RUN/PROC_END/PROC_ERROR
**/
/*===========================================================================*/
/**/
{
  unsigned char ret = PROC_RUN;
  unsigned long ltemp;
  
  switch(Status.step)
  {
  case STEP_0:
    StartTimer(T_RAMP_DW);
    pwm_duty = pwm_duty_max;    
    Status.step++;
    break;
    
  case STEP_1:    
    if (StatusTimer(T_DELAY_ACTIVATE_RELAIS) == T_ELAPSED)
    {
      MOTOR_RELAIS = OFF;
      StopTimer(T_DELAY_ACTIVATE_RELAIS);
    }
    
    if (StatusTimer(T_TEST_MOTOR_BRAKING) == T_ELAPSED)
    {
      NotRunningTimer(T_TEST_MOTOR_BRAKING);
      pwm_duty_ramp = pwm_duty;
      pwm_duty = PWM_DC_MOTOR_MAXIMUM_DUTY;
      StartTimer(T_TEST_MOTOR);
      Status.step = STEP_3;
    }  
    else if ((StatusTimer(T_RAMP_DW) == T_RUNNING) || (StatusTimer(T_RAMP_DW) == T_ELAPSED))
    {
      time = ReadTimer(T_RAMP_DW);
      if (time >= Durata[T_RAMP_DW])
      {
        pwm_duty = PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_DW;    
        Status.step++;
      }
      else
      {
        ltemp =((unsigned long)(Durata[T_RAMP_DW] - time)*pwm_duty_max + (unsigned long)(PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_DW * time))/Durata[T_RAMP_DW];
        pwm_duty = (unsigned char)(ltemp);
      }      
    }
    break;
    
  case STEP_2:
    pwm_duty = PWM_DC_MOTOR_MINIMUM_DUTY_RAMP_DW;
    setLED(LED_ON);    
    ret = PROC_END;
    break;
  
  case STEP_3:
    if (StatusTimer(T_TEST_MOTOR) == T_ELAPSED)
    {
      if (!MOTOR_VOLTAGE_PRESENCE)
      {
        //Force output to zero
        Status.step = STEP_2;      
      }
      else
      {
        //Continue ramp
        Status.step = STEP_1; 
        pwm_duty = pwm_duty_ramp;
      }
      MOTOR_RELAIS = OFF;
    }  
    break;  
    
  }
  DC_MotorOn(pwm_duty);
  return ret;
}

static unsigned char DC_MotorTesting(void)
/**/
/*===========================================================================*/
/**
**   @brief Returns result of motor test procedure
**
**   @param void
**
**   @return PROC_RUN/PROC_END/PROC_ERROR
**/
/*===========================================================================*/
/**/
{
  unsigned char ret = PROC_RUN;
  
  switch(Status.step)
  {
   case STEP_0:
    StartTimer(T_DELAY_BEFORE_TEST_MOTOR);
    Status.step++;
    break;
  case STEP_1:
    if (StatusTimer(T_DELAY_BEFORE_TEST_MOTOR) == T_ELAPSED)
    {
      if (IS_DOOR_OPEN())
      {
        #ifndef DISABLE_SECURITY_TEST
        //Module disabled - OC1 pin controlled by GPIO register
        OC1CON = 0x0000;
        DC_MOTOR = ON;
        StartTimer(T_TEST_MOTOR);
        Status.step++;
        #else
        Status.step = STEP_2;
        #endif
      }
      else
      {
        Status.step = STEP_4;
        testRetry = 0;
      }  
    }
    break;
    
  case STEP_2:   
    if (StatusTimer(T_TEST_MOTOR) == T_ELAPSED)
    {
      StopTimer(T_TEST_MOTOR);
      if (IS_DOOR_OPEN())
      {
        //Check 24 V presence
        if (MOTOR_VOLTAGE_PRESENCE)
        {
          if (testRetry < N_MAX_TEST_RETRY)
          {
          testRetry++;
          Status.step = STEP_5;
          StartTimer(T_RETRY_TEST_MOTOR);
          }
          else
          {  
          Status.step = STEP_3;
          }
        }
        else
        {
          Status.step = STEP_4;
          testRetry = 0;
        }  
      }
      else
      {
        Status.step = STEP_4;
        testRetry = 0;
      }  
      DC_MOTOR = OFF;
      //Module enabled - OC1 pin controlled by ouput compare
      OC1CON = 0x0006;
    }  
    break;
    
  case STEP_3:   
    ret = PROC_ERROR;
    break;
  
  case STEP_4:   
    ret = PROC_END;
    break;  
  
  case STEP_5:   
    if (StatusTimer(T_RETRY_TEST_MOTOR) == T_ELAPSED)
    {
      Status.step = STEP_1;
    }  
    break;    
    
  }
  return ret;
}

void setLED(unsigned char type)
/**/
/*===========================================================================*/
/**
**   @brief  Set LED type 
**
**   @param  void
**
**   @return void
**/
/*===========================================================================*/
/**/
{
  switch(type)
  {
  case LED_BLINK_SLOW:
    Durata[T_OFF_BLINK]= DELAY_LED_BLINK_OFF_SLOW;
    Durata[T_ON_BLINK] = DELAY_LED_BLINK_ON_SLOW;
    break;

  case LED_BLINK_FAST:
    Durata[T_OFF_BLINK]= DELAY_LED_BLINK_OFF_FAST;
    Durata[T_ON_BLINK] = DELAY_LED_BLINK_ON_FAST;
    break;    
  }

  LEDType = type;
}



static void EV_LedManager(void)
/**/
/*===========================================================================*/
/**
**   @brief EV LED blink management
**
**   @param void
**
**   @return void
**/
/*===========================================================================*/
/**/
{
  switch (LEDType)
  {
  case LED_OFF:
    LEDStatus = OFF;
    StopTimer(T_ON_BLINK);
    StopTimer(T_OFF_BLINK);
    break;
  
  case LED_ON:
    LEDStatus = ON;
    StopTimer(T_ON_BLINK);
    StopTimer(T_OFF_BLINK);
    break;  

  case LED_BLINK_SLOW:
  case LED_BLINK_FAST:

    if ( (StatusTimer(T_OFF_BLINK) == T_HALTED) && (StatusTimer(T_ON_BLINK) == T_HALTED))
    {
      LEDStatus = OFF;
      StartTimer(T_OFF_BLINK);
    }
    else if (StatusTimer(T_OFF_BLINK) == T_ELAPSED)
    {
      LEDStatus = ON;
      StartTimer(T_ON_BLINK);
      StopTimer(T_OFF_BLINK);
    }
    else if (StatusTimer(T_ON_BLINK) == T_ELAPSED)
    {
      LEDStatus = OFF;
      StartTimer(T_OFF_BLINK);
      StopTimer(T_ON_BLINK);
    }
    break;
  
  }
  EV_LED = LEDStatus;
}  
static void DC_MotorOn(unsigned char x)
/**/
/*===========================================================================*/
/**
**   @brief Returns result of motor test procedure
**
**   @param void
**
**   @return PROC_RUN/PROC_END/PROC_ERROR
**/
/*===========================================================================*/
/**/
{
  unsigned long ltemp;
  ltemp = (PERIOD_VALUE_DC_MOTOR*x)/100;
  OC1RS = (unsigned short)ltemp;
}  
void colorManager(void)
/*
*//*=====================================================================*//**
**      @brief sequencer of the module
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  StatusManager(); 
  EV_LedManager();
}
