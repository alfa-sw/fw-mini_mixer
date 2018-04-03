/*
*//*=====================================================================*//**
**
**      Nome del file  : TIMERMG.C
**
**      Descrizione    : Inclusioni relative al modulo TIMERMG.C
**
**      Progetto       : Alfa color tester
**
*//*=====================================================================*//**
*/


/*===== INCLUSIONI ========================================================= */

#include "MACRO.H"
#include "TimerMg.h"

/*====== MACRO LOCALI ====================================================== */

/*====== TIPI LOCALI ======================================================== */

/*====== VARIABILI LOCALI =================================================== */
static unsigned short MonTimeBase;

/*====== VARIABILI GLOBALI =================================================== */
unsigned short TimeBase;

timerstype TimStr[N_TIMERS];

unsigned short Durata[N_TIMERS] = {
  /*  1  */   DELAY_WAIT_SLAVE,
  /*  2  */   DELAY_FILTER_DIG_IN,
  /*  3  */   DELAY_RAMP_UP,
  /*  4  */   DELAY_RAMP_DW,
  /*  5  */   DELAY_AUTOMATIC_TX,
  /*  6  */   DELAY_RETRY_TEST_MOTOR,
  /*  7  */   DELAY_INTRA_FRAMES,
  /*  8  */   DELAY_FILTER_RESERVE_DIG_IN,
  /*  9  */   DELAY_INIT_DONE,
  /*  10  */  DELAY_STIRRING_RUN,
  /*  11  */  DELAY_ADC_SAMPLE,
  /*  12  */  DELAY_SHUTDOWN,
  /*  13  */  DELAY_TEST_MOTOR_BRAKING,
  /*  14  */  DELAY_TEST_MOTOR,
  /*  15  */  DELAY_SLAVE_WAIT_LINK_TIMER,
  /*  16  */  DELAY_COUNT_RAMP,
  /*  17  */  DELAY_ON_BLINK_TIME,
  /*  18  */  DELAY_OFF_BLINK_TIME,  
  /*  19  */  DELAY_BEFORE_TEST_MOTOR,
  /*  20  */  DELAY_ACTIVATE_RELAIS,
};
/*====== COSTANTI LOCALI =================================================== */

/*====== DEFINIZIONE FUNZIONI LOCALI ======================================= */
void TimerMg (void);
void TimerInit (void);



void TimerMg(void)
/*
*//*=====================================================================*//**
**
**      @brief Sequencer of the  TIMERMG  module
**
**      @param void
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
  unsigned char temp;


  MonTimeBase = TimeBase;


  for (temp = 0; temp < N_TIMERS; temp++)
  {
    if (TimStr[temp].Flg == T_RUNNING)
    {
      if ((MonTimeBase - TimStr[temp].InitBase) >= Durata[temp])
      {
        TimStr[temp].Flg = T_ELAPSED;
      }
    }

    if (TimStr[temp].Flg == T_STARTED)
    {
      TimStr[temp].InitBase = MonTimeBase;
      TimStr[temp].Flg = T_RUNNING;
    }

  }

}/*end TimerMg*/


unsigned short ReadTimer (unsigned char timer)
/*
*//*=====================================================================*//**
**
**      @brief Returns the time elapsed from start timer
**
**
**      @param timer timer identifier (from 0 to N_TIMERS-1)
**
**      @retval time total elapsed
**
**
*//*=====================================================================*//**
*/
{
  unsigned short TimeTot;
  TimeTot = (unsigned short)(TimeBase - TimStr[timer].InitBase);
  return (TimeTot);
} /* end ReadTimer */
