/*
*//*=====================================================================*//**
**
**      Nome del file  : TIMERMG.H
**
**      Descrizione    : Inclusioni relative al modulo TIMERMG.C
**
**      Progetto       : alfa color tester
**
*//*=====================================================================*//**
*/


enum{
/*  1 */ T_SLAVE_WAIT_TIMER,
/*  2 */ T_DEL_FILTER_DIG_IN,
/*  3 */ T_RAMP_UP,
/*  4 */ T_RAMP_DW, 
/*  5 */ T_AUTOMATIC_TX,
/*  6 */ T_RETRY_TEST_MOTOR,
/*  7 */ T_DELAY_INTRA_FRAMES,
/*  8 */ T_DELAY_FILTER_RESERVE_DIG_IN,
/*  9 */ T_DELAY_INIT_DONE,
/* 10 */ T_STIRRING_RUN,
/* 11 */ T_ADC_SAMPLE,
/* 12 */ T_DELAY_SHUTDOWN, 
/* 13 */ T_TEST_MOTOR_BRAKING,
/* 14 */ T_TEST_MOTOR,
/* 15 */ T_SLAVE_WAIT_LINK_TIMER,
/* 16 */ T_COUNT_RAMP,
/* 17 */ T_ON_BLINK,
/* 18 */ T_OFF_BLINK,
/* 19 */ T_DELAY_BEFORE_TEST_MOTOR,
/* 20 */ T_DELAY_ACTIVATE_RELAIS,
	N_TIMERS

};
#define NOT_RUNNING -2
#define START_TIMER 1
#define STOP_TIMER  0
#define T_CLEAR_IF_ELAPSED 1
#define T_START_IF_ELAPSED 2
#define T_READ 0
#define T_ELAPSED 	-1
#define T_RUNNING 	2
#define T_HALTED 	0
#define T_STARTED 	1
#define T_NOT_RUNNING 	-2


/* Base tempi uguale a 2ms --> Quindi 1s == 500 impulsi da 2ms */
#define  T_BASE 2 //ms

#define  CONV_SEC_COUNT  500L    // 1 sec = 1000 ms, 1 count each 2 ms ->1000/2 = 500

#ifdef DEBUG_TOUT_SERIALE
/*  1 */    #define DELAY_WAIT_SLAVE   20000 /*40 sec*/
#else
/*  1 */ 		#define DELAY_WAIT_SLAVE   1500 /*3 sec*/
#endif
/*  2 */ #define DELAY_FILTER_DIG_IN      5
/*  3 */ #define DELAY_RAMP_UP 1500 /*3 sec*/
/*  4 */ #define DELAY_RAMP_DW 4000 /*8 sec*/
/*  5 */ #define DELAY_AUTOMATIC_TX 500
/*  6 */ #define DELAY_RETRY_TEST_MOTOR 10 /*20 msec*/
/*  7 */ #define DELAY_INTRA_FRAMES 2
#if defined(BASE_COLOR_ACTUATOR) 
/*  8 */ #define DELAY_FILTER_RESERVE_DIG_IN 30000 /*1 min*/
#else
/*  8 */ #define DELAY_FILTER_RESERVE_DIG_IN 1
#endif
/*  9 */ #define DELAY_INIT_DONE 100
/* 10 */ #define DELAY_STIRRING_RUN 7500 //
/* 11 */ #define DELAY_ADC_SAMPLE 2
/* 12 */ #define DELAY_SHUTDOWN 5 /* 10 ms */
/* 13 */ #define DELAY_TEST_MOTOR_BRAKING   1000 /*2 sec*/
/* 14 */ #define DELAY_TEST_MOTOR   1 /*2 msec*/
/* 15 */ #define DELAY_SLAVE_WAIT_LINK_TIMER  5000 /*10 sec*/
/* 16 */ #define DELAY_COUNT_RAMP  5 /*10 msec*/
/* 17 */ #define DELAY_ON_BLINK_TIME  250 /*500 msec*/
/* 18 */ #define DELAY_OFF_BLINK_TIME  250 /*500 msec*/
/* 19 */ #define DELAY_BEFORE_TEST_MOTOR  100 /*200 msec*/
/* 20 */ #define DELAY_ACTIVATE_RELAIS  1000 /*2 sec*/

typedef struct {
  signed char Flg;
  unsigned short InitBase;
} timerstype;

extern unsigned short TimeBase;
extern timerstype TimStr[N_TIMERS];
extern unsigned short Durata[N_TIMERS];
extern void TimerMg (void);
extern unsigned short ReadTimer(unsigned char timer);
//extern void StartTimer(unsigned char Timer);
//extern signed char StatusTimer(unsigned char Timer);

#define StopTimer(Timer)    (TimStr[Timer].Flg = STOP_TIMER)
#define StartTimer(Timer)   (TimStr[Timer].Flg = START_TIMER)
#define NotRunningTimer(Timer) (TimStr[Timer].Flg = NOT_RUNNING)
#define StatusTimer(Timer)  (TimStr[Timer].Flg)
