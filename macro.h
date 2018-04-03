/*
*//*=====================================================================*//**
**
**      Nome del file  : macro.h
**
**      Descrizione    : Definizione macro
**
**      Progetto       : Haemotronic - Haemodrain
**
**      Data           : 30/09/2010
**
**      Autore         : A. Meliota
**
*//*=====================================================================*//**
*/


/*===== PROTEZIONE PER UNICA INCLUSIONE =====================================*/
#ifndef _MACRO_H_
#define _MACRO_H_

/*===== MACRO DI PROGRAMMAZIONE ===========================================*/
/* Definisce il tipo di macchina */
// #define COLOR_LAB_MACHINE

/* Definisce il tipo di slave */
#if defined (COLOR_LAB_MACHINE)
// #define B1_BASE  1
// #define B2_BASE  1
// #define B3_BASE  1
#define B4_BASE  1
// #define B5_BASE  1
// #define B6_BASE  1
// #define B7_BASE  1
// #define B8_BASE  1
// #define C1_COLOR 1
// #define C2_COLOR 1
// #define C3_COLOR 1
// #define C4_COLOR 1
// #define C5_COLOR 1
// #define C6_COLOR 1
// #define C7_COLOR 1
// #define C8_COLOR 1
// #define C9_COLOR 1
// #define C10_COLOR 1
// #define C11_COLOR 1
// #define C12_COLOR 1
// #define C13_COLOR 1
// #define C14_COLOR 1
// #define C15_COLOR 1
// #define C16_COLOR 1
// #define C17_COLOR 1
// #define C18_COLOR 1
// #define C19_COLOR 1
// #define C20_COLOR 1
// #define C21_COLOR 1
// #define C22_COLOR 1
// #define C23_COLOR 1
// #define C24_COLOR 1
// #define AUTOCAP 1 
#else
// #define B1_BASE  1
// #define B2_BASE  1
// #define C1_COLOR 1
// #define C2_COLOR 1
// #define C3_COLOR 1
// #define C4_COLOR 1
// #define C5_COLOR 1
// #define C6_COLOR 1
// #define C7_COLOR 1
// #define C8_COLOR 1
#define C9_COLOR 1
// #define C10_COLOR 1
// #define C11_COLOR 1
// #define C12_COLOR 1
// #define MOVE_X_AXIS 1
// #define MOVE_Y_AXIS 1
// #define STORAGE_CONTAINER1 1
// #define STORAGE_CONTAINER2 1
// #define STORAGE_CONTAINER3 1
// #define STORAGE_CONTAINER4 1
// #define PLUG_COVER1 1
// #define PLUG_COVER2 1 
// #define AUTOCAP 1 
#endif

#if defined(MOVE_X_AXIS) || defined(MOVE_Y_AXIS)
	#define MOTION_XY_ACTUATOR 1
#elif defined(STORAGE_CONTAINER1) || defined(STORAGE_CONTAINER2)	|| defined(STORAGE_CONTAINER3) || defined(STORAGE_CONTAINER4)		
  	#define STORAGE_CONTAINER_ACTUATOR 1
#elif defined(PLUG_COVER1) || defined(PLUG_COVER2)  	
  	#define PLUG_COVER_ACTUATOR 1
#elif defined(AUTOCAP) 
  	#define AUTOCAP_ACTUATOR 1
#else
  	#define BASE_COLOR_ACTUATOR 1
#endif

#if defined(B1_BASE) || defined(B2_BASE) || defined(B3_BASE) || defined(B4_BASE) || defined(B5_BASE) || defined(B6_BASE)|| defined(B7_BASE) || defined(B8_BASE)
 	#define BASE_ACTUATOR 1
#endif
/*=======================================================================*/
/* se definita considera le coordinate del banco Medicon per le posizioni xy */
// #define XY_BANCO_MEDICON

/*Macro per disabilitare il controllo del crc della flash*/
#define DISABLE_CRC_FLASH 1

// #define NANOTEC_MOTOR 1

/* se definita considera il motore cablato separatamente e non collegato a meccanica */
// #define SKIP_HOME_ERROR

/*Macro per lavorare in modalità debug sullo slave*/
// #define DEBUG_SLAVE

/* Simulate a Display CPU self test failure */
// #define FORCE_DISPLAY_CPU_FAILURE

/* Forces simulation of 50% packet loss from Master */
/* #define FORCE_SERIAL_PACKET_LOSS */

// se abilitata chiama la 	RTDM_ProcessMsgs() anzicchè serialCommManager();
//#define RTDM_PROCESS_MSG 1

// Macro Per utilizzare il pulsante presente sulla demo-board
//#define ENABLE_BUTTON_EVENT

/*Macro per abilitare la trasmissione seriale di un pacchetto indipendentemente dalla
 ricezione di una domanda dal display*/
#define SKIP_TOUT_SERIALE 1

/*Macro per abilitare la trasmissione seriale di un pacchetto indipendentemente dalla
 ricezione di una domanda dal display*/
// #define SKIP_START_POS_ERROR 1

/*Macro per allungare il tempo di timeout caduta di comunicazione seriale lato slaves
in modo da poter lavorare in debugger lato display*/
#define DEBUG_TOUT_SERIALE 1

/* Macro per misure ti tempo tramite commutazione pin RB6 */
// #define MEASURE_CHANGE_PAGE_TIME

/* Macro per abilitare controllo POS  in loop chiuso */
// #define DEBUG_POS_CTRL_CLOSED_LOOP 1

/* Macro per abilitare controllo vel  in loop chiuso */
// #define DEBUG_SPEED_CTRL_CLOSED_LOOP 1

/* Macro per abilitare acquisizione dati risposta al gradino in loop chiuso */
// #define DEBUG_RISPOSTA_GRADINO 1

/* Macro per disabilitare il test di sicurezza */
// #define DISABLE_SECURITY_TEST 1

/*===== DEFINE GENERICHE ====================================================*/
#ifndef MIN
#define MIN(x,y) ((x) <= (y) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x,y) ((x) <= (y) ? (y) : (x))
#endif

#define LSN(x) ((x) & 0x0F)           // Least Significant Nibble
#define MSN(x) (((x) & 0xF0) >> 4)    // Most Significant Nibble
#define LSB(x) ((x) & 0x00FF)         // Least Significant Byte
#define MSB(x) (((x) & 0xFF00) >> 8)  // Most Significant Byte
#define LSW(x) ((unsigned long)(x) & 0x0000FFFFL)         // Least Significant Word
#define MSW(x) (((unsigned long)(x) & 0xFFFF0000) >> 16)  // Most Significant Word
#define MSB_MSW(x) (((x) & 0xFF000000L) >> 24)  // Most Significant Byte of Most Significant Word
#define LSB_MSW(x) (((x) & 0x00FF0000L) >> 16)  // Least Significant Byte of Most Significant Word
#define MSB_LSW(x) (((x) & 0x0000FF00L) >> 8)   // Most Significant Byte of Least Significant Word
#define LSB_LSW(x) (((x) & 0x000000FFL))        // Least Significant Byte of Least Significant Word

#define TO_ASCII(x)   ((unsigned char)(x) + 0x30)
#define FROM_ASCII(x) ((unsigned char)(x) - 0x30)

#define TO_SHORT(x,y)     (((unsigned short)(x) << 8) | (y))
#define TO_LONG(x,y,k,h)  (((unsigned long)(x) << 24) | ((unsigned long)(y) << 16) | ((unsigned long)(k) << 8) | (h))

/* Thresholded equals op: x ~(t)= y iff | x - y | <= t
   works with signed and unsigned values. t must be non-negative. */
#define THRES_EQ(x,y,t) (((y)<(x)) ? ((x)-(y)<=(t)) : ((y)-(x)<=(t)))

/* These macros are intended for debugging purposes *only* */
#define HALT()    \
  do {            \
    while(1)      \
    {             \
      Nop();      \
    }             \
  }               \
  while (0)

#define ASSERT(x) \
  do {            \
    if (!(x))     \
    {             \
      HALT();     \
    }             \
  }               \
  while (0)

/*Macro per potenze di 10*/
#define TEN_0_POW         1
#define TEN_1_POW         10
#define TEN_2_POW         100
#define TEN_3_POW         1000
#define TEN_4_POW         10000L
#define TEN_5_POW         100000L
#define TEN_6_POW         1000000L
#define TEN_7_POW         10000000L
#define TEN_8_POW         100000000L

#define MASK_BITS_0_14    0x7FFF  /*01111111 11111111*/
#define MASK_BITS_15      0x8000  /*10000000 00000000*/
/*==============================================================*/

#define TRUE 1
#define FALSE 0
#define NOT_TEST_DONE 3

#define DISABLE   0
#define ENABLE    1

#define N_DEBUG_SAMPLES 20

// se abilitata elabora tutti e 24 i bit del convertitore, altrimenti considera solo i 16 più significativi
//#define  _24_BIT_CONVERSION 1


/*===== PROTEZIONE PER UNICA INCLUSIONE (FINE FILE) =========================*/
#endif
