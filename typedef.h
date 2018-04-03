/**/
/*============================================================================*/
/**
**      @file      TYPEDEF.H
**
**      @brief     Dichiarazione di tipo
**
**      @version   alfa color tester
**/
/*============================================================================*/
/**/

/*===== PROTEZIONE PER UNICA INCLUSIONE =====================================*/
#ifndef _TYPEDEF_H_
#define _TYPEDEF_H_

/*===== INCLUSIONI ========================================================= */
#include "macro.h"
/*========================== STEPPER_MOTOR ================================= */
/*========================== DEFINIZIONI GENERALI DI TIPO ================== */

/*!  Union   */



typedef union __attribute__ ((packed))
{
  unsigned short allflags;
  struct
  {
    unsigned short unused:16;
  } bit;             
} statusFlags_t;


typedef union __attribute__ ((packed))
{
  unsigned short uword;
  signed short   sword;
  unsigned char  byte[2];
} unionWord_t;


typedef union __attribute__ ((packed))
{
  unsigned long  udword;
  signed long    sdword;
  unsigned short word[2];
  unsigned char  byte[4];
} unionDWord_t;


/*!  Struct   */
typedef struct
{
  char level;
  char phase;  
  char step;
  char errorCode;
} status_t;


/*===== PROTEZIONE PER UNICA INCLUSIONE (FINE FILE) =========================*/
#endif
