/**/
/*============================================================================*/
/**
**      @file      CONST.H
**
**      @brief     dichiarazione costanti globali
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#include "typedef.h"

/*===== PROTEZIONE PER UNICA INCLUSIONE =====================================*/
#ifndef _CONST_H_
#define _CONST_H_

/*===== COSTANTI GENERICHE ====================================================*/
#define BY    0
#define WD    1

#define NSK   3

extern void DummyP (void);  
//extern __psv__ const BootloaderPointers_T  * BLT_Pointer;
extern const unsigned long Pow10[];



extern const unsigned long Pow10[];
extern const unsigned short ShortBit[];

extern const unsigned short CRC_TABLE[]; 
extern const unsigned short MASK_BIT [];
extern const unsigned char MASK_BIT_8[];

extern const unsigned char MaskNibble[2];
extern const unsigned char MaskBitOr[8];
extern const unsigned char MaskBitAnd[8];



/*===== PROTEZIONE PER UNICA INCLUSIONE (FINE FILE) =========================*/
#endif



