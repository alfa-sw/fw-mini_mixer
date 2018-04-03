/**/
/*============================================================================*/
/**
**      @file      RAM.C
**
**      @brief     dichiarazione variabili globali
**
**      @version   Alfa Color tester
**/
/*============================================================================*/
/**/


/* ==== INCLUSIONI ========================================================= */
#include "typedef.h"
#include "macro.h"



/*
*******************************************************************************
** NOTA BENE:
** questo file sfrutta un "trucco" per poter inserire le variabili in ram in
** un solo punto:
** le variabili vanno inserite soltanto nel file RAM.H
** l'unica accortezza necessaria e' di far precedere a ogni dichiarazione di
** variabile l'espressione RAM_EXTERN, e di definire la macro RAM_EXTERN_DISABLE
** prima di includere il file RAM.H
******************************************************************************
*/
#define RAM_EXTERN_DISABLE
#include "RAM.H"


