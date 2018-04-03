/*
*//*=====================================================================*//**
**
**      Nome del file  : SPI.h
**
**      Descrizione    : Header di SPI.c
**
**      Progetto       : Haemotronic - Haemodrain
**
**      Data           : 07/02/2010
**
**      Autore         : A. Puppio
**
*//*=====================================================================*//**
*/

#ifndef _SPI_H_
#define _SPI_H_

/*===== INCLUSIONI ========================================================= */

#include "p33FJ32MC204.h"

/*===== DICHIARAZIONI LOCALI ================================================*/

/*===== DEFINE GENERICHE ====================================================*/
#define BUFF_SIZE 3

#ifdef PUMP_SLAVE 
#define INIT_VALUE 0
#else
#define INIT_VALUE 1
#endif

#define    Init_Select_Channel()        LATBbits.LATB1= INIT_VALUE; TRISBbits.TRISB1 = 0;	// output pin
#define    SELECTOR_INPUT_CHANNEL       LATBbits.LATB1  // A0 

#define    Init_Speed_Converter()       LATBbits.LATB2=1; TRISBbits.TRISB2 = 0;	// output pin


// settaggio Gain 128

#define    Init_Gain1_Converter()       LATAbits.LATA4=1; TRISAbits.TRISA4 = 0;			//must be output
#define    Init_Gain0_Converter()       LATAbits.LATA7=1; TRISAbits.TRISA7 = 0;			//must be output 

#define   ADC_DATA_READY     		         PORTCbits.RC1   //INPUT PIN 

#define    Init_Sck1()                   TRISCbits.TRISC0 = 0;// output pin

/*===== TIPI ================================================================*/



#ifdef _24_BIT_CONVERSION
typedef struct
{
  long data_from_AD;
  unsigned char readCounter;
  unsigned char newData;
 
} Ad_read_t;

#else

typedef struct
{
  short data_from_AD;
  unsigned char readCounter;
  unsigned char newData;
 
} Ad_read_t;
#endif
typedef struct
{
  unsigned char buffer[3];
  unsigned char length;
  unsigned char index;

} Spi_buffer_t;

/*===== PROTOTIPI FUNZIONI ==================================================*/
extern void initSPI(void);
extern void ADConverterRLCMng(void);
extern void ADConverterPumpMng(void);
extern void ADConverterUFCMng(void);
#endif
