/*
*//*=====================================================================*//**
**
**      Nome del file  : SPI.c
**
**      Descrizione    : funzioni per gestire l' SPI
**
**      Progetto       : Haemotronic - Haemodrain
**
**      Data           : 07/02/2011
**
**      Autore         : A. Puppio
**
*//*=====================================================================*//**
*/

/*======================== INCLUSIONI ======================================= */
#include "MACRO.H"
#include "ram.h"
#include "stepper.h" // vi è la definizione di Nop

#include <string.h>

/* ===== MACRO LOCALI ====================================================== */

#define DATA_READ_SETTLE 6
/* definisce il numero di campioni su cui effettuare la media, da settare
in maniera appropriata */
#define N_SAMPLE 1

#define GAIN 128L
#define CONV_KG_TO_G 1000L
#define F_MOL_PR 100000L

#ifdef  LOAD_CELL_FS_5Kg
#define F_MOL_WEIGHT 1087L
#else
#define F_MOL_WEIGHT 1500L
#endif

#define LOAD_CELL_COEFF  (F_MOL_WEIGHT*CONV_KG_TO_G/GAIN)
#define LOAD_CELL_RES_GAIN 100
#define PR_COEFF  (F_MOL_PR/GAIN)
#define _23_BIT_MAX_VALUE 8388607L // (2^23 -1)
#define _15_BIT_MAX_VALUE 32767L // (2^15 -1)
/* da settare opportunamente, è il numero di misure da effettuare prima di
fare uno switch tra i sensori*/
#define NUMBER_MEASURE_FOR_SENSOR 2
#define DELAY_50_ns()           Nop();Nop();
#define DELAY_100_ns()          Nop();Nop();Nop();Nop();

#define OFFSET_ZERO_WEIGHT 0 //21 /*g*/
/* ===== TIPI LOCALI ======================================================= */
/* ===== VARIABILI LOCALI ================================================== */
static unsigned char input_channel;
/* ===== COSTANTI LOCALI =================================================== */
/* ===== VARIABILI LOCALI ================================================== */
static Spi_buffer_t txBufferSPI, rxBufferSPI;
static Ad_read_t  ad_read, *ad_read_p;

/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */
void SpiSendData(Spi_buffer_t * txBuffer);
void SpiReadData(Spi_buffer_t * rxBuffer);
void initSpiBuffer(Spi_buffer_t * buffer);
void initAd_read(Ad_read_t * init_Ad_read);
Ad_read_t * Read_AD_Converter(void);

/* ===== DEFINIZIONE FUNZIONI LOCALI ======================================= */



void initSPI(void)
/*
*//*=====================================================================*//**
**      @brief Initialize the Serial Peripheral Interface (SPI)
**
**      @param void
**
**      @retval void
**
*//*=====================================================================*//**
*/

{
/*-------------------------------------------------------------------------*/
/*   Remapping dei pin  SCK1 e DRDY/DOUT */
  // SCK1-> RP16/RC0
  RPOR8bits.RP16R = 8;
  // SDI1 -> RP17/RC1
  RPINR20bits.SDI1R = 17;
/*--------------------------------------------------------------------------*/

  _SPIEN = 0;
  // settiamo Fsck= 5 Mhz
  _SPRE   = 0; // 3 bit to set secondary Prescale . 0=8:1
  _PPRE   = 3; // 2 bit to set primary Prescale. 3= 1:1
  _MSTEN  = 1; // 1: Master mode; 0: slave mode

  SPI1CON1bits.MODE16=0 ;/*_MODE16 = 1; */// 0: communication is byte-wide; 1: word-wide
  _CKE    = 0; // Clock Edge Select bit. Depende on the slave
  _CKP    = 0; // Clock Polarity Select bit. Depende on the slave
  _SMP    = 0; // Data Input Sample Phase. Depende on the slave

  _DISSCK = 0; // Internal SPI clock is enabled
  _DISSDO = 0; //SD0 pin is controlled by module
  _SSEN   = 0; // 1: pin used for slave mode
  // SPI1STAT  register configuration
  _SPIEN = 1; // Enable SPI module

  initSpiBuffer(&rxBufferSPI);
  initSpiBuffer(&txBufferSPI);

  initAd_read(&ad_read);

  // inizializzazione  pin  d'ingresso al convertitore
  Init_Select_Channel()
  input_channel = INIT_VALUE;
  Init_Speed_Converter()
  Init_Gain1_Converter()
  Init_Gain0_Converter()
  Init_Sck1()

}


void initSpiBuffer(Spi_buffer_t * buffer)
/*
*//*=====================================================================*//**
**      @brief  init buffer
**
**      @param  buffer pointer to the buffer
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  memset(buffer->buffer, 0, BUFF_SIZE);
  buffer->index = 0;
  buffer->length = BUFF_SIZE;
}


void initAd_read(Ad_read_t * init_Ad_read)
/*
*//*=====================================================================*//**
**      @brief  init Ad_read_t structure
**
**      @param  pointer to Ad_read_t structure
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  init_Ad_read -> data_from_AD = 0;
  init_Ad_read -> readCounter  = 0;
  init_Ad_read -> newData = FALSE;
}


void SpiSendData(Spi_buffer_t * txBuffer)
/*
*//*=====================================================================*//**
**      @brief Send data
**
**      @param  txBuffer pointer to tx buffer
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  while(_SPITBF);
  // write  data
  SPI1BUF =txBuffer->buffer[txBuffer->index++];
}

void SpiReadData(Spi_buffer_t * rxBuffer)
/*
*//*=====================================================================*//**
**      @brief Read data
**
**      @param  rxBuffer pointer to rx buffer
**
**      @retval void
**
*//*=====================================================================*//**
*/
{
  while(! _SPIRBF);
  //read data
  rxBuffer->buffer[rxBuffer->index++] = SPI1BUF;
}

Ad_read_t * Read_AD_Converter(void)
/*
*//*=====================================================================*//**
**      @brief Read data from A/D converter
**
**      @param  void
**
**      @retval unsigned char
**
*//*=====================================================================*//**
*/
{
  unsigned char i;
  #ifdef _24_BIT_CONVERSION
  unsigned long tmp_data[BUFF_SIZE] = {0,0,0};
  #else
  unsigned short s_tmp_data;
  #endif

  if(!(ADC_DATA_READY))
  {
    for(i=0;i< BUFF_SIZE;i++)
    {
      SpiSendData(&txBufferSPI);
      SpiReadData(&rxBufferSPI);
      #ifdef _24_BIT_CONVERSION
      tmp_data[i]= rxBuffer.buffer[i];
      #endif
    }
    /* generazione "manuale" del 25 colpo di clock */
    /* 1) Al fine di non creare alee e quindi clock spurii (ved. fig. 10.1 del data-sheet),
          prima di disabilitare la periferica e quindi di "impadronirci" del pin RCO,
          settiamo Il pin RCO con l'ultimo valore settato dalla periferica " */
    LATCbits.LATC0=0;
    // Mettiamo 2 istruzioni di Nop in modo da essere " sicuri"
    // che LATCbits.LATC0=0 abbia avuto effetto quando andiamo a disabilitare la periferica
    DELAY_50_ns()
    /* 2) A questo punto disabilitiamo la periferica e quindi prendiamo controllo del pin RC0*/
    _SPIEN = 0;
    /*3) Creazione della forma d'onda del 25° colpo di clock*/
    LATCbits.LATC0=1;
    /*Lo facciamo rimanere alto per 100 ns*/
    DELAY_100_ns()
    /* Riportiamo il pin a 0*/
    LATCbits.LATC0=0;
    // Mettiamo 2 istruzioni di Nop in modo da essere " sicuri"
    // che LATCbits.LATC0=0 abbia avuto effetto quando andiamo ad abilitare la periferica
    DELAY_50_ns()
    /*Abilitiamo l'SPI*/
    _SPIEN = 1;

    #ifdef _24_BIT_CONVERSION
    tmp_data[0] <<= 16;
    tmp_data[1] <<= 8;
    ad_read.data_from_AD= tmp_data[0] | tmp_data[1] | tmp_data[2];
    #else

    s_tmp_data = 0x0000 | rxBufferSPI.buffer[0];
    s_tmp_data <<= 8;
    ad_read.data_from_AD = (signed short)(s_tmp_data | rxBufferSPI.buffer[1]);
    #endif


    ad_read.readCounter ++;
    initSpiBuffer(&rxBufferSPI);
    initSpiBuffer(&txBufferSPI);
    ad_read.newData = TRUE;


  }
  else
  {
    ad_read.newData = FALSE;
  }

    return(&ad_read);

}








