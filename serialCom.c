/**/
/*============================================================================*/
/**
**      @file      SerialCom.c
**
**      @brief     Modulo di comunicazione seriale
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

/*======================== INCLUSIONI ======================================= */
#include "macro.h"
#include "typedef.h"
#include "ram.h"
#include "const.h"
#include "serialCom.h"
#include "timerMg.h"
#include "color.h"

#include "UART.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ===== MACRO LOCALI ====================================================== */
#define ENABLE_TX_MULTIPROCESSOR LATAbits.LATA8

/**
 * @brief Stores a byte into RX uartBuffer, performs boundary
 * checking.
 *
 * @param uartBuffer_t buf, RX buffer
 * @param char c, the character to be stored
 */
#define STORE_BYTE(buf, c)                      \
  do {                                          \
    (buf).buffer[(buf).index ++ ] = (c);        \
    if ((buf).index >= BUFFER_SIZE)             \
    {                                           \
      SIGNAL_ERROR();                           \
    }                                           \
  } while(0)

/**
 * @brief Resets the receiver
 */
#define RESET_RECEIVER() \
  do {                                          \
    initBuffer(&rxBuffer);                      \
  } while (0)

/**
 * @brief Last char was an escape?
 */
#define IS_ESCAPE()                             \
  (rxBuffer.escape != FALSE)

/**
 * @brief Signal last char was an escape
 */
#define SIGNAL_ESCAPE()                         \
  do {                                          \
    rxBuffer.escape = TRUE;                     \
  } while (0)

/**
 * @brief Signal last char was not an escape
 */
#define CLEAR_ESCAPE()                          \
  do {                                          \
    rxBuffer.escape = FALSE;                    \
  } while (0)

/**
 * @brief Serial error?
 */
#define IS_ERROR()                              \
  (rxBuffer.bufferFlags.serialError != FALSE)

#define SIGNAL_ERROR()                          \
  do {                                          \
    rxBuffer.bufferFlags.serialError = TRUE;    \
  } while (0)

#define IS_FROM_DISPLAY(id)                     \
  ((id) < 100)

#define IS_FROM_SLAVE(id)                       \
  ((id) > 100)

/* These offsets apply ok, after payload unstuffying */
#define PUMP_STATE_OFS (4)
#define RLC_AIR_DETECTION_OFS (7)

/* ===== TIPI LOCALI ======================================================= */
/* ===== VARIABILI LOCALI ================================================== */
/* ===== COSTANTI LOCALI =================================================== */
/* ===== VARIABILI LOCALI ================================================== */
static uartBuffer_t rxBuffer;
static uartBuffer_t txBuffer;
static serialSlave_t serialSlave;
static unsigned char deviceID;

/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */
static void unstuffMessage();
static void rebuildMessage(unsigned char);
static void initBuffer(uartBuffer_t*);
static void makeMessage(void);
static void decodeMessage(void);
static void sendMessage(void);

/* ===== DEFINIZIONE FUNZIONI LOCALI ======================================= */
static void unstuffMessage()
/**/
/*===========================================================================*/
/**
**   @brief Performs byte unstuffying on rx buffer. This function is a
**   private service of rebuildMessage()
**
**/
/*===========================================================================*/
/**/
{
  unsigned char i, j, c;

  /* skip 3 bytes from frame head: [ STX, ID, LEN ] */
  unsigned char *p = rxBuffer.buffer + FRAME_PAYLOAD_START;

  /* i is the read index, j is the write index. For each iteration, j
   * is always incremented by 1, i may be incremented by 1 or 2,
   * depending on whether p[i] is a stuffed character or not. At the
   * end of the cycle (length bytes read) j is less than or equal to
   * i. (i - j) is the amount that must be subtracted to the payload
   * length. */

  i = j = 0;
  while (i < rxBuffer.length) {
    c = *(p + i);
    ++ i;

    if (c == ASCII_ESC)
    {
      c = *(p + i) - ASCII_ZERO;
      ++ i;

      if (!c)
      {
        *(p + j) = ASCII_ESC;
      }
      else
      {
        *(p + j) = c;
      }
    }
    else
    {
      *(p + j) = c;
    }

    ++ j;
  }

  /* done with unstuffying, now fix payload length. */
  rxBuffer.length -= (i - j);
}

static void rebuildMessage(unsigned char receivedByte)
/**/
/*=====================================================================*/
/**
**      @brief Called by  _U1RXInterrupt: update the rx buffer  with
**             subsequent received bytes
**
**      @param receivedByte received bytes
**
**      @retval void
**/
/*=====================================================================*/
/**/
{
  #ifdef FORCE_SERIAL_PACKET_LOSS
  static int _last_msg_from_display_was_lost;
  #endif

  if (! IS_ERROR())
  {
    switch(rxBuffer.status)
    {

    case WAIT_STX:
      if (receivedByte == ASCII_STX)
      {
        STORE_BYTE(rxBuffer, receivedByte);
        rxBuffer.status = WAIT_ID;
      }
      break;

    case WAIT_ID:
      STORE_BYTE(rxBuffer, receivedByte);
      deviceID = REMOVE_OFFSET(receivedByte);
      if (! IS_VALID_ID(deviceID))
      {
        SIGNAL_ERROR();
      }
      else
      {
        rxBuffer.status = WAIT_LENGTH;
      }
      break;

    case WAIT_LENGTH:
      STORE_BYTE(rxBuffer, receivedByte);
      if ( receivedByte < ADD_OFFSET( MIN_FRAME_SIZE) ||
           receivedByte > ADD_OFFSET( MAX_FRAME_SIZE) )
      {
        SIGNAL_ERROR();
      }
      else
      {
        /* The length embedded in the frame takes into account the
         * entire frame length, for ease of implementation of tx/rx
         * code. Here we discard the final 5 bytes (4 CRC + ETX). Later
         * on, after the crc check, we'll be able to discard also the
         * initial overhead [ STX, ID, LEN ] */
        rxBuffer.length  = REMOVE_OFFSET(receivedByte);
        rxBuffer.length -= FRAME_END_OVERHEAD;

        rxBuffer.status = WAIT_DATA;
      }
      break;

    case WAIT_DATA:
      /* check stuffying encoding */
      if (IS_ESCAPE())
      {
        /* ESC ZERO --> ESC, ESC TWO --> STX, ESC THREE --> ETX */
        if (receivedByte != ASCII_ZERO &&
            receivedByte != ASCII_TWO &&
            receivedByte != ASCII_THREE)
        {
          /* Ilegal encoding detected */
          SIGNAL_ERROR();
        }
        CLEAR_ESCAPE();
      }
      else
      {
        if (receivedByte == ASCII_ESC)
        {
          SIGNAL_ESCAPE();
        }
      }

      STORE_BYTE(rxBuffer, receivedByte);
      if (rxBuffer.index == rxBuffer.length)
      {
        rxBuffer.status = WAIT_CRC;
      }
      break;

    case WAIT_CRC:
      STORE_BYTE(rxBuffer, receivedByte);

      /* received four CRC bytes? */
      if (rxBuffer.index == FRAME_CRC_LENGTH + rxBuffer.length)
      {
        rxBuffer.status = WAIT_ETX;
      }
      break;

    case WAIT_ETX:
      #ifdef FORCE_SERIAL_PACKET_LOSS
      if ( IS_FROM_DISPLAY( deviceID ) && deviceID == MASTER_DEVICE_ID(slave_id)
                                       && (_last_msg_from_display_was_lost =
                                          !_last_msg_from_display_was_lost))
      #else
      if (receivedByte != ASCII_ETX || ! CHECK_CRC16(&rxBuffer))
      #endif
      {
        SIGNAL_ERROR();
      }
      else
      {
        STORE_BYTE(rxBuffer, receivedByte);
        rxBuffer.length -= FRAME_PAYLOAD_START;

        /* frame ok, we can now "unstuff" the payload */
        unstuffMessage();

        if (deviceID == MASTER_DEVICE_ID(slave_id))
        {
          rxBuffer.bufferFlags.rxCompleted = TRUE;
        }
       
        if (! rxBuffer.bufferFlags.rxCompleted)
        {
          SIGNAL_ERROR();
        }
      }
      break;

    default:
      SIGNAL_ERROR();
    } /* switch */
  } /* if (! IS_ERROR) */

  if (IS_ERROR())
  {
    RESET_RECEIVER();
  }
} /* rebuildMessage() */


static void makeMessage (void)
/*
*//*=====================================================================*//**
**      @brief Management of the fixed time window Display-slaves:
**             if the answer from the Involved slave is received,
**             the following actions are performed:
**             - update the serialSlave struct for the subsequent slave
**               interrogated,
**             - make the packet to be transmitted, calling the message
**               make function related to the new slave
**             If the time window is elapsed without answer and the number
**             of retries is lower than admitted:
**             - increase the number of retries for the current slave
**             - send again the message to the same slave
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  if ((rxBuffer.bufferFlags.decodeDone == TRUE )&& (txBuffer.bufferFlags.uartBusy == FALSE))
  {
    rxBuffer.bufferFlags.decodeDone = FALSE;
    initBuffer(&txBuffer);

    serialSlave.makeSerialMsg(&txBuffer,slave_id);

    txBuffer.bufferFlags.txReady = TRUE;
    StartTimer(T_SLAVE_WAIT_TIMER);
    StopTimer(T_SLAVE_WAIT_LINK_TIMER);
  }
  else if(rxBuffer.bufferFlags.decodeDone == FALSE && StatusTimer(T_SLAVE_WAIT_TIMER) == T_HALTED)
  {
    StartTimer(T_SLAVE_WAIT_TIMER);
  }
  else if(rxBuffer.bufferFlags.decodeDone == FALSE && StatusTimer(T_SLAVE_WAIT_TIMER)== T_ELAPSED
    && (StatusTimer(T_SLAVE_WAIT_LINK_TIMER) != T_RUNNING))
  {    
    #ifndef SKIP_TOUT_SERIALE
    Status.level= COLOR_TOUT_ERROR_ST;
    #endif
  }
}


static void decodeMessage(void)
/*
*//*=====================================================================*//**
**      @brief decode the received message, calling the decode
**             function related to the Involved slave: call to
**             serialSlave->decodeSerialMsg(&rxBuffer)
**
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  // A seconda dello stato e dello slave interrogato, chiamo le differenti
  // funzioni di decodifica
  if ( rxBuffer.bufferFlags.rxCompleted == TRUE)
  {
    if (StatusTimer(T_DELAY_INTRA_FRAMES) == T_HALTED)
    {
      StartTimer(T_DELAY_INTRA_FRAMES);
    }
    else if (StatusTimer(T_DELAY_INTRA_FRAMES) == T_ELAPSED)
    {
      serialSlave.decodeSerialMsg(&rxBuffer,slave_id);
      initBuffer(&rxBuffer);
      rxBuffer.bufferFlags.decodeDone = TRUE;
      StopTimer(T_DELAY_INTRA_FRAMES);
    }

  }

}


static void sendMessage(void)
/*
*//*=====================================================================*//**
**      @brief Start the transmission, enabling the UART 3 transmission
**             flag and filling the UART3 tx buffer with the first byte
**             to be transmitted
**
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  if(txBuffer.bufferFlags.txReady == TRUE)
  {
    if ((txBuffer.bufferFlags.uartBusy == TRUE) || (txBuffer.length > BUFFER_SIZE))
    {
      return;
    }
    // Enable Tx multiprocessor line
    ENABLE_TX_MULTIPROCESSOR=1;
    // Pulisco l'interrupt flag della trasmissione
    IFS0bits.U1TXIF = 0;
    // Abilito il flag UARTx En
    IEC0bits.U1TXIE = 1;

    // Scarico il primo byte nel buffer di trasmissione : Write data byte to lower byte of UxTXREG word
    // Take control of buffer
    txBuffer.bufferFlags.uartBusy = TRUE;

    while(U1STAbits.TRMT == 0);
    #ifdef DEBUG_SERIALE
    U1TXREG = TO_ASCII(txBuffer.buffer[txBuffer.index++]);
    #else
    U1TXREG = txBuffer.buffer[txBuffer.index++];
    #endif

  }
}


static void initBuffer(uartBuffer_t *buffer)
/*
*//*=====================================================================*//**
**      @brief init buffer
**
**      @param buffer pointer to the buffer
**
**      @retval void
*//*=====================================================================*//**
*/
{
  memset(buffer->buffer, 0, BUFFER_SIZE);
  buffer->bufferFlags.allFlags = 0;

  buffer->status = WAIT_STX;
  buffer->index = 0;
  buffer->length = 0;
  buffer->escape = FALSE;
}


void initSerialCom(void)
/*
*//*=====================================================================*//**
**      @brief Set UART3 registers; reset rreceiver and transmission
**             buffers and flags. Start the FIRST_LINK timer window
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  U1BRG = 85; // Clock_FREQ = 40MHz - BaudRate = 115200

  U1MODE = 0x08;
  U1STA = 0;

  // UART Enable
  U1MODEbits.UARTEN = 1;

  // Interrupt when char is tranferred into TSR Register: so transmit buffer is empty
  U1STAbits.UTXISEL1 = 0;
  U1STAbits.UTXISEL0 = 1;

  // Transmit Enable
  U1STAbits.UTXEN = 1;

  // Reset Interrupt flags
  IFS0bits.U1RXIF = 0;
  IFS0bits.U1TXIF = 0;

  // Rx start
  IEC0bits.U1RXIE = 1;

  // UART3 ENABLE TX MULTIPROCESSOR RD13
  ENABLE_TX_MULTIPROCESSOR=0;
  TRISAbits.TRISA8=0;

  initBuffer(&rxBuffer);
  initBuffer(&txBuffer);
  
  serialSlave.makeSerialMsg=&MakeColorMessage;
  serialSlave.decodeSerialMsg=&DecodeColorMessage;
 }


void serialCommManager(void)
/*
*//*=====================================================================*//**
**      @brief Sequencer of the module
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{

  decodeMessage();
  makeMessage();
  sendMessage();

}

#ifndef RTDM_PROCESS_MSG
/******************************************************************************/
/****************************** Interrupt Routine *****************************/
/******************************************************************************/

void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
/*
*//*=====================================================================*//**
**      @brief Interrupt in tx della UART1
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  if (_U1TXIE && _U1TXIF)
  {
    _U1TXIF = 0;

    if (txBuffer.index == txBuffer.length)
    {
      // Disable Tx multiprocessor line
      ENABLE_TX_MULTIPROCESSOR=0;
      // Disabilito il flag UARTx En
      IEC0bits.U1TXIE = 0;
      txBuffer.bufferFlags.uartBusy = FALSE;
      txBuffer.bufferFlags.txReady = FALSE;
    }
    else
    {
      #ifdef DEBUG_SERIALE
      U1TXREG = TO_ASCII(txBuffer.buffer[txBuffer.index++]);
      #else
      U1TXREG = txBuffer.buffer[txBuffer.index++];
      #endif
    }
  }
}


void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
/*
 *//*=====================================================================*//**
**      @brief Interrupt in tx della UART2
**
**      @param void
**
**      @retval void
*//*=====================================================================*//**
*/
{
  register unsigned char flushUart;

  if (_U1RXIE && _U1RXIF)
  {
    _U1RXIF = 0;

    /*Overrun Error*/
    if (U1STAbits.OERR)
    {
      /* Segnalazione Overrun Error */
      U1STAbits.OERR = 0;
      SIGNAL_ERROR();
    }

    /*Framing Error*/
    if (U1STAbits.FERR)
    {
      flushUart = U1RXREG;
      /* Segnalazione Framing Error */
      SIGNAL_ERROR();
    }

    rebuildMessage(U1RXREG);
  }
}
#endif

unsigned short CRCarea(unsigned char *pointer, unsigned short n_char,unsigned short CRCinit)
/*
**=============================================================================
**
**      Oggetto        : Calcola CRC di una zona di byte specificata
**                       dai parametri di ingresso
**
**      Parametri      : pointer      Indirizzo iniziale dell'area
**                                    da controllare
**                       n_char       Numero dei bytes da includere nel calcolo
**                       CRCinit      Valore iniziale di CRC ( = 0 se n_char
**				      copre l'intera zona da verificare,
**				      = CRCarea della zona precedente se
**				      si sta procedendo a blocchi
**
**      Ritorno        : CRCarea      Nuovo valore del CRC calcolato
**
**      Vers. - autore : 1.0  nuovo   G. Comai
**
**=============================================================================
*/
{

/* La routine proviene dalla dispensa "CRC Fundamentals", pagg. 196, 197. */

/* Nota sull'algoritmo:
       dato un vettore, se ne calcoli il CRC_16: se si accodano i 2 bytes del
       CRC_16 a tale vettore (low byte first!!!), il CRC_16 calcolato
       sul vettore così ottenuto DEVE valere zero.
  Tale proprietà può essere sfruttata nelle comunicazione seriali
  per verificare che un messaggio ricevuto,
  contenente in coda i 2 bytes del CRC_16 (calcolati dal trasmettitore),
  sia stato inviato correttamente: il CRC_16, calcolato dal ricevente
  sul messaggio complessivo deve valere zero. */

unsigned long i;
unsigned short index;
unsigned char psv_shadow;

/* save the PSVPAG */
psv_shadow = PSVPAG;
/* set the PSVPAG for accessing CRC_TABLE[] */
PSVPAG = __builtin_psvpage (CRC_TABLE);

for (i = 0; i < n_char; i++)
{
  index = ( (CRCinit ^ ( (unsigned short) *pointer & 0x00FF) ) & 0x00FF);
  CRCinit = ( (CRCinit >> 8) & 0x00FF) ^ CRC_TABLE[index];
  pointer = pointer + 1;
  /* Reset Watchdog*/
 // ClrWdt();
  } /* end for */

/* restore the PSVPAG for the compiler-managed PSVPAG */
PSVPAG = psv_shadow;

return CRCinit;

} /* end CRCarea */


void stuff_byte(unsigned char *buf, unsigned char *ndx, char c)
/**/
/*===========================================================================*/
/**
**   @brief Writes c at the ndx-th position of buf, performing byte
**   stuffying if necessary. (*ndx) is incremented accordingly.
**
**   @param buf, the output buffer
**   @param ndx, a pointer to the current writing position in the output buffer
**   @param c, the character to be written
**
**/
/*===========================================================================*/
/**/
{
  /* STX --> ESC TWO, ETX --> ESC THREE */
  if ((c == ASCII_STX) || (c == ASCII_ETX))
  {
    WRITE_BYTE(buf, *ndx, ASCII_ESC);
    WRITE_BYTE(buf, *ndx, c + ASCII_ZERO);
  }
  /* ESC --> ESC ZERO */
  else if (c == ASCII_ESC)
  {
    WRITE_BYTE(buf, *ndx, ASCII_ESC);
    WRITE_BYTE(buf, *ndx, ASCII_ZERO);
  }
  /* Regular char, nothing fancy here */
  else
  {
    WRITE_BYTE(buf, *ndx, c);
  }
}
