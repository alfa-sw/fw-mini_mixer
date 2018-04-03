/**/
/*============================================================================*/
/**
**      @file      SerialCom.h
**
**      @brief     Header file relativo a SerialCom.c
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#ifndef _SERIAL_COM_H_
#define _SERIAL_COM_H_

/*===== INCLUSIONI ========================================================= */
/*===== DICHIARAZIONI LOCALI ================================================*/
/*===== DEFINE GENERICHE ====================================================*/

/* reserved ASCII codes */
#define ASCII_STX (0x02)
#define ASCII_ETX (0x03)
#define ASCII_ESC (0x1B)

/* stuffying encodings */
#define ASCII_ZERO   ('0') /* ESC -> ESC ZERO  */
#define ASCII_TWO    ('2') /* STX -> ESC TWO   */
#define ASCII_THREE  ('3') /* ETX -> ESC THREE */

/* maximum payload size, calculated as slightly worse than the worst
 * case, with stuffying applied to *all* the bytes of the longest
 * message. */
#define BUFFER_SIZE (60)

#define FRAME_LENGTH_BYTE_POS    (2)
#define FRAME_PAYLOAD_START      (1 + FRAME_LENGTH_BYTE_POS)
#define FRAME_BGN_OVERHEAD       (FRAME_PAYLOAD_START)

#define FRAME_CRC_LENGTH         (4)
#define FRAME_END_OVERHEAD       (1 + FRAME_CRC_LENGTH)
#define MIN_FRAME_SIZE           (1 + FRAME_BGN_OVERHEAD + FRAME_END_OVERHEAD)
#define MAX_FRAME_SIZE           (BUFFER_SIZE)

/**
 * @brief The device ID for MASTER -> SLAVE msgs
 */
#define MASTER_DEVICE_ID(id)                    \
  (id)

/**
 * @brief The device ID for SLAVE -> MASTER msgs
 */
#define SLAVE_DEVICE_ID(id)                     \
  (100 + (id))

/**
 * @brief True iff ID is a valid query or reply ID
 */
#define IS_VALID_ID(id)                                 \
  (((0   < (id)) && ((id) <=       N_SLAVES)) ||        \
   ((100 < (id)) && ((id) <= 100 + N_SLAVES)))

/**
 * @brief All fixed position values are transferred with
 * a positive offset to avoid clashes with reserved bytes.
 */
#define ADD_OFFSET(c)                           \
  ((c) + 0x20)
#define REMOVE_OFFSET(c)                        \
  ((c) - 0x20)

/**
 * @brief low-level write c into buf and increment index.
 * @param buf, the buffer to write to
 * @param ndx, the index to write c at
 * @param c, the character to be written
 */
#define WRITE_BYTE(buf, ndx, c)                   \
  do {                                            \
    *((buf) + (ndx)) = (c);                       \
    ++ (ndx);                                     \
  } while (0)

/* just an alias for code uniformity */
#define STUFF_BYTE(buf, ndx, c) \
  stuff_byte((buf), &(ndx), (c))

/**
 * @brief Frame initialization
 */
#define FRAME_BEGIN(txb, idx, id)                               \
  do {                                                          \
    WRITE_BYTE((txb)->buffer, (idx), ASCII_STX);                \
    WRITE_BYTE((txb)->buffer, (idx), ADD_OFFSET( (id)));        \
    ++ (idx); /* reserved for pktlen */                         \
  } while (0)

/**
 * @brief Frame finalization
 */
#define FRAME_END(txb, idx)                                             \
  do {                                                                  \
    unionWord_t crc;                                                    \
                                                                        \
    /* fix pkt len */                                                   \
    (txb)->buffer [ FRAME_LENGTH_BYTE_POS ] =                           \
      ADD_OFFSET( FRAME_END_OVERHEAD + (idx));                          \
    (txb)->length = ( FRAME_END_OVERHEAD + (idx));                      \
                                                                        \
    /* crc16, sent one nibble at the time, w/ offset, big-endian */     \
    crc.uword = CRCarea((txb)->buffer, (idx), NULL);                    \
    WRITE_BYTE((txb)->buffer, (idx), ADD_OFFSET( MSN( crc.byte[1])));   \
    WRITE_BYTE((txb)->buffer, (idx), ADD_OFFSET( LSN( crc.byte[1])));   \
    WRITE_BYTE((txb)->buffer, (idx), ADD_OFFSET( MSN( crc.byte[0])));   \
    WRITE_BYTE((txb)->buffer, (idx), ADD_OFFSET( LSN( crc.byte[0])));   \
                                                                        \
    /* ETX = frame end */                                               \
    WRITE_BYTE((txb)->buffer, (idx), ASCII_ETX);                        \
  } while (0)

/**
 * @brief Check CRC16, CRC is expected to be sent over the wire in
 * Big-Endian, offseted format.
 * @return true iff CRC checks ok
 */
#define CHECK_CRC16(rxb) \
  ((((unsigned short)(REMOVE_OFFSET((rxb)->buffer[(rxb)->index - 4])) << 0xC) | \
    ((unsigned short)(REMOVE_OFFSET((rxb)->buffer[(rxb)->index - 3])) << 0x8) | \
    ((unsigned short)(REMOVE_OFFSET((rxb)->buffer[(rxb)->index - 2])) << 0x4) | \
    ((unsigned short)(REMOVE_OFFSET((rxb)->buffer[(rxb)->index - 1])) << 0x0) ) \
   == CRCarea((rxb)->buffer, (rxb)->length, NULL))

// define degli slaves
#if defined (COLOR_LAB_MACHINE)
enum
{
  B1_BASE_ID =   1,
  B2_BASE_ID, 
  B3_BASE_ID,
  B4_BASE_ID,
  B5_BASE_ID,
  B6_BASE_ID,
  B7_BASE_ID,
  B8_BASE_ID,
  C1_COLOR_ID,
  C2_COLOR_ID,
  C3_COLOR_ID,
  C4_COLOR_ID,
  C5_COLOR_ID,
  C6_COLOR_ID,
  C7_COLOR_ID,
  C8_COLOR_ID,
  C9_COLOR_ID,
  C10_COLOR_ID,
  C11_COLOR_ID,
  C12_COLOR_ID,
  C13_COLOR_ID,
  C14_COLOR_ID,
  C15_COLOR_ID,
  C16_COLOR_ID,
  C17_COLOR_ID,
  C18_COLOR_ID,
  C19_COLOR_ID,
  C20_COLOR_ID,
  C21_COLOR_ID,
  C22_COLOR_ID,
  C23_COLOR_ID,
  C24_COLOR_ID,
  GENERIC_ACT1_ID,
  GENERIC_ACT2_ID,
  GENERIC_ACT3_ID,
  GENERIC_ACT4_ID,
  GENERIC_ACT5_ID,
  GENERIC_ACT6_ID,
  GENERIC_ACT7_ID,
  GENERIC_ACT8_ID,  
  AUTOCAP_ID,
  GENERIC_ACT10_ID,
  GENERIC_ACT11_ID,
  GENERIC_ACT12_ID,
  GENERIC_ACT13_ID,
  GENERIC_ACT14_ID,
  GENERIC_ACT15_ID,
  GENERIC_ACT16_ID,  
  N_SLAVES
};
#else
enum
{
  B1_BASE_ID =   1,
  B2_BASE_ID,
  B3_BASE_ID,
  B4_BASE_ID,
  B5_BASE_ID,
  B6_BASE_ID,
  B7_BASE_ID,
  B8_BASE_ID,
  C1_COLOR_ID,
  C2_COLOR_ID,
  C3_COLOR_ID,
  C4_COLOR_ID,
  C5_COLOR_ID,
  C6_COLOR_ID,
  C7_COLOR_ID,
  C8_COLOR_ID,
  C9_COLOR_ID,
  C10_COLOR_ID,
  C11_COLOR_ID,
  C12_COLOR_ID,
  C13_COLOR_ID,
  C14_COLOR_ID,
  C15_COLOR_ID,
  C16_COLOR_ID,
  C17_COLOR_ID,
  C18_COLOR_ID,
  C19_COLOR_ID,
  C20_COLOR_ID,
  C21_COLOR_ID,
  C22_COLOR_ID,
  C23_COLOR_ID,
  C24_COLOR_ID,
  MOVE_X_AXIS_ID,
  MOVE_Y_AXIS_ID,
  STORAGE_CONTAINER1_ID,
  STORAGE_CONTAINER2_ID,
  STORAGE_CONTAINER3_ID,
  STORAGE_CONTAINER4_ID,
  PLUG_COVER_1_ID,
  PLUG_COVER_2_ID,
  AUTOCAP_ID,
  GENERIC_ACT10_ID,
  GENERIC_ACT11_ID,
  GENERIC_ACT12_ID,
  GENERIC_ACT13_ID,
  GENERIC_ACT14_ID,
  GENERIC_ACT15_ID,
  GENERIC_ACT16_ID,  
  N_SLAVES
};
#endif

// stati del buffer di ricezione
enum
{
  /* 0 */ WAIT_STX,
  /* 1 */ WAIT_ID,
  /* 2 */ WAIT_LENGTH,
  /* 3 */ WAIT_DATA,
  /* 4 */ WAIT_CRC,
  /* 5 */ WAIT_ETX,
};

/*===== TIPI ================================================================*/
typedef struct
{
  unsigned char buffer[BUFFER_SIZE];
  unsigned char length;
  unsigned char index;
  unsigned char status;
  unsigned char escape; // rx only
  union __attribute__ ((packed))
    {
      unsigned char allFlags;
      struct
      {
        unsigned char unused1     : 1;
        unsigned char txReady     : 1;
        unsigned char decodeDone  : 1;
        unsigned char rxCompleted : 1;
        unsigned char serialError : 1;
        unsigned char uartBusy    : 1;
        unsigned char startTx     : 1;
        unsigned char unused      : 1;
      };
    } bufferFlags;
} uartBuffer_t;


typedef struct
{
  void (*makeSerialMsg)(uartBuffer_t *, unsigned char);
  void (*decodeSerialMsg)(uartBuffer_t *,unsigned char);
  unsigned char numRetry;
} serialSlave_t;


/*===== PROTOTIPI FUNZIONI ==================================================*/
extern void initSerialCom(void);
extern void serialCommManager(void);
extern unsigned short CRCarea(unsigned char *pointer, unsigned short n_char,
                              unsigned short CRCinit);

extern void stuff_byte(unsigned char *buf, unsigned char *ndx, char c);

#endif
