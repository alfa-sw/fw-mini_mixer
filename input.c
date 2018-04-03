/**/
/*============================================================================*/
/**
**      @file      input.c
**
**      @brief     Campionamento ingressi digitali
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

/*===== INCLUSIONI ========================================================= */
#include "stepper.h"
#include "macro.h"
#include "ram.h"
#include "input.h"
#include "init.h"
#include "timermg.h"
#include "color.h"

/*====== MACRO LOCALI ====================================================== */
#define UPDATE_DIGITAL_STATUS(out,in,bitmask)                           \
  do {                                                                  \
    (out) = (((out) & ~(bitmask))) | ((in) & (bitmask));                \
  }                                                                     \
  while (0)



/* Macro per la definizione degli Array dei filtri */
#define INPUT_ARRAY 16

#define FILTER_WINDOW           5                      /* Larghezza della finestra del filtro */
#define FILTER_WINDOW_LENGTH    (FILTER_WINDOW-1)
#define FILTER_WINDOW_LOOP      (FILTER_WINDOW-2)
#define MAX_CHANGE              (FILTER_WINDOW/2)
#define MIN_COUNT               (FILTER_WINDOW*3/4)
#define ERRORE_FILTRO           2
#define LOW_FILTER              0
#define HIGH_FILTER             1
#define COUNT_RESET             0
#define MASK_FILTER_OFF         0x0000



#define VR_POSITIVE 33
#define VOLT_TO_MV  100L
#define _10_BIT_MAX_VALUE 1023L
#define N_MAX_AD_SAMPLE 8
#define SW_OFF_THR_0_VOLTAGE 700 /*mV*/
#define SW_OFF_THR_1_VOLTAGE 1500 /*mV*/

#define MASK_SEL_STATUS 0x001F

/*====== TIPI LOCALI ======================================================== */
/*====== VARIABILI LOCALI =================================================== */
DigInStatusType DigInStatus, DigInNotFiltered;

static unsigned char FILTRAGGIO_LOW[FILTER_WINDOW];
static unsigned char FILTRAGGIO_HIGH[FILTER_WINDOW];
static DigInStatusType OutputFilter;
static unsigned char  n_filter;
static unsigned char zero_counter, one_counter, ChangeStatus, Out_Status;
static signed char index_0, index_1;
static unsigned char DummyOutput_low, DummyOutput_high, shift;
static unsigned char fNewReading;
static unsigned char toggleStatus = 0;
/*====== COSTANTI LOCALI =================================================== */
const unsigned char MASK_BIT_8[]={0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

/* ===== PROTOTIPI FUNZIONI LOCALI ========================================= */
void inputManager(void);
static void ReadDigInStatus(void);
static void GetKeybStatus(void);
static void GetSelStatus(void);
// #if RLC_SLAVE || UFC_SLAVE
// static void ReadDigInClampStatus(void);
// #endif
static unsigned short FilterSensorInput(DigInStatusType InputFilter);

/*====== DEFINIZIONE FUNZIONI LOCALI ======================================= */
static void ReadDigInStatus(void)
/*
*//*=====================================================================*//**
**
**      @brief Reads digital inputs
**
**      @param void
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
  DigInNotFiltered.Bit.StatusType0 = TIME_SEL_BIT_0;
  DigInNotFiltered.Bit.StatusType1 = TIME_SEL_BIT_1;
  DigInNotFiltered.Bit.StatusType2 = TIME_SEL_BIT_2;
  DigInNotFiltered.Bit.StatusType3 = TIME_SEL_BIT_3;
  DigInNotFiltered.Bit.StatusType4 = TIME_SEL_BIT_4;
  
  DigInNotFiltered.Bit.StatusType5 = MOTOR_DRIVER_FAULT;
  DigInNotFiltered.Bit.StatusType6 = START_KEY;
  DigInNotFiltered.Bit.StatusType7 = DOOR_POS;
  DigInNotFiltered.Bit.StatusType8 = MOTOR_VOLTAGE_PRESENCE;
} /* end ReadDigInStatus() */



void inputManager(void)
/*
*//*=====================================================================*//**
**
**      @brief Sequencer of the INPUT module
**
**      @param void
**
**      @retval void
**
**
*//*=====================================================================*//**
*/
{
 
  if  (StatusTimer(T_DEL_FILTER_DIG_IN) == T_ELAPSED)
  {
    StartTimer(T_DEL_FILTER_DIG_IN);
    ReadDigInStatus();
    DigInStatus.word = FilterSensorInput(DigInNotFiltered);
    SelStatus = (unsigned char)(DigInStatus.word & MASK_SEL_STATUS);
  }
  GetKeybStatus();
  GetSelStatus();
  // #endif

  // AcquireSwitchOff();
} /* end inputManager() */

static unsigned short FilterSensorInput(DigInStatusType InputFilter)
/*
*//*=====================================================================*//**
**
**   @brief  Filter the digital inputs
**
**   @param  InputFilter input
**
**   @return ouput filtered
**
*//*=====================================================================*//**
*/
{
  unsigned char temp_uc;
  signed char temp_sc;

  if (n_filter < FILTER_WINDOW_LENGTH)  // n_filter = Finestra campioni del filtro
  {
    n_filter++;
  }
  else
  {
    n_filter = 0;
  }

  FILTRAGGIO_LOW[n_filter] = InputFilter.byte.low;
  FILTRAGGIO_HIGH[n_filter] = InputFilter.byte.high;

  for(temp_uc = 0 ; temp_uc < (INPUT_ARRAY / 2); temp_uc++) // INPUT_ARRAY = N° di ingressi da filtrare
  {
    shift = 0x1 << temp_uc;

    //ByteLow
    for(temp_sc = FILTER_WINDOW_LOOP; temp_sc >= 0; temp_sc--)
    {
      //Indice 0
      index_0 = n_filter - temp_sc;
      if (index_0 < 0)
      {
        index_0 += FILTER_WINDOW;
      }
      //Indice 1
      index_1 = n_filter - temp_sc - 1;
      if (index_1 < 0)
      {
        index_1 += FILTER_WINDOW;
      }

      if ( (FILTRAGGIO_LOW[index_0] ^ FILTRAGGIO_LOW[index_1]) & shift)
      {
        ChangeStatus++;
      }

      if ( FILTRAGGIO_LOW[index_0] & shift)
      {
        one_counter++;
      }
      else
      {
        zero_counter++;
      }

      if (temp_sc == 0)
      {
        if (FILTRAGGIO_LOW[index_1] & shift)
        {
          one_counter++;
        }
        else
        {
          zero_counter++;
        }
      }
    }

    if (ChangeStatus > MAX_CHANGE)
    {
      if (zero_counter >= MIN_COUNT)
      {
        Out_Status = LOW_FILTER;
      }
      else if (one_counter >= MIN_COUNT)
      {
        Out_Status = HIGH_FILTER;
      }
      else
      {
        Out_Status = ERRORE_FILTRO;
      }
    }
    else
    {
      if (zero_counter > one_counter)
      {
        Out_Status = LOW_FILTER;
      }
      else
      {
        Out_Status = HIGH_FILTER;
      }
    }

    zero_counter = COUNT_RESET;
    one_counter = COUNT_RESET;
    ChangeStatus = COUNT_RESET;

    // Segnale d'ingresso filtrato
    if (Out_Status != ERRORE_FILTRO)
    {
      if (!temp_uc)
      {
        DummyOutput_low = Out_Status;
      }
      else
      {
        DummyOutput_low |= (Out_Status << temp_uc);
      }
    }

    /*Byte High*/
    for(temp_sc = FILTER_WINDOW_LOOP; temp_sc >= 0 ; temp_sc--)
    {
      /*Indice 0*/
      index_0 = n_filter - temp_sc;
      if (index_0 < 0)
      {
        index_0 += FILTER_WINDOW;
      }
      /*Indice 1*/
      index_1 = n_filter - temp_sc - 1;
      if (index_1 < 0)
      {
        index_1 += FILTER_WINDOW;
      }

      if ( (FILTRAGGIO_HIGH[index_0] ^ FILTRAGGIO_HIGH[index_1]) & shift)
      {
        ChangeStatus++;
      }

      if (FILTRAGGIO_HIGH[index_0] & shift)
      {
        one_counter++;
      }
      else
      {
        zero_counter++;
      }

      if (temp_sc == 0)
      {
        if (FILTRAGGIO_HIGH[index_1] & shift)
        {
          one_counter++;
        }
        else
        {
          zero_counter++;
        }
      }
    }

    if (ChangeStatus > MAX_CHANGE)
    {
      if (zero_counter >= MIN_COUNT)
      {
        Out_Status = LOW_FILTER;
      }
      else if (one_counter >= MIN_COUNT)
      {
        Out_Status = HIGH_FILTER;
      }
      else
      {
        Out_Status = ERRORE_FILTRO;
      }
    }
    else
    {
      if (zero_counter > one_counter)
      {
        Out_Status = LOW_FILTER;
      }
      else
      {
        Out_Status = HIGH_FILTER;
      }
    }

    zero_counter = COUNT_RESET;
    one_counter = COUNT_RESET;
    ChangeStatus = COUNT_RESET;

    /* Segnale d'ingresso filtrato */
    if (Out_Status != ERRORE_FILTRO)
    {
      if (!temp_uc)
      {
        DummyOutput_high = Out_Status;
      }
      else
      {
        DummyOutput_high |= (Out_Status << temp_uc);
      }
    }
  }

  OutputFilter.byte.low = DummyOutput_low;
  OutputFilter.byte.high = DummyOutput_high;

  return (OutputFilter.word);
}


// static void AcquireSwitchOff(void)
// /*
// *//*=====================================================================*//**
// **
// **      @brief Aquire switch off signal
// **
// **      @param void
// **
// **      @retval void
// **
// **
// **
// *//*=====================================================================*//**
// */
// {
//   static unsigned char counter, fBufferFull;
//   static unsigned short buffer[N_MAX_AD_SAMPLE] = {0,0,0,0,0,0,0,0};

//   unsigned short ADCValue;
//   unsigned char j;
//   #ifdef PUMP_SLAVE
//   if(StatusPump.level == PUMP_AUTOTEST_ST && StatusPump.phase == PUMP_AUTOTEST_SWITCH_OFF)
//     #else
//   if(StatusClamp.level == CLAMP_AUTOTEST_SWITCH_OFF_ST)
//     #endif
//   {
//     if (StatusTimer(T_ADC_SAMPLE) == T_ELAPSED) //Ogni 30 sec vado a campionare il segnale analogico proveniente dalla batteria
//     {

//       StartTimer(T_ADC_SAMPLE);
//       ADCValue = 0;
//       SwitchOffAverage = 0;


//       IFS0bits.AD1IF = 0;         // clear ADC interrupt flag

//       while (!AD1CON1bits.DONE);// conversion done?
//       ADCValue = ADC1BUF0; // yes then get ADC value


//       if(counter < N_MAX_AD_SAMPLE )
//       {
//         buffer[counter++] =ADCValue;

//       }
//       else
//       {
//         counter = 0;
//         buffer[counter++] =ADCValue;
//         fBufferFull= TRUE;
//       }

//       if (fBufferFull)
//       {
//         for(j= 0; j< N_MAX_AD_SAMPLE; j++)
//         {
//           SwitchOffAverage +=  buffer[j];

//         }
//         SwitchOffAverage = SwitchOffAverage/ N_MAX_AD_SAMPLE;
//       }
//       else // vi entro solo la prima volta finchè non riempio il buffer, poi essendo circolare non vi entro più
//       {
//         for(j= 0; j< counter; j++)
//         {
//           SwitchOffAverage += buffer[j];

//         }

//         SwitchOffAverage = SwitchOffAverage / counter;

//       }
//       SwitchOffVoltage=((SwitchOffAverage * VR_POSITIVE * VOLT_TO_MV)) / _10_BIT_MAX_VALUE;
//       if (SwitchOffVoltage < SW_OFF_THR_0_VOLTAGE)
//       {
//       SwitchOffStatus = FALSE;
//       }
//       else if (SwitchOffVoltage >= SW_OFF_THR_0_VOLTAGE && SwitchOffVoltage < SW_OFF_THR_1_VOLTAGE )
//       {
//       SwitchOffStatus = TRUE;
//       }
//     }
//   }
// }

static void GetKeybStatus(void)
/*
*//*=====================================================================*//**
**
**      @brief Filter of the keys state
**
**      @param InputFilter input
**
**      @retval ouput filtered
**
**
**
*//*=====================================================================*//**
*/
{
  if (!IS_START_KEY_PRESSED())
  {
    fNewReading = TRUE;
    KeyboardStatus = NO_TASTI;
  }
  
  if (fNewReading)
  {
    if (IS_START_KEY_PRESSED())
    {      
      KeyboardStatus = KEY_START_STOP;
    }
    
    
    if (KeyboardStatus)
    {
      fNewReading = FALSE;
    }
  }
} /* end GetKeybStatus */

static void GetSelStatus(void)
/*
*//*=====================================================================*//**
**
**      @brief Filter of the keys state
**
**      @param InputFilter input
**
**      @retval ouput filtered
**
**
**
*//*=====================================================================*//**
*/
{
  switch(SelStatus)
  {
  case CONF_TIME_SEL_0:
    color.stirringDuration = STIRRING_TIME_25_SEC;
    break;
  case CONF_TIME_SEL_1:
    color.stirringDuration = STIRRING_TIME_40_SEC;
    break;
  case CONF_TIME_SEL_2:
    color.stirringDuration = STIRRING_TIME_55_SEC;
    break;
  case CONF_TIME_SEL_3:
    color.stirringDuration = STIRRING_TIME_85_SEC;
    break; 
  }  
} /* end GetKeybStatus */
