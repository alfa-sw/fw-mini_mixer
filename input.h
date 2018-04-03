/**/
/*============================================================================*/
/**
**      @file      input.h
**
**      @brief     Header file relativo a input.c
**
**      @version   Alfa color tester
**/
/*============================================================================*/
/**/

#ifndef _INPUT_H_
#define _INPUT_H_


/*Definizione SwitchStatusType*/
typedef union {
              unsigned char bytes[2];
              unsigned short word;
              struct {
                     unsigned short  StatusType0    			:1,
                                     StatusType1    			:1,
                                     StatusType2     			:1,   
                                     StatusType3     			:1,  
                                     StatusType4     			:1, 
                                     StatusType5     			:1, 
                                     StatusType6     			:1,   
                                     StatusType7    			:1,   
                                     StatusType8     			:1,                                      	 
                         						unused :7;
                     } Bit;
						  struct {
				              unsigned char low;
				              unsigned char high;
				             } byte;
              } DigInStatusType;



// Macro per KeyboardStatus
enum
{
  NO_TASTI	=0,
  KEY_START_STOP,
};  

#define CONF_TIME_SEL_0 0b00000
#define CONF_TIME_SEL_1 0b00001
#define CONF_TIME_SEL_2 0b00010
#define CONF_TIME_SEL_3 0b00100
#define CONF_TIME_SEL_4 0b01000 //n.u
#define CONF_TIME_SEL_5 0b10000 //n.u
#define CONF_TIME_SEL_6 0b00011 //n.u

extern void inputManager(void);
extern DigInStatusType DigInStatus, DigInNotFiltered;

#endif
