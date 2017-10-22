#ifndef __I2C_H
#define __I2C_H
/******************************************************************************
 // *                   I2C PERIPHERAL LIBRARY HEADER FILE
 ******************************************************************************
 * FileName:        		i2c.h
 * Dependencies:    	See include below
 * Processor:       		PIC18
 * Compiler:        		MCC18
 * Company:         		Microchip Technology, Inc.
 *
 * Software License Agreement
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *****************************************************************************/
 
#include "pconfig.h"

/* SSPCON1 REGISTER */
#define   SSPENB    			0b00100000  	/* Enable serial port and configures SCK, SDO, SDI*/
#define   SLAVE_7   			0b00000110     	/* I2C Slave mode, 7-bit address*/
#define   SLAVE_10  			0b00000111    	/* I2C Slave mode, 10-bit address*/
#define   MASTER    			0b00001000     	/* I2C Master mode */
#define   MASTER_FIRMW			0b00001011		//I2C Firmware Controlled Master mode (slave Idle)
#define   SLAVE_7_STSP_INT 		0b00001110		//I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
#define   SLAVE_10_STSP_INT 	0b00001111		//I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled

/* SSPSTAT REGISTER */
#define   SLEW_OFF  			0b10000000  	/* Slew rate disabled for 100kHz mode */
#define   SLEW_ON   			0b00000000  	/* Slew rate enabled for 400kHz mode  */


#if defined (I2C_V2) || defined (I2C_V3) || defined (I2C_V5) || defined (I2C_V6) || defined (I2C_V6_1)/* These versions have MSSP1 */

/* ***** I2C1 ***** */

/***********************************************************************************
Macro       : EnableIntI2C1

Include     : i2c.h

Description : Macro enables I2C  Interrupt
 
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define EnableIntI2C1                   (PIE1bits.SSP1IE = 1)

/***********************************************************************************
Macro       : DisableIntI2C1

Include     : i2c.h

Description : Macro disables I2C  Interrupt
 
Arguments   : None
 
Remarks     : None
***********************************************************************************/
#define DisableIntI2C1                   (PIE1bits.SSP1IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntI2C1(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C interrupt
 
Arguments   : priority - This input parameter is the level of interrupt priority.
		* 0 -- Low priority (Default Value)
		* 1 -- High Priority
 Remarks     : None
***********************************************************************************/
#define SetPriorityIntI2C1(priority)     (IPR1bits.SSP1IP = priority)

/*******************************************************************
Macro       : I2C1_Clear_Intr_Status_Bit

Include     : i2c.h

Description : Macro to Clear I2C  Interrupt Status bit

Arguments   : None

Remarks     : None
*******************************************************************/
#define I2C1_Clear_Intr_Status_Bit     (PIR1bits.SSP1IF = 0)

/*******************************************************************
Macro       : I2C1_Intr_Status

Include     : i2c.h 

Description : Macro to return I2C Interrupt Status

Arguments   : None

Remarks     : None
*******************************************************************/
#define I2C1_Intr_Status		PIR1bits.SSP1IF

/*******************************************************************
Macro       : StopI2C1()

Include     : i2c.h

Description : Macro to initiate stop condition

Arguments   : None

Remarks     : This macro initiates stop condition and waits till the stop signal
		sequence is terminated.This macro is applicable only to master
*******************************************************************/
#define StopI2C1()  SSP1CON2bits.PEN=1;while(SSP1CON2bits.PEN)

#define StopI2C StopI2C1

/*******************************************************************
Macro       : StartI2C1()

Include     : i2c.h

Description : Macro to initiate start condition

Arguments   : None

Remarks     : This macro initiates start condition and waits till the start signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define StartI2C1()  SSP1CON2bits.SEN=1;while(SSP1CON2bits.SEN)

#define StartI2C StartI2C1 

/*******************************************************************
Macro       : RestartI2C1()

Include     : i2c.h

Description : Macro to initiate Restart condition

Arguments   : None

Remarks     : This macro initiates Restart condition and waits till the Restart signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define RestartI2C1()  SSP1CON2bits.RSEN=1;while(SSP1CON2bits.RSEN)

#define RestartI2C RestartI2C1

/*******************************************************************
Macro       : NotAckI2C1()

Include     : i2c.h

Description : Macro to initiate negetive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates negetive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define NotAckI2C1()     SSP1CON2bits.ACKDT=1, SSP1CON2bits.ACKEN=1;while(SSP1CON2bits.ACKEN)

#define NotAckI2C NotAckI2C1

/*******************************************************************
Macro       : AckI2C1()

Include     : i2c.h

Description : Macro to initiate positive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates positive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define AckI2C1()        SSP1CON2bits.ACKDT=0, SSP1CON2bits.ACKEN=1;while(SSP1CON2bits.ACKEN)

#define AckI2C AckI2C1

/**********************************************************************************************
Macro :  IdleI2C1() 

Include            : i2c.h 

Description        : This Macro generates Wait condition until I2C bus is Idle.

Arguments          : None 

Remarks            : This Macro will be in a wait state until Start Condition Enable bit,
                     Stop Condition Enable bit, Receive Enable bit, Acknowledge Sequence
                     Enable bit of I2C Control register and Transmit Status bit I2C Status
                     register are clear. The IdleI2C function is required since the hardware
                     I2C peripheral does not allow for spooling of bus sequence. The I2C
                     peripheral must be in Idle state before an I2C operation can be initiated
                     or write collision will be generated.
***********************************************************************************************/
#define IdleI2C1()    while ((SSP1CON2 & 0x1F) | (SSP1STATbits.R_W))

#define IdleI2C IdleI2C1

/*********************************************************************
Macro  :  CloseI2C1()

Include            : i2c.h 

Description        : This Macro turns off the I2C module 

Arguments          : None 

Return Value       : None 

Remarks            : This Macro disables the I2C module.
*********************************************************************/
#define CloseI2C1()  SSP1CON1 &=0xDF

#define CloseI2C CloseI2C1

void OpenI2C1(  unsigned char sync_mode,  unsigned char slew );
#define OpenI2C OpenI2C1

/************************************************************************
Macro :  DataRdyI2C1() 

Include            : i2c.h 

Description        : This Macro provides status back to user if SSPxBUF
			register contain data. 
                     
Arguments          : None 

Remarks            : This Macro determines if there is any byte to read from
			SSPxBUF register.
*************************************************************************/
#define DataRdyI2C1()    (SSP1STATbits.BF)

#define DataRdyI2C DataRdyI2C1

unsigned char ReadI2C1( void );
#define ReadI2C ReadI2C1

/**************************************************************************
Macro       : getcI2C1

Description : macro is identical to ReadI2C1,#define to ReadI2C1 in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  getcI2C1  ReadI2C1
#define  getcI2C getcI2C1

unsigned char WriteI2C1(  unsigned char data_out );
#define WriteI2C WriteI2C1

/**************************************************************************
Macro       : putcI2C1

Description : macro is identical to WriteI2C1,#define to WriteI2C1 in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  putcI2C1  WriteI2C1
#define  putcI2C putcI2C1

unsigned char getsI2C1(  unsigned char *rdptr,  unsigned char length );
#define getsI2C getsI2C1

unsigned char putsI2C1(  unsigned char *wrptr );
#define putsI2C putsI2C1




//****************** I2C EEPROM Function prototype ******************************

unsigned char EEAckPolling1(  unsigned char control );
#define EEAckPolling EEAckPolling1

unsigned char EEByteWrite1(  unsigned char control,
                            unsigned char address,
                            unsigned char data );
#define EEByteWrite EEByteWrite1

unsigned int  EECurrentAddRead1(  unsigned char control );
#define EECurrentAddRead EECurrentAddRead1

unsigned char EEPageWrite1(  unsigned char control,
                            unsigned char address,
                            unsigned char *wrptr );
#define EEPageWrite EEPageWrite1

unsigned int  EERandomRead1(  unsigned char control,  unsigned char address );
#define EERandomRead EERandomRead1

unsigned char EESequentialRead1(  unsigned char control,
                                 unsigned char address,
                                 unsigned char *rdptr,
                                 unsigned char length );
#define EESequentialRead EESequentialRead1
#endif



#if defined (I2C_V3) || defined (I2C_V6) || defined (I2C_V6_1) /*This version has MSSP2*/
/* ***** I2C2 ***** */
#if defined (I2C_V3) || defined (I2C_V6)
/***********************************************************************************
Macro       : EnableIntI2C2

Include     : i2c.h

Description : Macro enables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntI2C2                   (PIE3bits.SSP2IE = 1)

/***********************************************************************************
Macro       : DisableIntI2C2

Include     : i2c.h

Description : Macro disables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntI2C2                   (PIE3bits.SSP2IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntI2C2(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
		* 0 -- Low priority (Default Value)
		* 1 -- High Priority
 Remarks     : None
***********************************************************************************/
#define SetPriorityIntI2C2(priority)     (IPR3bits.SSP2IP = priority)

/*******************************************************************
Macro       : I2C2_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C  Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C2_Clear_Intr_Status_Bit     (PIR3bits.SSP2IF = 0)

/*******************************************************************
Macro       : I2C2_Intr_Status

Include     : i2c.h 

Description : Macro to return I2C Interrupt Status  

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C2_Intr_Status		PIR3bits.SSP2IF

#elif defined(I2C_V6_1)

/***********************************************************************************
Macro       : EnableIntI2C2

Include     : i2c.h

Description : Macro enables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntI2C2                   (PIE2bits.SSP2IE = 1)

/***********************************************************************************
Macro       : DisableIntI2C2

Include     : i2c.h

Description : Macro disables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntI2C2                   (PIE2bits.SSP2IE = 0)

/***********************************************************************************
Macro       : SetPriorityIntI2C2(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
		* 0 -- Low priority (Default Value)
		* 1 -- High Priority
 Remarks     : None
***********************************************************************************/
#define SetPriorityIntI2C2(priority)     (IPR2bits.SSP2IP = priority)

/*******************************************************************
Macro       : I2C2_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C  Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C2_Clear_Intr_Status_Bit     (PIR2bits.SSP2IF = 0)

/*******************************************************************
Macro       : I2C2_Intr_Status

Include     : i2c.h 

Description : Macro to return I2C Interrupt Status  

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C2_Intr_Status		PIR2bits.SSP2IF

#endif

/*******************************************************************
Macro       : StopI2C2()

Include     : i2c.h

Description : Macro to initiate stop condition

Arguments   : None

Remarks     : This macro initiates stop condition and waits till the stop signal
		sequence is terminated.This macro is applicable only to master
*******************************************************************/
#define StopI2C2()  SSP2CON2bits.PEN=1;while(SSP2CON2bits.PEN)

/*******************************************************************
Macro       : StartI2C2()

Include     : i2c.h

Description : Macro to initiate start condition

Arguments   : None

Remarks     : This macro initiates start condition and waits till the start signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define StartI2C2()  SSP2CON2bits.SEN=1;while(SSP2CON2bits.SEN)

/*******************************************************************
Macro       : RestartI2C2()

Include     : i2c.h

Description : Macro to initiate Restart condition

Arguments   : None

Remarks     : This macro initiates Restart condition and waits till the Restart signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define RestartI2C2()  SSP2CON2bits.RSEN=1;while(SSP2CON2bits.RSEN)

/*******************************************************************
Macro       : NotAckI2C2()

Include     : i2c.h

Description : Macro to initiate negetive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates negetive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define NotAckI2C2()     SSP2CON2bits.ACKDT=1, SSP2CON2bits.ACKEN=1;while(SSP2CON2bits.ACKEN)

/*******************************************************************
Macro       : AckI2C2()

Include     : i2c.h

Description : Macro to initiate positive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates positive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define AckI2C2()        SSP2CON2bits.ACKDT=0, SSP2CON2bits.ACKEN=1;while(SSP2CON2bits.ACKEN)

#if defined (I2C_SFR_V1)
/**********************************************************************************************
Macro :  IdleI2C2() 

Include            : i2c.h 

Description        : This Macro generates Wait condition until I2C bus is Idle.

Arguments          : None 

Remarks            : This Macro will be in a wait state until Start Condition Enable bit,
                     Stop Condition Enable bit, Receive Enable bit, Acknowledge Sequence
                     Enable bit of I2C Control register and Transmit Status bit I2C Status
                     register are clear. The IdleI2C function is required since the hardware
                     I2C peripheral does not allow for spooling of bus sequence. The I2C
                     peripheral must be in Idle state before an I2C operation can be initiated
                     or write collision will be generated.
***********************************************************************************************/
#define IdleI2C2()    while ((SSP2CON2 & 0x1F) | (SSP2STATbits.R_NOT_W))
#else
/**********************************************************************************************
Macro :  IdleI2C2() 

Include            : i2c.h 

Description        : This Macro generates Wait condition until I2C bus is Idle.

Arguments          : None 

Remarks            : This Macro will be in a wait state until Start Condition Enable bit,
                     Stop Condition Enable bit, Receive Enable bit, Acknowledge Sequence
                     Enable bit of I2C Control register and Transmit Status bit I2C Status
                     register are clear. The IdleI2C function is required since the hardware
                     I2C peripheral does not allow for spooling of bus sequence. The I2C
                     peripheral must be in Idle state before an I2C operation can be initiated
                     or write collision will be generated.
***********************************************************************************************/
#define IdleI2C2()    while ((SSP2CON2 & 0x1F) | (SSP2STATbits.R_W))
#endif


/*********************************************************************
Macro  :  CloseI2C2()

Include            : i2c.h 

Description        : This Macro turns off the I2C module 

Arguments          : None 

Return Value       : None 

Remarks            : This Macro disables the I2C module.
*********************************************************************/
#define CloseI2C2()  SSP2CON1 &=0xDF

void OpenI2C2(  unsigned char sync_mode,  unsigned char slew );

/************************************************************************
Macro :  DataRdyI2C2() 

Include            : i2c.h 

Description        : This Macro provides status back to user if SSPxBUF
			register contain data. 
                     
Arguments          : None 

Remarks            : This Macro determines if there is any byte to read from
			SSPxBUF register.
*************************************************************************/
#define DataRdyI2C2()    (SSP2STATbits.BF)

unsigned char ReadI2C2( void );

/**************************************************************************
Macro       : getcI2C2

Description : macro is identical to ReadI2C2,#define to ReadI2C2 in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  getcI2C2  ReadI2C2

unsigned char WriteI2C2(  unsigned char data_out );

/**************************************************************************
Macro       : putcI2C2

Description : macro is identical to WriteI2C2,#define to WriteI2C2 in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  putcI2C2  WriteI2C2

unsigned char getsI2C2(  unsigned char *rdptr,  unsigned char length );

signed char putsI2C2(  unsigned char *wrptr );



//******************* I2C EEPROM Function Prototype ******************************

unsigned char EEAckPolling2(  unsigned char control );

unsigned char EEByteWrite2(  unsigned char control,
                            unsigned char address,
                            unsigned char data );

unsigned int  EECurrentAddRead2(  unsigned char control );

unsigned char EEPageWrite2(  unsigned char control,
                            unsigned char address,
                            unsigned char *wrptr );

unsigned int  EERandomRead2(  unsigned char control,  unsigned char address );

unsigned char EESequentialRead2(  unsigned char control,
                                 unsigned char address,
                                 unsigned char *rdptr,
                                 unsigned char length );
#endif



#if defined (I2C_V1) || defined (I2C_V4)    /* Parts with only one I2C, MSSP */

//******** I2C **********

/***********************************************************************************
Macro       : EnableIntI2C

Include     : i2c.h

Description : Macro enables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define EnableIntI2C                   (PIE1bits.SSPIE = 1)

#define EnableIntI2C1	EnableIntI2C

/***********************************************************************************
Macro       : DisableIntI2C

Include     : i2c.h

Description : Macro disables I2C  Interrupt 
 
Arguments   : None
 
Remarks     : None 
***********************************************************************************/
#define DisableIntI2C                   (PIE1bits.SSPIE = 0)

#define DisableIntI2C1	DisableIntI2C

/***********************************************************************************
Macro       : SetPriorityIntI2C(priority)
 
Include     : i2c.h
 
Description : Macro sets the priority level for I2C interrupt.
 
Arguments   : priority - This input parameter is the level of interrupt priority.
		* 0 -- Low priority (Default Value)
		* 1 -- High Priority
 Remarks     : None
***********************************************************************************/
#define SetPriorityIntI2C(priority)     (IPR1bits.SSPIP = priority)

#define SetPriorityIntI2C1	SetPriorityIntI2C

/*******************************************************************
Macro       : I2C_Clear_Intr_Status_Bit

Include     : i2c.h 

Description : Macro to Clear I2C  Interrupt Status bit 

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C_Clear_Intr_Status_Bit     (PIR1bits.SSPIF = 0)

#define I2C1_Clear_Intr_Status_Bit	I2C_Clear_Intr_Status_Bit

/*******************************************************************
Macro       : I2C_Intr_Status

Include     : i2c.h 

Description : Macro to return I2C Interrupt Status  

Arguments   : None 

Remarks     : None 
*******************************************************************/
#define I2C_Intr_Status		PIR1bits.SSPIF

#define I2C1_Intr_Status	I2C_Intr_Status

/**************************************************************************
Macro       : getcI2C

Description : macro is identical to ReadI2C,#define to ReadI2C in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  getcI2C  ReadI2C

/**************************************************************************
Macro       : putcI2C

Description : macro is identical to WriteI2C,#define to WriteI2C in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  putcI2C  WriteI2C

void IdleI2C( void );

void OpenI2C(  unsigned char sync_mode,  unsigned char slew );

unsigned char WriteI2C( unsigned char data_out );

signed char putsI2C(  unsigned char *wrptr );

unsigned char ReadI2C( void );

void CloseI2C( void );

#endif

#if defined (I2C_V1) 

/* CloseI2C
 * Disable SPI module
 */
//#define CloseI2C()  SSPCON1 &=0xDF

/* Idle I2C
 * Test if I2C module is idle
 */
//#define IdleI2C()    while ((SSPCON2 & 0x1F) | (SSPSTATbits.R_W))

/*******************************************************************
Macro       : StopI2C()

Include     : i2c.h

Description : Macro to initiate stop condition

Arguments   : None

Remarks     : This macro initiates stop condition and waits till the stop signal
		sequence is terminated.This macro is applicable only to master
*******************************************************************/
#define StopI2C()  SSPCON2bits.PEN=1;while(SSPCON2bits.PEN)

/*******************************************************************
Macro       : StartI2C()

Include     : i2c.h

Description : Macro to initiate start condition

Arguments   : None

Remarks     : This macro initiates start condition and waits till the start signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define StartI2C()  SSPCON2bits.SEN=1;while(SSPCON2bits.SEN)

/*******************************************************************
Macro       : RestartI2C()

Include     : i2c.h

Description : Macro to initiate Restart condition

Arguments   : None

Remarks     : This macro initiates Restart condition and waits till the Restart signal
		sequence is terminated. This macro is applicable only to master
*******************************************************************/
#define RestartI2C()  SSPCON2bits.RSEN=1;while(SSPCON2bits.RSEN)

/*******************************************************************
Macro       : NotAckI2C()

Include     : i2c.h

Description : Macro to initiate negetive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates negetive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define NotAckI2C()     SSPCON2bits.ACKDT=1;SSPCON2bits.ACKEN=1;while(SSPCON2bits.ACKEN)

/*******************************************************************
Macro       : AckI2C()

Include     : i2c.h

Description : Macro to initiate positive acknowledgement sequence

Arguments   : None

Remarks     : This macro initiates positive acknowledgement condition and 
		waits till the acknowledgement sequence is terminated.
		This macro is applicable only to master
*******************************************************************/
#define AckI2C()        SSPCON2bits.ACKDT=0;SSPCON2bits.ACKEN=1;while(SSPCON2bits.ACKEN)

/************************************************************************
Macro :  DataRdyI2C() 

Include            : i2c.h 

Description        : This Macro provides status back to user if SSPxBUF
			register contain data. 
                     
Arguments          : None 

Remarks            : This Macro determines if there is any byte to read from
			SSPxBUF register.
*************************************************************************/
#define DataRdyI2C()    (SSPSTATbits.BF)

/**************************************************************************
Macro       : putcI2C

Description : macro is identical to WriteI2C,#define to WriteI2C in i2c.h
 
Arguments   : None
 
Remarks     : None 
***************************************************************************/
#define  putcI2C  WriteI2C

unsigned char WriteI2C(  unsigned char data_out );

unsigned char getsI2C(  unsigned char *rdptr,  unsigned char length );




//******************* I2C EEPROM Function Prototype ******************************

unsigned char EEAckPolling(  unsigned char control );

unsigned char EEByteWrite(  unsigned char control,
                            unsigned char address,
                            unsigned char data );

unsigned int  EECurrentAddRead(  unsigned char control );

unsigned char EEPageWrite(  unsigned char control,
                            unsigned char address,
                            unsigned char *wrptr );

unsigned int  EERandomRead(  unsigned char control,  unsigned char address );

unsigned char EESequentialRead(  unsigned char control,
                                 unsigned char address,
                                 unsigned char *rdptr,
                                 unsigned char length );
#endif


#if defined (I2C_IO_V1 ) 
#define I2C_SCL	TRISCbits.TRISC3
#define I2C_SDA	TRISCbits.TRISC4
#endif

#if defined (I2C_IO_V2)
#define I2C_SCL	TRISCbits.TRISC5
#define I2C_SDA	TRISCbits.TRISC4
#endif

#if defined (I2C_IO_V3)
#define I2C_SCL	TRISBbits.TRISB1
#define I2C_SDA	TRISBbits.TRISB0
#endif

#if defined (I2C_IO_V4)
#define I2C_SCL	TRISBbits.TRISB6
#define I2C_SDA	TRISBbits.TRISB4

#endif

#if defined (I2C_IO_V5)
#define I2C1_SCL	TRISCbits.TRISC3
#define I2C1_SDA	TRISCbits.TRISC4
#endif

#if defined (I2C_IO_V6)
#define I2C2_SCL	TRISDbits.TRISD0
#define I2C2_SDA	TRISDbits.TRISD1
#endif

#if defined (I2C_IO_V7)
#define I2C2_SCL	TRISDbits.TRISD6
#define I2C2_SDA	TRISDbits.TRISD5
#endif

#endif

