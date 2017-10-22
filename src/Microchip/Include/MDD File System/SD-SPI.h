/********************************************************************
 * Copyright (C) 2017 Jean Wlodarski
 * 
 * KaZjjW at gmailcom
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * The original code is from Microchip, under the Microchip license.
********************************************************************/

/******************************************************************************
 *
 *                Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        SD-SPI.h
 * Dependencies:    GenericTypeDefs.h
 *                  FSconfig.h
 *                  FSDefs.h
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
 * Compiler:        C18/C30/C32
 * Company:         Microchip Technology, Inc.
 * Version:         1.3.0
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") for its PICmicro� Microcontroller is intended and
 * supplied to you, the Company�s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*****************************************************************************/

#ifndef SDMMC_H
#define SDMMC_H

#include "GenericTypeDefs.h"
#include "FSconfig.h"
#include "MDD File System/FSDefs.h"


    // Description: This macro is used to initialize a PIC18 SPI module with a 4x prescale divider
    #define   SYNC_MODE_FAST    0x00
    // Description: This macro is used to initialize a PIC18 SPI module with a 16x prescale divider
    #define   SYNC_MODE_MED     0x01
    // Description: This macro is used to initialize a PIC18 SPI module with a 64x prescale divider
    #define   SYNC_MODE_SLOW    0x02

/*****************************************************************/
/*                  Strcutures and defines                       */
/*****************************************************************/


#define EEPROM_SECTOR_PROT_REG_LOCKED   0b100000000
#define EEPROM_ERASE_PROGRAM_ERROR      0b001000000
#define EEPROM_WRITE_ENABLED            0b000000010
#define EEPROM_BUSY                     0b000000001

// ATMEL EEPROM SPI opcodes
#define OPCODE_READ_LF    			0x03    // Read data from memory array beginning at selected address, Low Frequency CLK
#define OPCODE_READ_HF    			0x0B    // Read data from memory array beginning at selected address, High Frequency CLK

#define OPCODE_WRITE   				0x02    // Write data to memory array beginning at selected address
#define OPCODE_WRDI   			 	0x04    // Reset the write enable latch (disable write operations)
#define OPCODE_WREN    				0x06    // Set the write enable latch (enable write operations)
#define OPCODE_RDSR    				0x05    // Read Status register
#define OPCODE_WRSR    				0x01    // Write Status register

#define	READ_STATUS_REGISTER		0x05	// read status register
#define BLOCK_4K_ERASE              0x20	// 4KBytes block erase
#define BLOCK_32K_ERASE             0x52	// 32KBytes block erase
#define BLOCK_64K_ERASE             0xD8	// 64KBytes block erase
#define CHIP_ERASE         	        0x60	// Chip erase
#define READ_DEVICE_ID              0x9F	// Read Manufacturer and Device ID
#define DEEP_PWR_DOWN               0xB9	// Deep Power-Down
#define DEEP_PWR_DOWN_RESUME        0xAB	// Resume from Deep Power-Down
#define READ_PROT_SECT_REGISTER     0x3C	//Read Sector Protection Registers
#define AAI_WRITE                   0xAD    //SSI EEPROM sequential write
//! Macro to disable MSSP
#define  DISABLE_MSSP() 	(SSP2CON1bits.SSPEN = 0)

//! Macro to enable MSSP
#define  ENABLE_MSSP()  	(SSP2CON1bits.SSPEN = 1)
/*! active low on the /CS pin selects the EEPROM device
  *    whereas active high de-selects the EEPROM device on the SPI bus
 */
#define CS_PIN_SELECT	(PORTAbits.RA5)

//! MSSP STATUS REGister
#define MSSP_STATUS_REG         (SSPSTAT)

//! MSSP CONFIGURE REG1
#define MSSP_CONFIGURE_REG1     (SSP2CON1)

//!I2C interrupt priority enable
#define INTERRUPT_PRIORITY_BIT  (RCONbits.IPEN)

//! global interrupt enable
#define GLOBAL_INT_ENABLE       (INTCONbits.GIE)

#ifndef FALSE
    #define FALSE   0
#endif
#ifndef TRUE
    #define TRUE    !FALSE
#endif

#define INPUT   1
#define OUTPUT  0


// Description: A delay prescaler
#define DELAY_PRESCALER   (BYTE)      8

// Description: An approximation of the number of cycles per delay loop of overhead
#define DELAY_OVERHEAD    (BYTE)      5

// Description: An approximate calculation of how many times to loop to delay 1 ms in the Delayms function
#define MILLISECDELAY   (WORD)      ((GetInstructionClock()/DELAY_PRESCALER/(WORD)1000) - DELAY_OVERHEAD)


#define SD_MODE_NORMAL  0
#define SD_MODE_HC      1


//Definition for a structure used when calling either MDD_SDSPI_AsyncReadTasks() 
//function, or the MDD_SDSPI_AsyncWriteTasks() function.
typedef struct
{
    WORD wNumBytes;         //Number of bytes to attempt to read or write in the next call to MDD_SDSPI_AsyncReadTasks() or MDD_SDSPI_AsyncWriteTasks.  May be updated between calls to the handler.
    DWORD dwBytesRemaining; //Should be initialized to the total number of bytes that you wish to read or write.  This value is allowed to be greater than a single block size of the media.
    BYTE* pBuffer;          //Pointer to where the read/written bytes should be copied to/from.  May be updated between calls to the handler function.
    DWORD dwAddress;        //Starting block address to read or to write to on the media.  Should only get initialized, do not modify after that.
    BYTE bStateVariable;    //State machine variable.  Should get initialized to ASYNC_READ_QUEUED or ASYNC_WRITE_QUEUED to start an operation.  After that, do not modify until the read or write is complete.
}ASYNC_IO;   


//Response codes for the MDD_SDSPI_AsyncReadTasks() function.
#define ASYNC_READ_COMPLETE             0x00
#define ASYNC_READ_BUSY                 0x01
#define ASYNC_READ_NEW_PACKET_READY     0x02
#define ASYNC_READ_ERROR                0xFF

//MDD_SDSPI_AsyncReadTasks() state machine variable values.  These are used internally to SD-SPI.c.
#define ASYNC_READ_COMPLETE             0x00
#define ASYNC_READ_QUEUED               0x01    //Initialize to this to start a read sequence
#define ASYNC_READ_WAIT_START_TOKEN     0x03
#define ASYNC_READ_NEW_PACKET_READY     0x02
#define ASYNC_READ_ABORT                0xFE
#define ASYNC_READ_ERROR                0xFF

//Possible return values when calling MDD_SDSPI_AsyncWriteTasks()
#define ASYNC_WRITE_COMPLETE        0x00
#define ASYNC_WRITE_SEND_PACKET     0x02
#define ASYNC_WRITE_BUSY            0x03
#define ASYNC_WRITE_ERROR           0xFF

//MDD_SDSPI_AsyncWriteTasks() state machine variable values.  These are used internally to SD-SPI.c.
#define ASYNC_WRITE_COMPLETE            0x00
#define ASYNC_WRITE_QUEUED              0x01    //Initialize to this to start a write sequence
#define ASYNC_WRITE_TRANSMIT_FIRST_PACKET     0x02
#define ASYNC_WRITE_TRANSMIT_SECOND_PACKET     0x03
#define ASYNC_WRITE_MEDIA_BUSY          0x04
#define ASYNC_STOP_TOKEN_SENT_WAIT_BUSY 0x05
#define ASYNC_WRITE_ABORT               0xFE
#define ASYNC_WRITE_ERROR               0xFF


//Constants
#define MEDIA_BLOCK_SIZE            512u  //Should always be 512 for v1 and v2 devices.
#define WRITE_RESPONSE_TOKEN_MASK   0x1F  //Bit mask to AND with the write token response byte from the media, to clear the don't care bits.



/***************************************************************************/
/*                               Macros                                    */
/***************************************************************************/

// Description: A macro to send clock cycles to dummy-read the CRC
//#define mReadCRC()              WriteSPIM(0xFF);WriteSPIM(0xFF);

// Description: A macro to send clock cycles to dummy-write the CRC
//#define mSendCRC()              WriteSPIM(0xFF);WriteSPIM(0xFF);

// Description: A macro to send 8 clock cycles for SD timing requirements
//#define mSend8ClkCycles()       WriteSPIM(0xFF);

/*****************************************************************************/
/*                                 Public Prototypes                         */
/*****************************************************************************/

//These are the public API functions provided by SD-SPI.c
BYTE MDD_SDSPI_MediaDetect(void);
MEDIA_INFORMATION * MDD_SDSPI_MediaInitialize(void);
DWORD MDD_SDSPI_ReadCapacity(void);
WORD MDD_SDSPI_ReadSectorSize(void);
void MDD_SDSPI_InitIO(void);
BYTE MDD_SDSPI_SectorRead(DWORD sector_addr, BYTE* buffer);
BYTE MDD_SDSPI_SectorWrite(DWORD sector_addr, BYTE* buffer, BYTE allowWriteToZero);
BYTE MDD_SDSPI_AsyncReadTasks(ASYNC_IO*);
BYTE MDD_SDSPI_AsyncWriteTasks(ASYNC_IO*);
BYTE MDD_SDSPI_WriteProtectState(void);
BYTE MDD_SDSPI_ShutdownMedia(void);
BYTE MMD_EEPROM_Busy(void); //=1 si busy, =0 si pas busy
BYTE EEPROM_Erase4K(DWORD address);
void Delayms(BYTE milliseconds);
BYTE EEPROM_EraseEEPROM(void);
unsigned char WriteSPIM( unsigned char data_out );
BYTE MDD_SDSPI_WriteLowPower(DWORD Address,BYTE* buffer);

#endif
