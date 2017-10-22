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
 *               Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        SD-SPI.c
 * Dependencies:    SD-SPI.h
 *                  string.h
 *                  FSIO.h
 *                  FSDefs.h
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33/PIC32
 * Compiler:        C18/C30/C32
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the "Company") for its PICmicro‚Äö√†√∂‚àö√Ü‚Äö√†√∂‚Äö√†√®‚âà√≠¬¨¬© Microcontroller is intended and
 * supplied to you, the Company‚Äö√†√∂‚àö√Ü‚Äö√†√∂‚Äö√†√®‚âà√≠¬¨¬©s customer, for use solely and
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
*****************************************************************************
 File Description:

 Change History:
  Rev     Description
  -----   -----------
  1.2.5   Fixed bug in the calculation of the capacity for v1.0 devices
  1.3.0   Improved media initialization sequence, for better card compatibility
          (especially with SDHC cards).
          Implemented SPI optimizations for data transfer rate improvement.
          Added new MDD_SDSPI_AsyncReadTasks() and MDD_SDSPI_AsyncWriteTasks() 
          API functions.  These are non-blocking, state machine based read/write
          handlers capable of considerably improved data throughput, particularly
          for multi-block reads and multi-block writes.
  1.3.2   Modified MDD_SDSPI_AsyncWriteTasks() so pre-erase command only gets
          used for multi-block write scenarios.   
  1.3.4   1) Added support for dsPIC33E & PIC24E controllers.
          2) #include "HardwareProfile.h" is moved up in the order.
          3) "SPI_INTERRUPT_FLAG_ASM" macro has to be defined in "HardwareProfile.h" file
             for PIC18 microcontrollers.Or else an error is generated while building
             the code.
                       "#define SPI_INTERRUPT_FLAG_ASM  PIR1, 3" is removed from SD-SPI.c
          4) Replaced "__C30" usage with "__C30__" .

********************************************************************/

#include "Compiler.h"
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "MDD File System/FSIO.h"
#include "MDD File System/FSDefs.h"
#include "MDD File System/SD-SPI.h"
#include "string.h"
#include "FSConfig.h"

/******************************************************************************
 * Global Variables
 *****************************************************************************/

// Description:  Used for the mass-storage library to determine capacity
DWORD MDD_SDSPI_finalLBA;
WORD gMediaSectorSize;
BYTE gSDMode;
static MEDIA_INFORMATION mediaInformation;
static ASYNC_IO ioInfo; //Declared global context, for fast/code efficient access
extern BYTE WriteEnable;



/******************************************************************************
 * Prototypes
 *****************************************************************************/
extern void Delayms(BYTE milliseconds);
BYTE MDD_SDSPI_ReadMedia(void);
MEDIA_INFORMATION * MDD_SDSPI_MediaInitialize(void);
//MMC_RESPONSE SendMMCCmd(BYTE cmd, DWORD address);

#if defined __C30__ || defined __C32__
    void OpenSPIM ( unsigned int sync_mode);
    void CloseSPIM( void );
    unsigned char WriteSPIM( unsigned char data_out );
#elif defined __18CXX
    void OpenSPIM ( unsigned char sync_mode);
    void CloseSPIM( void );
    unsigned char WriteSPIM( unsigned char data_out );

    unsigned char WriteSPIManual(unsigned char data_out);
    BYTE ReadMediaManual (void);
 //   MMC_RESPONSE SendMMCCmdManual(BYTE cmd, DWORD address);
#endif
void InitSPISlowMode(void);
BYTE MMD_EEPROM_Busy(void);

#if defined __18CXX
//Private function prototypes
#if defined USE_ATMEL_FLASH
static void PIC18_Optimized_SPI_Write_Packet(void);
static void PIC18_Optimized_SPI_Read_Packet(void);
static void PIC18_Optimized_SPI_Low_Power_Write_Packet(WORD Address,DWORD Addressbuffer);
#endif
#endif

//-------------Function name redirects------------------------------------------
//During the media initialization sequence, it is
//necessary to clock the media at no more than 400kHz SPI speeds, since some
//media types power up in open drain output mode and cannot run fast initially.
//On PIC18 devices, when the CPU is run at full frequency, the standard SPI 
//prescalars cannot reach a low enough SPI frequency.  Therefore, we initialize
//the card at low speed using bit-banged SPI on PIC18 devices.  On 
//PIC32/PIC24/dsPIC devices, the SPI module is flexible enough to reach <400kHz
//speeds regardless of CPU frequency, and therefore bit-banged code is not 
//necessary.  Therefore, we use function redirects where necessary, to point to
//the proper SPI related code, given the processor type.

#if defined __18CXX
    #define SendMediaSlowCmd    SendMMCCmdManual
    #define WriteSPISlow        WriteSPIManual
#else
    #define SendMediaSlowCmd    SendMMCCmd
    #define WriteSPISlow        WriteSPIM
#endif
//------------------------------------------------------------------------------


//These funcions are based on the ones to manage a SD CARD, modified to work with an ATMEL EEPROM

/*********************************************************
  Function:
    BYTE MDD_SDSPI_MediaDetect
  Summary:
    Determines whether an SD card is present (the EEPROM is always present, of course)
 
  Remarks:
    None                                                  
  *********************************************************/

BYTE MDD_SDSPI_MediaDetect (void)
{
    return 1;

}//end MediaDetect



/*********************************************************
  Function:
    WORD MDD_SDSPI_ReadSectorSize (void)
  Summary:
 Returns the hard-coded EEPROM sector size
  *********************************************************/

WORD MDD_SDSPI_ReadSectorSize(void)
{
    return (MEDIA_SECTOR_SIZE);
}


/*********************************************************
  Function:
    DWORD MDD_SDSPI_ReadCapacity (void)
  Summary:
 Returns the hard-coded EEPROM capacity
  *********************************************************/
DWORD MDD_SDSPI_ReadCapacity(void)
{
    return(MEDIA_CAPACITY);
}


/*********************************************************
  Function:
    WORD MDD_SDSPI_InitIO (void)
  Summary:
    Initializes the I/O lines connected to the EEPROM
    and enables all the Write modes
  *********************************************************/

void MDD_SDSPI_InitIO (void)
{
    BYTE Buff;

    SD_CS  = 1;
    SD_CS_TRIS  = 0;	// define /SS pin direction as O/p

    SPICLOCKLAT  = 1;	// setting/clearing the output latch depending upon Clock Idle state (i.e high/low)
    SPICLOCKTRIS  = 0;	// define SCL direction as O/p

    SPIOUTLAT  = 1;
    SPIOUTTRIS = 0;	// define SDO direction as O/p

    SPIINTRIS = 1;	// define SDI direction as I/p

    SSP2STAT = 0b11000000;
    SSP2CON1 = 0b00000001;

    SD_CS  = 0;
    Nop();
    //! enable MSSP (SPI)  module
    ENABLE_MSSP();

#if defined USE_ATMEL_FLASH
    SD_CS = 1;
    Nop();
    WriteSPIM( DEEP_PWR_DOWN_RESUME  ); //Just in case, exit the sleep mode
    Nop();
    SD_CS = 1;

//    MMD_EEPROM_Busy();

    SD_CS = 1;
    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
    }
    //SD_CS = 1;
   Buff = MDD_SDSPI_ReadMedia();
  
   if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   {
     SD_CS = 1;
 
    if (WriteSPIM( OPCODE_WREN) != 0)
        SD_CS = 1;
   }

    SD_CS = 1;

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
       SD_CS = 1;

   Buff = MDD_SDSPI_ReadMedia();

   if ((Buff & 0b00001000) | (Buff & 0b00001100) ) //Test is the software protection is on
   {
     SD_CS = 1;
     Nop();
     Nop();
     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        SD_CS = 1;
     if (WriteSPIM( 0x00 ) != 0)
        SD_CS = 1;
     SD_CS = 1;

   }
#endif
   SD_CS = 1;
}

/*********************************************************
  Function:
    BYTE EEPROM_EnableWrites(void)
  Summary:
    (re)enables the writes to the EEPROM (after an erase/write)
  *********************************************************/
BYTE EEPROM_EnableWrites(void)
{
    BYTE Buff=0xFF;
    MMD_EEPROM_Busy();
    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
     Nop();
     Nop();

     Nop();
     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();

   if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   {
     SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
   }
     Nop();
     Nop();
 

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();
    SD_CS = 1;

   if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   {
       SD_CS = 1;
       return 0;
   }

    SD_CS = 1;
    return 1;

}

/*********************************************************
  Function:
    BYTE EEPROM_Erase4K(void)
  Summary:
    
  Conditions:
 Erases the smallest EEPROM portion possible: 4Kb
  Input:
    None
  Return:
    None
  Side Effects:
    None.
  Description:
    
  Remarks:
    None
  *********************************************************/


BYTE EEPROM_Erase4K(DWORD address)
{
  BYTE Buff;
  DWORD_VAL temporary;

  if(!EEPROM_EnableWrites())
      return 0;

  temporary.Val = address;
#if defined USE_SST_FLASH
      SD_CS = 1;
      if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
     Nop();
     Nop();
#endif
  //Write the Erase 4Kb command followed by the address

    SD_CS = 1;
     Nop();
     Nop();
     Nop();
     Nop();
     Nop();
    if (WriteSPIM( BLOCK_4K_ERASE ) != 0)
        {
        SD_CS = 1;
        return 0;
    }

   if (WriteSPIM( temporary.v[2] ) != 0)
    {
       SD_CS = 1;
      return 0;
    }

   if (WriteSPIM( temporary.v[1] ) != 0)
    {
       SD_CS = 1;
      return 0;
    }

   if (WriteSPIM( temporary.v[0] ) != 0)
    {
       SD_CS = 1;
      return 0;
    }

    SD_CS = 1;
    Nop();
    Nop();
    Nop();
    MMD_EEPROM_Busy();
 
    Nop();
   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();
       SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
   
    SD_CS = 1;
    return 1;
}


/*********************************************************
  Function:
    BYTE EEPROM_EraseEEPROM(void)
  Summary:
 Erases all the EEPROM contents and waits till it's over (internal operation)
  *********************************************************/

BYTE EEPROM_EraseEEPROM(void)
{
  BYTE Buff;
    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();
   SD_CS = 1;
   if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   {
     SD_CS = 1;
     Nop();
     Nop();
  
    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
#if defined USE_ATMEL_FLASH
   }
     #endif
   
     Nop();
     Nop();
     Nop();
     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        }
     SD_CS = 1;
     Nop();
     Nop();

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();
	SD_CS = 1;
   if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   {
     SD_CS = 1;
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
   }
#if defined USE_SST_FLASH
   }
     #endif
     Nop();
     Nop();
     #if defined USE_SST_FLASH
      SD_CS = 1;
      if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;
     Nop();
     Nop();
#endif
SD_CS = 0;
     Nop();
     Nop();
     Nop();
     Nop();
     Nop();
    if (WriteSPIM( CHIP_ERASE ) != 0)
        {
        SD_CS = 1;
        return 0;
        }

   SD_CS = 1;
   Nop();
 

   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }
    
	Buff = MDD_SDSPI_ReadMedia(); 
   while ( (Buff&0b00000001) == 1)
   {
	//mLED_1_On();
	Buff = MDD_SDSPI_ReadMedia();  
	}
	
     Nop();
     Nop();
   SD_CS = 1;
     Nop();
     Nop();
     Nop();
     Nop();

	//mLED_1_Off();
   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

   Buff = MDD_SDSPI_ReadMedia();
    SD_CS = 1;
    return 1;
}
/*********************************************************
  Function:
    BYTE MDD_SDSPI_ShutdownMedia (void)
  Summary:

  *********************************************************/

BYTE MDD_SDSPI_ShutdownMedia(void)
{
    // close the spi bus
    CloseSPIM();
    
    // deselect the device
    SD_CS = 1;

    return 0;
}



/*********************************************************
  Function:
   BYTE MMD_EEPROM_Busy(void)
  Summary:
 * Tests the EEPROM busy status, waits till it's over.

  *********************************************************/
BYTE MMD_EEPROM_Busy(void)
{
BYTE Buff;
   SD_CS = 1;
   Nop();

   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }
    
    Buff = MDD_SDSPI_ReadMedia();

   while ( (Buff&0b00000001) == 1 )
   {
       if(USB_POWER)
        mLED_LP_On();
       if(!USB_POWER)
        mLED_LP_On();
       Buff = MDD_SDSPI_ReadMedia();
       Nop();
       Nop();
       Nop();
       Nop();
       Nop();
       Nop();
       Nop();
   }

     if(USB_POWER)
         mLED_LP_Off();
     if(!USB_POWER)
         mLED_LP_Off();

      SD_CS = 1;

}


/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_SectorRead (DWORD sector_addr, BYTE * buffer)
  Summary:
    Reads a sector of data from an SD card.
  Conditions:
    The MDD_SectorRead function pointer must be pointing towards this function.
  Input:
    sector_addr - The address of the sector on the card.
    buffer -      The buffer where the retrieved data will be stored.  If
                  buffer is NULL, do not store the data anywhere.
  Return Values:
    TRUE -  The sector was read successfully
    FALSE - The sector could not be read
  Side Effects:
    None
  Description:
    The MDD_SDSPI_SectorRead function reads a sector of data bytes (512 bytes) 
    of data from the SD card starting at the sector address and stores them in 
    the location pointed to by 'buffer.'
  Remarks:
    The card expects the address field in the command packet to be a byte address.
    The sector_addr value is converted to a byte address by shifting it left nine
    times (multiplying by 512).
    
    This function performs a synchronous read operation.  In other words, this
    function is a blocking function, and will not return until either the data
    has fully been read, or, a timeout or other error occurred.
  ***************************************************************************************/
#if defined USE_ATMEL_FLASH
BYTE MDD_SDSPI_SectorRead(DWORD sector_addr, BYTE* buffer)
{
    ASYNC_IO info;
    BYTE status;
    
    //Initialize info structure for using the MDD_SDSPI_AsyncReadTasks() function.
    info.wNumBytes = 512;
    info.dwBytesRemaining = 512;
    info.pBuffer = buffer;
    info.dwAddress = sector_addr;
    info.bStateVariable = ASYNC_READ_QUEUED;
    
    //Blocking loop, until the state machine finishes reading the sector,
    //or a timeout or other error occurs.  MDD_SDSPI_AsyncReadTasks() will always
    //return either ASYNC_READ_COMPLETE or ASYNC_READ_FAILED eventually 
    //(could take awhile in the case of timeout), so this won't be a totally
    //infinite blocking loop.
    while(1)
    {
        status = MDD_SDSPI_AsyncReadTasks(&info);
        if(status == ASYNC_READ_COMPLETE)
        {
            return TRUE;
        }
        else if(status == ASYNC_READ_ERROR)
        {
            return FALSE;
        } 
    }       

    //Impossible to get here, but we will return a value anyay to avoid possible 
    //compiler warnings.
    return FALSE;
}    

#endif

#if defined USE_SST_FLASH

BYTE MDD_SDSPI_SectorRead(DWORD sector_addr, BYTE* buffer)
{
 //   ASYNC_IO info;
    BYTE status,tmp;
    DWORD_VAL temporary;
    WORD ptrBuffer;


    MMD_EEPROM_Busy();
   
    if(sector_addr < LOGGING_SECTOR)
        sector_addr <<= 12;//9
    else
    {
       sector_addr = sector_addr - LOGGING_SECTOR;
       sector_addr <<= 9;
       sector_addr = sector_addr + LOGGING_SECTOR_1000;

    }

    SD_CS = 1;
    Nop();
    Nop();
    Nop();
WriteSPIM( READ_STATUS_REGISTER );

    tmp = MDD_SDSPI_ReadMedia();

SD_CS = 1;
Nop();
Nop();
    if (WriteSPIM( OPCODE_READ_HF ) != 0)		//High speed array read command
    {
       SD_CS = 1;
      return FALSE;
    }

    temporary.Val = sector_addr;	//MSB of the address
    if (WriteSPIM( temporary.v[2] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[1] ) != 0)		//Middle byte
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[0] ) != 0)		//LSB
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( 0xFF ) != 0)				//Don't care padding bits
    {
       SD_CS = 1;
      return FALSE;
    }

    ptrBuffer = 0;
    while(ptrBuffer < 0x200)
    {
        buffer[ptrBuffer] = MDD_SDSPI_ReadMedia();
        ptrBuffer++;

    }

    SD_CS = 1;

    return TRUE;
   
}
#endif


/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_AsyncReadTasks(ASYNC_IO* info)
  Summary:
    Speed optimized, non-blocking, state machine based read function that reads 
    data packets from the media, and copies them to a user specified RAM buffer.
  Pre-Conditions:
    The ASYNC_IO structure must be initialized correctly, prior to calling
    this function for the first time.  Certain parameters, such as the user
    data buffer pointer (pBuffer) in the ASYNC_IO struct are allowed to be changed
    by the application firmware, in between each call to MDD_SDSPI_AsyncReadTasks().
    Additionally, the media and microcontroller SPI module should have already 
    been initalized before using this function.  This function is mutually
    exclusive with the MDD_SDSPI_AsyncWriteTasks() function.  Only one operation
    (either one read or one write) is allowed to be in progress at a time, as
    both functions share statically allocated resources and monopolize the SPI bus.
  Input:
    ASYNC_IO* info -        A pointer to a ASYNC_IO structure.  The 
                            structure contains information about how to complete
                            the read operation (ex: number of total bytes to read,
                            where to copy them once read, maximum number of bytes
                            to return for each call to MDD_SDSPI_AsyncReadTasks(), etc.).
  Return Values:
    BYTE - Returns a status byte indicating the current state of the read 
            operation. The possible return values are:
            
            ASYNC_READ_BUSY - Returned when the state machine is busy waiting for
                             a data start token from the media.  The media has a
                             random access time, which can often be quite long
                             (<= ~3ms typ, with maximum of 100ms).  No data
                             has been copied yet in this case, and the application
                             should keep calling MDD_SDSPI_AsyncReadTasks() until either
                             an error/timeout occurs, or ASYNC_READ_NEW_PACKET_READY
                             is returned.
            ASYNC_READ_NEW_PACKET_READY -   Returned after a single packet, of
                                            the specified size (in info->numBytes),
                                            is ready to be read from the 
                                            media and copied to the user 
                                            specified data buffer.  Often, after
                                            receiving this return value, the 
                                            application firmware would want to
                                            update the info->pReceiveBuffer pointer
                                            before calling MDD_SDSPI_AsyncReadTasks()
                                            again.  This way, the application can
                                            begin fetching the next packet worth
                                            of data, while still using/consuming
                                            the previous packet of data.
            ASYNC_READ_COMPLETE - Returned when all data bytes in the read 
                                 operation have been read and returned successfully,
                                 and the media is now ready for the next operation.
            ASYNC_READ_ERROR - Returned when some failure occurs.  This could be
                               either due to a media timeout, or due to some other
                               unknown type of error.  In this case, the 
                               MDD_SDSPI_AsyncReadTasks() handler will terminate
                               the read attempt and will try to put the media 
                               back in a default state, ready for a new command.  
                               The application firmware may then retry the read
                               attempt (if desired) by re-initializing the 
                               ASYNC_IO structure and setting the 
                               bStateVariable = ASYNC_READ_QUEUED.

            
  Side Effects:
    Uses the SPI bus and the media.  The media and SPI bus should not be
    used by any other function until the read operation has either completed
    successfully, or returned with the ASYNC_READ_ERROR condition.
  Description:
    Speed optimized, non-blocking, state machine based read function that reads 
    data packets from the media, and copies them to a user specified RAM buffer.
    This function uses the multi-block read command (and stop transmission command) 
    to perform fast reads of data.  The total amount of data that will be returned 
    on any given call to MDD_SDSPI_AsyncReadTasks() will be the info->numBytes parameter.
    However, if the function is called repeatedly, with info->bytesRemaining set
    to a large number, this function can successfully fetch data sizes >> than
    the block size (theoretically anything up to ~4GB, since bytesRemaining is 
    a 32-bit DWORD).  The application firmware should continue calling 
    MDD_SDSPI_AsyncReadTasks(), until the ASYNC_READ_COMPLETE value is returned 
    (or ASYNC_READ_ERROR), even if it has already received all of the data expected.
    This is necessary, so the state machine can issue the CMD12 (STOP_TRANMISSION) 
    to terminate the multi-block read operation, once the total expected number 
    of bytes have been read.  This puts the media back into the default state 
    ready for a new command.
    
    During normal/successful operations, calls to MDD_SDSPI_AsyncReadTasks() 
    would typically return:
    1. ASYNC_READ_BUSY - repeatedly up to several milliseconds, then 
    2. ASYNC_READ_NEW_PACKET_READY - repeatedly, until 512 bytes [media read 
        block size] is received, then 
    3. Back to ASYNC_READ_BUSY (for awhile, may be short), then
    4. Back to ASYNC_READ_NEW_PACKET_READY (repeatedly, until the next 512 byte
       boundary, then back to #3, etc.
    5. After all data is received successfully, then the function will return 
       ASYNC_READ_COMPLETE, for all subsequent calls (until a new read operation
       is started, by re-initializing the ASYNC_IO structure, and re-calling
       the function).
    
  Remarks:
    This function will monopolize the SPI module during the operation.  Do not
    use the SPI module for any other purpose.

  *****************************************************************************/
#if defined USE_ATMEL_FLASH
BYTE MDD_SDSPI_AsyncReadTasks(ASYNC_IO* info)
{
    BYTE bData;
//    MMC_RESPONSE response;
    DWORD_VAL *temporary; //To make byte shifts for address bytes to send to the EEPROM
    static WORD blockCounter;
    static DWORD longTimeoutCounter;
    static BOOL SingleBlockRead;
    
    //Check what state we are in, to decide what to do.
    switch(info->bStateVariable)
    {
        case ASYNC_READ_COMPLETE:
            return ASYNC_READ_COMPLETE;
        case ASYNC_READ_QUEUED:
            //Start the read request.  
            
            //Initialize some variables we will use later.
            blockCounter = MEDIA_BLOCK_SIZE; //Counter will be used later for block boundary tracking
            ioInfo = *info; //Get local copy of structure, for quicker access with less code size

            MMD_EEPROM_Busy();

            
            if(ioInfo.dwAddress < LOGGING_SECTOR)
                ioInfo.dwAddress <<= 12;//9
            else
            {
               ioInfo.dwAddress = ioInfo.dwAddress - LOGGING_SECTOR;
               ioInfo.dwAddress <<= 9;
               ioInfo.dwAddress = ioInfo.dwAddress + LOGGING_SECTOR_1000;

            }

            if (WriteSPIM( OPCODE_READ_HF ) != 0)		//High speed array read command
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_READ_ERROR;
              return ASYNC_READ_ERROR;
            }

            temporary = (DWORD_VAL*)&ioInfo.dwAddress;	//MSB of the address
            if (WriteSPIM( temporary->v[2] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_READ_ERROR;
              return ASYNC_READ_ERROR;
            }

            if (WriteSPIM( temporary->v[1] ) != 0)		//Middle byte
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_READ_ERROR;
              return ASYNC_READ_ERROR;
            }

            if (WriteSPIM( temporary->v[0] ) != 0)		//LSB
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_READ_ERROR;
              return ASYNC_READ_ERROR;
            }

            if (WriteSPIM( 0xFF ) != 0)				//Don't care padding bits
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_READ_ERROR;
              return ASYNC_READ_ERROR;
            }
            
            
            //Don't de-assert CS here, the first bit to be read will appear at the next CLK pulse.


               info->bStateVariable =  ASYNC_READ_NEW_PACKET_READY;
               return ASYNC_READ_NEW_PACKET_READY;
       
        case ASYNC_READ_NEW_PACKET_READY:
            //We have sent the READ_MULTI_BLOCK command and have successfully
            //received the data start token byte.  Therefore, we are ready
            //to receive raw data bytes from the media.

            //We sent the "read array command"
            //Now we have to send the address


            if(ioInfo.dwBytesRemaining != 0x00000000)
            {
                //Re-update local copy of pointer and number of bytes to read in this
                //call.  These parameters are allowed to change between packets.
                ioInfo.wNumBytes = info->wNumBytes;
                ioInfo.pBuffer = info->pBuffer;
           	    


                //Now read a ioInfo.wNumBytes packet worth of SPI bytes, 
                //and place the received bytes in the user specified pBuffer.
                //This operation directly dictates data thoroughput in the 
                //application, therefore optimized code should be used for each 
                //processor type.
       
                PIC18_Optimized_SPI_Read_Packet();
                 	    //Update counters for state tracking and loop exit condition tracking.

                ioInfo.dwBytesRemaining -= ioInfo.wNumBytes;
                blockCounter -= ioInfo.wNumBytes;

                return ASYNC_READ_NEW_PACKET_READY;
            }//if(ioInfo.dwBytesRemaining != 0x00000000)
            else
            {
            
                SD_CS = 1;  //De-select media
    
                info->bStateVariable = ASYNC_READ_COMPLETE;
                return ASYNC_READ_COMPLETE;
            }

            
        case ASYNC_READ_ABORT:
            //If the application firmware wants to cancel a read request.
            info->bStateVariable = ASYNC_READ_ERROR;
        
        case ASYNC_READ_ERROR:
             SD_CS = 1;  //De-select media
        default:
            //Some error must have happened.
            SD_CS = 1;  //De-select media

            return ASYNC_READ_ERROR;

    }//switch(info->stateVariable)    
    
    //Should never get to here.  All pathways should have already returned.
    return ASYNC_READ_ERROR;
}    


#endif

/*****************************************************************************
  Function:
    static void PIC18_Optimized_SPI_Read_Packet(void)
  Summary:

  *****************************************************************************/
#if defined USE_ATMEL_FLASH
static void PIC18_Optimized_SPI_Read_Packet(void)
{
    static WORD FSR0Save;
    static WORD PRODSave;

    //Make sure the SPI_INTERRUPT_FLAG_ASM has been correctly defined, for the SPI
    //module that is actually being used in the hardware.
    #ifndef SPI_INTERRUPT_FLAG_ASM
        #error "Please define SPI_INTERRUPT_FLAG_ASM.  Double click this message for more info."
        //In the HardwareProfile - [platform name].h file for your project, please
        //add a "#define SPI_INTERRUPT_FLAG_ASM  PIRx, y" definition, where
        //PIRx is the PIR register holding the SSPxIF flag for the SPI module being used
        //to interface with the SD/MMC card, and y is the bit number for the SSPxIF bit (ex: 0-7).
    #endif

    //Make sure at least one byte needs to be read.
    if(ioInfo.wNumBytes == 0)
    {
        return;
    }

    //Context save C compiler managed registers that we will modify in this function.
    FSR0Save = FSR0;    
    PRODSave = PROD;    
    
    //Using PRODH and PRODL as convenient 16-bit access bank counter
    PROD = ioInfo.wNumBytes;    //ioInfo.wNumBytes holds the total number of bytes
                                //this function will read from SPI.
    //Going to use the FSR0 directly.  This is non-conventional, but delivers
    //better performance than using a normal C managed software RAM pointer.
    FSR0 = (WORD)ioInfo.pBuffer;

    //Initiate the first SPI operation
    WREG = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = 0xFF;

    //Highly speed efficient SPI read loop, written in inline assembly
    //language for best performance.  Total number of bytes that will be fetched
    //is exactly == the value of ioInfo.wNumBytes prior to calling this function.
    _asm
        bra     ASMSPIReadLoopEntryPoint
    
ASMSPIReadLoop:
        //Wait until last hardware SPI transaction is complete
        btfss   SPI_INTERRUPT_FLAG_ASM, 0
        bra     -2
        bcf     SPI_INTERRUPT_FLAG_ASM, 0

        //Save received byte and start the next transfer
        movf    SPIBUF, 0, 0    //Copy SPIBUF byte into WREG
        setf    SPIBUF, 0       //Write 0xFF to SPIBUF, to start a SPI transaction
        movwf   POSTINC0, 0     //Write the last received byte to the user's RAM buffer
    
ASMSPIReadLoopEntryPoint:
        //Now decrement 16-bit counter for loop exit test condition
        movlw   0x00
        decf    PRODL, 1, 0     //Decrement LSB
        subwfb  PRODH, 1, 0     //Decrement MSB, only if borrow from LSB decrement
        //Check if anymore bytes remain to be sent
        movf    PRODL, 0, 0     //copy PRODL to WREG
        iorwf   PRODH, 0, 0     //Z bit will be set if both PRODL and PRODH == 0x00
        bnz     ASMSPIReadLoop  //Go back and loop if our counter isn't = 0x0000.

        //Wait until the very last SPI transaction is complete and save the byte
        btfss   SPI_INTERRUPT_FLAG_ASM, 0
        bra     -2
        movff   SPIBUF, POSTINC0
    _endasm

    SPI_INTERRUPT_FLAG = 0;	 

    //Context restore C compiler managed registers
    PROD = PRODSave;
    FSR0 = FSR0Save;    
}    
#endif





/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_AsyncWriteTasks(ASYNC_IO* info)
  Summary:
    Speed optimized, non-blocking, state machine based write function that writes
    data from the user specified buffer, onto the media, at the specified 
    media block address.
  Pre-Conditions:
    The ASYNC_IO structure must be initialized correctly, prior to calling
    this function for the first time.  Certain parameters, such as the user
    data buffer pointer (pBuffer) in the ASYNC_IO struct are allowed to be changed
    by the application firmware, in between each call to MDD_SDSPI_AsyncWriteTasks().
    Additionally, the media and microcontroller SPI module should have already 
    been initalized before using this function.  This function is mutually
    exclusive with the MDD_SDSPI_AsyncReadTasks() function.  Only one operation
    (either one read or one write) is allowed to be in progress at a time, as
    both functions share statically allocated resources and monopolize the SPI bus.
  Input:
    ASYNC_IO* info -        A pointer to a ASYNC_IO structure.  The 
                            structure contains information about how to complete
                            the write operation (ex: number of total bytes to write,
                            where to obtain the bytes from, number of bytes
                            to write for each call to MDD_SDSPI_AsyncWriteTasks(), etc.).
  Return Values:
    BYTE - Returns a status byte indicating the current state of the write 
            operation. The possible return values are:
            
            ASYNC_WRITE_BUSY - Returned when the state machine is busy waiting for
                             the media to become ready to accept new data.  The 
                             media has write time, which can often be quite long
                             (a few ms typ, with maximum of 250ms).  The application
                             should keep calling MDD_SDSPI_AsyncWriteTasks() until either
                             an error/timeout occurs, ASYNC_WRITE_SEND_PACKET
                             is returned, or ASYNC_WRITE_COMPLETE is returned.
            ASYNC_WRITE_SEND_PACKET -   Returned when the MDD_SDSPI_AsyncWriteTasks()
                                        handler is ready to consume data and send
                                        it to the media.  After ASYNC_WRITE_SEND_PACKET
                                        is returned, the application should make certain
                                        that the info->wNumBytes and pBuffer parameters
                                        are correct, prior to calling 
                                        MDD_SDSPI_AsyncWriteTasks() again.  After
                                        the function returns, the application is
                                        then free to write new data into the pBuffer
                                        RAM location. 
            ASYNC_WRITE_COMPLETE - Returned when all data bytes in the write
                                 operation have been written to the media successfully,
                                 and the media is now ready for the next operation.
            ASYNC_WRITE_ERROR - Returned when some failure occurs.  This could be
                               either due to a media timeout, or due to some other
                               unknown type of error.  In this case, the 
                               MDD_SDSPI_AsyncWriteTasks() handler will terminate
                               the write attempt and will try to put the media 
                               back in a default state, ready for a new command.  
                               The application firmware may then retry the write
                               attempt (if desired) by re-initializing the 
                               ASYNC_IO structure and setting the 
                               bStateVariable = ASYNC_WRITE_QUEUED.

            
  Side Effects:
    Uses the SPI bus and the media.  The media and SPI bus should not be
    used by any other function until the read operation has either completed
    successfully, or returned with the ASYNC_WRITE_ERROR condition.
  Description:
    Speed optimized, non-blocking, state machine based write function that writes 
    data packets to the media, from a user specified RAM buffer.

  *****************************************************************************/
#if defined USE_ATMEL_FLASH
BYTE MDD_SDSPI_AsyncWriteTasks(ASYNC_IO* info)
{
    static BYTE data_byte;
    static WORD blockCounter;
    static DWORD WriteTimeout;
    static BYTE command;
    DWORD preEraseBlockCount;
    BYTE    Buff;
    DWORD_VAL *temporary;
    
    //Check what state we are in, to decide what to do.
    switch(info->bStateVariable)
    {
        case ASYNC_WRITE_COMPLETE:
            return ASYNC_WRITE_COMPLETE;
        case ASYNC_WRITE_QUEUED:
            //Initiate the write sequence.
            blockCounter = MEDIA_BLOCK_SIZE;    //Initialize counter.  Will be used later for block boundary tracking.

            //Copy input structure into a statically allocated global instance 
            //of the structure, for faster local access of the parameters with 
            //smaller code size.
            ioInfo = *info;

            //Check if the EEPROM is protected and write-protected
            //Send the write command
            //And the start address
            //Check if we wrote more than the EEPROM's sector size
 

            if(ioInfo.dwAddress < LOGGING_SECTOR)
            {
                ioInfo.dwAddress <<= 12;//9
                if( !(EEPROM_Erase4K((DWORD_VAL*)&ioInfo.dwAddress) ))
                {
                   SD_CS = 1;
                  info->bStateVariable = ASYNC_WRITE_ERROR;
                  return ASYNC_WRITE_ERROR;
                }
            }
            else
            {
                if(WriteEnable)
                {
                ioInfo.dwAddress = ioInfo.dwAddress - LOGGING_SECTOR;
                ioInfo.dwAddress <<= 9;
                ioInfo.dwAddress = ioInfo.dwAddress + LOGGING_SECTOR_1000;
                }
                else
                {
                                  info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
                }

            }

               if(ioInfo.dwBytesRemaining == 0)
                {
                    info->bStateVariable = ASYNC_WRITE_COMPLETE;
                    SD_CS = 1;
                    return ASYNC_WRITE_COMPLETE;
                }
 
 
           MMD_EEPROM_Busy();
                if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

           Buff = MDD_SDSPI_ReadMedia();

           if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  | !(Buff&EEPROM_WRITE_ENABLED) | (Buff&EEPROM_BUSY) | (Buff&0b00001000) | (Buff&0b00000100))
           {
            
             Nop();
             if (WriteSPIM( OPCODE_WRSR  ) != 0)
                {
                SD_CS = 1;
                }
             if (WriteSPIM( 0x00 ) != 0)
                {
                SD_CS = 1;
                }
             SD_CS = 1;
           }

           //Here, we know the EEPROM is un-protected and ready to receive commands
           if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
           {
             SD_CS = 1;
             Nop();
             Nop();
             Nop();
          
            if (WriteSPIM( OPCODE_WREN) != 0)
                {
                SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
                }
           }
             SD_CS = 1;
             Nop();
        
            if (WriteSPIM( OPCODE_WRITE  ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            temporary = (DWORD_VAL*)&ioInfo.dwAddress;
            if (WriteSPIM( temporary->v[2] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            if (WriteSPIM( temporary->v[1] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            if (WriteSPIM( temporary->v[0] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }
            info->bStateVariable = ASYNC_WRITE_TRANSMIT_FIRST_PACKET;
            return ASYNC_WRITE_TRANSMIT_FIRST_PACKET;

        case ASYNC_WRITE_TRANSMIT_FIRST_PACKET:

            //Now send a packet of raw data bytes to the media, over SPI.
            //This code directly impacts data thoroughput in a significant way.
            //Special care should be used to make sure this code is speed optimized.

                PIC18_Optimized_SPI_Write_Packet();

               
 
            //Keep track of variables for loop/state exit conditions.
            ioInfo.dwBytesRemaining -= ioInfo.wNumBytes;
            blockCounter -= ioInfo.wNumBytes;
            info->dwBytesRemaining = ioInfo.dwBytesRemaining;
            info->dwAddress = ioInfo.dwAddress;
            info->wNumBytes = ioInfo.wNumBytes;
            //If we finished to write all the data, exit. If not, start again at the EEPROM command and address "case"
            if(ioInfo.dwBytesRemaining == 0)
            {
                info->bStateVariable = ASYNC_WRITE_COMPLETE;
                SD_CS = 1;
                return ASYNC_WRITE_COMPLETE;
            }
            else
            {
                SD_CS = 1;
                info->bStateVariable = ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
                return ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
            }



            
        case ASYNC_WRITE_TRANSMIT_SECOND_PACKET:
           if(ioInfo.dwBytesRemaining == 0)
            {
                info->bStateVariable = ASYNC_WRITE_COMPLETE;
                SD_CS = 1;
                return ASYNC_WRITE_COMPLETE;
            }
            MMD_EEPROM_Busy();
            if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

           Buff = MDD_SDSPI_ReadMedia();

           if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  || (Buff&EEPROM_BUSY) || (Buff&0b00001000) || (Buff&0b00000100))
           {
            SD_CS = 1;
             if (WriteSPIM( OPCODE_WRSR  ) != 0)
                {
                SD_CS = 1;
                }
             if (WriteSPIM( 0x00 ) != 0)
                {
                SD_CS = 1;
                }
             SD_CS = 1;

             if (WriteSPIM( OPCODE_WREN) != 0)
                {
                SD_CS = 1;
                return 0;
                }
             SD_CS = 1;



            info->bStateVariable = ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
            return ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
           }

           //Here, we know the EEPROM is un-protected and ready to receive commands
           if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
           {
             SD_CS = 1;
             Nop();
        
            if (WriteSPIM( OPCODE_WREN) != 0)
                {
                SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
                }
            SD_CS = 1;
            info->bStateVariable = ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
            return ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
           }
             SD_CS = 1;
             Nop();
             Nop();
       
            if (WriteSPIM( OPCODE_WRITE  ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            temporary = (DWORD_VAL*)&ioInfo.dwAddress;
            if (WriteSPIM( temporary->v[2] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            if (WriteSPIM( temporary->v[1] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }

            if (WriteSPIM( temporary->v[0] ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }


            //Now send a packet of raw data bytes to the media, over SPI.
            //This code directly impacts data thoroughput in a significant way.
            //Special care should be used to make sure this code is speed optimized.

                PIC18_Optimized_SPI_Write_Packet();

            //Keep track of variables for loop/state exit conditions.
            ioInfo.dwBytesRemaining -= ioInfo.wNumBytes;
            blockCounter -= ioInfo.wNumBytes;
            info->dwBytesRemaining = ioInfo.dwBytesRemaining;
            info->dwAddress = ioInfo.dwAddress;
            info->wNumBytes = ioInfo.wNumBytes;
            //If we finished to write all the data, exit. If not, start again at the EEPROM command and address "case"
            if(ioInfo.dwBytesRemaining == 0)
            {
                info->bStateVariable = ASYNC_WRITE_COMPLETE;
                SD_CS = 1;
                return ASYNC_WRITE_COMPLETE;

            }
            else
            {
                SD_CS = 1;
                info->bStateVariable = ASYNC_WRITE_TRANSMIT_SECOND_PACKET;
                return ASYNC_WRITE_TRANSMIT_SECOND_PACKET;

            }

        case ASYNC_WRITE_MEDIA_BUSY:

            if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
            {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_ERROR;
              return ASYNC_WRITE_ERROR;
            }
           Buff = MDD_SDSPI_ReadMedia();

           if (Buff&EEPROM_BUSY)
           {
               SD_CS = 1;
              info->bStateVariable = ASYNC_WRITE_QUEUED; //EPROM Busy. Exit the function, but without error (could be writing a previous page)
              return ASYNC_WRITE_QUEUED;

           }
    
        
        case ASYNC_STOP_TOKEN_SENT_WAIT_BUSY:
            break;
       
        case ASYNC_WRITE_ABORT:
      
            info->bStateVariable = ASYNC_WRITE_ERROR; 
            //Fall through to default case.
        default:
            //Used for ASYNC_WRITE_ERROR case.
            return ASYNC_WRITE_ERROR; 
    }//switch(info->stateVariable)    
    

    //Should never execute to here.  All pathways should have a hit a return already.
    info->bStateVariable = ASYNC_WRITE_ABORT;
    return ASYNC_WRITE_BUSY;
} 

#endif
  
/*****************************************************************************
  Function:
    static void PIC18_Optimized_SPI_Write_Packet(void)
  Summary:
    A private function intended for use internal to the SD-SPI.c file.
    This function writes a specified number of bytes to the SPI module,
    at high speed for optimum throughput, copied from the user specified RAM
    buffer.
    This function is only implemented and used on PIC18 devices.
  Pre-Conditions:
    The ioInfo.wNumBytes must be pre-initialized prior to calling 
    PIC18_Optimized_SPI_Write_Packet().
    Additionally, the ioInfo.pBuffer must also be pre-initialized, prior
    to calling PIC18_Optimized_SPI_Write_Packet().
  Input:
    ioInfo.wNumBytes global variable, initialized to the number of bytes to send
    ioInfo.pBuffer global variable, initialized to point to the RAM location that
        contains the data to send out the SPI port
  Return Values:
    None
  Side Effects:
    None
  Description:
    A private function intended for use internal to the SD-SPI.c file.
    This function writes a specified number of bytes to the SPI module,
    at high speed for optimum throughput, copied from the user specified RAM
    buffer.
    This function is only implemented and used on PIC18 devices.
  Remarks:
    This function is speed optimized, using inline assembly language code, and
    makes use of C compiler managed resources.  It is currently written to work
    with the Microchip MPLAB C18 compiler, and may need modification if built
    with a different PIC18 compiler.
  *****************************************************************************/
#if defined USE_ATMEL_FLASH
static void PIC18_Optimized_SPI_Write_Packet(void)
{
    static BYTE bData;
    static WORD FSR0Save;
    static WORD PRODSave;

    //Make sure the SPI_INTERRUPT_FLAG_ASM has been correctly defined, for the SPI
    //module that is actually being used in the hardware.
    #ifndef SPI_INTERRUPT_FLAG_ASM
        #error Please add "#define SPI_INTERRUPT_FLAG_ASM  PIRx, Y" to your hardware profile.  Replace x and Y with appropriate numbers for your SPI module interrupt flag.
    #endif
    
    //Make sure at least one byte needs copying.
    if(ioInfo.wNumBytes == 0)
    {
        ioInfo.dwBytesRemaining = 0;
        return;
    }    

    //Context save C compiler managed registers.
    FSR0Save = FSR0; 
    PRODSave = PROD;
    //Using PRODH and PRODL as 16 bit loop counter.  These are convenient since
    //they are always in the access bank.
          if(ioInfo.wNumBytes > 0x100)  //That's the first part of 512bytes
            {
            PROD = 0x100;
            //ioInfo.wNumBytes = ioInfo.wNumBytes - 0x100;
            ioInfo.dwAddress = ioInfo.dwAddress + 0x100;
            FSR0 = (WORD)ioInfo.pBuffer;
            ioInfo.wNumBytes = ioInfo.wNumBytes - 0x100;
            }
          else        //The second part of the 512 bytes
            {
            PROD = ioInfo.wNumBytes;
            FSR0 = (WORD)ioInfo.pBuffer + 0x100;
             }
 
                  
    _asm
        movf    POSTINC0, 0, 0  //Fetch next byte to send and store in WREG
        bra     ASMSPIXmitLoopEntryPoint
ASMSPIXmitLoop:    
        movf    POSTINC0, 0, 0  //Pre-Fetch next byte to send and temporarily store in WREG
        //Wait until last hardware SPI transaction is complete
        btfss   SPI_INTERRUPT_FLAG_ASM, 0
        bra     -2
        
ASMSPIXmitLoopEntryPoint:
        //Start the next SPI transaction
        bcf     SPI_INTERRUPT_FLAG_ASM, 0   //Clear interrupt flag
        movwf   SPIBUF, 0       //Write next byte to transmit to SSPBUF
        
        //Now decrement byte counter for loop exit condition
        movlw   0x00
        decf    PRODL, 1, 0     //Decrement LSB
        subwfb  PRODH, 1, 0     //Decrement MSB, only if borrow from LSB decrement
        //Check if anymore bytes remain to be sent
        movf    PRODL, 0, 0     //copy PRODL to WREG
        iorwf   PRODH, 0, 0     //Z bit will be set if both PRODL and PRODH == 0x00
        bnz     ASMSPIXmitLoop  //Go back and loop if our counter isn't = 0x0000.
    _endasm

    //Wait until the last SPI transaction is really complete.  
    //Above loop jumps out after the last byte is started, but not finished yet.
    while(!SPI_INTERRUPT_FLAG);

    //Leave SPI module in a "clean" state, ready for next transaction.
    bData = SPIBUF;         //Dummy read to clear BF flag.
    SPI_INTERRUPT_FLAG = 0; //Clear interrupt flag.
    SD_CS = 1;
    //Restore C compiler managed registers that we modified
    PROD = PRODSave;
    FSR0 = FSR0Save;
    ioInfo.wNumBytes = ioInfo.wNumBytes;// - 0x100;
}
#endif
/*
#if defined USE_SST_FLASH
void PIC18_Optimized_SPI_Write_Packet(void)
{
    BYTE clear;
    WORD ptrBuff;
    //static WORD FSR0Save;
    //static WORD PRODSave;

    //Make sure the SPI_INTERRUPT_FLAG_ASM has been correctly defined, for the SPI
    //module that is actually being used in the hardware.
    #ifndef SPI_INTERRUPT_FLAG_ASM
        #error Please add "#define SPI_INTERRUPT_FLAG_ASM  PIRx, Y" to your hardware profile.  Replace x and Y with appropriate numbers for your SPI module interrupt flag.
    #endif

    //Make sure at least one byte needs copying.
    if(ioInfo.wNumBytes == 0)
    {
        ioInfo.dwBytesRemaining = 0;
        return;
    }

    //Context save C compiler managed registers.
//    FSR0Save = FSR0;
//    PRODSave = PROD;
    //Using PRODH and PRODL as 16 bit loop counter.  These are convenient since
    //they are always in the access bank.
    

//Write the first two bytes
    ptrBuff = 0;
    SD_CS = 0;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = (BYTE)ioInfo.pBuffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = ioInfo.pBuffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    SD_CS = 1; //Validate the written data
    Nop();
    Nop();

    while(ptrBuff < 0x100)
    {
    SD_CS = 0;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = AAI_WRITE;
    while (!SPI_INTERRUPT_FLAG);

    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = ioInfo.pBuffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);

    ptrBuff++;

    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = ioInfo.pBuffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    SD_CS = 1; //Validate the written data
    Nop();
    Nop();
    }

 ioInfo.wNumBytes = ioInfo.wNumBytes  - ptrBuff;

}



#endif
   

*/



/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_SectorWrite (DWORD sector_addr, BYTE * buffer, BYTE allowWriteToZero)
  Summary:
    Writes a sector of data to an SD card.
  Conditions:
    The MDD_SectorWrite function pointer must be pointing to this function.
  Input:
    sector_addr -      The address of the sector on the card.
    buffer -           The buffer with the data to write.
    allowWriteToZero -
                     - TRUE -  Writes to the 0 sector (MBR) are allowed
                     - FALSE - Any write to the 0 sector will fail.
  Return Values:
    TRUE -  The sector was written successfully.
    FALSE - The sector could not be written.
  Side Effects:
    None.
  Description:
    The MDD_SDSPI_SectorWrite function writes one sector of data (512 bytes) 
    of data from the location pointed to by 'buffer' to the specified sector of 
    the SD card.
  Remarks:
    The card expects the address field in the command packet to be a byte address.
    The sector_addr value is ocnverted to a byte address by shifting it left nine
    times (multiplying by 512).
  ***************************************************************************************/
#if defined USE_ATMEL_FLASH
BYTE MDD_SDSPI_SectorWrite(DWORD sector_addr, BYTE* buffer, BYTE allowWriteToZero)
{
    static ASYNC_IO info;
    BYTE status;
    
    if(allowWriteToZero == FALSE)
    {
        if(sector_addr == 0x00000000)
        {
            return FALSE;
        }    
    }

    //Initialize structure so we write a single sector worth of data.
    info.wNumBytes = 512;
    info.dwBytesRemaining = 512;
    info.pBuffer = buffer;
    info.dwAddress = sector_addr;
    info.bStateVariable = ASYNC_WRITE_QUEUED;
    
    //Repeatedly call the write handler until the operation is complete (or a
    //failure/timeout occurred).
    while(1)
    {
        status = MDD_SDSPI_AsyncWriteTasks(&info);
        if(status == ASYNC_WRITE_COMPLETE)
        {
           // MDD_SDSPI_SectorRead( sector_addr, buffer);
            return TRUE;
        }    
        else if(status == ASYNC_WRITE_ERROR)
        {
            return FALSE;
        }
    }    
    return TRUE;
}    


#endif

#if defined USE_SST_FLASH
BYTE MDD_SDSPI_SectorWrite(DWORD sector_addr, BYTE* buffer, BYTE allowWriteToZero)
{
  //  static ASYNC_IO info;
    BYTE status,Buff, clear;
    WORD ptrBuff;
    DWORD_VAL temporary;


    if(allowWriteToZero == FALSE)
    {
        if(sector_addr == 0x00000000)
        {
            return FALSE;
        }
    }

    if(sector_addr < LOGGING_SECTOR)
    {
        sector_addr <<= 12;//9
        if( !(EEPROM_Erase4K(sector_addr) ))
        {
           SD_CS = 1;
           return FALSE;
        }
    }
    else
    {

        sector_addr = sector_addr - LOGGING_SECTOR;
        sector_addr <<= 9;
        sector_addr = sector_addr + LOGGING_SECTOR_1000;


    }



   MMD_EEPROM_Busy();
   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

   Buff = MDD_SDSPI_ReadMedia();

   if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  | !(Buff&EEPROM_WRITE_ENABLED) | (Buff&EEPROM_BUSY) | (Buff&0b00001000) | (Buff&0b00000100))
   {

     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        }
     SD_CS = 1;
   }

   //Here, we know the EEPROM is un-protected and ready to receive commands
 //  if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   //{
     SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
          SD_CS = 1;
          return FALSE;
        }
   //}
     SD_CS = 1;
     Nop();

    if (WriteSPIM( AAI_WRITE  ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    temporary.Val = sector_addr;
    if (WriteSPIM( temporary.v[2] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[1] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[0] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }


    //We wrote the address to write. Write the first bytes and continue the AAI process
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    ptrBuff = 0;
   // SD_CS = 0;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = (BYTE)buffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = (BYTE)buffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    Nop();
    SD_CS = 1; //Validate the written data

    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();

    MMD_EEPROM_Busy();


    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    //We have to resend the AAI command and the two next bytes. And so on.
    while(ptrBuff < 0x200)
    {
        SD_CS = 0;
        clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = AAI_WRITE;
        while (!SPI_INTERRUPT_FLAG);

    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
        clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = (BYTE)buffer[ptrBuff];
        while (!SPI_INTERRUPT_FLAG);

        ptrBuff++;

        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
           clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = (BYTE)buffer[ptrBuff];
        while (!SPI_INTERRUPT_FLAG);
        ptrBuff++;
        SD_CS = 1; //Validate the written data
        Nop();
        Nop();
        MMD_EEPROM_Busy();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    }

    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    
    //Write-protect the EEPROM to finish the write.
    MMD_EEPROM_Busy();
    WriteSPIM(OPCODE_WRDI );
    SD_CS = 1;
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    MMD_EEPROM_Busy();
    SD_CS = 1;
    
    return TRUE;
}


#endif
/*******************************************************************************
  Function:
    BYTE MDD_SDSPI_WriteProtectState
  Summary:
    Indicates whether the card is write-protected.
  Conditions:
    The MDD_WriteProtectState function pointer must be pointing to this function.
  Input:
    None.
  Return Values:
    TRUE -  The card is write-protected
    FALSE - The card is not write-protected
  Side Effects:
    None.
  Description:
    The MDD_SDSPI_WriteProtectState function will determine if the SD card is
    write protected by checking the electrical signal that corresponds to the
    physical write-protect switch.
  Remarks:
    None
*******************************************************************************/

BYTE MDD_SDSPI_WriteProtectState(void)
{
    if(WriteEnable == TRUE)
        return(FALSE);
    else
        return(FALSE);//As we're managing the write enable inside the write functions
    //always report the drive is write enabled.
}


/*******************************************************************************
  Function:
    void Delayms (BYTE milliseconds)
  Summary:
    Delay.
  Conditions:
    None.
  Input:
    BYTE milliseconds - Number of ms to delay
  Return:
    None.
  Side Effects:
    None.
  Description:
    The Delayms function will delay a specified number of milliseconds.  Used for SPI
    timing.
  Remarks:
    Depending on compiler revisions, this function may not delay for the exact 
    time specified.  This shouldn't create a significant problem.
*******************************************************************************/

void Delayms(BYTE milliseconds)
{
    BYTE    ms;
    DWORD   count;
    
    ms = milliseconds;
    while (ms--)
    {
        count = MILLISECDELAY;
        while (count--);
    }
    Nop();
    return;
}


/*******************************************************************************
  Function:
    void CloseSPIM (void)
  Summary:
    Disables the SPI module.
  Conditions:
    None.
  Input:
    None.
  Return:
    None.
  Side Effects:
    None.
  Description:
    Disables the SPI module.
  Remarks:
    None.
*******************************************************************************/

void CloseSPIM (void)
{
    SD_CS  = 1;
    SD_CS_TRIS  = 1;	// define /SS pin direction as O/p

    SPICLOCKLAT  = 1;	// setting/clearing the output latch depending upon Clock Idle state (i.e high/low)
    SPICLOCKTRIS  = 1;	// define SCL direction as O/p

    SPIOUTLAT  = 1;
    SPIOUTTRIS = 1;	// define SDO direction as O/p

    SPIINTRIS = 1;	// define SDI direction as I/p

    SPICON1 &= 0xDF;

}



/*****************************************************************************
  Function:
    unsigned char WriteSPIM (unsigned char data_out)
  Summary:
    Writes data to the SD card.
  Conditions:
    None.
  Input:
    data_out - The data to write.
  Return:
    0.
  Side Effects:
    None.
  Description:
    The WriteSPIM function will write a byte of data from the microcontroller to the
    SD card.
  Remarks:
    None.
  ***************************************************************************************/

unsigned char WriteSPIM( unsigned char data_out )
{
    BYTE clear;
    
    SD_CS = 0;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = data_out;
    if (SPICON1 & 0x80)
        return -1;
    else
        while (!SPI_INTERRUPT_FLAG);
    return 0;
}



/*****************************************************************************
  Function:
    BYTE MDD_SDSPI_ReadMedia (void)
  Summary:
    Reads a byte of data from the SD card.
  Conditions:
    None.
  Input:
    None.
  Return:
    The byte read.
  Side Effects:
    None.
  Description:
    The MDD_SDSPI_ReadMedia function will read one byte from the SPI port.
  Remarks:
    This function replaces ReadSPI, since some implementations of that function
    will initialize SSPBUF/SPIBUF to 0x00 when reading.  The card expects 0xFF.
  ***************************************************************************************/
BYTE MDD_SDSPI_ReadMedia(void)
{
    BYTE clear;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = 0xFF;
    while (!SPI_INTERRUPT_FLAG);
    return SPIBUF;
}

/*****************************************************************************
  Function:
    void OpenSPIM (unsigned int sync_mode)
  Summary:
    Initializes the SPI module
  Conditions:
    None.
  Input:
    sync_mode - Input parameter that sets the SPI mode/speed.
  Return:
    None.
  Side Effects:
    None.
  Description:
    The OpenSPIM function will enable and configure the SPI module.
  Remarks:
    None.
  ***************************************************************************************/

void OpenSPIM (unsigned char sync_mode)

{
 
    SD_CS  = 1;
    SD_CS_TRIS  = 0;	// define /SS pin direction as O/p

    SPICLOCKLAT  = 1;	// setting/clearing the output latch depending upon Clock Idle state (i.e high/low)
    SPICLOCKTRIS  = 0;	// define SCL direction as O/p

    SPIOUTLAT  = 1;
    SPIOUTTRIS = 0;	// define SDO direction as O/p

    SPIINTRIS = 1;	// define SDI direction as I/p
    ENABLE_MSSP();
}


#ifdef __18CXX
// Description: Delay value for the manual SPI clock
#define MANUAL_SPI_CLOCK_VALUE             1


/*****************************************************************************
  Function:
    MEDIA_INFORMATION *  MDD_SDSPI_MediaInitialize (void)
  Summary:
 
********************************************************************************/

MEDIA_INFORMATION *  MDD_SDSPI_MediaInitialize(void)
{
    //Yeah, whatever dude, I'm an EEPROM.
    mediaInformation.validityFlags.bits.sectorSize = TRUE;
    mediaInformation.sectorSize = MEDIA_SECTOR_SIZE;

    mediaInformation.errorCode = MEDIA_NO_ERROR;
    return &mediaInformation;

}//end MediaInitialize







#if defined USE_ATMEL_FLASH
BYTE MDD_SDSPI_WriteLowPower(DWORD Address, BYTE* buffer)
{

    BYTE    Buff;
    DWORD_VAL temporary;
    WORD AddressBuffData; //Pointer to the data in the RAM we want to write to the EEPROM (containing the logs)
    DWORD TempAddress;      //Address within the buffer, so we write its second part in the second write to the EEPROM.


    MDD_SDSPI_InitIO();

   
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 1;

    //Check what state we are in, to decide what to do.
    TempAddress = 0x100;
    AddressBuffData = (WORD)buffer;
    if(Address < LOGGING_SECTOR)
    {
        Address <<= 12;
    }
    else
    {
        Address = Address - LOGGING_SECTOR;
        Address <<= 9;
        Address = Address + LOGGING_SECTOR_1000;
    }


    MMD_EEPROM_Busy();

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
      return 0;
    }

    Buff = MDD_SDSPI_ReadMedia();

    if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  | !(Buff&EEPROM_WRITE_ENABLED) | (Buff&EEPROM_BUSY) | (Buff&0b00001000) | (Buff&0b00000100))
    {

     Nop();
     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        }
     SD_CS = 1;
    }

    //Here, we know the EEPROM is un-protected and ready to receive commands
    if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
    {
     SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
    }
     SD_CS = 1;

    if (WriteSPIM( OPCODE_WRITE  ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    temporary.Val = Address;

    if (WriteSPIM( (BYTE)(temporary.v[2]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    if (WriteSPIM( (BYTE)(temporary.v[1]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    if (WriteSPIM( (BYTE)(temporary.v[0]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }


  
    PIC18_Optimized_SPI_Low_Power_Write_Packet(TempAddress,AddressBuffData);

    
    Address += 0x100;
    AddressBuffData += 0x100;
    MMD_EEPROM_Busy();

    if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    Buff = MDD_SDSPI_ReadMedia();

    if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  || (Buff&EEPROM_BUSY) || (Buff&0b00001000) || (Buff&0b00000100))
    {
     SD_CS = 1;
     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        }
     SD_CS = 1;

     if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
     SD_CS = 1;

    }

    //Here, we know the EEPROM is un-protected and ready to receive commands
    if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
    {
     SD_CS = 1;
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
        SD_CS = 1;
        return 0;
        }
    SD_CS = 1;

    }

    SD_CS = 1;

    if (WriteSPIM( OPCODE_WRITE  ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    temporary.Val = Address;

    if (WriteSPIM( (BYTE)(temporary.v[2]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    if (WriteSPIM( (BYTE)(temporary.v[1]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }

    if (WriteSPIM( (BYTE)(temporary.v[0]) ) != 0)
    {
       SD_CS = 1;
       return 0;
    }


        PIC18_Optimized_SPI_Low_Power_Write_Packet(TempAddress,AddressBuffData);

    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 0;
}

static void PIC18_Optimized_SPI_Low_Power_Write_Packet(WORD Address,DWORD Addressbuffer)
{
    static BYTE bData;
    static WORD FSR0Save;
    static WORD PRODSave;
    //Context save C compiler managed registers.
    FSR0Save = FSR0;
    PRODSave = PROD;
    //Using PRODH and PRODL as 16 bit loop counter.  These are convenient since
    //they are always in the access bank.

    PROD = Address;
    //ioInfo.wNumBytes = ioInfo.wNumBytes - 0x100;
    //Address += 0x100;
    FSR0 = (WORD)Addressbuffer;
    //Addressbuffer += 0x100;

    _asm
        movf    POSTINC0, 0, 0  //Fetch next byte to send and store in WREG
        bra     ASMSPIXmitLoopEntryPoint
ASMSPIXmitLoop:
        movf    POSTINC0, 0, 0  //Pre-Fetch next byte to send and temporarily store in WREG
        //Wait until last hardware SPI transaction is complete
        btfss   SPI_INTERRUPT_FLAG_ASM, 0
        bra     -2

ASMSPIXmitLoopEntryPoint:
        //Start the next SPI transaction
        bcf     SPI_INTERRUPT_FLAG_ASM, 0   //Clear interrupt flag
        movwf   SPIBUF, 0       //Write next byte to transmit to SSPBUF

        //Now decrement byte counter for loop exit condition
        movlw   0x00
        decf    PRODL, 1, 0     //Decrement LSB
        subwfb  PRODH, 1, 0     //Decrement MSB, only if borrow from LSB decrement
        //Check if anymore bytes remain to be sent
        movf    PRODL, 0, 0     //copy PRODL to WREG
        iorwf   PRODH, 0, 0     //Z bit will be set if both PRODL and PRODH == 0x00
        bnz     ASMSPIXmitLoop  //Go back and loop if our counter isn't = 0x0000.
    _endasm

    //Wait until the last SPI transaction is really complete.
    //Above loop jumps out after the last byte is started, but not finished yet.
    while(!SPI_INTERRUPT_FLAG);

    //Leave SPI module in a "clean" state, ready for next transaction.
    bData = SPIBUF;         //Dummy read to clear BF flag.
    SPI_INTERRUPT_FLAG = 0; //Clear interrupt flag.
    SD_CS = 1;
    //Restore C compiler managed registers that we modified
    PROD = PRODSave;
    FSR0 = FSR0Save;

}

#endif

#if defined USE_SST_FLASH

BYTE MDD_SDSPI_WriteLowPower(DWORD Address, BYTE* buffer)
{
  BYTE status,Buff, clear;
    WORD ptrBuff;
    DWORD_VAL temporary;
    //BYTE    Buff;
    //DWORD_VAL temporary;
    //WORD AddressBuffData; //Pointer to the data in the RAM we want to write to the EEPROM (containing the logs)
    //DWORD TempAddress;      //Address within the buffer, so we write its second part in the second write to the EEPROM.


    MDD_SDSPI_InitIO();


    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 1;


    if(Address < LOGGING_SECTOR)
    {
        Address <<= 12;
    }
    else
    {
        Address = Address - LOGGING_SECTOR;
        Address <<= 9;
        Address = Address + LOGGING_SECTOR_1000;
    }


   MMD_EEPROM_Busy();
   if (WriteSPIM( READ_STATUS_REGISTER ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

   Buff = MDD_SDSPI_ReadMedia();

   if ( (Buff&EEPROM_SECTOR_PROT_REG_LOCKED)  | !(Buff&EEPROM_WRITE_ENABLED) | (Buff&EEPROM_BUSY) | (Buff&0b00001000) | (Buff&0b00000100))
   {

     if (WriteSPIM( OPCODE_WRSR  ) != 0)
        {
        SD_CS = 1;
        }
     if (WriteSPIM( 0x00 ) != 0)
        {
        SD_CS = 1;
        }
     SD_CS = 1;
   }

   //Here, we know the EEPROM is un-protected and ready to receive commands
 //  if (!(Buff & EEPROM_WRITE_ENABLED ) )//Test if write to EEPROM is enabled
   //{
     SD_CS = 1;
     Nop();
     Nop();
     Nop();

    if (WriteSPIM( OPCODE_WREN) != 0)
        {
          SD_CS = 1;
          return FALSE;
        }
   //}
     SD_CS = 1;
     Nop();

    if (WriteSPIM( AAI_WRITE  ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    temporary.Val = Address ;
    if (WriteSPIM( temporary.v[2] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[1] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }

    if (WriteSPIM( temporary.v[0] ) != 0)
    {
       SD_CS = 1;
      return FALSE;
    }


    //We wrote the address to write. Write the first bytes and continue the AAI process

    ptrBuff = 0;
   // SD_CS = 0;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = (BYTE)buffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    clear = SPIBUF;
    SPI_INTERRUPT_FLAG = 0;
    SPIBUF = (BYTE)buffer[ptrBuff];
    while (!SPI_INTERRUPT_FLAG);
    ptrBuff++;
    SD_CS = 1; //Validate the written data
    Nop();
    Nop();
    MMD_EEPROM_Busy();


   Nop();
    //We have to resend the AAI command and the two next bytes. And so on.
    while(ptrBuff < 0x200)
    {
        SD_CS = 0;
        clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = AAI_WRITE;
        while (!SPI_INTERRUPT_FLAG);


        clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = (BYTE)buffer[ptrBuff];
        while (!SPI_INTERRUPT_FLAG);

        ptrBuff++;

        clear = SPIBUF;
        SPI_INTERRUPT_FLAG = 0;
        SPIBUF = (BYTE)buffer[ptrBuff];
        while (!SPI_INTERRUPT_FLAG);
        ptrBuff++;
        SD_CS = 1; //Validate the written data
        MMD_EEPROM_Busy();
    }

    Nop();
    //Write-protect the EEPROM to finish the write.
    MMD_EEPROM_Busy();
    WriteSPIM(OPCODE_WRDI );
    SD_CS = 1;


    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 0;
}


#endif
