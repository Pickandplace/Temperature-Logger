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
 * FileName:        FSconfig.h
 * Dependencies:    None
 * Processor:       PIC18/PIC24/dsPIC30/dsPIC33
 * Compiler:        C18/C30
 * Company:         Microchip Technology, Inc.
 * Version:         1.0.0
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the ÔøΩCompanyÔøΩ) for its PICmicroÔøΩ Microcontroller is intended and
 * supplied to you, the CompanyÔøΩs customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN ÔøΩAS ISÔøΩ CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*****************************************************************************/
/********************************************************************
 *The code added to this file belongs to Jean Wloradski and should not
 *be distribued, copied, modified or used without his authorization.
 *All right reserved. For private use only
********************************************************************/

#ifndef _FS_DEF_

#include "HardwareProfile.h"

/***************************************************************************/
/*   Note:  There are likely pin definitions present in the header file    */
/*          for your device (SP-SPI.h, CF-PMP.h, etc).  You may wish to    */
/*          specify these as well                                          */
/***************************************************************************/

// The FS_MAX_FILES_OPEN #define is only applicable when Dynamic
// memeory allocation is not used (FS_DYNAMIC_MEM not defined).
// Defines how many concurent open files can exist at the same time.
// Takes up static memory. If you do not need to open more than one
// file at the same time, then you should set this to 1 to reduce
// memory usage
#define FS_MAX_FILES_OPEN 	2
/************************************************************************/

// The size of a sector
// Must be 512, 1024, 2048, or 4096
// 512 bytes is the value used by most cards
#define MEDIA_SECTOR_SIZE 		512
/************************************************************************/

//#define FILES_ADDRESS 0x2000
 //#define MDD_INTERNAL_FLASH_DRIVE_CAPACITY 14
#define MEDIA_CAPACITY  		0x0670    //MINIMUM=28
// Calculation: (0x100000 - 0x2F000)/0x200
// (Total EEPROM size - Start of the Logging zone)/Sector size.
#define PARTITION_SIZE (DWORD)(MEDIA_CAPACITY  - 1)

//#define MDD_INTERNAL_FLASH_PARTITION_SIZE = MEDIA_CAPACITY - 0x0001

#define MDD_INTERNAL_FLASH_MAX_NUM_FILES_IN_ROOT 0x03
#define DIRECTORY_LOCATION      0x0C//0C
#define FAT_LOCATION            0x02
#define FIRST_DATA_SECTOR       0x2C//2A
#define LOGGING_SECTOR          0x2F    //2DHere, we start to manage the disk ourselves: Any write by the OS will fail.
#define LOGGING_SECTOR_1000     0x2F000 //The sectors are now normally grouped (0x200 instead of the 4KB of erase zone)
#define RESERVED_SECTORS        0x01
#define SECTORS_PER_FAT         0x05 //04
#define NUMBER_OF_FATS          0x02
#define HIDDEN_SECTORS          0x01


// Comment this line out if you don't intend to write data to the card
#define ALLOW_WRITES
/************************************************************************/

// Comment this line out if you don't intend to format your card
// Writes must be enabled to use the format function
//#define ALLOW_FORMATS
/************************************************************************/

// Uncomment this definition if you're using directories
// Writes must be enabled to use directories
//#define ALLOW_DIRS
/************************************************************************/

// Allows the use of FSfopenpgm, FSremovepgm, etc with PIC18
#if defined(__18CXX)
 //   #define ALLOW_PGMFUNCTIONS
#endif
/************************************************************************/

// Allows the use of the FSfprintf function
// Writes must be enabled to use the FSprintf function
//#define ALLOW_FSFPRINTF
/************************************************************************/

// If FAT32 support required then uncomment the following
//#define SUPPORT_FAT32
/* ******************************************************************************************************* */



// Select how you want the timestamps to be updated
// Use the Real-time clock peripheral to set the clock
// You must configure the RTC in your application code
//#define USEREALTIMECLOCK
// The user will update the timing variables manually using the SetClockVars function
// The user should set the clock before they create a file or directory (Create time),
// and before they close a file (last access time, last modified time)
//#define USERDEFINEDCLOCK
// Just increment the time- this will not produce accurate times and dates
#define INCREMENTTIMESTAMP


#ifdef USE_PIC18
	#ifdef USEREALTIMECLOCK
		#error The PIC18 architecture does not have a Real-time clock and calander module
	#endif
#endif

#ifdef ALLOW_PGMFUNCTIONS
	#ifndef USE_PIC18
		#error The pgm functions are unneccessary when not using PIC18
	#endif
#endif

#ifndef USEREALTIMECLOCK
    #ifndef USERDEFINEDCLOCK
        #ifndef INCREMENTTIMESTAMP
            #error Please enable USEREALTIMECLOCK, USERDEFINEDCLOCK, or INCREMENTTIMESTAMP
        #endif
    #endif
#endif




#ifdef __18CXX
    /* Define the locations for the dataBuffer and FATbuffer; PLEASE CHECK THE LINKER FILE */
    #define DATA_BUFFER_ADDRESS      0x800
    #define FAT_BUFFER_ADDRESS       0xA00
#endif


// Function definitions
// Associate the physical layer functions with the correct physical layer
#ifdef USE_SD_INTERFACE_WITH_SPI       // SD-SPI.c and .h

    #define MDD_MediaInitialize     MDD_SDSPI_MediaInitialize
    #define MDD_MediaDetect         MDD_SDSPI_MediaDetect
    #define MDD_SectorRead          MDD_SDSPI_SectorRead
    #define MDD_SectorWrite         MDD_SDSPI_SectorWrite
    #define MDD_InitIO              MDD_SDSPI_InitIO
    #define MDD_ShutdownMedia       MDD_SDSPI_ShutdownMedia
    #define MDD_WriteProtectState   MDD_SDSPI_WriteProtectState
    #define MDD_ReadSectorSize      MDD_SDSPI_ReadSectorSize
    #define MDD_ReadCapacity        MDD_SDSPI_ReadCapacity
    #define MDD_FINAL_SPI_SPEED     2000000

#endif

#endif
