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

#include "Compiler.h"
#include "GenericTypeDefs.h"
/*
const rom char Instr0[0xFF] =
{
'S','t','i','c','k',' ','n','o','t',' ','p','l','u','g','g','e','d',':',0x0D,0x0A,
'S','T','A','R','T',':',' ','P','u','s','h',' ','t','h','e',' ','b','u','t','t','o','n','.',' ','T','h','e',' ','L','E','D',' ','f','l','a','s','h','e','s',' ','3',' ','t','i','m','e','s','.',0x0D,0x0A,
'S','T','O','P',':',' ','P','u','s','h',' ','t','h','e',' ','b','u','t','t','o','n','.',' ','T','h','e',' ','L','E','D',' ','f','l','a','s','h','e','s',' ','2',' ','t','i','m','e','s','.',0x0D,0x0A,
0x0D,0x0A,' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0x0D,0x0A,
'E','R','A','S','E',' ','L','O','G',':',' ','H','o','l','d',' ','t','h','e',' ','b','u','t','t','o','n',' ','w','h','i','l','e',' ','p','l','u','g','g','i','n','g',' ','t','h','e',' ','s','t','i','c','k',0x0D,0x0A,
'W','h','e','n',' ','c','o','n','f','i','g','u','r','e','d',',',' ','t','h','e',' ','L','E','D',' ','b','l','i','n','k','s','.',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
0x00
};

*/
const rom char Instr0[0xFF] =
{
'S','t','a','r','t',' ','l','o','g','g','i','n','g',':',' ','P','r','e','s','s',
' ','t','h','e',' ','b','u','t','t','o','n',' ','w','h','e','n',' ','p','e','r','i','o','d',' ',
'c','o','n','f','i','g','u','r','e','d','.',' ','T','h','e',' ','L','E','D',' ','f','l','a','s','h','e','s',' ',
't','h','r','e','e',' ','t','i','m','e','s','.',0x0D,0x0A,
'P','a','u','s','e',' ','l','o','g','g','i','n','g',':',' ','P','r','e','s','s',' ','t','h','e',' ',
'b','u','t','t','o','n','.',' ','T','h','e',' ','L','E','D',' ','f','l','a','s','h','e','s',' ','t','w','o',' ',
't','i','m','e','s','.',0x0D,0x0A,
'F','o','r','m','a','t',':',' ','P','l','u','g',' ','t','o',' ','U','S','B',' ','w','i','t','h','o','u','t',' ',
't','h','e',' ','b','a','t','t','e','r','y',' ','w','h','i','l','e',' ','h','o','l','d','i','n','g',' ',
't','h','e',' ','b','u','t','t','o','n','.',0x0D,0x0A,
' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',
0x00
};



const far rom char Config0[0xC9] =
{
'#','T','H','I','S',' ','I','S',' ','T','H','E',' ','C','O','N','F','I','G','U','R','A','T','I','O','N',' ','F','I','L','E',
0x0D,0x0A,
'S','e','t','t','i','n','g',' ','t','h','e',' ','t','e','m','p','e','r','a','t','u','r','e',' ','l','o','g','g','i','n','g',' ','p','e','r','i','o','d',':','i','n',' ','s','e','c','o','n','d','s',0x0D,0x0A,'5','<','p','e','r','i','o','d','<','8','6','4','0','0',' ',0x0D,0x0A,
'D','o','n','t',' ','f','o','r','g','e','t',' ','t','o',' ','a','d','d',' ','a',' ','"','#','"',' ','b','e','f','o','r','e',' ','t','h','e',' ','n','u','m','b','e','r','.',0x0D,0x0A,
'#','6','0',0x0D,0x0A,
0x0D,0x0A,
'#','E','N','D',' ','O','F',' ','T','H','E',' ','C','O','N','F','I','G','U','R','A','T','I','O','N',' ','F','I','L','E',0x0D,0x0A,
' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',0x0D,0x0A
};


const far rom char CONFIG_FILE_ID_START[32] = "#THIS IS THE CONFIGURATION FILE";
const far rom char CONFIG_FILE_ID_END [31]=   "#END OF THE CONFIGURATION FILE";
const far rom char CONFIG_FILE_NAME[7] =      "CONFIG";
//const  rom char LOW_BATTERY_STR[12] =  "LOW BATTERY";
const  rom char LOG_NOT_STARTED[8] =  "NO DATA";