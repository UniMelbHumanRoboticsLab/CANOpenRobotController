/*******************************************************************************

   File - CO_OD.c/CO_OD.h
   CANopen Object Dictionary.

   Copyright (C) 2004-2008 Janez Paternoster

   License: GNU Lesser General Public License (LGPL).

   <http://canopennode.sourceforge.net>

   (For more information see <CO_SDO.h>.)

   This file is part of CANopenNode, an opensource CANopen Stack.
   Project home page is <https://github.com/CANopenNode/CANopenNode>.
   For more information on CANopen see <http://www.can-cia.org/>.
 
   CANopenNode is free and open source software: you can redistribute
   it and/or modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation, either version 2 of the
   License, or (at your option) any later version.
  
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
  
   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
  
   Following clarification and special exception to the GNU General Public
   License is included to the distribution terms of CANopenNode:
  
   Linking this library statically or dynamically with other modules is
   making a combined work based on this library. Thus, the terms and
   conditions of the GNU General Public License cover the whole combination.
  
   As a special exception, the copyright holders of this library give
   you permission to link this library with independent modules to
   produce an executable, regardless of the license terms of these
   independent modules, and to copy and distribute the resulting
   executable under terms of your choice, provided that you also meet,
   for each linked independent module, the terms and conditions of the
   license of that module. An independent module is a module which is
   not derived from or based on this library. If you modify this
   library, you may extend this exception to your version of the
   library, but you are not obliged to do so. If you do not wish
   to do so, delete this exception statement from your version.
 
   This file was automatically generated with libedssharp Object
   Dictionary Editor v0.6-xdd-alpha-81-gb562769
   DON'T EDIT THIS FILE MANUALLY !!!!
*******************************************************************************/

#include "CO_OD.h"

#include "CO_SDO.h"
#include "CO_driver.h"

/*******************************************************************************
   DEFINITION AND INITIALIZATION OF OBJECT DICTIONARY VARIABLES
*******************************************************************************/

/***** Definition for ROM variables ********************************************/
struct sCO_OD_ROM CO_OD_ROM = {
    CO_OD_FIRST_LAST_WORD,

    /*100c*/ 0x00,
    /*1012*/ 0x0000L,

    CO_OD_FIRST_LAST_WORD,
};

/***** Definition for RAM variables ********************************************/
struct sCO_OD_RAM CO_OD_RAM = {
    CO_OD_FIRST_LAST_WORD,

    /*1000*/ 0x0000L,
    /*1001*/ 0x0L,
    /*1002*/ 0x0000L,
    /*1003*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1005*/ 0x40000080,
    /*1006*/ 0xF4240,
    /*1007*/ 0x0000L,
    /*1008*/ {'C', 'A', 'N', 'o', 'p', 'e', 'n', 'N', 'o', 'd', 'e'},
    /*1009*/ {'3', '.', '0', '0'},
    /*100a*/ {'3', '.', '0', '0'},
    /*100d*/ 0x0L,
    /*1010*/ {0x0003L},
    /*1011*/ {0x0001L},
    /*1013*/ 0x0000L,
    /*1014*/ 0x0080L,
    /*1015*/ 0x64,
    /*1016*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1017*/ 0x3E8,
    /*1018*/ {0x4L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1019*/ 0x0L,
    /*1029*/ {0x0L, 0x0L, 0x1L, 0x0L, 0x0L, 0x0L},
    /*1200*/ {{0x2L, 0x0600L, 0x0580L}},
    /*1280*/ {{0x3L, 0x0000L, 0x0000L, 0x0L}},
    /*1400*/ {{0x2L, 0x0181L, 0xffL},
              /*1401*/ {0x2L, 0x0182L, 0xffL},
              /*1402*/ {0x2L, 0x0183L, 0xffL},
              /*1403*/ {0x2L, 0x0184L, 0xffL},
              /*1404*/ {0x2L, 0x0185L, 0xfeL},
              /*1405*/ {0x2L, 0x0186L, 0xfeL},
              /*1406*/ {0x2L, 0x0281L, 0xfeL},
              /*1407*/ {0x2L, 0x0282L, 0xfeL},
              /*1408*/ {0x2L, 0x0283L, 0xfeL},
              /*1409*/ {0x2L, 0x0284L, 0xfeL},
              /*140a*/ {0x2L, 0x0285L, 0xfeL},
              /*140b*/ {0x2L, 0x0286L, 0xfeL},
              /*140c*/ {0x2L, 0x0381L, 0xfeL},
              /*140d*/ {0x2L, 0x0382L, 0xfeL},
              /*140e*/ {0x2L, 0x0383L, 0xfeL},
              /*140f*/ {0x2L, 0x0384L, 0xfeL},
              /*1410*/ {0x2L, 0x0191L, 0xfeL},
              /*1411*/ {0x2L, 0x0192L, 0xfeL},
              /*1412*/ {0x2L, 0x0193L, 0xfeL},
              /*1413*/ {0x2L, 0x80000000L, 0xfeL},
              /*1414*/ {0x2L, 0x80000000L, 0xfeL},
              /*1415*/ {0x2L, 0x80000000L, 0xfeL},
              /*1416*/ {0x2L, 0x80000000L, 0xfeL},
              /*1417*/ {0x2L, 0x80000000L, 0xfeL},
              /*1418*/ {0x2L, 0x80000000L, 0xfeL},
              /*1419*/ {0x2L, 0x80000000L, 0xfeL},
              /*141a*/ {0x2L, 0x80000000L, 0xfeL},
              /*141b*/ {0x2L, 0x80000000L, 0xfeL},
              /*141c*/ {0x2L, 0x80000000L, 0xfeL},
              /*141d*/ {0x2L, 0x80000000L, 0xfeL},
              /*141e*/ {0x2L, 0x80000000L, 0xfeL},
              /*141f*/ {0x2L, 0x80000000L, 0xfeL}},
    /*1600*/ {{0x1L, 0x60410110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1601*/ {0x1L, 0x60410210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1602*/ {0x1L, 0x60410310L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1603*/ {0x1L, 0x60410410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1604*/ {0x1L, 0x60410510L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1605*/ {0x1L, 0x60410610L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1606*/ {0x2L, 0x60640120L, 0x606c0120L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1607*/ {0x2L, 0x60640220L, 0x606c0220L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1608*/ {0x2L, 0x60640320L, 0x606c0320L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1609*/ {0x2L, 0x60640420L, 0x606c0420L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160a*/ {0x2L, 0x60640520L, 0x606c0520L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160b*/ {0x2L, 0x60640620L, 0x606c0620L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160c*/ {0x1L, 0x60770110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160d*/ {0x1L, 0x60770210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160e*/ {0x1L, 0x60770310L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*160f*/ {0x1L, 0x60770410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1610*/ {0x1L, 0x60030010L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1611*/ {0x1L, 0x60040010L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1612*/ {0x1L, 0x60050010L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1613*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1614*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1615*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1616*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1617*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1618*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1619*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161a*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161b*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161c*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161d*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161e*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*161f*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L}},
    /*1800*/ {{0x6L, 0x0201L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1801*/ {0x6L, 0x0202L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1802*/ {0x6L, 0x0203L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1803*/ {0x6L, 0x0204L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1804*/ {0x6L, 0x0205L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1805*/ {0x6L, 0x0206L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1806*/ {0x6L, 0x0301L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1807*/ {0x6L, 0x0302L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1808*/ {0x6L, 0x0303L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1809*/ {0x6L, 0x0304L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180a*/ {0x6L, 0x0305L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180b*/ {0x6L, 0x0306L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180c*/ {0x6L, 0x0401L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180d*/ {0x6L, 0x0402L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180e*/ {0x6L, 0x0403L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*180f*/ {0x6L, 0x0404L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1810*/ {0x6L, 0x0405L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1811*/ {0x6L, 0x0406L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1812*/ {0x6L, 0x0211L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1813*/ {0x6L, 0x0212L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1814*/ {0x6L, 0x0501L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1815*/ {0x6L, 0x0502L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1816*/ {0x6L, 0x0503L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1817*/ {0x6L, 0x0504L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1818*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*1819*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181a*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181b*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181c*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181d*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181e*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L},
              /*181f*/ {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L}},
    /*1a00*/ {{0x1L, 0x60400110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a01*/ {0x1L, 0x60400210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a02*/ {0x1L, 0x60400310L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a03*/ {0x1L, 0x60400410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a04*/ {0x1L, 0x60400510L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a05*/ {0x1L, 0x60400610L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a06*/ {0x1L, 0x607a0120L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a07*/ {0x1L, 0x607a0220L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a08*/ {0x1L, 0x607a0320L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a09*/ {0x1L, 0x607a0420L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0a*/ {0x1L, 0x607a0520L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0b*/ {0x1L, 0x607a0620L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0c*/ {0x1L, 0x60ff0120L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0d*/ {0x1L, 0x60ff0220L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0e*/ {0x1L, 0x60ff0320L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a0f*/ {0x1L, 0x60ff0420L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a10*/ {0x1L, 0x60ff0520L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a11*/ {0x1L, 0x60ff0620L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a12*/ {0x1L, 0x60010010L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a13*/ {0x1L, 0x60020010L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a14*/ {0x1L, 0x60710110L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a15*/ {0x1L, 0x60710210L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a16*/ {0x1L, 0x60710310L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a17*/ {0x1L, 0x60710410L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a18*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a19*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1a*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1b*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1c*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1d*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1e*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
              /*1a1f*/ {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L}},
    /*1f80*/ 0x0000L,
    /*1f81*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*1f82*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
    /*1f89*/ 0x0000L,
    /*2100*/ {0x0L},
    /*2101*/ 0x20L,
    /*2102*/ 0xfa,
    /*2103*/ 0x00,
    /*2104*/ 0x00,
    /*2106*/ 0x0000L,
    /*2107*/ {0x3e8, 0x00, 0x00, 0x00, 0x00},
    /*2108*/ {0x00},
    /*2109*/ {0x00},
    /*2110*/ {0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*2111*/ {0x0001L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*2112*/ {0x0001L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*2120*/ {0x5L, 0x1234567890abcdefL, 0x234567890abcdef1L, 12.345, 456.789, 0},
    /*2130*/ {0x3L, {'-'}, 0x00000000L, 0x0000L},
    /*2209*/ {0x4L, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*2301*/ {{0x8L, 0x03e8L, 0x0L, {'T', 'r', 'a', 'c', 'e', '1'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2302*/ {0x8L, 0x03e8L, 0x0L, {'T', 'r', 'a', 'c', 'e', '2'}, {'g', 'r', 'e', 'e', 'n'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2303*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2304*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2305*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2306*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2307*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2308*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2309*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230a*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230b*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230c*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230d*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230e*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*230f*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2310*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2311*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2312*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2313*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2314*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2315*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2316*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2317*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2318*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2319*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231a*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231b*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231c*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231d*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231e*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*231f*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L},
              /*2320*/ {0x8L, 0x03e8L, 0x0L, {'n', 'a', 'm', 'e'}, {'r', 'e', 'd'}, 0x0000L, 0x0L, 0x0L, 0x0000L}},
    /*2400*/ 0x0L,
    /*2401*/ {{0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2402*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2403*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2404*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2405*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2406*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2407*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2408*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2409*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240a*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240b*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240c*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240d*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240e*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*240f*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2410*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2411*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2412*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2413*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2414*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2415*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2416*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2417*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2418*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2419*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241a*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241b*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241c*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241d*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241e*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*241f*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L},
              /*2420*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0, 0x0000L}},
    /*6000*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
    /*6001*/ 0x0000,
    /*6002*/ 0x0000,
    /*6003*/ 0x0000,
    /*6004*/ 0x0000,
    /*6005*/ 0x0000,
    /*6040*/ {0x6L, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*6041*/ {0x6L, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*6064*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*606c*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*6077*/ {0x4L, 0x0000, 0x0000, 0x0000, 0x0000},
    /*607a*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*60ff*/ {0x6L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L},
    /*6071*/ {0x4L, 0x0000, 0x0000, 0x0000, 0x0000},
    /*6200*/ {0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L, 0x0L},
    /*6401*/ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    /*6411*/ {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    CO_OD_FIRST_LAST_WORD,
};

/***** Definition for EEPROM variables ********************************************/
struct sCO_OD_EEPROM CO_OD_EEPROM = {
    CO_OD_FIRST_LAST_WORD,

    CO_OD_FIRST_LAST_WORD,
};

/*******************************************************************************
   STRUCTURES FOR RECORD TYPE OBJECTS
*******************************************************************************/

/*0x1018*/ const CO_OD_entryRecord_t OD_record1018[5] = {
    {(void *)&CO_OD_RAM.identity.maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.identity.vendorID, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.productCode, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.revisionNumber, 0x86, 0x4},
    {(void *)&CO_OD_RAM.identity.serialNumber, 0x86, 0x4},
};

/*0x1200*/ const CO_OD_entryRecord_t OD_record1200[3] = {
    {(void *)&CO_OD_RAM.SDOServerParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.SDOServerParameter[0].COB_IDClientToServer, 0x86, 0x4},
    {(void *)&CO_OD_RAM.SDOServerParameter[0].COB_IDServerToClient, 0x86, 0x4},
};

/*0x1280*/ const CO_OD_entryRecord_t OD_record1280[4] = {
    {(void *)&CO_OD_RAM.SDOClientParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].COB_IDClientToServer, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].COB_IDServerToClient, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.SDOClientParameter[0].nodeIDOfTheSDOServer, 0x0e, 0x1},
};

/*0x1400*/ const CO_OD_entryRecord_t OD_record1400[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[0].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[0].transmissionType, 0x0e, 0x1},
};

/*0x1401*/ const CO_OD_entryRecord_t OD_record1401[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[1].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[1].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[1].transmissionType, 0x0e, 0x1},
};

/*0x1402*/ const CO_OD_entryRecord_t OD_record1402[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[2].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[2].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[2].transmissionType, 0x0e, 0x1},
};

/*0x1403*/ const CO_OD_entryRecord_t OD_record1403[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[3].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[3].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[3].transmissionType, 0x0e, 0x1},
};

/*0x1404*/ const CO_OD_entryRecord_t OD_record1404[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[4].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[4].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[4].transmissionType, 0x0e, 0x1},
};

/*0x1405*/ const CO_OD_entryRecord_t OD_record1405[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[5].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[5].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[5].transmissionType, 0x0e, 0x1},
};

/*0x1406*/ const CO_OD_entryRecord_t OD_record1406[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[6].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[6].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[6].transmissionType, 0x0e, 0x1},
};

/*0x1407*/ const CO_OD_entryRecord_t OD_record1407[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[7].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[7].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[7].transmissionType, 0x0e, 0x1},
};

/*0x1408*/ const CO_OD_entryRecord_t OD_record1408[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[8].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[8].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[8].transmissionType, 0x0e, 0x1},
};

/*0x1409*/ const CO_OD_entryRecord_t OD_record1409[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[9].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[9].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[9].transmissionType, 0x0e, 0x1},
};

/*0x140a*/ const CO_OD_entryRecord_t OD_record140a[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[10].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[10].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[10].transmissionType, 0x0e, 0x1},
};

/*0x140b*/ const CO_OD_entryRecord_t OD_record140b[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[11].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[11].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[11].transmissionType, 0x0e, 0x1},
};

/*0x140c*/ const CO_OD_entryRecord_t OD_record140c[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[12].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[12].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[12].transmissionType, 0x0e, 0x1},
};

/*0x140d*/ const CO_OD_entryRecord_t OD_record140d[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[13].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[13].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[13].transmissionType, 0x0e, 0x1},
};

/*0x140e*/ const CO_OD_entryRecord_t OD_record140e[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[14].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[14].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[14].transmissionType, 0x0e, 0x1},
};

/*0x140f*/ const CO_OD_entryRecord_t OD_record140f[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[15].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[15].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[15].transmissionType, 0x0e, 0x1},
};

/*0x1410*/ const CO_OD_entryRecord_t OD_record1410[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[16].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[16].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[16].transmissionType, 0x0e, 0x1},
};

/*0x1411*/ const CO_OD_entryRecord_t OD_record1411[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[17].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[17].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[17].transmissionType, 0x0e, 0x1},
};

/*0x1412*/ const CO_OD_entryRecord_t OD_record1412[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[18].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[18].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[18].transmissionType, 0x0e, 0x1},
};

/*0x1413*/ const CO_OD_entryRecord_t OD_record1413[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[19].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[19].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[19].transmissionType, 0x0e, 0x1},
};

/*0x1414*/ const CO_OD_entryRecord_t OD_record1414[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[20].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[20].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[20].transmissionType, 0x0e, 0x1},
};

/*0x1415*/ const CO_OD_entryRecord_t OD_record1415[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[21].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[21].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[21].transmissionType, 0x0e, 0x1},
};

/*0x1416*/ const CO_OD_entryRecord_t OD_record1416[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[22].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[22].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[22].transmissionType, 0x0e, 0x1},
};

/*0x1417*/ const CO_OD_entryRecord_t OD_record1417[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[23].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[23].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[23].transmissionType, 0x0e, 0x1},
};

/*0x1418*/ const CO_OD_entryRecord_t OD_record1418[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[24].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[24].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[24].transmissionType, 0x0e, 0x1},
};

/*0x1419*/ const CO_OD_entryRecord_t OD_record1419[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[25].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[25].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[25].transmissionType, 0x0e, 0x1},
};

/*0x141a*/ const CO_OD_entryRecord_t OD_record141a[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[26].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[26].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[26].transmissionType, 0x0e, 0x1},
};

/*0x141b*/ const CO_OD_entryRecord_t OD_record141b[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[27].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[27].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[27].transmissionType, 0x0e, 0x1},
};

/*0x141c*/ const CO_OD_entryRecord_t OD_record141c[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[28].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[28].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[28].transmissionType, 0x0e, 0x1},
};

/*0x141d*/ const CO_OD_entryRecord_t OD_record141d[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[29].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[29].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[29].transmissionType, 0x0e, 0x1},
};

/*0x141e*/ const CO_OD_entryRecord_t OD_record141e[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[30].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[30].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[30].transmissionType, 0x0e, 0x1},
};

/*0x141f*/ const CO_OD_entryRecord_t OD_record141f[3] = {
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[31].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[31].COB_IDUsedByRPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOCommunicationParameter[31].transmissionType, 0x0e, 0x1},
};

/*0x1600*/ const CO_OD_entryRecord_t OD_record1600[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[0].mappedObject8, 0x8e, 0x4},
};

/*0x1601*/ const CO_OD_entryRecord_t OD_record1601[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[1].mappedObject8, 0x8e, 0x4},
};

/*0x1602*/ const CO_OD_entryRecord_t OD_record1602[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[2].mappedObject8, 0x8e, 0x4},
};

/*0x1603*/ const CO_OD_entryRecord_t OD_record1603[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[3].mappedObject8, 0x8e, 0x4},
};

/*0x1604*/ const CO_OD_entryRecord_t OD_record1604[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[4].mappedObject8, 0x8e, 0x4},
};

/*0x1605*/ const CO_OD_entryRecord_t OD_record1605[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[5].mappedObject8, 0x8e, 0x4},
};

/*0x1606*/ const CO_OD_entryRecord_t OD_record1606[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[6].mappedObject8, 0x8e, 0x4},
};

/*0x1607*/ const CO_OD_entryRecord_t OD_record1607[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[7].mappedObject8, 0x8e, 0x4},
};

/*0x1608*/ const CO_OD_entryRecord_t OD_record1608[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[8].mappedObject8, 0x8e, 0x4},
};

/*0x1609*/ const CO_OD_entryRecord_t OD_record1609[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[9].mappedObject8, 0x8e, 0x4},
};

/*0x160a*/ const CO_OD_entryRecord_t OD_record160a[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[10].mappedObject8, 0x8e, 0x4},
};

/*0x160b*/ const CO_OD_entryRecord_t OD_record160b[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[11].mappedObject8, 0x8e, 0x4},
};

/*0x160c*/ const CO_OD_entryRecord_t OD_record160c[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[12].mappedObject8, 0x8e, 0x4},
};

/*0x160d*/ const CO_OD_entryRecord_t OD_record160d[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[13].mappedObject8, 0x8e, 0x4},
};

/*0x160e*/ const CO_OD_entryRecord_t OD_record160e[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[14].mappedObject8, 0x8e, 0x4},
};

/*0x160f*/ const CO_OD_entryRecord_t OD_record160f[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[15].mappedObject8, 0x8e, 0x4},
};

/*0x1610*/ const CO_OD_entryRecord_t OD_record1610[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[16].mappedObject8, 0x8e, 0x4},
};

/*0x1611*/ const CO_OD_entryRecord_t OD_record1611[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[17].mappedObject8, 0x8e, 0x4},
};

/*0x1612*/ const CO_OD_entryRecord_t OD_record1612[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[18].mappedObject8, 0x8e, 0x4},
};

/*0x1613*/ const CO_OD_entryRecord_t OD_record1613[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[19].mappedObject8, 0x8e, 0x4},
};

/*0x1614*/ const CO_OD_entryRecord_t OD_record1614[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[20].mappedObject8, 0x8e, 0x4},
};

/*0x1615*/ const CO_OD_entryRecord_t OD_record1615[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[21].mappedObject8, 0x8e, 0x4},
};

/*0x1616*/ const CO_OD_entryRecord_t OD_record1616[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[22].mappedObject8, 0x8e, 0x4},
};

/*0x1617*/ const CO_OD_entryRecord_t OD_record1617[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[23].mappedObject8, 0x8e, 0x4},
};

/*0x1618*/ const CO_OD_entryRecord_t OD_record1618[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[24].mappedObject8, 0x8e, 0x4},
};

/*0x1619*/ const CO_OD_entryRecord_t OD_record1619[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[25].mappedObject8, 0x8e, 0x4},
};

/*0x161a*/ const CO_OD_entryRecord_t OD_record161a[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[26].mappedObject8, 0x8e, 0x4},
};

/*0x161b*/ const CO_OD_entryRecord_t OD_record161b[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[27].mappedObject8, 0x8e, 0x4},
};

/*0x161c*/ const CO_OD_entryRecord_t OD_record161c[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[28].mappedObject8, 0x8e, 0x4},
};

/*0x161d*/ const CO_OD_entryRecord_t OD_record161d[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[29].mappedObject8, 0x8e, 0x4},
};

/*0x161e*/ const CO_OD_entryRecord_t OD_record161e[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[30].mappedObject8, 0x8e, 0x4},
};

/*0x161f*/ const CO_OD_entryRecord_t OD_record161f[9] = {
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.RPDOMappingParameter[31].mappedObject8, 0x8e, 0x4},
};

/*0x1800*/ const CO_OD_entryRecord_t OD_record1800[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[0].SYNCStartValue, 0x0e, 0x1},
};

/*0x1801*/ const CO_OD_entryRecord_t OD_record1801[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[1].SYNCStartValue, 0x0e, 0x1},
};

/*0x1802*/ const CO_OD_entryRecord_t OD_record1802[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[2].SYNCStartValue, 0x0e, 0x1},
};

/*0x1803*/ const CO_OD_entryRecord_t OD_record1803[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[3].SYNCStartValue, 0x0e, 0x1},
};

/*0x1804*/ const CO_OD_entryRecord_t OD_record1804[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[4].SYNCStartValue, 0x0e, 0x1},
};

/*0x1805*/ const CO_OD_entryRecord_t OD_record1805[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[5].SYNCStartValue, 0x0e, 0x1},
};

/*0x1806*/ const CO_OD_entryRecord_t OD_record1806[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[6].SYNCStartValue, 0x0e, 0x1},
};

/*0x1807*/ const CO_OD_entryRecord_t OD_record1807[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[7].SYNCStartValue, 0x0e, 0x1},
};

/*0x1808*/ const CO_OD_entryRecord_t OD_record1808[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[8].SYNCStartValue, 0x0e, 0x1},
};

/*0x1809*/ const CO_OD_entryRecord_t OD_record1809[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[9].SYNCStartValue, 0x0e, 0x1},
};

/*0x180a*/ const CO_OD_entryRecord_t OD_record180a[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[10].SYNCStartValue, 0x0e, 0x1},
};

/*0x180b*/ const CO_OD_entryRecord_t OD_record180b[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[11].SYNCStartValue, 0x0e, 0x1},
};

/*0x180c*/ const CO_OD_entryRecord_t OD_record180c[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[12].SYNCStartValue, 0x0e, 0x1},
};

/*0x180d*/ const CO_OD_entryRecord_t OD_record180d[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[13].SYNCStartValue, 0x0e, 0x1},
};

/*0x180e*/ const CO_OD_entryRecord_t OD_record180e[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[14].SYNCStartValue, 0x0e, 0x1},
};

/*0x180f*/ const CO_OD_entryRecord_t OD_record180f[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[15].SYNCStartValue, 0x0e, 0x1},
};

/*0x1810*/ const CO_OD_entryRecord_t OD_record1810[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[16].SYNCStartValue, 0x0e, 0x1},
};

/*0x1811*/ const CO_OD_entryRecord_t OD_record1811[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[17].SYNCStartValue, 0x0e, 0x1},
};

/*0x1812*/ const CO_OD_entryRecord_t OD_record1812[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[18].SYNCStartValue, 0x0e, 0x1},
};

/*0x1813*/ const CO_OD_entryRecord_t OD_record1813[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[19].SYNCStartValue, 0x0e, 0x1},
};

/*0x1814*/ const CO_OD_entryRecord_t OD_record1814[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[20].SYNCStartValue, 0x0e, 0x1},
};

/*0x1815*/ const CO_OD_entryRecord_t OD_record1815[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[21].SYNCStartValue, 0x0e, 0x1},
};

/*0x1816*/ const CO_OD_entryRecord_t OD_record1816[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[22].SYNCStartValue, 0x0e, 0x1},
};

/*0x1817*/ const CO_OD_entryRecord_t OD_record1817[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[23].SYNCStartValue, 0x0e, 0x1},
};

/*0x1818*/ const CO_OD_entryRecord_t OD_record1818[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[24].SYNCStartValue, 0x0e, 0x1},
};

/*0x1819*/ const CO_OD_entryRecord_t OD_record1819[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[25].SYNCStartValue, 0x0e, 0x1},
};

/*0x181a*/ const CO_OD_entryRecord_t OD_record181a[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[26].SYNCStartValue, 0x0e, 0x1},
};

/*0x181b*/ const CO_OD_entryRecord_t OD_record181b[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[27].SYNCStartValue, 0x0e, 0x1},
};

/*0x181c*/ const CO_OD_entryRecord_t OD_record181c[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[28].SYNCStartValue, 0x0e, 0x1},
};

/*0x181d*/ const CO_OD_entryRecord_t OD_record181d[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[29].SYNCStartValue, 0x0e, 0x1},
};

/*0x181e*/ const CO_OD_entryRecord_t OD_record181e[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[30].SYNCStartValue, 0x0e, 0x1},
};

/*0x181f*/ const CO_OD_entryRecord_t OD_record181f[7] = {
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].transmissionType, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].inhibitTime, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].compatibilityEntry, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].eventTimer, 0x8e, 0x2},
    {(void *)&CO_OD_RAM.TPDOCommunicationParameter[31].SYNCStartValue, 0x0e, 0x1},
};

/*0x1a00*/ const CO_OD_entryRecord_t OD_record1a00[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[0].mappedObject8, 0x8e, 0x4},
};

/*0x1a01*/ const CO_OD_entryRecord_t OD_record1a01[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[1].mappedObject8, 0x8e, 0x4},
};

/*0x1a02*/ const CO_OD_entryRecord_t OD_record1a02[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[2].mappedObject8, 0x8e, 0x4},
};

/*0x1a03*/ const CO_OD_entryRecord_t OD_record1a03[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[3].mappedObject8, 0x8e, 0x4},
};

/*0x1a04*/ const CO_OD_entryRecord_t OD_record1a04[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[4].mappedObject8, 0x8e, 0x4},
};

/*0x1a05*/ const CO_OD_entryRecord_t OD_record1a05[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[5].mappedObject8, 0x8e, 0x4},
};

/*0x1a06*/ const CO_OD_entryRecord_t OD_record1a06[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[6].mappedObject8, 0x8e, 0x4},
};

/*0x1a07*/ const CO_OD_entryRecord_t OD_record1a07[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[7].mappedObject8, 0x8e, 0x4},
};

/*0x1a08*/ const CO_OD_entryRecord_t OD_record1a08[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[8].mappedObject8, 0x8e, 0x4},
};

/*0x1a09*/ const CO_OD_entryRecord_t OD_record1a09[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[9].mappedObject8, 0x8e, 0x4},
};

/*0x1a0a*/ const CO_OD_entryRecord_t OD_record1a0a[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[10].mappedObject8, 0x8e, 0x4},
};

/*0x1a0b*/ const CO_OD_entryRecord_t OD_record1a0b[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[11].mappedObject8, 0x8e, 0x4},
};

/*0x1a0c*/ const CO_OD_entryRecord_t OD_record1a0c[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[12].mappedObject8, 0x8e, 0x4},
};

/*0x1a0d*/ const CO_OD_entryRecord_t OD_record1a0d[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[13].mappedObject8, 0x8e, 0x4},
};

/*0x1a0e*/ const CO_OD_entryRecord_t OD_record1a0e[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[14].mappedObject8, 0x8e, 0x4},
};

/*0x1a0f*/ const CO_OD_entryRecord_t OD_record1a0f[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[15].mappedObject8, 0x8e, 0x4},
};

/*0x1a10*/ const CO_OD_entryRecord_t OD_record1a10[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[16].mappedObject8, 0x8e, 0x4},
};

/*0x1a11*/ const CO_OD_entryRecord_t OD_record1a11[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[17].mappedObject8, 0x8e, 0x4},
};

/*0x1a12*/ const CO_OD_entryRecord_t OD_record1a12[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[18].mappedObject8, 0x8e, 0x4},
};

/*0x1a13*/ const CO_OD_entryRecord_t OD_record1a13[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[19].mappedObject8, 0x8e, 0x4},
};

/*0x1a14*/ const CO_OD_entryRecord_t OD_record1a14[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[20].mappedObject8, 0x8e, 0x4},
};

/*0x1a15*/ const CO_OD_entryRecord_t OD_record1a15[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[21].mappedObject8, 0x8e, 0x4},
};

/*0x1a16*/ const CO_OD_entryRecord_t OD_record1a16[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[22].mappedObject8, 0x8e, 0x4},
};

/*0x1a17*/ const CO_OD_entryRecord_t OD_record1a17[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[23].mappedObject8, 0x8e, 0x4},
};

/*0x1a18*/ const CO_OD_entryRecord_t OD_record1a18[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[24].mappedObject8, 0x8e, 0x4},
};

/*0x1a19*/ const CO_OD_entryRecord_t OD_record1a19[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[25].mappedObject8, 0x8e, 0x4},
};

/*0x1a1a*/ const CO_OD_entryRecord_t OD_record1a1a[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[26].mappedObject8, 0x8e, 0x4},
};

/*0x1a1b*/ const CO_OD_entryRecord_t OD_record1a1b[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[27].mappedObject8, 0x8e, 0x4},
};

/*0x1a1c*/ const CO_OD_entryRecord_t OD_record1a1c[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[28].mappedObject8, 0x8e, 0x4},
};

/*0x1a1d*/ const CO_OD_entryRecord_t OD_record1a1d[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[29].mappedObject8, 0x8e, 0x4},
};

/*0x1a1e*/ const CO_OD_entryRecord_t OD_record1a1e[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[30].mappedObject8, 0x8e, 0x4},
};

/*0x1a1f*/ const CO_OD_entryRecord_t OD_record1a1f[9] = {
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject1, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject2, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject3, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject4, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject5, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject6, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject7, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.TPDOMappingParameter[31].mappedObject8, 0x8e, 0x4},
};

/*0x2120*/ const CO_OD_entryRecord_t OD_record2120[6] = {
    {(void *)&CO_OD_RAM.testVar.maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.testVar.I64, 0x9e, 0x8},
    {(void *)&CO_OD_RAM.testVar.U64, 0x9e, 0x8},
    {(void *)&CO_OD_RAM.testVar.R32, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.testVar.R64, 0x9e, 0x8},
    {(void *)0, 0x0e, 0x0},
};

/*0x2130*/ const CO_OD_entryRecord_t OD_record2130[4] = {
    {(void *)&CO_OD_RAM.time.maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.time.string, 0x06, 0x1},
    {(void *)&CO_OD_RAM.time.epochTimeBaseMs, 0x86, 0x8},
    {(void *)&CO_OD_RAM.time.epochTimeOffsetMs, 0x9e, 0x4},
};

/*0x2209*/ const CO_OD_entryRecord_t OD_record2209[7] = {
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor1, 0xbe, 0x2},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor2, 0xbe, 0x2},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor3, 0xbe, 0x2},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor4, 0xbe, 0x2},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor5, 0xbe, 0x2},
    {(void *)&CO_OD_RAM.motorTempSensorVoltages.motor6, 0xbe, 0x2},
};

/*0x2301*/ const CO_OD_entryRecord_t OD_record2301[9] = {
    {(void *)&CO_OD_RAM.traceConfig[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[0].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[0].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[0].threshold, 0x8e, 0x4},
};

/*0x2302*/ const CO_OD_entryRecord_t OD_record2302[9] = {
    {(void *)&CO_OD_RAM.traceConfig[1].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[1].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[1].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[1].threshold, 0x8e, 0x4},
};

/*0x2303*/ const CO_OD_entryRecord_t OD_record2303[9] = {
    {(void *)&CO_OD_RAM.traceConfig[2].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[2].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[2].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[2].threshold, 0x8e, 0x4},
};

/*0x2304*/ const CO_OD_entryRecord_t OD_record2304[9] = {
    {(void *)&CO_OD_RAM.traceConfig[3].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[3].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[3].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[3].threshold, 0x8e, 0x4},
};

/*0x2305*/ const CO_OD_entryRecord_t OD_record2305[9] = {
    {(void *)&CO_OD_RAM.traceConfig[4].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[4].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[4].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[4].threshold, 0x8e, 0x4},
};

/*0x2306*/ const CO_OD_entryRecord_t OD_record2306[9] = {
    {(void *)&CO_OD_RAM.traceConfig[5].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[5].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[5].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[5].threshold, 0x8e, 0x4},
};

/*0x2307*/ const CO_OD_entryRecord_t OD_record2307[9] = {
    {(void *)&CO_OD_RAM.traceConfig[6].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[6].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[6].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[6].threshold, 0x8e, 0x4},
};

/*0x2308*/ const CO_OD_entryRecord_t OD_record2308[9] = {
    {(void *)&CO_OD_RAM.traceConfig[7].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[7].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[7].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[7].threshold, 0x8e, 0x4},
};

/*0x2309*/ const CO_OD_entryRecord_t OD_record2309[9] = {
    {(void *)&CO_OD_RAM.traceConfig[8].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[8].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[8].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[8].threshold, 0x8e, 0x4},
};

/*0x230a*/ const CO_OD_entryRecord_t OD_record230a[9] = {
    {(void *)&CO_OD_RAM.traceConfig[9].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[9].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[9].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[9].threshold, 0x8e, 0x4},
};

/*0x230b*/ const CO_OD_entryRecord_t OD_record230b[9] = {
    {(void *)&CO_OD_RAM.traceConfig[10].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[10].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[10].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[10].threshold, 0x8e, 0x4},
};

/*0x230c*/ const CO_OD_entryRecord_t OD_record230c[9] = {
    {(void *)&CO_OD_RAM.traceConfig[11].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[11].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[11].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[11].threshold, 0x8e, 0x4},
};

/*0x230d*/ const CO_OD_entryRecord_t OD_record230d[9] = {
    {(void *)&CO_OD_RAM.traceConfig[12].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[12].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[12].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[12].threshold, 0x8e, 0x4},
};

/*0x230e*/ const CO_OD_entryRecord_t OD_record230e[9] = {
    {(void *)&CO_OD_RAM.traceConfig[13].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[13].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[13].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[13].threshold, 0x8e, 0x4},
};

/*0x230f*/ const CO_OD_entryRecord_t OD_record230f[9] = {
    {(void *)&CO_OD_RAM.traceConfig[14].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[14].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[14].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[14].threshold, 0x8e, 0x4},
};

/*0x2310*/ const CO_OD_entryRecord_t OD_record2310[9] = {
    {(void *)&CO_OD_RAM.traceConfig[15].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[15].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[15].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[15].threshold, 0x8e, 0x4},
};

/*0x2311*/ const CO_OD_entryRecord_t OD_record2311[9] = {
    {(void *)&CO_OD_RAM.traceConfig[16].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[16].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[16].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[16].threshold, 0x8e, 0x4},
};

/*0x2312*/ const CO_OD_entryRecord_t OD_record2312[9] = {
    {(void *)&CO_OD_RAM.traceConfig[17].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[17].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[17].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[17].threshold, 0x8e, 0x4},
};

/*0x2313*/ const CO_OD_entryRecord_t OD_record2313[9] = {
    {(void *)&CO_OD_RAM.traceConfig[18].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[18].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[18].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[18].threshold, 0x8e, 0x4},
};

/*0x2314*/ const CO_OD_entryRecord_t OD_record2314[9] = {
    {(void *)&CO_OD_RAM.traceConfig[19].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[19].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[19].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[19].threshold, 0x8e, 0x4},
};

/*0x2315*/ const CO_OD_entryRecord_t OD_record2315[9] = {
    {(void *)&CO_OD_RAM.traceConfig[20].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[20].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[20].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[20].threshold, 0x8e, 0x4},
};

/*0x2316*/ const CO_OD_entryRecord_t OD_record2316[9] = {
    {(void *)&CO_OD_RAM.traceConfig[21].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[21].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[21].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[21].threshold, 0x8e, 0x4},
};

/*0x2317*/ const CO_OD_entryRecord_t OD_record2317[9] = {
    {(void *)&CO_OD_RAM.traceConfig[22].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[22].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[22].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[22].threshold, 0x8e, 0x4},
};

/*0x2318*/ const CO_OD_entryRecord_t OD_record2318[9] = {
    {(void *)&CO_OD_RAM.traceConfig[23].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[23].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[23].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[23].threshold, 0x8e, 0x4},
};

/*0x2319*/ const CO_OD_entryRecord_t OD_record2319[9] = {
    {(void *)&CO_OD_RAM.traceConfig[24].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[24].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[24].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[24].threshold, 0x8e, 0x4},
};

/*0x231a*/ const CO_OD_entryRecord_t OD_record231a[9] = {
    {(void *)&CO_OD_RAM.traceConfig[25].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[25].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[25].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[25].threshold, 0x8e, 0x4},
};

/*0x231b*/ const CO_OD_entryRecord_t OD_record231b[9] = {
    {(void *)&CO_OD_RAM.traceConfig[26].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[26].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[26].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[26].threshold, 0x8e, 0x4},
};

/*0x231c*/ const CO_OD_entryRecord_t OD_record231c[9] = {
    {(void *)&CO_OD_RAM.traceConfig[27].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[27].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[27].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[27].threshold, 0x8e, 0x4},
};

/*0x231d*/ const CO_OD_entryRecord_t OD_record231d[9] = {
    {(void *)&CO_OD_RAM.traceConfig[28].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[28].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[28].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[28].threshold, 0x8e, 0x4},
};

/*0x231e*/ const CO_OD_entryRecord_t OD_record231e[9] = {
    {(void *)&CO_OD_RAM.traceConfig[29].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[29].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[29].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[29].threshold, 0x8e, 0x4},
};

/*0x231f*/ const CO_OD_entryRecord_t OD_record231f[9] = {
    {(void *)&CO_OD_RAM.traceConfig[30].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[30].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[30].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[30].threshold, 0x8e, 0x4},
};

/*0x2320*/ const CO_OD_entryRecord_t OD_record2320[9] = {
    {(void *)&CO_OD_RAM.traceConfig[31].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].size, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[31].axisNo, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].name, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].color, 0x8e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].map, 0x8e, 0x4},
    {(void *)&CO_OD_RAM.traceConfig[31].format, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].trigger, 0x0e, 0x1},
    {(void *)&CO_OD_RAM.traceConfig[31].threshold, 0x8e, 0x4},
};

/*0x2401*/ const CO_OD_entryRecord_t OD_record2401[7] = {
    {(void *)&CO_OD_RAM.trace[0].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[0].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[0].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[0].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[0].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[0].triggerTime, 0x9e, 0x4},
};

/*0x2402*/ const CO_OD_entryRecord_t OD_record2402[7] = {
    {(void *)&CO_OD_RAM.trace[1].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[1].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[1].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[1].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[1].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[1].triggerTime, 0x9e, 0x4},
};

/*0x2403*/ const CO_OD_entryRecord_t OD_record2403[7] = {
    {(void *)&CO_OD_RAM.trace[2].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[2].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[2].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[2].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[2].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[2].triggerTime, 0x9e, 0x4},
};

/*0x2404*/ const CO_OD_entryRecord_t OD_record2404[7] = {
    {(void *)&CO_OD_RAM.trace[3].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[3].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[3].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[3].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[3].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[3].triggerTime, 0x9e, 0x4},
};

/*0x2405*/ const CO_OD_entryRecord_t OD_record2405[7] = {
    {(void *)&CO_OD_RAM.trace[4].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[4].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[4].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[4].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[4].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[4].triggerTime, 0x9e, 0x4},
};

/*0x2406*/ const CO_OD_entryRecord_t OD_record2406[7] = {
    {(void *)&CO_OD_RAM.trace[5].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[5].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[5].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[5].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[5].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[5].triggerTime, 0x9e, 0x4},
};

/*0x2407*/ const CO_OD_entryRecord_t OD_record2407[7] = {
    {(void *)&CO_OD_RAM.trace[6].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[6].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[6].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[6].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[6].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[6].triggerTime, 0x9e, 0x4},
};

/*0x2408*/ const CO_OD_entryRecord_t OD_record2408[7] = {
    {(void *)&CO_OD_RAM.trace[7].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[7].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[7].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[7].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[7].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[7].triggerTime, 0x9e, 0x4},
};

/*0x2409*/ const CO_OD_entryRecord_t OD_record2409[7] = {
    {(void *)&CO_OD_RAM.trace[8].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[8].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[8].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[8].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[8].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[8].triggerTime, 0x9e, 0x4},
};

/*0x240a*/ const CO_OD_entryRecord_t OD_record240a[7] = {
    {(void *)&CO_OD_RAM.trace[9].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[9].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[9].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[9].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[9].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[9].triggerTime, 0x9e, 0x4},
};

/*0x240b*/ const CO_OD_entryRecord_t OD_record240b[7] = {
    {(void *)&CO_OD_RAM.trace[10].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[10].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[10].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[10].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[10].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[10].triggerTime, 0x9e, 0x4},
};

/*0x240c*/ const CO_OD_entryRecord_t OD_record240c[7] = {
    {(void *)&CO_OD_RAM.trace[11].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[11].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[11].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[11].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[11].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[11].triggerTime, 0x9e, 0x4},
};

/*0x240d*/ const CO_OD_entryRecord_t OD_record240d[7] = {
    {(void *)&CO_OD_RAM.trace[12].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[12].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[12].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[12].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[12].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[12].triggerTime, 0x9e, 0x4},
};

/*0x240e*/ const CO_OD_entryRecord_t OD_record240e[7] = {
    {(void *)&CO_OD_RAM.trace[13].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[13].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[13].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[13].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[13].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[13].triggerTime, 0x9e, 0x4},
};

/*0x240f*/ const CO_OD_entryRecord_t OD_record240f[7] = {
    {(void *)&CO_OD_RAM.trace[14].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[14].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[14].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[14].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[14].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[14].triggerTime, 0x9e, 0x4},
};

/*0x2410*/ const CO_OD_entryRecord_t OD_record2410[7] = {
    {(void *)&CO_OD_RAM.trace[15].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[15].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[15].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[15].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[15].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[15].triggerTime, 0x9e, 0x4},
};

/*0x2411*/ const CO_OD_entryRecord_t OD_record2411[7] = {
    {(void *)&CO_OD_RAM.trace[16].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[16].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[16].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[16].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[16].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[16].triggerTime, 0x9e, 0x4},
};

/*0x2412*/ const CO_OD_entryRecord_t OD_record2412[7] = {
    {(void *)&CO_OD_RAM.trace[17].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[17].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[17].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[17].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[17].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[17].triggerTime, 0x9e, 0x4},
};

/*0x2413*/ const CO_OD_entryRecord_t OD_record2413[7] = {
    {(void *)&CO_OD_RAM.trace[18].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[18].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[18].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[18].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[18].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[18].triggerTime, 0x9e, 0x4},
};

/*0x2414*/ const CO_OD_entryRecord_t OD_record2414[7] = {
    {(void *)&CO_OD_RAM.trace[19].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[19].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[19].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[19].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[19].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[19].triggerTime, 0x9e, 0x4},
};

/*0x2415*/ const CO_OD_entryRecord_t OD_record2415[7] = {
    {(void *)&CO_OD_RAM.trace[20].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[20].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[20].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[20].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[20].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[20].triggerTime, 0x9e, 0x4},
};

/*0x2416*/ const CO_OD_entryRecord_t OD_record2416[7] = {
    {(void *)&CO_OD_RAM.trace[21].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[21].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[21].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[21].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[21].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[21].triggerTime, 0x9e, 0x4},
};

/*0x2417*/ const CO_OD_entryRecord_t OD_record2417[7] = {
    {(void *)&CO_OD_RAM.trace[22].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[22].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[22].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[22].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[22].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[22].triggerTime, 0x9e, 0x4},
};

/*0x2418*/ const CO_OD_entryRecord_t OD_record2418[7] = {
    {(void *)&CO_OD_RAM.trace[23].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[23].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[23].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[23].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[23].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[23].triggerTime, 0x9e, 0x4},
};

/*0x2419*/ const CO_OD_entryRecord_t OD_record2419[7] = {
    {(void *)&CO_OD_RAM.trace[24].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[24].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[24].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[24].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[24].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[24].triggerTime, 0x9e, 0x4},
};

/*0x241a*/ const CO_OD_entryRecord_t OD_record241a[7] = {
    {(void *)&CO_OD_RAM.trace[25].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[25].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[25].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[25].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[25].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[25].triggerTime, 0x9e, 0x4},
};

/*0x241b*/ const CO_OD_entryRecord_t OD_record241b[7] = {
    {(void *)&CO_OD_RAM.trace[26].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[26].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[26].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[26].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[26].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[26].triggerTime, 0x9e, 0x4},
};

/*0x241c*/ const CO_OD_entryRecord_t OD_record241c[7] = {
    {(void *)&CO_OD_RAM.trace[27].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[27].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[27].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[27].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[27].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[27].triggerTime, 0x9e, 0x4},
};

/*0x241d*/ const CO_OD_entryRecord_t OD_record241d[7] = {
    {(void *)&CO_OD_RAM.trace[28].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[28].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[28].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[28].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[28].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[28].triggerTime, 0x9e, 0x4},
};

/*0x241e*/ const CO_OD_entryRecord_t OD_record241e[7] = {
    {(void *)&CO_OD_RAM.trace[29].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[29].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[29].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[29].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[29].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[29].triggerTime, 0x9e, 0x4},
};

/*0x241f*/ const CO_OD_entryRecord_t OD_record241f[7] = {
    {(void *)&CO_OD_RAM.trace[30].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[30].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[30].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[30].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[30].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[30].triggerTime, 0x9e, 0x4},
};

/*0x2420*/ const CO_OD_entryRecord_t OD_record2420[7] = {
    {(void *)&CO_OD_RAM.trace[31].maxSubIndex, 0x06, 0x1},
    {(void *)&CO_OD_RAM.trace[31].size, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[31].value, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[31].min, 0x9e, 0x4},
    {(void *)&CO_OD_RAM.trace[31].max, 0x9e, 0x4},
    {(void *)0, 0x06, 0x0},
    {(void *)&CO_OD_RAM.trace[31].triggerTime, 0x9e, 0x4},
};

/*0x6040*/ const CO_OD_entryRecord_t OD_record6040[7] = {
    {(void *)&CO_OD_RAM.controlWords.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.controlWords.motor1, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor2, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor3, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor4, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor5, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor6, 0xfe, 0x2},
};

/*0x6041*/ const CO_OD_entryRecord_t OD_record6041[7] = {
    {(void *)&CO_OD_RAM.statusWords.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.statusWords.motor1, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor2, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor3, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor4, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor5, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.statusWords.motor6, 0xfe, 0x2},
};

/*0x6064*/ const CO_OD_entryRecord_t OD_record6064[7] = {
    {(void *)&CO_OD_RAM.actualMotorPositions.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor4, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor5, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorPositions.motor6, 0xfe, 0x4},
};

/*0x606c*/ const CO_OD_entryRecord_t OD_record606c[7] = {
    {(void *)&CO_OD_RAM.actualMotorVelocities.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor4, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor5, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorVelocities.motor6, 0xfe, 0x4},
};

/*0x6077*/ const CO_OD_entryRecord_t OD_record6077[5] = {
    {(void *)&CO_OD_RAM.actualMotorTorques.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.actualMotorTorques.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorTorques.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorTorques.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualMotorTorques.motor4, 0xfe, 0x4},
};

/*0x607a*/ const CO_OD_entryRecord_t OD_record607a[7] = {
    {(void *)&CO_OD_RAM.targetMotorPositions.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor4, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor5, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorPositions.motor6, 0xfe, 0x4},
};

/*0x60ff*/ const CO_OD_entryRecord_t OD_record60ff[7] = {
    {(void *)&CO_OD_RAM.targetMotorVelocities.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor4, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor5, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorVelocities.motor6, 0xfe, 0x4},
};

/*0x6071*/ const CO_OD_entryRecord_t OD_record6071[5] = {
    {(void *)&CO_OD_RAM.targetMotorTorques.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.targetMotorTorques.motor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorTorques.motor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorTorques.motor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.targetMotorTorques.motor4, 0xfe, 0x4},
};

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/
extern const CO_OD_entry_t CO_OD[CO_OD_NoOfElements] = {

    {0x1000, 0x00, 0x86, 4, (void *)&CO_OD_RAM.deviceType},
    {0x1001, 0x00, 0x26, 1, (void *)&CO_OD_RAM.errorRegister},
    {0x1002, 0x00, 0xa6, 4, (void *)&CO_OD_RAM.manufacturerStatusRegister},
    {0x1003, 0x08, 0x8e, 4, (void *)&CO_OD_RAM.preDefinedErrorField[0]},
    {0x1005, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.COB_ID_SYNCMessage},
    {0x1006, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.communicationCyclePeriod},
    {0x1007, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.synchronousWindowLength},
    {0x1008, 0x00, 0x86, 11, (void *)&CO_OD_RAM.manufacturerDeviceName},
    {0x1009, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerHardwareVersion},
    {0x100a, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerSoftwareVersion},
    {0x100c, 0x00, 0x85, 2, (void *)&CO_OD_ROM.guardTime},
    {0x100d, 0x00, 0x06, 1, (void *)&CO_OD_RAM.lifeTimeFactor},
    {0x1010, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.storeParameters[0]},
    {0x1011, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.restoreDefaultParameters[0]},
    {0x1012, 0x00, 0x85, 4, (void *)&CO_OD_ROM.COB_ID_TIME},
    {0x1013, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.highResolutionTimeStamp},
    {0x1014, 0x00, 0x86, 4, (void *)&CO_OD_RAM.COB_ID_EMCY},
    {0x1015, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.inhibitTimeEMCY},
    {0x1016, 0x04, 0x8e, 4, (void *)&CO_OD_RAM.consumerHeartbeatTime[0]},
    {0x1017, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.producerHeartbeatTime},
    {0x1018, 0x04, 0x00, 0, (void *)&OD_record1018},
    {0x1019, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.synchronousCounterOverflowValue},
    {0x1029, 0x06, 0x0e, 1, (void *)&CO_OD_RAM.errorBehavior[0]},
    {0x1200, 0x02, 0x00, 0, (void *)&OD_record1200},
    {0x1280, 0x03, 0x00, 0, (void *)&OD_record1280},
    {0x1400, 0x02, 0x00, 0, (void *)&OD_record1400},
    {0x1401, 0x02, 0x00, 0, (void *)&OD_record1401},
    {0x1402, 0x02, 0x00, 0, (void *)&OD_record1402},
    {0x1403, 0x02, 0x00, 0, (void *)&OD_record1403},
    {0x1404, 0x02, 0x00, 0, (void *)&OD_record1404},
    {0x1405, 0x02, 0x00, 0, (void *)&OD_record1405},
    {0x1406, 0x02, 0x00, 0, (void *)&OD_record1406},
    {0x1407, 0x02, 0x00, 0, (void *)&OD_record1407},
    {0x1408, 0x02, 0x00, 0, (void *)&OD_record1408},
    {0x1409, 0x02, 0x00, 0, (void *)&OD_record1409},
    {0x140a, 0x02, 0x00, 0, (void *)&OD_record140a},
    {0x140b, 0x02, 0x00, 0, (void *)&OD_record140b},
    {0x140c, 0x02, 0x00, 0, (void *)&OD_record140c},
    {0x140d, 0x02, 0x00, 0, (void *)&OD_record140d},
    {0x140e, 0x02, 0x00, 0, (void *)&OD_record140e},
    {0x140f, 0x02, 0x00, 0, (void *)&OD_record140f},
    {0x1410, 0x02, 0x00, 0, (void *)&OD_record1410},
    {0x1411, 0x02, 0x00, 0, (void *)&OD_record1411},
    {0x1412, 0x02, 0x00, 0, (void *)&OD_record1412},
    {0x1413, 0x02, 0x00, 0, (void *)&OD_record1413},
    {0x1414, 0x02, 0x00, 0, (void *)&OD_record1414},
    {0x1415, 0x02, 0x00, 0, (void *)&OD_record1415},
    {0x1416, 0x02, 0x00, 0, (void *)&OD_record1416},
    {0x1417, 0x02, 0x00, 0, (void *)&OD_record1417},
    {0x1418, 0x02, 0x00, 0, (void *)&OD_record1418},
    {0x1419, 0x02, 0x00, 0, (void *)&OD_record1419},
    {0x141a, 0x02, 0x00, 0, (void *)&OD_record141a},
    {0x141b, 0x02, 0x00, 0, (void *)&OD_record141b},
    {0x141c, 0x02, 0x00, 0, (void *)&OD_record141c},
    {0x141d, 0x02, 0x00, 0, (void *)&OD_record141d},
    {0x141e, 0x02, 0x00, 0, (void *)&OD_record141e},
    {0x141f, 0x02, 0x00, 0, (void *)&OD_record141f},
    {0x1600, 0x08, 0x00, 0, (void *)&OD_record1600},
    {0x1601, 0x08, 0x00, 0, (void *)&OD_record1601},
    {0x1602, 0x08, 0x00, 0, (void *)&OD_record1602},
    {0x1603, 0x08, 0x00, 0, (void *)&OD_record1603},
    {0x1604, 0x08, 0x00, 0, (void *)&OD_record1604},
    {0x1605, 0x08, 0x00, 0, (void *)&OD_record1605},
    {0x1606, 0x08, 0x00, 0, (void *)&OD_record1606},
    {0x1607, 0x08, 0x00, 0, (void *)&OD_record1607},
    {0x1608, 0x08, 0x00, 0, (void *)&OD_record1608},
    {0x1609, 0x08, 0x00, 0, (void *)&OD_record1609},
    {0x160a, 0x08, 0x00, 0, (void *)&OD_record160a},
    {0x160b, 0x08, 0x00, 0, (void *)&OD_record160b},
    {0x160c, 0x08, 0x00, 0, (void *)&OD_record160c},
    {0x160d, 0x08, 0x00, 0, (void *)&OD_record160d},
    {0x160e, 0x08, 0x00, 0, (void *)&OD_record160e},
    {0x160f, 0x08, 0x00, 0, (void *)&OD_record160f},
    {0x1610, 0x08, 0x00, 0, (void *)&OD_record1610},
    {0x1611, 0x08, 0x00, 0, (void *)&OD_record1611},
    {0x1612, 0x08, 0x00, 0, (void *)&OD_record1612},
    {0x1613, 0x08, 0x00, 0, (void *)&OD_record1613},
    {0x1614, 0x08, 0x00, 0, (void *)&OD_record1614},
    {0x1615, 0x08, 0x00, 0, (void *)&OD_record1615},
    {0x1616, 0x08, 0x00, 0, (void *)&OD_record1616},
    {0x1617, 0x08, 0x00, 0, (void *)&OD_record1617},
    {0x1618, 0x08, 0x00, 0, (void *)&OD_record1618},
    {0x1619, 0x08, 0x00, 0, (void *)&OD_record1619},
    {0x161a, 0x08, 0x00, 0, (void *)&OD_record161a},
    {0x161b, 0x08, 0x00, 0, (void *)&OD_record161b},
    {0x161c, 0x08, 0x00, 0, (void *)&OD_record161c},
    {0x161d, 0x08, 0x00, 0, (void *)&OD_record161d},
    {0x161e, 0x08, 0x00, 0, (void *)&OD_record161e},
    {0x161f, 0x08, 0x00, 0, (void *)&OD_record161f},
    {0x1800, 0x06, 0x00, 0, (void *)&OD_record1800},
    {0x1801, 0x06, 0x00, 0, (void *)&OD_record1801},
    {0x1802, 0x06, 0x00, 0, (void *)&OD_record1802},
    {0x1803, 0x06, 0x00, 0, (void *)&OD_record1803},
    {0x1804, 0x06, 0x00, 0, (void *)&OD_record1804},
    {0x1805, 0x06, 0x00, 0, (void *)&OD_record1805},
    {0x1806, 0x06, 0x00, 0, (void *)&OD_record1806},
    {0x1807, 0x06, 0x00, 0, (void *)&OD_record1807},
    {0x1808, 0x06, 0x00, 0, (void *)&OD_record1808},
    {0x1809, 0x06, 0x00, 0, (void *)&OD_record1809},
    {0x180a, 0x06, 0x00, 0, (void *)&OD_record180a},
    {0x180b, 0x06, 0x00, 0, (void *)&OD_record180b},
    {0x180c, 0x06, 0x00, 0, (void *)&OD_record180c},
    {0x180d, 0x06, 0x00, 0, (void *)&OD_record180d},
    {0x180e, 0x06, 0x00, 0, (void *)&OD_record180e},
    {0x180f, 0x06, 0x00, 0, (void *)&OD_record180f},
    {0x1810, 0x06, 0x00, 0, (void *)&OD_record1810},
    {0x1811, 0x06, 0x00, 0, (void *)&OD_record1811},
    {0x1812, 0x06, 0x00, 0, (void *)&OD_record1812},
    {0x1813, 0x06, 0x00, 0, (void *)&OD_record1813},
    {0x1814, 0x06, 0x00, 0, (void *)&OD_record1814},
    {0x1815, 0x06, 0x00, 0, (void *)&OD_record1815},
    {0x1816, 0x06, 0x00, 0, (void *)&OD_record1816},
    {0x1817, 0x06, 0x00, 0, (void *)&OD_record1817},
    {0x1818, 0x06, 0x00, 0, (void *)&OD_record1818},
    {0x1819, 0x06, 0x00, 0, (void *)&OD_record1819},
    {0x181a, 0x06, 0x00, 0, (void *)&OD_record181a},
    {0x181b, 0x06, 0x00, 0, (void *)&OD_record181b},
    {0x181c, 0x06, 0x00, 0, (void *)&OD_record181c},
    {0x181d, 0x06, 0x00, 0, (void *)&OD_record181d},
    {0x181e, 0x06, 0x00, 0, (void *)&OD_record181e},
    {0x181f, 0x06, 0x00, 0, (void *)&OD_record181f},
    {0x1a00, 0x08, 0x00, 0, (void *)&OD_record1a00},
    {0x1a01, 0x08, 0x00, 0, (void *)&OD_record1a01},
    {0x1a02, 0x08, 0x00, 0, (void *)&OD_record1a02},
    {0x1a03, 0x08, 0x00, 0, (void *)&OD_record1a03},
    {0x1a04, 0x08, 0x00, 0, (void *)&OD_record1a04},
    {0x1a05, 0x08, 0x00, 0, (void *)&OD_record1a05},
    {0x1a06, 0x08, 0x00, 0, (void *)&OD_record1a06},
    {0x1a07, 0x08, 0x00, 0, (void *)&OD_record1a07},
    {0x1a08, 0x08, 0x00, 0, (void *)&OD_record1a08},
    {0x1a09, 0x08, 0x00, 0, (void *)&OD_record1a09},
    {0x1a0a, 0x08, 0x00, 0, (void *)&OD_record1a0a},
    {0x1a0b, 0x08, 0x00, 0, (void *)&OD_record1a0b},
    {0x1a0c, 0x08, 0x00, 0, (void *)&OD_record1a0c},
    {0x1a0d, 0x08, 0x00, 0, (void *)&OD_record1a0d},
    {0x1a0e, 0x08, 0x00, 0, (void *)&OD_record1a0e},
    {0x1a0f, 0x08, 0x00, 0, (void *)&OD_record1a0f},
    {0x1a10, 0x08, 0x00, 0, (void *)&OD_record1a10},
    {0x1a11, 0x08, 0x00, 0, (void *)&OD_record1a11},
    {0x1a12, 0x08, 0x00, 0, (void *)&OD_record1a12},
    {0x1a13, 0x08, 0x00, 0, (void *)&OD_record1a13},
    {0x1a14, 0x08, 0x00, 0, (void *)&OD_record1a14},
    {0x1a15, 0x08, 0x00, 0, (void *)&OD_record1a15},
    {0x1a16, 0x08, 0x00, 0, (void *)&OD_record1a16},
    {0x1a17, 0x08, 0x00, 0, (void *)&OD_record1a17},
    {0x1a18, 0x08, 0x00, 0, (void *)&OD_record1a18},
    {0x1a19, 0x08, 0x00, 0, (void *)&OD_record1a19},
    {0x1a1a, 0x08, 0x00, 0, (void *)&OD_record1a1a},
    {0x1a1b, 0x08, 0x00, 0, (void *)&OD_record1a1b},
    {0x1a1c, 0x08, 0x00, 0, (void *)&OD_record1a1c},
    {0x1a1d, 0x08, 0x00, 0, (void *)&OD_record1a1d},
    {0x1a1e, 0x08, 0x00, 0, (void *)&OD_record1a1e},
    {0x1a1f, 0x08, 0x00, 0, (void *)&OD_record1a1f},
    {0x1f80, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.NMTStartup},
    {0x1f81, 0x7f, 0x8e, 4, (void *)&CO_OD_RAM.slaveAssignment[0]},
    {0x1f82, 0x7f, 0x0e, 1, (void *)&CO_OD_RAM.requestNMT[0]},
    {0x1f89, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.bootTime},
    {0x2100, 0x00, 0xa6, 10, (void *)&CO_OD_RAM.errorStatusBits},
    {0x2101, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.CANNodeID},
    {0x2102, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.CANBitRate},
    {0x2103, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.SYNCCounter},
    {0x2104, 0x00, 0x86, 2, (void *)&CO_OD_RAM.SYNCTime},
    {0x2106, 0x00, 0x86, 4, (void *)&CO_OD_RAM.powerOnCounter},
    {0x2107, 0x05, 0x8e, 2, (void *)&CO_OD_RAM.performance[0]},
    {0x2108, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.temperature[0]},
    {0x2109, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.voltage[0]},
    {0x2110, 0x20, 0x8e, 4, (void *)&CO_OD_RAM.variableInt32[0]},
    {0x2111, 0x10, 0x8e, 4, (void *)&CO_OD_RAM.variableROM_Int32[0]},
    {0x2112, 0x10, 0x8e, 4, (void *)&CO_OD_RAM.variableNV_Int32[0]},
    {0x2120, 0x05, 0x00, 0, (void *)&OD_record2120},
    {0x2130, 0x03, 0x00, 0, (void *)&OD_record2130},
    {0x2209, 0x06, 0x00, 0, (void *)&OD_record2209},
    {0x2301, 0x08, 0x00, 0, (void *)&OD_record2301},
    {0x2302, 0x08, 0x00, 0, (void *)&OD_record2302},
    {0x2303, 0x08, 0x00, 0, (void *)&OD_record2303},
    {0x2304, 0x08, 0x00, 0, (void *)&OD_record2304},
    {0x2305, 0x08, 0x00, 0, (void *)&OD_record2305},
    {0x2306, 0x08, 0x00, 0, (void *)&OD_record2306},
    {0x2307, 0x08, 0x00, 0, (void *)&OD_record2307},
    {0x2308, 0x08, 0x00, 0, (void *)&OD_record2308},
    {0x2309, 0x08, 0x00, 0, (void *)&OD_record2309},
    {0x230a, 0x08, 0x00, 0, (void *)&OD_record230a},
    {0x230b, 0x08, 0x00, 0, (void *)&OD_record230b},
    {0x230c, 0x08, 0x00, 0, (void *)&OD_record230c},
    {0x230d, 0x08, 0x00, 0, (void *)&OD_record230d},
    {0x230e, 0x08, 0x00, 0, (void *)&OD_record230e},
    {0x230f, 0x08, 0x00, 0, (void *)&OD_record230f},
    {0x2310, 0x08, 0x00, 0, (void *)&OD_record2310},
    {0x2311, 0x08, 0x00, 0, (void *)&OD_record2311},
    {0x2312, 0x08, 0x00, 0, (void *)&OD_record2312},
    {0x2313, 0x08, 0x00, 0, (void *)&OD_record2313},
    {0x2314, 0x08, 0x00, 0, (void *)&OD_record2314},
    {0x2315, 0x08, 0x00, 0, (void *)&OD_record2315},
    {0x2316, 0x08, 0x00, 0, (void *)&OD_record2316},
    {0x2317, 0x08, 0x00, 0, (void *)&OD_record2317},
    {0x2318, 0x08, 0x00, 0, (void *)&OD_record2318},
    {0x2319, 0x08, 0x00, 0, (void *)&OD_record2319},
    {0x231a, 0x08, 0x00, 0, (void *)&OD_record231a},
    {0x231b, 0x08, 0x00, 0, (void *)&OD_record231b},
    {0x231c, 0x08, 0x00, 0, (void *)&OD_record231c},
    {0x231d, 0x08, 0x00, 0, (void *)&OD_record231d},
    {0x231e, 0x08, 0x00, 0, (void *)&OD_record231e},
    {0x231f, 0x08, 0x00, 0, (void *)&OD_record231f},
    {0x2320, 0x08, 0x00, 0, (void *)&OD_record2320},
    {0x2400, 0x00, 0x1e, 1, (void *)&CO_OD_RAM.traceEnable},
    {0x2401, 0x06, 0x00, 0, (void *)&OD_record2401},
    {0x2402, 0x06, 0x00, 0, (void *)&OD_record2402},
    {0x2403, 0x06, 0x00, 0, (void *)&OD_record2403},
    {0x2404, 0x06, 0x00, 0, (void *)&OD_record2404},
    {0x2405, 0x06, 0x00, 0, (void *)&OD_record2405},
    {0x2406, 0x06, 0x00, 0, (void *)&OD_record2406},
    {0x2407, 0x06, 0x00, 0, (void *)&OD_record2407},
    {0x2408, 0x06, 0x00, 0, (void *)&OD_record2408},
    {0x2409, 0x06, 0x00, 0, (void *)&OD_record2409},
    {0x240a, 0x06, 0x00, 0, (void *)&OD_record240a},
    {0x240b, 0x06, 0x00, 0, (void *)&OD_record240b},
    {0x240c, 0x06, 0x00, 0, (void *)&OD_record240c},
    {0x240d, 0x06, 0x00, 0, (void *)&OD_record240d},
    {0x240e, 0x06, 0x00, 0, (void *)&OD_record240e},
    {0x240f, 0x06, 0x00, 0, (void *)&OD_record240f},
    {0x2410, 0x06, 0x00, 0, (void *)&OD_record2410},
    {0x2411, 0x06, 0x00, 0, (void *)&OD_record2411},
    {0x2412, 0x06, 0x00, 0, (void *)&OD_record2412},
    {0x2413, 0x06, 0x00, 0, (void *)&OD_record2413},
    {0x2414, 0x06, 0x00, 0, (void *)&OD_record2414},
    {0x2415, 0x06, 0x00, 0, (void *)&OD_record2415},
    {0x2416, 0x06, 0x00, 0, (void *)&OD_record2416},
    {0x2417, 0x06, 0x00, 0, (void *)&OD_record2417},
    {0x2418, 0x06, 0x00, 0, (void *)&OD_record2418},
    {0x2419, 0x06, 0x00, 0, (void *)&OD_record2419},
    {0x241a, 0x06, 0x00, 0, (void *)&OD_record241a},
    {0x241b, 0x06, 0x00, 0, (void *)&OD_record241b},
    {0x241c, 0x06, 0x00, 0, (void *)&OD_record241c},
    {0x241d, 0x06, 0x00, 0, (void *)&OD_record241d},
    {0x241e, 0x06, 0x00, 0, (void *)&OD_record241e},
    {0x241f, 0x06, 0x00, 0, (void *)&OD_record241f},
    {0x2420, 0x06, 0x00, 0, (void *)&OD_record2420},
    {0x6000, 0x08, 0x0e, 1, (void *)&CO_OD_RAM.readInput8Bit[0]},
    {0x6001, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.currentState},
    {0x6002, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.currentMovement},
    {0x6003, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.nextMovement},
    {0x6004, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.goButton},
    {0x6005, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.HB},
    {0x6040, 0x06, 0x00, 0, (void *)&OD_record6040},
    {0x6041, 0x06, 0x00, 0, (void *)&OD_record6041},
    {0x6064, 0x06, 0x00, 0, (void *)&OD_record6064},
    {0x606c, 0x06, 0x00, 0, (void *)&OD_record606c},
    {0x6071, 0x04, 0x00, 0, (void *)&OD_record6071},
    {0x6077, 0x04, 0x00, 0, (void *)&OD_record6077},
    {0x607a, 0x06, 0x00, 0, (void *)&OD_record607a},
    {0x60ff, 0x06, 0x00, 0, (void *)&OD_record60ff},
    {0x6200, 0x08, 0x0e, 1, (void *)&CO_OD_RAM.writeOutput8Bit[0]},
    {0x6401, 0x0c, 0x8e, 2, (void *)&CO_OD_RAM.readAnalogueInput16Bit[0]},
    {0x6411, 0x08, 0x8e, 2, (void *)&CO_OD_RAM.writeAnalogueOutput16Bit[0]},
};
