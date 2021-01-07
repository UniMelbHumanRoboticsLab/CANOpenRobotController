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
    /*7001*/ {0x4L, 0x0000, 0x0000, 0x0000, 0x0000},
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


/*0x1400*/ CO_OD_entryRecord_t OD_record1400[3] = {
    {(void *)&(OD_RPDOCommunicationParameter[0]->maxSubIndex), 0x06, 0x1},
    {(void *)&(OD_RPDOCommunicationParameter[0]->COB_IDUsedByRPDO), 0x8e, 0x4},
    {(void *)&(OD_RPDOCommunicationParameter[0]->transmissionType), 0x0e, 0x1},
};


OD_RPDOCommunicationParameter_t RPDOCommParamOff = {0x2L, 0x80000000L, 0xffL};
OD_RPDOMappingParameter_t RPDOMapParamOff = {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

OD_RPDOCommunicationParameter_t *OD_RPDOCommunicationParameter[CO_NO_RPDO] = {&RPDOCommParamOff};
OD_RPDOMappingParameter_t *OD_RPDOMappingParameter[CO_NO_RPDO] = {&RPDOMapParamOff};

/*0x1400*/ CO_OD_entryRecord_t OD_recordRPDOCommff[3] = {
    {(void *)&(RPDOCommParamOff.maxSubIndex), 0x06, 0x1},
    {(void *)&(RPDOCommParamOff.COB_IDUsedByRPDO), 0x8e, 0x4},
    {(void *)&(RPDOCommParamOff.transmissionType), 0x0e, 0x1},
};

/*0x1600*/ CO_OD_entryRecord_t OD_recordRPDOMapOff[9] = {
    {(void *)&RPDOMapParamOff.numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&RPDOMapParamOff.mappedObject1, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject2, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject3, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject4, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject5, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject6, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject7, 0x8e, 0x4},
    {(void *)&RPDOMapParamOff.mappedObject8, 0x8e, 0x4},
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

/*0x6040*/ CO_OD_entryRecord_t OD_record6040[7] = {
    {(void *)&CO_OD_RAM.controlWords.numberOfMotors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.controlWords.motor1, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor2, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor3, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor4, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor5, 0xfe, 0x2},
    {(void *)&CO_OD_RAM.controlWords.motor6, 0xfe, 0x2},
};

/*0x6041*/ CO_OD_entryRecord_t OD_record6041[7] = {
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

/*0x7001*/ const CO_OD_entryRecord_t OD_record7001[5] = {
    {(void *)&CO_OD_RAM.actualSensorForces.numberOfSensors, 0x06, 0x1},
    {(void *)&CO_OD_RAM.actualSensorForces.sensor1, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualSensorForces.sensor2, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualSensorForces.sensor3, 0xfe, 0x4},
    {(void *)&CO_OD_RAM.actualSensorForces.sensor4, 0xfe, 0x4},
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
    {0x7001, 0x04, 0x00, 1, (void *)&OD_record7001},
    {0x6200, 0x08, 0x0e, 1, (void *)&CO_OD_RAM.writeOutput8Bit[0]},
    {0x6401, 0x0c, 0x8e, 2, (void *)&CO_OD_RAM.readAnalogueInput16Bit[0]},
    {0x6411, 0x08, 0x8e, 2, (void *)&CO_OD_RAM.writeAnalogueOutput16Bit[0]},
};

CO_OD_entry_t CO_OD[CO_OD_NoOfElements];

bool_t CO_OD_set_entry(uint16_t element_, uint16_t index_, uint8_t maxSubIndex_, uint16_t attribute_, uint16_t length_, void *pData_) {
    CO_OD[element_].index = index_;
    CO_OD[element_].maxSubIndex = maxSubIndex_;
    CO_OD[element_].attribute = attribute_;
    CO_OD[element_].length = length_;
    CO_OD[element_].pData = pData_;
    return true;
}

bool_t CO_configure(void) {
    int i;
    CO_OD_set_entry(0, 0x1000, 0x00, 0x86, 4, (void *)&CO_OD_RAM.deviceType);
    CO_OD_set_entry(1, 0x1001, 0x00, 0x26, 1, (void *)&CO_OD_RAM.errorRegister);
    CO_OD_set_entry(2, 0x1002, 0x00, 0xa6, 4, (void *)&CO_OD_RAM.manufacturerStatusRegister);
    CO_OD_set_entry(3, 0x1003, 0x08, 0x8e, 4, (void *)&CO_OD_RAM.preDefinedErrorField[0]);
    CO_OD_set_entry(4, 0x1005, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.COB_ID_SYNCMessage);
    CO_OD_set_entry(5, 0x1006, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.communicationCyclePeriod);
    CO_OD_set_entry(6, 0x1007, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.synchronousWindowLength);
    CO_OD_set_entry(7, 0x1008, 0x00, 0x86, 11, (void *)&CO_OD_RAM.manufacturerDeviceName);
    CO_OD_set_entry(8, 0x1009, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerHardwareVersion);
    CO_OD_set_entry(9, 0x100a, 0x00, 0x86, 4, (void *)&CO_OD_RAM.manufacturerSoftwareVersion);
    CO_OD_set_entry(10, 0x100c, 0x00, 0x85, 2, (void *)&CO_OD_ROM.guardTime);
    CO_OD_set_entry(11, 0x100d, 0x00, 0x06, 1, (void *)&CO_OD_RAM.lifeTimeFactor);
    CO_OD_set_entry(12, 0x1010, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.storeParameters[0]);
    CO_OD_set_entry(13, 0x1011, 0x01, 0x8e, 4, (void *)&CO_OD_RAM.restoreDefaultParameters[0]);
    CO_OD_set_entry(14, 0x1012, 0x00, 0x85, 4, (void *)&CO_OD_ROM.COB_ID_TIME);
    CO_OD_set_entry(15, 0x1013, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.highResolutionTimeStamp);
    CO_OD_set_entry(16, 0x1014, 0x00, 0x86, 4, (void *)&CO_OD_RAM.COB_ID_EMCY);
    CO_OD_set_entry(17, 0x1015, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.inhibitTimeEMCY);
    CO_OD_set_entry(18, 0x1016, 0x04, 0x8e, 4, (void *)&CO_OD_RAM.consumerHeartbeatTime[0]);
    CO_OD_set_entry(19, 0x1017, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.producerHeartbeatTime);
    CO_OD_set_entry(20, 0x1018, 0x04, 0x00, 0, (void *)&OD_record1018);
    CO_OD_set_entry(21, 0x1019, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.synchronousCounterOverflowValue);
    CO_OD_set_entry(22, 0x1029, 0x06, 0x0e, 1, (void *)&CO_OD_RAM.errorBehavior[0]);
    CO_OD_set_entry(23, 0x1200, 0x02, 0x00, 0, (void *)&OD_record1200);
    CO_OD_set_entry(24, 0x1280, 0x03, 0x00, 0, (void *)&OD_record1280);
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + i, 0x1400 + i, 0x02, 0x00, 0, (void *)&OD_recordRPDOCommff);
    }
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO + i, 0x1600 + i, 0x08, 0x00, 0, (void *)&OD_recordRPDOMapOff);

    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + i, 0x1800 + i, 0x06, 0x00, 0, (void *)&OD_record1800);
    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + CO_NO_TPDO + i, 0x1a00 + i, 0x08, 0x00, 0, (void *)&OD_record1a00);
    }
    // PDOs go here
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 1, 0x1f80, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.NMTStartup);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 2, 0x1f81, 0x7f, 0x8e, 4, (void *)&CO_OD_RAM.slaveAssignment[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 3, 0x1f82, 0x7f, 0x0e, 1, (void *)&CO_OD_RAM.requestNMT[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 4, 0x1f89, 0x00, 0x8e, 4, (void *)&CO_OD_RAM.bootTime);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 5, 0x2100, 0x00, 0xa6, 10, (void *)&CO_OD_RAM.errorStatusBits);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 6, 0x2101, 0x00, 0x0e, 1, (void *)&CO_OD_RAM.CANNodeID);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 7, 0x2102, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.CANBitRate);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 8, 0x2103, 0x00, 0x8e, 2, (void *)&CO_OD_RAM.SYNCCounter);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 9, 0x2104, 0x00, 0x86, 2, (void *)&CO_OD_RAM.SYNCTime);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 10, 0x2106, 0x00, 0x86, 4, (void *)&CO_OD_RAM.powerOnCounter);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 11, 0x2107, 0x05, 0x8e, 2, (void *)&CO_OD_RAM.performance[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 12, 0x2108, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.temperature[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 13, 0x2109, 0x01, 0x8e, 2, (void *)&CO_OD_RAM.voltage[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 14, 0x2110, 0x20, 0x8e, 4, (void *)&CO_OD_RAM.variableInt32[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 15, 0x2111, 0x10, 0x8e, 4, (void *)&CO_OD_RAM.variableROM_Int32[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 16, 0x2112, 0x10, 0x8e, 4, (void *)&CO_OD_RAM.variableNV_Int32[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 17, 0x2120, 0x05, 0x00, 0, (void *)&OD_record2120);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 18, 0x2130, 0x03, 0x00, 0, (void *)&OD_record2130);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 19, 0x2209, 0x06, 0x00, 0, (void *)&OD_record2209);
    // Trace Config - might delete
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 20, 0x2301, 0x08, 0x00, 0, (void *)&OD_record2301);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 21, 0x2302, 0x08, 0x00, 0, (void *)&OD_record2302);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 22, 0x2303, 0x08, 0x00, 0, (void *)&OD_record2303);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 23, 0x2304, 0x08, 0x00, 0, (void *)&OD_record2304);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 24, 0x2305, 0x08, 0x00, 0, (void *)&OD_record2305);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 25, 0x2306, 0x08, 0x00, 0, (void *)&OD_record2306);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 26, 0x2307, 0x08, 0x00, 0, (void *)&OD_record2307);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 27, 0x2308, 0x08, 0x00, 0, (void *)&OD_record2308);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 28, 0x2309, 0x08, 0x00, 0, (void *)&OD_record2309);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 29, 0x230a, 0x08, 0x00, 0, (void *)&OD_record230a);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 30, 0x230b, 0x08, 0x00, 0, (void *)&OD_record230b);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 31, 0x230c, 0x08, 0x00, 0, (void *)&OD_record230c);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 32, 0x230d, 0x08, 0x00, 0, (void *)&OD_record230d);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 33, 0x230e, 0x08, 0x00, 0, (void *)&OD_record230e);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 34, 0x230f, 0x08, 0x00, 0, (void *)&OD_record230f);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 35, 0x2310, 0x08, 0x00, 0, (void *)&OD_record2310);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 36, 0x2311, 0x08, 0x00, 0, (void *)&OD_record2311);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 37, 0x2312, 0x08, 0x00, 0, (void *)&OD_record2312);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 38, 0x2313, 0x08, 0x00, 0, (void *)&OD_record2313);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 39, 0x2314, 0x08, 0x00, 0, (void *)&OD_record2314);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 40, 0x2315, 0x08, 0x00, 0, (void *)&OD_record2315);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 41, 0x2316, 0x08, 0x00, 0, (void *)&OD_record2316);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 42, 0x2317, 0x08, 0x00, 0, (void *)&OD_record2317);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 43, 0x2318, 0x08, 0x00, 0, (void *)&OD_record2318);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 44, 0x2319, 0x08, 0x00, 0, (void *)&OD_record2319);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 45, 0x231a, 0x08, 0x00, 0, (void *)&OD_record231a);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 46, 0x231b, 0x08, 0x00, 0, (void *)&OD_record231b);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 47, 0x231c, 0x08, 0x00, 0, (void *)&OD_record231c);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 48, 0x231d, 0x08, 0x00, 0, (void *)&OD_record231d);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 49, 0x231e, 0x08, 0x00, 0, (void *)&OD_record231e);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 50, 0x231f, 0x08, 0x00, 0, (void *)&OD_record231f);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 51, 0x2320, 0x08, 0x00, 0, (void *)&OD_record2320);
    // Trace Enable
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 52, 0x2400, 0x00, 0x1e, 1, (void *)&CO_OD_RAM.traceEnable);
    // Trace
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 53, 0x2401, 0x06, 0x00, 0, (void *)&OD_record2401);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 54, 0x2402, 0x06, 0x00, 0, (void *)&OD_record2402);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 55, 0x2403, 0x06, 0x00, 0, (void *)&OD_record2403);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 56, 0x2404, 0x06, 0x00, 0, (void *)&OD_record2404);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 57, 0x2405, 0x06, 0x00, 0, (void *)&OD_record2405);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 58, 0x2406, 0x06, 0x00, 0, (void *)&OD_record2406);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 59, 0x2407, 0x06, 0x00, 0, (void *)&OD_record2407);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 60, 0x2408, 0x06, 0x00, 0, (void *)&OD_record2408);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 61, 0x2409, 0x06, 0x00, 0, (void *)&OD_record2409);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 62, 0x240a, 0x06, 0x00, 0, (void *)&OD_record240a);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 63, 0x240b, 0x06, 0x00, 0, (void *)&OD_record240b);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 64, 0x240c, 0x06, 0x00, 0, (void *)&OD_record240c);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 65, 0x240d, 0x06, 0x00, 0, (void *)&OD_record240d);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 66, 0x240e, 0x06, 0x00, 0, (void *)&OD_record240e);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 67, 0x240f, 0x06, 0x00, 0, (void *)&OD_record240f);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 68, 0x2410, 0x06, 0x00, 0, (void *)&OD_record2410);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 69, 0x2411, 0x06, 0x00, 0, (void *)&OD_record2411);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 70, 0x2412, 0x06, 0x00, 0, (void *)&OD_record2412);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 71, 0x2413, 0x06, 0x00, 0, (void *)&OD_record2413);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 72, 0x2414, 0x06, 0x00, 0, (void *)&OD_record2414);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 73, 0x2415, 0x06, 0x00, 0, (void *)&OD_record2415);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 74, 0x2416, 0x06, 0x00, 0, (void *)&OD_record2416);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 75, 0x2417, 0x06, 0x00, 0, (void *)&OD_record2417);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 76, 0x2418, 0x06, 0x00, 0, (void *)&OD_record2418);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 77, 0x2419, 0x06, 0x00, 0, (void *)&OD_record2419);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 78, 0x241a, 0x06, 0x00, 0, (void *)&OD_record241a);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 79, 0x241b, 0x06, 0x00, 0, (void *)&OD_record241b);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 80, 0x241c, 0x06, 0x00, 0, (void *)&OD_record241c);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 81, 0x241d, 0x06, 0x00, 0, (void *)&OD_record241d);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 82, 0x241e, 0x06, 0x00, 0, (void *)&OD_record241e);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 83, 0x241f, 0x06, 0x00, 0, (void *)&OD_record241f);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 84, 0x2420, 0x06, 0x00, 0, (void *)&OD_record2420);
    // Extra data stores
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85, 0x6000, 0x08, 0x0e, 1, (void *)&CO_OD_RAM.readInput8Bit[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 86, 0x6001, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.currentState);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 87, 0x6002, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.currentMovement);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 88, 0x6003, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.nextMovement);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 89, 0x6004, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.goButton);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 90, 0x6005, 0x00, 0xfe, 2, (void *)&CO_OD_RAM.HB);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 91, 0x6040, 0x06, 0x00, 0, (void *)&OD_record6040);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 92, 0x6041, 0x06, 0x00, 0, (void *)&OD_record6041);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 93, 0x6064, 0x06, 0x00, 0, (void *)&OD_record6064);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 94, 0x606c, 0x06, 0x00, 0, (void *)&OD_record606c);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 95, 0x6071, 0x04, 0x00, 0, (void *)&OD_record6071);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 96, 0x6077, 0x04, 0x00, 0, (void *)&OD_record6077);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 97, 0x607a, 0x06, 0x00, 0, (void *)&OD_record607a);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 98, 0x60ff, 0x06, 0x00, 0, (void *)&OD_record60ff);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 99, 0x7001, 0x04, 0x00, 1, (void *)&OD_record7001);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 100, 0x6200, 0x08, 0x0e, 1, (void *)&CO_OD_RAM.writeOutput8Bit[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 101, 0x6401, 0x0c, 0x8e, 2, (void *)&CO_OD_RAM.readAnalogueInput16Bit[0]);
    CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 102, 0x6411, 0x08, 0x8e, 2, (void *)&CO_OD_RAM.writeAnalogueOutput16Bit[0]);

    return true;
}