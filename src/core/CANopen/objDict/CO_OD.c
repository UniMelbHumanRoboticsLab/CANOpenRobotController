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

// Parameters setting the RPDO to off
OD_RPDOCommunicationParameter_t RPDOCommParamOff = {0x2L, 0x80000000L, 0xffL};
OD_RPDOMappingParameter_t RPDOMapParamOff = {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

// Array of pointers to all RPDOs (initialised to off to start with)
OD_RPDOCommunicationParameter_t *OD_RPDOCommunicationParameter[CO_NO_RPDO] = {&RPDOCommParamOff};
OD_RPDOMappingParameter_t *OD_RPDOMappingParameter[CO_NO_RPDO] = {&RPDOMapParamOff};


// OD_record for Entries that are off
/*0x1400*/ CO_OD_entryRecord_t OD_recordRPDOCommOff[3] = {
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

// Parameters setting the TPDO to off
OD_TPDOCommunicationParameter_t TPDOCommParamOff = {0x6L, 0x80000000L, 0xfeL, 0x00, 0x0L, 0x00, 0x0L};
OD_TPDOMappingParameter_t TPDOMapParamOff = {0x0L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L, 0x0000L};

// Array of pointers to all TPDOs (initialised to off to start with)
OD_TPDOCommunicationParameter_t *OD_TPDOCommunicationParameter[CO_NO_TPDO] = {&TPDOCommParamOff};
OD_TPDOMappingParameter_t *OD_TPDOMappingParameter[CO_NO_TPDO] = {&TPDOMapParamOff};

/*0x1800*/ const CO_OD_entryRecord_t OD_recordTPDOCommOff[7] = {
    {(void *)&TPDOCommParamOff.maxSubIndex, 0x06, 0x1},
    {(void *)&TPDOCommParamOff.COB_IDUsedByTPDO, 0x8e, 0x4},
    {(void *)&TPDOCommParamOff.transmissionType, 0x0e, 0x1},
    {(void *)&TPDOCommParamOff.inhibitTime, 0x8e, 0x2},
    {(void *)&TPDOCommParamOff.compatibilityEntry, 0x0e, 0x1},
    {(void *)&TPDOCommParamOff.eventTimer, 0x8e, 0x2},
    {(void *)&TPDOCommParamOff.SYNCStartValue, 0x0e, 0x1},
};

/*0x1a00*/ const CO_OD_entryRecord_t OD_recordTPDOMapOff[9] = {
    {(void *)&TPDOMapParamOff.numberOfMappedObjects, 0x0e, 0x1},
    {(void *)&TPDOMapParamOff.mappedObject1, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject2, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject3, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject4, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject5, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject6, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject7, 0x8e, 0x4},
    {(void *)&TPDOMapParamOff.mappedObject8, 0x8e, 0x4},
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

INTEGER16 junkData =8;

/*Junk data Test*/ const CO_OD_entryRecord_t OD_DummyDataStoreLocation[9] = {
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
    {(void *)&junkData, 0xfe, 0x2},
};

/*******************************************************************************
   OBJECT DICTIONARY
*******************************************************************************/

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


    // Initialise all PDOs to off
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + i, 0x1400 + i, 0x02, 0x00, 0, (void *)&OD_recordRPDOCommOff);
    }
    for (i = 0; i < CO_NO_RPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO + i, 0x1600 + i, 0x08, 0x00, 0, (void *)&OD_recordRPDOMapOff);

    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + i, 0x1800 + i, 0x06, 0x00, 0, (void *)&OD_recordTPDOCommOff);
    }
    for (i = 0; i < CO_NO_TPDO; i = i + 1) {
        CO_OD_set_entry(25 + CO_NO_RPDO * 2 + CO_NO_TPDO + i, 0x1a00 + i, 0x08, 0x00, 0, (void *)&OD_recordTPDOMapOff);
    }

    // Also modify the pointers to the mapping parameters
    for (int i = 0; i < CO_NO_RPDO; i++) {
        OD_RPDOCommunicationParameter[i] = &RPDOCommParamOff;
        OD_RPDOMappingParameter[i] = &RPDOMapParamOff;
    }
    for (int i = 0; i < CO_NO_TPDO; i++) {
        OD_TPDOCommunicationParameter[i] = &TPDOCommParamOff;
        OD_TPDOMappingParameter[i] = &TPDOMapParamOff;
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
    // Extra data stores for RPDO Data
    for (int i = 0; i < CO_NO_RPDO; i++) {
        CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85+i, 0x6000+i, 0x08, 0x00, 0, (void *)&OD_DummyDataStoreLocation);
    }

    // Extra data stores for TPDO data
    for (int i = 0; i < CO_NO_TPDO; i++) {
        CO_OD_set_entry(24 + 2 * CO_NO_RPDO + 3 * CO_NO_TPDO + 85 + i, 0x6000 + CO_NO_RPDO + i, 0x08, 0x00, 0, (void *)&OD_DummyDataStoreLocation);
    }
    return true;
}

// Should return RPDO number
/**
 * @brief Sets up an RPDO. Should need:
 * - Number of Objects to Map and Links to where the object should be mapped to (or a matrix of pointers to variables...) (testRecord?)
 * 
 * @return int RPDO number
 */
int currRPDO = 0;

int CO_setRPDO(OD_RPDOCommunicationParameter_t *RPDOcommPara, OD_RPDOMappingParameter_t *RPDOmapparam, CO_OD_entryRecord_t *RPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *RPDOmapparamEntry) {
    // Should check that the COB-ID is not being used at the moment
    // Could also add a flag which says whether it should be checked or not

    if (currRPDO < CO_NO_RPDO){
        // Iterate through the Mapped Objects to set the parameters
        //This is super hacky and crap.. seriously... why did they set it up in this way? 
        uint32_t *pMap = &RPDOmapparam->mappedObject1; 
        for (int i = 0; i < RPDOmapparam->numberOfMappedObjects; i++) {
            uint32_t map = *pMap;

            // Change it to 0x6000
            *pMap = (0x60000000 + currRPDO*0x10000)| (0x0000FFFF & map);
            pMap++;
        }

        // Change the OD entry
        CO_OD[25 + currRPDO].pData = (void *)RPDOCommEntry;
        CO_OD[25 + CO_NO_RPDO+ currRPDO].pData = (void *)RPDOmapparamEntry;

        // Change the Mapping Parameter Entry
        OD_RPDOCommunicationParameter[currRPDO] = RPDOcommPara;
        OD_RPDOMappingParameter[currRPDO] = RPDOmapparam;

        // Change the relevant OD location
        CO_OD[24 + 2 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85 + currRPDO].pData = (void *)dataStoreRecord;

        // increment counter, but return the original value
        currRPDO = currRPDO +1;
        return currRPDO;
    }
    return -1; // Error  - too many PDOs defined
}

// Should return RPDO number
/**
 * @brief Sets up an TPDO. Should need:
 * - Number of Objects to Map and Links to where the object should be mapped to (or a matrix of pointers to variables...) (testRecord?)
 * 
 * @return int TPDO number
 */
int currTPDO = 0;
int CO_setTPDO(OD_TPDOCommunicationParameter_t *TPDOcommPara, OD_TPDOMappingParameter_t *TPDOmapparam, CO_OD_entryRecord_t *TPDOCommEntry, CO_OD_entryRecord_t *dataStoreRecord, CO_OD_entryRecord_t *TPDOmapparamEntry) {
    // Should check that the COB-ID is not being used at the moment
    // Could also add a flag which says whether it should be checked or not

    if (currTPDO < CO_NO_TPDO) {
        // Iterate through the Mapped Objects to set the parameters
        //This is super hacky and crap.. seriously... why did they set it up in this way?
        uint32_t *pMap = &TPDOmapparam->mappedObject1;
        for (int i = 0; i < TPDOmapparam->numberOfMappedObjects; i++) {
            uint32_t map = *pMap;

            // Change it to 0x6000
            *pMap = (0x60000000 + (CO_NO_RPDO+currTPDO) * 0x10000) | (0x0000FFFF & map);
            pMap++;
        }

        // Change the OD entry
        CO_OD[25 + 2*CO_NO_RPDO +currTPDO].pData = (void *)TPDOCommEntry;
        CO_OD[25 + 2*CO_NO_RPDO + CO_NO_TPDO + currTPDO].pData = (void *)TPDOmapparamEntry;

        // Change the Mapping Parameter Entry
        OD_TPDOCommunicationParameter[currTPDO] = TPDOcommPara;
        OD_TPDOMappingParameter[currTPDO] =TPDOmapparam;

        // Change the relevant OD location
        CO_OD[24 + 3 * CO_NO_RPDO + 2 * CO_NO_TPDO + 85 + currTPDO].pData = (void *)dataStoreRecord;
        // increment counter, but return the original value
        currTPDO = currTPDO + 1;
        return currTPDO;
    }
    return -1;  // Error  - too many PDOs defined
}