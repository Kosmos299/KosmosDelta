/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

// beta25 = 3740 K, R25 = 47 kOhm, Pull-up = 4.7 kOhm, "VISHAY NTCALUG03A473HC"
const temp_entry_t temptable_47[] PROGMEM = {
  { OV(  24), 300 },
  { OV(  26), 295 },
  { OV(  27), 290 },
  { OV(  29), 285 },
  { OV(  31), 280 },
  { OV(  32), 275 },
  { OV(  34), 270 },
  { OV(  37), 265 },
  { OV(  39), 260 },
  { OV(  42), 255 },
  { OV(  44), 250 },
  { OV(  47), 245 },
  { OV(  51), 240 },
  { OV(  54), 235 },
  { OV(  58), 230 },
  { OV(  62), 225 },
  { OV(  67), 220 },
  { OV(  72), 215 },
  { OV(  78), 210 },
  { OV(  84), 205 },
  { OV(  90), 200 },
  { OV(  97), 195 },
  { OV( 105), 190 },
  { OV( 114), 185 },
  { OV( 124), 180 },
  { OV( 134), 175 },
  { OV( 145), 170 },
  { OV( 157), 165 },
  { OV( 171), 160 },
  { OV( 186), 155 },
  { OV( 202), 150 },
  { OV( 220), 145 },
  { OV( 239), 140 },
  { OV( 260), 135 },
  { OV( 282), 130 },
  { OV( 307), 125 },
  { OV( 333), 120 },
  { OV( 361), 115 },
  { OV( 391), 110 },
  { OV( 423), 105 },
  { OV( 456), 100 },
  { OV( 490),  95 },
  { OV( 526),  90 },
  { OV( 563),  85 },
  { OV( 600),  80 },
  { OV( 637),  75 },
  { OV( 674),  70 },
  { OV( 710),  65 },
  { OV( 745),  60 },
  { OV( 778),  55 },
  { OV( 809),  50 },
  { OV( 839),  45 },
  { OV( 865),  40 },
  { OV( 889),  35 },
  { OV( 911),  30 },
  { OV( 930),  25 },
  { OV( 947),  20 },
  { OV( 961),  15 },
  { OV( 973),  10 },
  { OV( 983),   5 },
  { OV( 992),   0 },
  { OV( 998),  -5 },
  { OV(1004), -10 },
  { OV(1008), -15 }
};