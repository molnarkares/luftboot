/*
 * This file is part of the Paparazzi UAV project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2011-2012 Piotr Esden-Tempski <piotr@esden.net>
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
 */

#ifndef CRC32_ZIP_H
#define CRC32_ZIP_H


/* This function calculates the crc32 the same way as
 * python binascii.crc32
 * */
const uint32_t Polynomial = 0xEDB88320;

static uint32_t crc32_bitwise(const void* data, size_t length, uint32_t previousCrc32){
     uint32_t crc = ~previousCrc32; // same as previousCrc32 ^ 0xFFFFFFFF
     unsigned char* current = (unsigned char*) data;
     while (length--){
         crc ^= *current++;
         uint32_t j;
         for (j = 0; j < 8; j++) {
             if (crc & 1)
                 crc = (crc >> 1) ^ Polynomial;
             else
                 crc = crc >> 1;
         }
     }
     return ~crc; // same as crc ^ 0xFFFFFFFF
}

#endif
