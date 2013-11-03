#!/usr/bin/python
# coding=utf-8

# Base Python File (test_crc.py)
# Created: ven. 01 nov. 2013 18:58:53 CET
# Version: 1.0
#

import array
import binascii
import crcmod
import zlib
import numpy


def stm32_crc(data):
    polynomial = 0x04C11DB7
    crc = 0xFFFFFFFF
    if len(data) % 4 != 0:
        raise ValueError("stm32_crc Error: data is not a multiple of 4")
    a = array.array('I')
    if a.itemsize != 4:  # Depends on implementation
        raise ValueError("stm32_crc Error: itemsize in not 4, change to get size of 4")
    a.fromstring(data)
    #data32 = [struct.unpack_from('<L', data, i) for i in range(0, len(data), 4)]

    for d in a:
        crc ^= d
        for i in range(32):
            if crc & 0x80000000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
    return crc & 0xFFFFFFFF

#p = 0x04C11DB7
#d = 0
#for i in range(32):
#    d <<= 1
#    if p & (1 << i):
#        d |= 1
#print(hex(d))

print("")
data = open("filename.bin", 'rb').read()[:2048]
#data = array.array('B',[2])*4
#a32 = array.array('I')
#a32.fromstring(data)


stm32_crcmod = crcmod.mkCrcFun(0x104C11DB7, initCrc=0xFFFFFFFF, rev=True) #,xorOut=0xFFFFFFFF)
print("slow right = {:08X}".format(stm32_crc(data)))
print("crcmod impl = {:08X}".format(stm32_crcmod(data)))
print("zlib impl = {:08X}".format(zlib.crc32(data,0xFFFFFFFF)))
print("binascii impl = {:08X}".format(binascii.crc32(data,0xFFFFFFFF)))
