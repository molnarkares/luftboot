#!/usr/bin/env python
#
# stm32_mem.py: STM32 memory access using USB DFU class
# Copyright (C) 2011  Black Sphere Technologies
# Written by Gareth McMullin <gareth@blacksphere.co.nz>
# Modified by Felix Ruess <felix.ruess@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function

from time import sleep
import struct
from sys import stdout, exit
from os import path

from optparse import OptionParser

import dfu
import time

from binascii import crc32
from progressbar import ProgressBar, Percentage, ETA, Bar


APP_ADDRESS = 0x08002000
SECTOR_SIZE = 2048

CMD_GETCOMMANDS = 0x00
CMD_SETADDRESSPOINTER = 0x21
CMD_ERASE = 0x41
CMD_CRC = 0x51  # arbritrary value not in the DFU protocol
CMD_FULL_CRC = 0x52  # arbritrary value not in the DFU protocol

valid_manufacturers = ["STMicroelectronics",
                       "Black Sphere Technologies",
                       "TUDelft MavLab. 2012->13",
                       "Transition Robotics Inc."]

# construct a dict that transform error_code into text
err_text = dict((eval("dfu." + name), name) for name in vars(dfu)
                if name.startswith('DFU_STATUS_ERROR'))


# Helper function to print text error from code
def print_error(err_code):
    error = err_text.get(err_code, "Unknown Error")
    print("\nDFU error: {}".format(error))


# helper function that loops waiting for a DFU state
def stm32_wait_for_state(dev, state):
    while True:
        status = dev.get_status()
        if status.bStatus != dfu.DFU_STATUS_OK:
            print_error(status.bStatus)
            return False
        if status.bState == dfu.STATE_DFU_DOWNLOAD_BUSY:
            sleep(status.bwPollTimeout / 1000.0)
        if status.bState == state:
            return True


def stm32_erase(dev, addr):
    erase_cmd = struct.pack("<BL", CMD_ERASE, addr)
    dev.download(0, erase_cmd)
    return stm32_wait_for_state(dev, dfu.STATE_DFU_DOWNLOAD_IDLE)


def stm32_send_next_crc(dev, crc):
    crc_cmd = struct.pack("<BL", CMD_CRC, crc)
    dev.download(0, crc_cmd)
    return stm32_wait_for_state(dev, dfu.STATE_DFU_DOWNLOAD_IDLE)


def stm32_setAddr(dev, addr):
    addr_cmd = struct.pack("<BL", CMD_SETADDRESSPOINTER, addr)
    dev.download(0, addr_cmd)
    return stm32_wait_for_state(dev, dfu.STATE_DFU_DOWNLOAD_IDLE)


def stm32_write(dev, data):
    dev.download(2, data)
    return stm32_wait_for_state(dev, dfu.STATE_DFU_DOWNLOAD_IDLE)


def stm32_full_crc(dev, addr, length, crc):
    crc_cmd = struct.pack("<BLLL", CMD_FULL_CRC, addr, length, crc)
    dev.download(0, crc_cmd)
    return stm32_wait_for_state(dev, dfu.STATE_DFU_DOWNLOAD_IDLE)


def stm32_manifest(dev):
    dev.download(0, "")
    while True:
        try:
            status = dev.get_status()
        except:
            return
        sleep(status.bwPollTimeout / 1000.0)
        if status.bState == dfu.STATE_DFU_MANIFEST:
            break


def get_valid_DFU_dev():
    stm32devs = []
    devs = dfu.finddevs()
    for dev in devs:
        try:
            dfudev = dfu.dfu_device(*dev)
        except:
            if options.verbose:
                print("Could not open dfu device %s id %04x:%04x "
                      "maybe the os driver is claiming it?" %
                      (dev[0].filename, dev[0].idVendor, dev[0].idProduct))
            continue
        try:
            man = dfudev.handle.getString(dfudev.dev.iManufacturer, 30)
            product = dfudev.handle.getString(dfudev.dev.iProduct, 30)
            serial = dfudev.handle.getString(dfudev.dev.iSerialNumber, 40)
        except Exception as e:
            print("whoops... could not get device description.")
            print("exception:", e)
            continue

        if options.verbose:
            print("Found dfu device %s: id %04x:%04x %s - %s - %s" %
                  (dfudev.dev.filename, dfudev.dev.idVendor,
                   dfudev.dev.idProduct, man, product, serial))

        if man in valid_manufacturers:
            if options.product == "any":
                stm32devs.append((dfudev, man, product, serial))
            elif options.product == "Lisa/Lia":
                if "Lisa/M" in product or "Lia" in product or "Fireswarm" in product:
                    stm32devs.append((dfudev, man, product, serial))
    return stm32devs


def wait_for_valid_DFU():
    for i in range(1, 60):
        stm32devs = get_valid_DFU_dev()
        if stm32devs:
            break
        print('.', end="")
        stdout.flush()
        time.sleep(0.5)
    print("")
    if not stm32devs:
        print("No valid DFU devices found!")
        exit(1)
    return stm32devs


def print_copyright():
    print("")
    print("USB Device Firmware Upgrade - Host Utility -- version 1.3")
    print("Copyright (C) 2011  Black Sphere Technologies")
    print("Copyright (C) 2012  Transition Robotics Inc.")
    print("License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>")
    print("")


if __name__ == "__main__":
    usage = "Usage: %prog [options] firmware.bin" + "\n" + "Run %prog --help to list the options."
    parser = OptionParser(usage, version='%prog version 1.3')
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose")
    parser.add_option("--product", type="choice", choices=["any", "Lisa/Lia"],
                      action="store", default="Lisa/Lia",
                      help="only upload to device where idProduct contains PRODUCT\n"
                      "choices: (any, Lisa/Lia), default: Lisa/Lia")
    parser.add_option("--addr", type="int", action="store", dest="addr", default=APP_ADDRESS,
                      help="Upload start address (default: 0x08002000)")
    parser.add_option("-n", "--dry-run", action="store_true",
                      help="Dry run to check which board is found without actually flashing.")
    (options, args) = parser.parse_args()

    if len(args) != 1:
        parser.error("incorrect number of arguments")
    else:
        if path.isfile(args[0]):
            binfile = args[0]
        else:
            parser.error("Binary file " + args[0] + " not found")

    if options.verbose:
        print_copyright()

    #wait a few second until a valid DFU device is found
    stm32devs = wait_for_valid_DFU()

    # use first potential board as target
    target, man, product, serial = stm32devs[0]

    print("Using device: %s - %s - %s" % (
          man, product, serial))

    # if it's a dry run only, don't actually flash, just exit now
    if options.dry_run:
        print("Dry run, done.")
        exit(0)

    try:
        state = target.get_state()
    except:
        print("Failed to read device state! Assuming APP_IDLE")
        state = dfu.STATE_APP_IDLE
    if state == dfu.STATE_APP_IDLE:
        target.detach()
        print("Run again to upgrade firmware.")
        exit(0)

    target.make_idle()

    try:
        bin = open(binfile, "rb").read()
    except:
        print("Could not open binary file.")
        raise

    remainder = len(bin) % SECTOR_SIZE
    print("Reading {}: {} bytes".format(path.basename(binfile),
                                        len(bin)))

    addr = options.addr
    bin_crc = crc32(bin) & 0xffffffff
    bin_base_addr = options.addr
    bin_len = len(bin)
    print("Programming memory from 0x%08X...\r" % addr)
    stdout.flush()
    # writing stats here
    widgets = ['Writing: ', Percentage(), ' ', ETA(), ' ', Bar()]
    pbar = ProgressBar(widgets=widgets, maxval=len(bin), term_width=79).start()
    while bin:
        pbar.update(pbar.maxval - len(bin))
        stdout.flush()
        # erase block and set address
        trynb = 5
        while trynb > 0:
            if not stm32_erase(target, addr):
                trynb -= 1
                continue
            break
        if trynb == 0:
            print("Error, tryed 5 times to erase page at {:08X} without success.")
            exit(-1)

        # compute CRC, send it, and write
        crc = crc32(bin[:SECTOR_SIZE]) & 0xffffffff
        trynb = 5
        while trynb > 0:
            if not stm32_send_next_crc(target, crc):
                trynb -= 1
                continue
            if not stm32_write(target, bin[:SECTOR_SIZE]):
                trynb -= 1
                continue
            break
        if trynb == 0:
            print("Error, tryed 5 times to program page at {:08X} without success.")
            exit(-1)
        bin = bin[SECTOR_SIZE:]
        addr += SECTOR_SIZE

    pbar.update(pbar.maxval - len(bin))
    pbar.finish()
    stdout.flush()

    if stm32_full_crc(target, bin_base_addr, bin_len, bin_crc):
        print("\nFull Program verification (CRC32) is OK")
    else:
        print("\nFull Program verficiation ERROR, CRC32 is invalid")
        print("Programming NOT compete!\n")
        exit(-1)

    # Run the downloaded program
    stm32_manifest(target)
    print("All operations complete!\n")
    exit(0)
