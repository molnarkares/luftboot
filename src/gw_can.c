/* extension to DFU fimrware updater
 * that enables gatewaying to CAN nodes
 * Copyright (C) 2013 Karoly Molnar <molnarkares@gmail.com>
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
 */

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/cm3/nvic.h>

#define CAN_REQUEST_ID	0x67D
#define CAN_RESPONSE_ID	0x5FD
#define CAN_BL_BROADCAST_ID	0x1
#define CAN_BL_RESPONSE_ID	0x2
#define CAN_BRD_TIMEOUT	(10*9000)	// 10 ms
#define CAN_GW_TIMEOUT 	(50*9000)	// 50 ms

#define NODE_SECTOR_SIZE	(4*1024)
#define NODE_SECTOR_NUM		8
#define NODE_FLASH_START	(0x00000000)
#define NODE_FLASH_END		(NODE_FLASH_START + (NODE_SECTOR_NUM*NODE_SECTOR_SIZE))
#define NODE_RAM_START		(0x10001000)
#define NODE_ERASE_TIMEOUT  (120*9000)	// 120 ms
#define SYSTICK_TIMEOUT_100MS	900000

typedef struct {
	uint32_t id;
	uint8_t data[8];
	uint32_t length;
	volatile bool received;
} can_msg_t;

can_msg_t * can_received_frame = NULL;

static bool gw_can_sendrec(can_msg_t *tx_msg, can_msg_t *rx_msg_ref, uint32_t timeout,
bool check_id, bool check_len, uint8_t check_data_map);
static bool gw_can_sendrec_w(can_msg_t *tx_msg, can_msg_t *rx_msg_ref, uint32_t timeout,
bool check_id, bool check_len, uint8_t check_data_map);
bool gw_can_erase_sector(uint32_t address);
static void gw_can_init(uint32_t baud);
bool gw_can_bl_request(uint16_t node);
bool gw_can_flash_program(uint32_t address, uint8_t* data, uint16_t len);

void gw_can_init(uint32_t baud) {
    uint32_t brp;
	rcc_peripheral_enable_clock(&RCC_APB1ENR,
	RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
	AFIO_MAPR &= ~AFIO_MAPR_CAN2_REMAP;	//PB12 PB13

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIO_BANK_CAN2_RX, GPIO_MODE_INPUT,
	GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX);
//	gpio_set(GPIO_BANK_CAN2_RX, GPIO_CAN2_RX);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIO_BANK_CAN2_TX, GPIO_MODE_OUTPUT_50_MHZ,
	GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX);

	/* NVIC setup. */
	can_reset(CAN2);


	switch (baud) {
	case 100:
		brp = 20;
		break;
	case 1000:
		brp = 2;
		break;
	default:
		brp = 2;
		break;
	}

	/* 36 / 20 / (10+7+1) = 100 kHz */
	can_init(   CAN2,
                false, /* TTCM: Time triggered comm mode? */
                true, /* ABOM: Automatic bus-off management? */
                false, /* AWUM: Automatic wakeup mode? */
                false, /* NART: No automatic retransmission? */
                false, /* RFLM: Receive FIFO locked mode? */
                false, /* TXFP: Transmit FIFO priority? */
                CAN_BTR_SJW_3TQ,
                CAN_BTR_TS1_10TQ,
                CAN_BTR_TS2_7TQ, brp, /* BRP+1: Baud rate prescaler */
                false, /* loopback mode */
                false); /* silent mode */

	/* CAN filter 0 init. */
	can_filter_id_mask_32bit_init(  CAN1,
	                                14,
                                    CAN_RESPONSE_ID << CAN_RIxR_STID_SHIFT,
                                    CAN_RIxR_STID_MASK << CAN_RIxR_STID_SHIFT,
                                    0,
                                    true);
	/* Enable CAN RX interrupt. */
	nvic_enable_irq(NVIC_CAN2_RX0_IRQ);
	nvic_set_priority(NVIC_CAN2_RX0_IRQ, 1);
}

/* BL request is a two stage operation:
 *
 * 1. Gateway is requesting BL mode by transmitting to the
 * particular node PROG CAN ID on 1Mbps
 * 2. After waiting for the successful transmission the GW changes to 100kbps
 * 3. Then the gateway is changing transmits Device Type Request
 * 4. Expected response is ASCII "LPC1"
 * 5. After that it performs Flash command unlock on the node
 *
 */

bool gw_can_bl_request(uint16_t node) {
	can_msg_t tx_frame, rx_frame;

	/* start on 1Mbps and send commands not the actove nodes */
	gw_can_init(1000);

	/* send PREPARE BL command to all nodes. This shall put all nodes to silent/
	 * listen only except the one that is targeted.
	 */
	tx_frame.id = CAN_BL_BROADCAST_ID;
	//tx_frame.length = 8;
	*(uint32_t*) &tx_frame.data[0] = 0x00000020;
	*(uint16_t*) &tx_frame.data[4] = node;
	*(uint16_t*) &tx_frame.data[6] = 100;

	rx_frame.id = CAN_BL_RESPONSE_ID;
	rx_frame.length = 8;
	*(uint32_t*) &tx_frame.data[0] = 0x00000021;
	*(uint16_t*) &tx_frame.data[4] = node;
	*(uint16_t*) &tx_frame.data[6] = 100;

	if (gw_can_sendrec_w(&tx_frame, &rx_frame, CAN_GW_TIMEOUT, true, true,
			0xff)) {
		uint32_t timeout_val, now;
		/* wait for 10ms to enable time for all nodes to make the transition to 100kbps*/
		timeout_val = systick_get_value();
		do {
			now = systick_get_value();
			if (now > timeout_val) {
				timeout_val += SYSTICK_TIMEOUT_100MS;
			}
		} while ((timeout_val - now) > CAN_BRD_TIMEOUT);
	}

	/* change to 100kbps and talk to the LPCx CANOPEN Boot Loader */
	gw_can_init(100);
	/*
	 * Request from flash loader:
	 * ID = 0x67D Len = 8 Data = 0x40 0x00 0x10 0x00 0x00 0x00 0x00 0x00
	 *
	 */
	tx_frame.id = CAN_REQUEST_ID;
	*(uint32_t*) (&tx_frame.data[0]) = 0x00100040;
	*(uint32_t*) (&tx_frame.data[4]) = 0;

	/* Expected response:
	 * ID = 0x5FD Len = 8 Data = 0x43 0x00 0x10 0x00 0x4C 0x50 0x43 0x31
	 *
	 */
	//rx_frame.id = CAN_RESPONSE_ID;
	*(uint32_t*) (&rx_frame.data[0]) = 0x00100043;
	*(uint32_t*) (&rx_frame.data[4]) = 0x3143504c;

	if (!gw_can_sendrec_w(&tx_frame, &rx_frame, CAN_GW_TIMEOUT, false, true,
			0xff)) {
		return false;
	}
	/* unlock request */
	*(uint32_t*) (&tx_frame.data[0]) = 0x0050002b;
	*(uint32_t*) (&tx_frame.data[4]) = 0x00005a5a;
	*(uint32_t*) (&rx_frame.data[0]) = 0x00500060;
	*(uint32_t*) (&rx_frame.data[4]) = 0;

	if (gw_can_sendrec_w(&tx_frame, &rx_frame, CAN_GW_TIMEOUT, false, true,
			0b00000101)) {
		/* unlock request ACKed */
		return true;
	}

	return false;
}

/* interrupt handler for CAN frame reception.
 */
void can2_rx0_isr(void) {
	/* check if there is any buffer configured */
	if (can_received_frame != NULL) {
		/* avoid overrun */
		if (!can_received_frame->received) {
			bool tmp;
			uint32_t fmi;
			uint8_t length;
			can_receive(CAN2, 0, true, &can_received_frame->id, &tmp, &tmp,
					&fmi, &length, can_received_frame->data);
			can_received_frame->length = length;
			can_received_frame->received = true;
		}
	}
	nvic_clear_pending_irq(NVIC_CAN2_RX0_IRQ);

}

static bool gw_can_sendrec_w(can_msg_t *tx_msg, can_msg_t *rx_msg_ref,
		uint32_t timeout,
		bool check_id, bool check_len, uint8_t check_data_map) {
	bool retval;
	retval = gw_can_sendrec(tx_msg, rx_msg_ref, timeout, check_id, check_len,
			check_data_map);
	return retval;
}

static bool gw_can_sendrec(can_msg_t *tx_msg, can_msg_t *rx_msg_ref,
		uint32_t timeout,
		bool check_id, bool check_len, uint8_t check_data_map) {
	uint32_t timeout_val;

	can_msg_t rx_msg;
	int data_idx;

	//can_disable_irq(CAN2, CAN_IER_FMPIE0);
	can_received_frame = &rx_msg;
	//can_enable_irq(CAN2, CAN_IER_FMPIE0);
	can_received_frame->received = false;
	timeout_val = systick_get_value();

	/* transmit the request and wait for 50 ms to see if the node responds */
	can_transmit(CAN2, tx_msg->id, false, false, /*tx_msg->length*/ 8, tx_msg->data);
	do {
		uint32_t now = systick_get_value();
		if (now > timeout_val) {
			timeout_val += SYSTICK_TIMEOUT_100MS;
		}
		if ((timeout_val - now) > timeout) {
			return false;
		}
	} while (can_received_frame->received == false);

	can_received_frame->received = false;

	//can_disable_irq(CAN2, CAN_IER_FMPIE0);
	can_received_frame = NULL;
	//can_enable_irq(CAN2, CAN_IER_FMPIE0);

	if (check_id && (rx_msg.id != rx_msg_ref->id)) {
		return false;
	}
	if (check_len && (rx_msg.length != rx_msg_ref->length)) {
		return false;
	}
	for (data_idx = 0; data_idx < 8; data_idx++) {
		if (check_data_map & (1 << data_idx)) {
			if (rx_msg.data[data_idx] != rx_msg_ref->data[data_idx]) {
				return false;
			}
		}
	}
	memcpy(rx_msg_ref, &rx_msg, sizeof(can_msg_t));
	return true;
}

bool gw_can_erase_sector(uint32_t address) {
	can_msg_t tx_frame, rx_frame;
	/* sectors are erased one by one */
	uint32_t sector;

	/* sector size of NODE and STM32 is differnt:
	 * STM32 is using 2kBytes while
	 * LPC11C is using 4K sectors
	 * Therefore we accept erase requests for the node
	 * only on the 4K boundaries to avoid erasing of already
	 * programmed data
	 */
	if (address % NODE_SECTOR_SIZE) {
		//mimic successful erase
		return true;
	}
	sector = address / NODE_SECTOR_SIZE;
	tx_frame.id = CAN_REQUEST_ID;
	//tx_frame.length = 8;
	//rx_frame.id = CAN_RESPONSE_ID;
	rx_frame.length = 8;

	/* sector erase is a two step process:
	 * 1. flash prepare command
	 * 2. erase command
	 */
	/* flash prepare write command */
	*(uint32_t*) (&tx_frame.data[0]) = 0x0050202b;
	tx_frame.data[4] = (uint8_t) sector;
	tx_frame.data[5] = (uint8_t) sector;
	*(uint16_t*) (&tx_frame.data[6]) = 0;

	/* prepare expected response */
	*(uint32_t *) (&rx_frame.data[0]) = 0x00502060;

	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* write preparation request not ACKed */
		return false;
	}

	/* flash erase command
	 * commented out lines are kept for documentation but
	 * these are same as the previous request
	 */
	//	tx_frame.data[0] = 0x2b;
	tx_frame.data[1] = 0x30;
	//	tx_frame.data[2] = 0x50;
	//	tx_frame.data[3] = 0x00;

	//	tx_frame.data[4] = sector;
	//	tx_frame.data[5] = sector;
	//	tx_frame.data[6] = 0;
	//	tx_frame.data[7] = 0;

	/* unlock expected response */
	//	rx_frame.data[0] = 0x60;
	rx_frame.data[1] = 0x30;
	//	rx_frame.data[2] = 0x50;
	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        NODE_ERASE_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* erase request not ACKed */
		return false;
	}
	return true;

}
bool gw_can_flash_program(uint32_t address, uint8_t* data, uint16_t len) {
	can_msg_t tx_frame, rx_frame;
	int idx = 0;
	int toggle = 0;
	int data_ctr;
	uint16_t len_save = len;
	// calculate sector id (each sectors are 4Kbytes long
	uint32_t sector = address / NODE_SECTOR_SIZE;
	tx_frame.id = CAN_REQUEST_ID;
	//tx_frame.length = 8;
	//rx_frame.id = CAN_RESPONSE_ID;
	rx_frame.length = 8;

	/* flash prepare write command */
	*(uint32_t*) (&tx_frame.data[0]) = 0x0050202b;

	tx_frame.data[4] = (uint8_t) sector;
	tx_frame.data[5] = (uint8_t) sector;
	*(uint16_t*) (&tx_frame.data[6]) = 0;

	/* expected response */
	*(uint32_t*) (&rx_frame.data[0]) = 0x00502060;

	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* write preparation request not ACKed */
		return false;
	}

	/* RAM write address command */
	*(uint32_t*) (&tx_frame.data[0]) = 0x00501523;
	*(uint32_t*) (&tx_frame.data[4]) = NODE_RAM_START;

	/* expected response */
	*(uint32_t*) (&rx_frame.data[0]) = 0x00501560;

	if (!gw_can_sendrec_w(&tx_frame, &rx_frame, CAN_GW_TIMEOUT, false, true,
			0b00000111)) {
		/* RAM write address request not ACKed */
		return false;
	}

	//	 Segmented download command
	*(uint32_t*) (&tx_frame.data[0]) = 0x011f5021;
	*(uint16_t*) (&tx_frame.data[4]) = len;
	*(uint16_t*) (&tx_frame.data[6]) = 0;

	//expected response
	*(uint32_t*) (&rx_frame.data[0]) = 0x001f5060;

	if (!gw_can_sendrec_w(&tx_frame, &rx_frame, CAN_GW_TIMEOUT, false, true,
			0b00000111)) {
		//segmented write first frame not acked
		return false;
	}
	/* segmented ack is one byte long */
	rx_frame.length = 1;
	while (len > 7) {
		//	 Segmented download command
		tx_frame.data[0] = toggle;
		memcpy(&tx_frame.data[1], &data[idx], 7);

		//expected response
		rx_frame.data[0] = 0x20 | toggle;

		if (!gw_can_sendrec_w(  &tx_frame,
		                        &rx_frame,
		                        CAN_GW_TIMEOUT,
		                        false,
		                        true,
		                        0b00000001))
		{
			//		 RAM write address request not ACKed
			return false;
		}
		len -= 7;
		idx += 7;
		toggle = (toggle != 0) ? 0 : (1 << 4);
	}
	//	 Segmented download command
	tx_frame.data[0] = toggle | ((7 - len) << 1) | 0x01;
	memcpy(&tx_frame.data[1], &data[idx], len);
	//expected response
	rx_frame.data[0] = 0x20 | toggle;

	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000001))
	{
		//		 RAM write address request not ACKed
		return false;
	}

	/* Initiate programming bytes to flash */

	/* Flash Address (DST) */
	*(uint32_t*) (&tx_frame.data[0]) = 0x01505023;
	*(uint32_t*) (&tx_frame.data[4]) = address;

	/* expected response */
	*(uint32_t*) (&rx_frame.data[0]) = 0x00505060;
	rx_frame.length = 8;

	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* Flash address request not ACKed */
		return false;
	}

	/* RAM Address (SRC) */

	//	tx_frame.data[0] = 0x23;
	//	tx_frame.data[1] = 0x50;
	//	tx_frame.data[2] = 0x50;
	tx_frame.data[3] = 0x02;
	*(uint32_t*) (&tx_frame.data[4]) = NODE_RAM_START;

	/* expected response */
	//	rx_frame.data[0] = 0x60;
	//	rx_frame.data[1] = 0x50;
	//	rx_frame.data[2] = 0x50;
	//	rx_frame.length = 8;
	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* RAM  address request not ACKed */
		return false;
	}

	/* length and initiate programming */
	*(uint32_t*) (&tx_frame.data[0]) = 0x0350502b;
	*(uint16_t*) (&tx_frame.data[4]) = len_save;
	*(uint16_t*) (&tx_frame.data[6]) = 0;

	/* expected response */
	*(uint32_t*) (&rx_frame.data[0]) = 0x00505060;
	rx_frame.length = 8;

	if (!gw_can_sendrec_w(  &tx_frame,
	                        &rx_frame,
	                        CAN_GW_TIMEOUT,
	                        false,
	                        true,
	                        0b00000111))
	{
		/* size address request not ACKed */
		return false;
	}

	return true;
}
