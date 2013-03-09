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

#define LED1_PORT		GPIOA
#define LED1_PIN		GPIO10

#define LED2_PORT		GPIOC
#define LED2_PIN		GPIO15

#define USBDETECT_PORT	GPIOA
#define USBDETECT_PIN	GPIO9

#ifndef VERSION
#define VERSION         ""
#endif

#ifndef DEV_SERIAL
#define DEV_SERIAL      "NSERIAL"
#endif

#define APP_ADDRESS	0x08002000
#define SECTOR_SIZE	2048

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

#define FLASH_OBP_RDP 0x1FFFF800
#define FLASH_OBP_WRP10 0x1FFFF808
/* Defines user option register as per Table 5 on Page 55 of RM0008 (STM32 Reference Manual) */
#define FLASH_OBP_DATA0 0x1FFFF804

#define FLASH_OBP_RDP_KEY 0x5aa5

static const char dev_serial[] __attribute__ ((section (".devserial"))) = DEV_SERIAL;

/* We need a special large control buffer for this device: */
u8 usbd_control_buffer[SECTOR_SIZE];

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static void gpio_init(void);
static void gpio_uninit(void);
static bool gpio_force_bootloader(void);
static void led_advance(void);
static void led_set(int id, int on);

static char *get_dev_unique_id(char *serial_no);
static u8 usbdfu_getstatus(u32 *bwPollTimeout);
static void usbdfu_getstatus_complete(usbd_device *device, struct usb_setup_data *req);
static int usbdfu_control_request(usbd_device *device, struct usb_setup_data *req, u8 **buf,
		u16 *len, void (**complete)(usbd_device *device, struct usb_setup_data *req));


#define CAN_REQUEST_ID	0x67D
#define CAN_RESPONSE_ID	0x5FD
#define CAN_GW_TIMEOUT 	(50*9000)	// 5 * 10ms
#define SYSTICK_TIMEOUT_100MS	900000


static void gw_can_init(uint32_t baud);
static bool can_bl_request(uint16_t node);

static struct {
	u8 buf[sizeof(usbd_control_buffer)];
	u16 len;
	u32 addr;
	u16 blocknum;
} prog;

typedef struct {
	u32 gpioport;
	u16 gpios;
}gpio_config_t;

gpio_config_t io_cfg[] = {
			{LED1_PORT, LED1_PIN},
			{LED2_PORT, LED2_PIN}};

typedef struct {
	u32 id;
	u8 data[8];
	u8 length;
	volatile bool received;
} can_rx_t;

can_rx_t can_received_frame;

const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1D50,
	.idProduct = 0x600F,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = SECTOR_SIZE,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char serial_no[24+8];

static const char *usb_strings[] = {
	"Lyorak",
	"BME UAV",
	"Papilot " VERSION,
	serial_no,
	/* This string is used by ST Microelectronics' DfuSe utility */
	"@Internal Flash   /0x08000000/4*002Ka,124*002Kg"
};

static u8 usbdfu_getstatus(u32 *bwPollTimeout)
{
	switch(usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;

	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;

	default:
		return DFU_STATUS_OK;
	}
}

static void usbdfu_getstatus_complete(usbd_device *device, struct usb_setup_data *req)
{
	int i;
	(void)req;

	switch(usbdfu_state) {
	case STATE_DFU_DNBUSY:
		if(prog.blocknum == 0)
		{
			u32 bl_address = *(u32*)(prog.buf+1);
			if (bl_address < 0x8002000)
			{
				u16 node = (u16)(bl_address>>16);
				// we will gateway to CAN nodes
				if(!can_bl_request(node))
				{
					usbd_ep_stall_set(device, 0, 1);
					return;
				}
			}else if (bl_address >= 0x8040000)
			{
				// out of range
				usbd_ep_stall_set(device, 0, 1);
				return;
			}
			flash_unlock();
			switch(prog.buf[0]) {
			case CMD_ERASE:
				flash_erase_page(*(u32*)(prog.buf+1));
			case CMD_SETADDR:
				prog.addr = *(u32*)(prog.buf+1);
			}
		} else {
			u32 baseaddr = prog.addr +
				((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			for(i = 0; i < prog.len; i += 2)
				flash_program_half_word(baseaddr + i,
						*(u16*)(prog.buf+i));
		}
		flash_lock();

		/* We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
		 */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;

	case STATE_DFU_MANIFEST:
		/* USB device must detach, we just reset... */
		scb_reset_system();
		return; /* Will never return */
	default:
		return;
	}
}

static int usbdfu_control_request(usbd_device *device, struct usb_setup_data *req, u8 **buf,
		u16 *len, void (**complete)(usbd_device *device, struct usb_setup_data *req))
{

	if((req->bmRequestType & 0x7F) != 0x21)
		return 0; /* Only accept class request */

	switch(req->bRequest) {
	case DFU_DNLOAD:
		if((len == NULL) || (*len == 0)) {
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return 1;
		} else {
			/* Copy download data for use on GET_STATUS */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return 1;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE */
		if(usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state */
		usbdfu_state = STATE_DFU_IDLE;
		return 1;
	case DFU_UPLOAD:
		/* Upload not supported for now */
		return 0;
	case DFU_GETSTATUS: {
		u32 bwPollTimeout = 0; /* 24-bit integer in DFU class spec */

		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0;	/* iString not used here */
		*len = 6;

		*complete = usbdfu_getstatus_complete;

		return 1;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision */
		*buf[0] = usbdfu_state;
		*len = 1;
		return 1;
	}

	return 0;
}

static void gpio_init(void)
{
	int io_ctr;
	rcc_peripheral_enable_clock(&RCC_AHBENR, RCC_AHBENR_OTGFSEN);
	/* Enable GPIOA, GPIOB, GPIOC, and AFIO clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
						  RCC_APB2ENR_IOPBEN |
						  RCC_APB2ENR_IOPCEN |
						  RCC_APB2ENR_AFIOEN);
	/* LED1 */
	/* Set GPIO10 (in GPIO port A) to 'output push-pull'. */
	for(io_ctr = 0; io_ctr <(sizeof(io_cfg)/sizeof(io_cfg[0]));io_ctr++)
	{
		gpio_set_mode(io_cfg[io_ctr].gpioport, GPIO_MODE_OUTPUT_50_MHZ,
				GPIO_CNF_OUTPUT_PUSHPULL, io_cfg[io_ctr].gpios);
		gpio_set(io_cfg[io_ctr].gpioport,io_cfg[io_ctr].gpios);
	}

	/* USB detect pin */
	gpio_clear(USBDETECT_PORT, USBDETECT_PIN);
	gpio_set_mode(USBDETECT_PORT, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_PULL_UPDOWN, USBDETECT_PIN);
	gpio_clear(USBDETECT_PORT, USBDETECT_PIN);

}

static void gpio_uninit(void)
{
	int io_ctr;
	/* Enable GPIOA, GPIOB, GPIOC, and AFIO clocks. */
	for(io_ctr = 0; io_ctr <(sizeof(io_cfg)/sizeof(io_cfg[0]));io_ctr++)
	{
		gpio_set_mode(io_cfg[io_ctr].gpioport, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_FLOAT, io_cfg[io_ctr].gpios);
	}
	/* USB detect */
	gpio_set_mode(USBDETECT_PORT, GPIO_MODE_INPUT,
				GPIO_CNF_INPUT_FLOAT, USBDETECT_PIN);

	rcc_peripheral_disable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPAEN |
						  RCC_APB2ENR_IOPBEN |
						  RCC_APB2ENR_IOPCEN |
						  RCC_APB2ENR_AFIOEN);
}


static void led_set(int id, int on)
{
	if (on) {
		gpio_clear(io_cfg[id].gpioport,io_cfg[id].gpios);
	} else {
		gpio_set(io_cfg[id].gpioport,io_cfg[id].gpios);
	}
}

static void led_advance(void)
{
	static int state = 0;
	led_set(0,(state & 0x8));
	state++;
}

static bool gpio_force_bootloader(void)
{
	return gpio_get(USBDETECT_PORT, USBDETECT_PIN);
}

int main(void)
{

	gpio_init();

	/* Check if the application is valid. */
	if ((*(volatile u32 *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {
		if(!gpio_force_bootloader())
		{
			gpio_uninit();
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;

			/* Initialise master stack pointer. */
//			asm volatile("msr msp, %0"::"g"
//					     (*(volatile u32 *)APP_ADDRESS));
			// TODO possibly it is not needed

			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
	    }
	}

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gw_can_init(100);
	//can_transmit(CAN2,0x67d,false,false,8,txdata);

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(SYSTICK_TIMEOUT_100MS);
	systick_interrupt_enable();
	systick_counter_enable();

	get_dev_unique_id(serial_no);

	usbd_device *device = usbd_init(&stm32f107_usb_driver, &dev, &config, usb_strings,4);

	usbd_set_control_buffer_size(device, sizeof(usbd_control_buffer));
	usbd_register_control_callback( device,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
	if (can_bl_request(0))
	{
		led_set(1,1);
	}
	while (1)
	{
		usbd_poll(device);
	}
}

static char *get_dev_unique_id(char *s)
{
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	int i;

	for(i = 0; i < 7; i++) {
		s[i] = dev_serial[i];
	}
	s[i] = ' ';

	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 24; i+=2) {
		s[i+8] = ((*unique_id >> 4) & 0xF) + '0';
		s[i+8+1] = (*unique_id++ & 0xF) + '0';
	}
	for(i = 0; i < 24; i++)
		if(s[i+8] > '9')
			s[i+8] += 'A' - '9' - 1;

	return s;
}

void sys_tick_handler()
{
	led_advance();
}

static void gw_can_init(uint32_t baud)
{
	/* Enable peripheral clocks. */
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_AFIOEN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
	//TODO: these two above are possibly already enabled
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN2EN);
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_CAN1EN);
	AFIO_MAPR &= ~AFIO_MAPR_CAN2_REMAP;	//PB12 PB13

	/* Configure CAN pin: RX (input pull-up). */
	gpio_set_mode(GPIO_BANK_CAN2_RX, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX);
	gpio_set(GPIO_BANK_CAN2_RX, GPIO_CAN2_RX);

	/* Configure CAN pin: TX. */
	gpio_set_mode(GPIO_BANK_CAN2_TX, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX);

	/* NVIC setup. */
	can_reset(CAN2);

	u32 brp;

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
	can_init(CAN2,
			   false,           /* TTCM: Time triggered comm mode? */
			   true,            /* ABOM: Automatic bus-off management? */
			   false,           /* AWUM: Automatic wakeup mode? */
			   false,           /* NART: No automatic retransmission? */
			   false,           /* RFLM: Receive FIFO locked mode? */
			   false,           /* TXFP: Transmit FIFO priority? */
			   CAN_BTR_SJW_3TQ,
			   CAN_BTR_TS1_10TQ,
			   CAN_BTR_TS2_7TQ,
			   brp,               /* BRP+1: Baud rate prescaler */
			   false,           /* loopback mode */
			   false);          /* silent mode */

	  /* CAN filter 0 init. */
//	can_filter_id_mask_32bit_init(CAN1,		/* it is always for CAN1 only */
//	                                14,     /* Filter ID , first for CAN2 */
//	                                0,     /* CAN ID */
//	                                0,     /* CAN ID mask */
//	                                0,     /* FIFO assignment (here: FIFO0) */
//	                                true); /* Enable the filter. */
	can_filter_id_mask_32bit_init(CAN1,14,CAN_RESPONSE_ID,0,0,true);
	/* Enable CAN RX interrupt. */
	/*nvic_enable_irq(NVIC_CAN2_RX0_IRQ);
	nvic_set_priority(NVIC_CAN2_RX0_IRQ, 1);
    can_enable_irq(CAN2, CAN_IER_FMPIE0);*/
}

/* BL request is a two stage operation:
 *
 * 1. Gateway is requesting BL mode by transmitting to the
 * particular node PROG CAN ID on 1Mbps
 * 2. After waiting for the successful transmission the GW changes to 100kbps
 * 3. Then the gateway is changing transmits Device Type Request
 * 4. Expected response is ASCII "LPC1"
 *
 */

static bool can_bl_request(uint16_t node)
{
	u32 id, fmi;
	bool ext, rtr,received;
	u8 length, data[8];
	u8 candata[8];
	u32 timeout_val;

	// temp
	/*
	 * Request from flash loader:
	 * ID = 0x67D Len = 8 Data = 0x40 0x00 0x10 0x00 0x00 0x00 0x00 0x00
	 *
	 * Expected response:
	 * ID = 0x5FD Len = 8 Data = 0x43 0x00 0x10 0x00 0x4C 0x50 0x43 0x31
	 * */

	candata[0] = 0x40;
	candata[1] = 0;
	candata[2] = 0x10;
	candata[3] = 0;
	candata[4] = 0;
	candata[5] = 0;
	candata[6] = 0;
	candata[7] = 0;

	can_received_frame.received = false;
	timeout_val = systick_get_value();
	can_transmit(CAN2,CAN_REQUEST_ID,false,false,8,candata);
	led_set(1,1);
	do
	{
		u32 now =systick_get_value();
		if( now > timeout_val)
		{
			timeout_val += SYSTICK_TIMEOUT_100MS;
		}
		if ((timeout_val - now) > CAN_GW_TIMEOUT)
		{
			led_set(1,0);
			return false;
		}
		can_receive(CAN2, 0, true, &can_received_frame.id, &ext, &rtr, &fmi,
							&can_received_frame.length, can_received_frame.data);
	}while (can_received_frame.length != 8);
	//}while(can_received_frame.received == false);
	led_set(1,0);
	can_received_frame.received = false;

	candata[0] = 0x43;
	candata[1] = 0x00;
	candata[2] = 0x10;
	candata[3] = 0x00;
	candata[4] = 'L';
	candata[5] = 'P';
	candata[6] = 'C';
	candata[7] = '1';
	if((can_received_frame.id == CAN_RESPONSE_ID) &&
			(can_received_frame.length == 8))
	{
		if(!memcmp(can_received_frame.data,candata,8))
		{
			return true;
		}
	}
	return false;
}


void can2_rx0_isr(void)
{
	bool ext,rtr;
	u32 fmi;
	/* avoid overrun */
	if(!can_received_frame.received)
	{
		can_receive(CAN2, 0, true, &can_received_frame.id, &ext, &rtr, &fmi,
					&can_received_frame.length, can_received_frame.data);
		can_received_frame.received = true;
	}
}
