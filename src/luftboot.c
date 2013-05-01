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

#define APP_ADDRESS		0x08002000
#define APP_END_ADDRESS	0x08040000
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
void led_set(int id, int on);

static char *get_dev_unique_id(char *serial_no);
static u8 usbdfu_getstatus(u32 *bwPollTimeout);
static void usbdfu_getstatus_complete(usbd_device *device, struct usb_setup_data *req);
static int usbdfu_control_request(usbd_device *device, struct usb_setup_data *req, u8 **buf,
		u16 *len, void (**complete)(usbd_device *device, struct usb_setup_data *req));


#define SYSTICK_TIMEOUT_100MS	900000

extern bool gw_can_erase_sector(u32 address);
extern void gw_can_init(uint32_t baud);
extern bool gw_can_bl_request(uint16_t node);
extern bool gw_can_flash_program(u32 address, u8* data, u16 len);


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
		break;

	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		usbdfu_state = STATE_DFU_MANIFEST;
		break;

	default:
		break;
	}
	return DFU_STATUS_OK;
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
			if (bl_address < APP_ADDRESS)
			{
				u16 node = (u16)(bl_address>>16);
				// we will gateway to CAN nodes
				if(!gw_can_bl_request(node))
				{
					usbd_ep_stall_set(device, 0, 1);
					break;
				}
			}else if (bl_address >= APP_END_ADDRESS)
			{
				// out of range
				usbd_ep_stall_set(device, 0, 1);
				break;
			}
			switch(prog.buf[0]) {
			case CMD_ERASE:
				if (bl_address < APP_ADDRESS)
				{
					if(!gw_can_erase_sector(bl_address))
					{
						usbd_ep_stall_set(device, 0, 1);
					}
				}else
				{
					flash_unlock();
					flash_erase_page(bl_address);
					flash_lock();
				}
				break;
			case CMD_SETADDR:
				prog.addr = bl_address;
				break;
			}
		} else
		{
			u32 baseaddr = prog.addr + ((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			if(baseaddr >= APP_ADDRESS) // program stm32
			{
				flash_unlock();
				for(i = 0; i < prog.len; i += 2)
				{
					flash_program_half_word(baseaddr + i,
										*(u16*)(prog.buf+i));
				}
				flash_lock();
			}else // gateway
			{
				if(!gw_can_flash_program(baseaddr,prog.buf,prog.len))
				{
					usbd_ep_stall_set(device, 0, 1);
				}
			}
		}

	/*	 We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
	*/
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		break;
	case STATE_DFU_MANIFEST:
			 //USB device must detach, we just reset...
		scb_reset_system();
		break;  //Will never return
	default:
		break;
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
		/* Return state with no state transition*/
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
	/* LED pins */
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


void led_set(int id, int on)
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
	int i,ctr;
	/* one read appears to be unstable. Reason unknown
	 * Multiple reads provide adequate stability to
	 * determine whether USB is connected
	 */
	for(i = 0,ctr=0; i < 100;i++)
	{
		if(gpio_get(USBDETECT_PORT, USBDETECT_PIN))
		{
			ctr++;
		}
	}
	return (i == ctr);//gpio_get(USBDETECT_PORT, USBDETECT_PIN);
}

int main(void)
{

	gpio_init();

	/* Check if the application is valid. */
	if ((*(volatile u32 *)APP_ADDRESS & 0x2FFE0000) == 0x20000000)
	{
		if(!gpio_force_bootloader())
		{
			gpio_uninit();
			/* Set vector table base address. */
			SCB_VTOR = APP_ADDRESS & 0xFFFF;

			/* Jump to application. */
			(*(void (**)())(APP_ADDRESS + 4))();
	    }
	}

	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	gw_can_init(1000);

	systick_set_clocksource(STK_CTRL_CLKSOURCE_AHB_DIV8);
	systick_set_reload(SYSTICK_TIMEOUT_100MS);
	systick_interrupt_enable();
	systick_counter_enable();

	get_dev_unique_id(serial_no);

	usbd_device *device = usbd_init(&stm32f107_usb_driver, &dev, &config, usb_strings,5,usbd_control_buffer,SECTOR_SIZE);

/*	usbd_set_control_buffer_size(device, sizeof(usbd_control_buffer)); */
	usbd_register_control_callback( device,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
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
	for(i = 0; i < 24; i++) {
		if(s[i+8] > '9') {
			s[i+8] += 'A' - '9' - 1;
		}
	}
	return s;
}

void sys_tick_handler()
{
	led_advance();
}
