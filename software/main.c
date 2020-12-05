/*
 * USB-Joystickadapter Firmware
 *    
 * based on project hid-mouse, a very simple HID example by Christian Starkjohann, OBJECTIVE DEVELOPMENT Software GmbH
 * and ATtiny2313 USB Joyadapter firmware by Grigori Goronzy.    
 * adapted for ATtiny4313, changes for ATtiny2313 with new compiler by Andreas Paul 05/2011
 * complete rewrite by Philipp Lang 01/2015
 * 
 * Name: main.c
 * Project: USB-Joystickadapter
 * Author: Andreas Paul, Philipp Lang (Christian Starkjohann, Grigori Goronzy) 
 * Creation Date: 2015-01-31
 * 
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id: $
 */

/*
  Wiring             | Port | 7    6    5    4    3    2    1    0    |
  -------------------+------+-----------------------------------------+
  Left joystick      | PB   | But2 But1 Down Up   Rght Left -    -    |
  Right Joystick     | PD   | -    Down Up   Rght Left -    But2 But1 |
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "usbdrv.h"

/* USB configuration descriptor */
PROGMEM const char usbDescriptorConfiguration[] = {
  9,          /* sizeof(usbDescriptorConfiguration): length of descriptor in bytes */
  USBDESCR_CONFIG,    /* descriptor type */
  9 + (2*(9+9+7)), 0, /* total length of data returned (including inlined descriptors) */
  2,          /* number of interfaces in this configuration */
  1,          /* index of this configuration */
  0,          /* configuration name string index */
#if USB_CFG_IS_SELF_POWERED
  (1 << 7) | USBATTR_SELFPOWER,       /* attributes */
#else
  (1 << 7),                           /* attributes */
#endif
  USB_CFG_MAX_BUS_POWER/2,            /* max USB current in 2mA units */

/* interface 1 descriptor follows inline: */
  9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
  USBDESCR_INTERFACE, /* descriptor type */
  0,          /* index of this interface */
  0,          /* alternate setting for this interface */
  1,          /* endpoints excl 0: number of endpoint descriptors to follow */
  USB_CFG_INTERFACE_CLASS,
  USB_CFG_INTERFACE_SUBCLASS,
  USB_CFG_INTERFACE_PROTOCOL,
  3,          /* string index for interface */
  9,          /* sizeof(usbDescrHID): length of descriptor in bytes */
  USBDESCR_HID,   /* descriptor type: HID */
  0x01, 0x01, /* BCD representation of HID version */
  0x00,       /* target country code */
  0x01,       /* number of HID Report (or other HID class) Descriptor infos to follow */
  0x22,       /* descriptor type: report */
  USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0,  /* total length of report descriptor */

/* endpoint descriptor for endpoint 1 */
  7,          /* sizeof(usbDescrEndpoint) */
  USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
  (char)0x81, /* IN endpoint number 1 */
  0x03,       /* attrib: Interrupt endpoint */
  8, 0,       /* maximum packet size */
  USB_CFG_INTR_POLL_INTERVAL, /* in ms */

/* interface 2 descriptor follows inline: */
  9,          /* sizeof(usbDescrInterface): length of descriptor in bytes */
  USBDESCR_INTERFACE, /* descriptor type */
  1,          /* index of this interface */
  0,          /* alternate setting for this interface */
  1,          /* endpoints excl 0: number of endpoint descriptors to follow */
  USB_CFG_INTERFACE_CLASS,
  USB_CFG_INTERFACE_SUBCLASS,
  USB_CFG_INTERFACE_PROTOCOL,
  4,          /* string index for interface */
  9,          /* sizeof(usbDescrHID): length of descriptor in bytes */
  USBDESCR_HID,   /* descriptor type: HID */
  0x01, 0x01, /* BCD representation of HID version */
  0x00,       /* target country code */
  0x01,       /* number of HID Report (or other HID class) Descriptor infos to follow */
  0x22,       /* descriptor type: report */
  USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH, 0,  /* total length of report descriptor */

/* endpoint descriptor for endpoint 3 */
  7,          /* sizeof(usbDescrEndpoint) */
  USBDESCR_ENDPOINT,  /* descriptor type = endpoint */
  (char)(0x80 | USB_CFG_EP3_NUMBER), /* IN endpoint number 3 */
  0x03,       /* attrib: Interrupt endpoint */
  8, 0,       /* maximum packet size */
  USB_CFG_INTR_POLL_INTERVAL, /* in ms */
};

PROGMEM const char usbHidReportDescriptor[42] = {
  0x05, 0x01,   // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,   // USAGE (Joystick)
  0xa1, 0x01,   // COLLECTION (Application)
  0x09, 0x01,   //   USAGE (Pointer)
  0xa1, 0x00,   //   COLLECTION (Physical)
  0x09, 0x30,   //     USAGE (X)
  0x09, 0x31,   //     USAGE (Y)
  0x15, 0xff,   //     LOGICAL_MINIMUM (-1)
  0x25, 0x01,   //     LOGICAL_MAXIMUM (1)
  0x75, 0x03,   //     REPORT_SIZE (3)
  0x95, 0x02,   //     REPORT_COUNT (2)
  0x81, 0x02,   //     INPUT (Data,Var,Abs)
  0xc0,         //   END_COLLECTION
  0x05, 0x09,   //   USAGE_PAGE (Button)
  0x19, 0x01,   //   USAGE_MINIMUM (Button 1)
  0x29, 0x02,   //   USAGE_MAXIMUM (Button 2)
  0x15, 0x00,   //   LOGICAL_MINIMUM (0)
  0x25, 0x01,   //   LOGICAL_MAXIMUM (1)
  0x75, 0x01,   //   REPORT_SIZE (1)
  0x95, 0x02,   //   REPORT_COUNT (2)
  0x81, 0x02,   //   INPUT (Data,Var,Abs)
  0xc0,         // END_COLLECTION
};

static uchar ReportBuffer[2] = { 0x02, 0x02 }; // initialize with impossible values
static uchar IdleRate = 0; // infinite
static const char Desc0Str[] = { 4, 3, 0x09, 0x04 };
static PROGMEM const char VendorStr[]   = "hexagons.de";
static PROGMEM const char JoystickStr[] = "Retro-Joystick #x";

usbMsgLen_t usbFunctionDescriptor(struct usbRequest *rq)
{
  static char buf[(sizeof(JoystickStr) > sizeof(VendorStr) ? sizeof(JoystickStr) : sizeof(VendorStr))*2];

  uchar ix = rq->wValue.bytes[0];
  const char* s;
  uchar l;
  uchar ifchar = 0;
  if (ix == 0) { // String0
    usbMsgPtr = (usbMsgPtr_t)Desc0Str;
    return 4;
  }
  else if (ix == 1) { // Vendor
    s = VendorStr;
    l = sizeof(VendorStr) * 2;
  }
  else if (ix == 2) { // Product
    s = JoystickStr;
    l = (sizeof(JoystickStr) - 3) * 2;
  }
  else if (ix == 3 || ix == 4) { // Interface 1 or 2
    s = JoystickStr;
    l = sizeof(JoystickStr) * 2;
    ifchar = '1' + ix - 3;
  }
  else {
    return 0;
  }
  
  // copy from ROM to RAM with char to wchar conversion
  char* d = buf + 2;
  char c;
  while ((c = pgm_read_byte(s++)) != 0) {
    *d = c;
    d += 2;
  }
  
  buf[0] = l; // length
  buf[1] = 3; // string descriptor mark
  if (ifchar)
    buf[(sizeof(JoystickStr)-1) * 2] = ifchar; // patch interface number
  usbMsgPtr = (usbMsgPtr_t)buf;
  return l;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
  usbRequest_t *rq = (void *)data;
  if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
    // class request type
    switch (rq->bRequest) {
    case USBRQ_HID_GET_REPORT:
      // wValue: ReportType, ReportID
      // wIndex: interface
      usbMsgPtr = &ReportBuffer[rq->wIndex.word & 1];
      return 1;
    case USBRQ_HID_GET_IDLE:
      usbMsgPtr = &IdleRate;
      return 1;
    case USBRQ_HID_SET_IDLE:
      // ignore
      return 0;
    }
  }
  return 0;
}

static uchar encodeReport(uchar pin)
{
  pin = ~pin;
  uchar r = pin & 0xc0; // buttons
  if (pin & _BV(5)) // down
    r |= 0x08;
  if (pin & _BV(4)) // up
    r |= 0x38;
  if (pin & _BV(3)) // right
    r |= 0x01;
  if (pin & _BV(2)) // left
    r |= 0x07;
  return r;
}

int __attribute__((noreturn)) main(void)
{
  // reset status: port bits are inputs without pull-up.
  // enable pull-up on joystick pins, keep D+/D-
  PORTB = 0xfc;
  PORTD = 0x7b;
  wdt_enable(WDTO_1S);
  
  // enforce re-enumeration, do this while interrupts are disabled!
  // fake USB disconnect for > 250 ms
  usbInit();
  usbDeviceDisconnect();
  uchar i = 0;
  while (--i)
    _delay_ms(1);
  
  // the port mapping can be selected by holding the left joystick while plugging-in the adapter:
  // button1 + left: left joystick will be reported as joystick #1
  // button1 + right: right joystick will be reported as joystick #1
  // the mapping is stored in the eeprom
  uchar swapPorts = eeprom_read_byte((uint8_t *)0);
  uchar newSwapPorts = swapPorts;
  i = PINB & 0x7c;
  if (i == 0x38) // button1 + left => no swap
    newSwapPorts = 0;
  else if (i == 0x34) // button1 + right => swap
    newSwapPorts = 1;
  if (newSwapPorts != swapPorts) {
    eeprom_write_byte((uint8_t *)0, newSwapPorts);
    while ((PINB & 0xfc) != 0xfc) // wait until joystick is back in null position (indicates success to user)
      wdt_reset();
  }
  
  usbDeviceConnect();
  sei();

  // main loop
  for (;;) {
    wdt_reset();

    usbPoll();
    
    uchar r1 = encodeReport(PINB);
    uchar x = PIND & 0x7b;
    uchar r2 = encodeReport((x >> 1) | (x << 6));
    if (swapPorts) {
      x = r1;
      r1 = r2;
      r2 = x;
    }

    if (ReportBuffer[0] != r1) {
      ReportBuffer[0] = r1;
      usbSetInterrupt(&ReportBuffer[0], 1);
    }
    if (ReportBuffer[1] != r2) {
      ReportBuffer[1] = r2;
      usbSetInterrupt3(&ReportBuffer[1], 1);
    }
  }
}
