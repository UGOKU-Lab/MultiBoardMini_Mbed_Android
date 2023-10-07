/* Copyright (C) 2011 Circuits At Home, LTD. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

Contact information
-------------------

Circuits At Home, LTD
Web      :  http://www.circuitsathome.com
e-mail   :  support@circuitsathome.com
 */
/* MAX3421E-based USB Host Library header file */

//warning
//#define _usb_h_
//#define MBED_H

#ifndef usbhost_h
#define usbhost_h

//#define DEBUGMODE
#ifdef DEBUGMODE
#define DEBUG(x, ...) printf("[%s:%d]" x "\n", __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__);
#else
#define DEBUG(...) while (0);
#endif

#if !defined(_usb_h_) // || defined(_USBHOST_H_)
#error "Never include usbhost.h directly; include Usb.h instead"
#else
#define _USBHOST_H_

#if USING_SPI4TEENSY3
#include <spi4teensy3.h>
#include <sys/types.h>
#endif

typedef enum
{
    vbus_on = 0,
    vbus_off = GPX_VBDET
} VBUS_t;

class MAX3421E : public SPI
{
    //static
    uint8_t vbusState;

public:
    MAX3421E(PinName mosi, PinName miso, PinName sclk, PinName ssel, PinName intr);
    void regWr(uint8_t reg, uint8_t data);
    uint8_t *bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p);
    void gpioWr(uint8_t data);
    uint8_t regRd(uint8_t reg);
    uint8_t *bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p);
    uint8_t gpioRd();
    uint8_t gpioRdOutput();
    uint16_t reset();
    int8_t Init();
    int8_t Init(int mseconds);

    void vbusPower(VBUS_t state)
    {
        regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | state));
    }

    uint8_t getVbusState(void)
    {
        return vbusState;
    };
    void busprobe();
    uint8_t GpxHandler();
    uint8_t IntHandler();
    uint8_t Task();


private:
    DigitalOut _ss;
    DigitalIn _intr;

};


#endif // _USBHOST_H_
#endif //usbhost_h