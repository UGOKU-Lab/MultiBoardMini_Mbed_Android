#include "Usb.h"
#include "usbhost.h"

//#define DEBUGMODE
#ifdef DEBUGMODE
#define DEBUG(x, ...) printf("[%s:%d]" x "\n", __PRETTY_FUNCTION__, __LINE__, ##__VA_ARGS__);
#else
#define DEBUG(...) while (0);
#endif



/* constructor */
MAX3421E::MAX3421E(PinName mosi, PinName miso, PinName sclk, PinName ssel, PinName intr) : SPI(mosi, miso, sclk), _ss(ssel), _intr(intr)
{
    _ss = 1;
    
    //_spi.mode = SPI_MODE_MASTER;
    format(8, 0);
    frequency(26000000);
}

/* write single byte into MAX3421 register */
void MAX3421E::regWr(uint8_t reg, uint8_t data)
{
    uint8_t c[2];

    _ss = 0;

    c[0] = reg | 0x02;
    c[1] = data;
    
    SPI::write(c[0]);
    SPI::write(c[1]);
    
    //HAL_SPI_Transmit(&SPI_Handle, c, 2, HAL_MAX_DELAY);
    DEBUG("-              wirte %2x, %2x\n", c[0], c[1]);
    
    //DEBUG("w %d, %d\n", c[0], c[1]);
    
    _ss = 1;
}

/* multiple-byte write                            */

/* returns a pointer to memory position after last written */
uint8_t *MAX3421E::bytesWr(uint8_t reg, uint8_t nbytes, uint8_t *data_p)
{
    //uint8_t data = reg | 0x02;

    _ss = 0;
    
    uint8_t data = reg | 0x02;
    
    SPI::write(data);
    while (nbytes)
    {
        SPI::write(*data_p);
        nbytes--;
        data_p++; // advance data pointer
    }

    _ss = 1;
    return data_p;
}
/* GPIO write */
/*GPIO byte is split between 2 registers, so two writes are needed to write one byte */
/* GPOUT bits are in the low nibble. 0-3 in IOPINS1, 4-7 in IOPINS2 */

void MAX3421E::gpioWr(uint8_t data)
{
    regWr(rIOPINS1, data);
    data >>= 4;
    regWr(rIOPINS2, data);
    return;
}

/* single host register read    */

uint8_t MAX3421E::regRd(uint8_t reg)
{
    _ss = 0;
    
    SPI::write(reg);
    uint8_t rv = SPI::write(0);

    DEBUG("-              read %2x\n", rv);
    _ss = 1;

    return rv;
}
/* multiple-byte register read  */

/* returns a pointer to a memory position after last read   */
uint8_t *MAX3421E::bytesRd(uint8_t reg, uint8_t nbytes, uint8_t *data_p)
{
    _ss = 0;
    

    SPI::write(reg);
    memset(data_p, 0, nbytes);
    while (nbytes)
    {
        *data_p = SPI::write(0);
        data_p++;
        nbytes--;
    }
    
    _ss = 1;

    return data_p;
}

/* GPIO read. See gpioWr for explanation */

/** @brief  Reads the current GPI input values
*   @retval uint8_t Bitwise value of all 8 GPI inputs
*/
/* GPIN pins are in high nibbles of IOPINS1, IOPINS2    */

uint8_t MAX3421E::gpioRd()
{
    uint8_t gpin = 0;
    gpin = regRd(rIOPINS2);         //pins 4-7
    gpin &= 0xf0;                   //clean lower nibble
    gpin |= (regRd(rIOPINS1) >> 4); //shift low bits and OR with upper from previous operation.
    return (gpin);
}

/** @brief  Reads the current GPI output values
*   @retval uint8_t Bitwise value of all 8 GPI outputs
*/
/* GPOUT pins are in low nibbles of IOPINS1, IOPINS2    */

uint8_t MAX3421E::gpioRdOutput()
{
    uint8_t gpout = 0;
    gpout = regRd(rIOPINS1);         //pins 0-3
    gpout &= 0x0f;                   //clean upper nibble
    gpout |= (regRd(rIOPINS2) << 4); //shift high bits and OR with lower from previous operation.
    return (gpout);
}

/* reset MAX3421E. Returns number of cycles it took for PLL to stabilize after reset
  or zero if PLL haven't stabilized in 65535 cycles */

uint16_t MAX3421E::reset()
{
    uint16_t i = 0;
    regWr(rUSBCTL, bmCHIPRES);
    regWr(rUSBCTL, 0x00);
    while (++i)
    {
        if ((regRd(rUSBIRQ) & bmOSCOKIRQ))
        {
            break;
        }
    }
    return (i);
}

/* initialize MAX3421E. Set Host mode, pullups, and stuff. Returns 0 if success, -1 if not */

int8_t MAX3421E::Init()
{
    XMEM_ACQUIRE_SPI();
    // Moved here.
    // you really should not init hardware in the constructor when it involves locks.
    // Also avoids the vbus flicker issue confusing some devices.
    /* pin and peripheral setup */

    _ss = 1;

    XMEM_RELEASE_SPI();
    /* MAX3421E - full-duplex SPI, level interrupt */
    // GPX pin on. Moved here, otherwise we flicker the vbus.
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

    if (reset() == 0)
    { //OSCOKIRQ hasn't asserted in time
        return (-1);
    }

    regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

    regWr(rHIEN, bmCONDETIE | bmFRAMEIE); //connection detection

    /* check if device is connected */
    regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
    while (!(regRd(rHCTL) & bmSAMPLEBUS))
        ; //wait for sample operation to finish

    busprobe(); //check if anything is connected

    regWr(rHIRQ, bmCONDETIRQ); //clear connection detect interrupt
    regWr(rCPUCTL, 0x01);      //enable interrupt pin

    return (0);
}

/* initialize MAX3421E. Set Host mode, pullups, and stuff. Returns 0 if success, -1 if not */

int8_t MAX3421E::Init(int mseconds)
{
    XMEM_ACQUIRE_SPI();
    // Moved here.
    // you really should not init hardware in the constructor when it involves locks.
    // Also avoids the vbus flicker issue confusing some devices.
    /* pin and peripheral setup */

    _ss = 1;

    XMEM_RELEASE_SPI();
    /* MAX3421E - full-duplex SPI, level interrupt, vbus off */
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL | GPX_VBDET));

    if (reset() == 0)
    { //OSCOKIRQ hasn't asserted in time
        return (-1);
    }

    // Delay a minimum of 1 second to ensure any capacitors are drained.
    // 1 second is required to make sure we do not smoke a Microdrive!
    if (mseconds < 1000)
        mseconds = 1000;
    delay(mseconds);

    regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST); // set pull-downs, Host

    regWr(rHIEN, bmCONDETIE | bmFRAMEIE); //connection detection

    /* check if device is connected */
    regWr(rHCTL, bmSAMPLEBUS); // sample USB bus
    while (!(regRd(rHCTL) & bmSAMPLEBUS))
        ; //wait for sample operation to finish

    busprobe(); //check if anything is connected

    regWr(rHIRQ, bmCONDETIRQ); //clear connection detect interrupt
    regWr(rCPUCTL, 0x01);      //enable interrupt pin

    // GPX pin on. This is done here so that busprobe will fail if we have a switch connected.
    regWr(rPINCTL, (bmFDUPSPI | bmINTLEVEL));

    return (0);
}

/* probe bus to determine device presence and speed and switch host to this speed */

void MAX3421E::busprobe()
{
    uint8_t bus_sample;
    DEBUG("busprobe()\n");
    bus_sample = regRd(rHRSL);             //Get J,K status
    bus_sample &= (bmJSTATUS | bmKSTATUS); //zero the rest of the byte
    switch (bus_sample)
    { //start full-speed or low-speed host
    case (bmJSTATUS):
        if ((regRd(rMODE) & bmLOWSPEED) == 0)
        {
            regWr(rMODE, MODE_FS_HOST); //start full-speed host
            vbusState = FSHOST;
        }
        else
        {
            regWr(rMODE, MODE_LS_HOST); //start low-speed host
            vbusState = LSHOST;
        }
        break;
    case (bmKSTATUS):
        if ((regRd(rMODE) & bmLOWSPEED) == 0)
        {
            regWr(rMODE, MODE_LS_HOST); //start low-speed host
            vbusState = LSHOST;
        }
        else
        {
            regWr(rMODE, MODE_FS_HOST); //start full-speed host
            vbusState = FSHOST;
        }
        break;
    case (bmSE1): //illegal state
        vbusState = SE1;
        break;
    case (bmSE0): //disconnected state
        regWr(rMODE, bmDPPULLDN | bmDMPULLDN | bmHOST | bmSEPIRQ);
        vbusState = SE0;
        break;
    } //end switch( bus_sample )
}

/* MAX3421 state change task and interrupt handler */

uint8_t MAX3421E::Task(void)
{
    uint8_t rcode = 0;
    uint8_t pinvalue;
    //USB_HOST_SERIAL.print("Vbus state: ");
    //USB_HOST_SERIAL.println( vbusState, HEX );

    pinvalue = _intr.read();

    //pinvalue = digitalRead( MAX_INT );
    if (pinvalue == 0)
    {
        rcode = IntHandler();
    }
    //    pinvalue = digitalRead( MAX_GPX );
    //    if( pinvalue == LOW ) {
    //        GpxHandler();
    //    }
    //    usbSM();                                //USB state machine
    return (rcode);
}

uint8_t MAX3421E::IntHandler()
{
    uint8_t HIRQ;
    uint8_t HIRQ_sendback = 0x00;

    DEBUG("IntHandler\n");
    HIRQ = regRd(rHIRQ); //determine interrupt source
    //if( HIRQ & bmFRAMEIRQ ) {               //->1ms SOF interrupt handler
    //    HIRQ_sendback |= bmFRAMEIRQ;
    //}//end FRAMEIRQ handling
    if (HIRQ & bmCONDETIRQ)
    {
        busprobe();
        HIRQ_sendback |= bmCONDETIRQ;
    }
    // End HIRQ interrupts handling, clear serviced IRQs
    regWr(rHIRQ, HIRQ_sendback);
    return (HIRQ_sendback);
}