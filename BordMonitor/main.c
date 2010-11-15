/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include "ibus.h"
#include <uart.h>
#include <i2cmaster.h>
#include <avr/wdt.h>
#include "display.h"
#include "buttons.h"
#include "leds.h"
#include "emul_mid.h"
#include "config.h"
#include "bootloader.h"

// global tick counter
ticks_t g_tickNumber = 0;

// current adc value
uint8_t g_photoSensor = 0;
uint8_t g_backLightDimmer = 0;

// channel to read adc values (0 = photo sensor, 1 = light dimmer)
uint8_t g_adcCurrentChannel = 0;

// current emulation mode (0=OpenBM, 1=MID, 2=BMBT)
//typedef enum _EmulationMode
//{
//    OPENBM = 0,
//    MID = 1,
//    BMBT = 2
//}EmulationMode;

//EmulationMode g_emulationMode = MID;

//------------------------------------------------------------------------------
// Initialize analog hardware (read photo sensor, read bg light)
//------------------------------------------------------------------------------
void adc_init(void)
{
    g_photoSensor = 0;
    g_backLightDimmer = 0;
    g_adcCurrentChannel = 0;
    //g_emulationMode = MID;
    
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 109KHz sample rate @ 14MHz
    ADMUX = (1 << REFS0); // Set ADC reference to AVCC
    ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
    ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2); // enable reading on ADC7
    SFIOR &= ~(1 << ADTS2) & ~(1 << ADTS1) & ~(1 << ADTS0);  // Set ADC to Free-Running Mode
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    ADCSRA |= (1 << ADEN);  // Enable ADC

    // start reading from ADC
    ADCSRA |= (1 << ADSC);
}

//------------------------------------------------------------------------------
// Reset cpu by watchdog, full reset is performed
//------------------------------------------------------------------------------
void resetCPU(void)
{
    cli();
    wdt_enable(WDTO_15MS);
    while(1){};
}

//------------------------------------------------------------------------------
// Send Version string over ibus
//------------------------------------------------------------------------------
void ibus_sendVersion(void)
{
    
}

//------------------------------------------------------------------------------
// This method will be called by IBus stack when new message recieved
//------------------------------------------------------------------------------
void ibus_MessageCallback(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    // transfer message further to current emulation state
    //switch(g_emulationMode)
    //{
    //case OPENBM:
    //    emul_openbm_on_bus_msg(src,dst,msg,msglen);
    //    break;
    //case MID:
        emul_mid_on_bus_msg(src,dst,msg,msglen);
    //    break;
    //case BMBT:
    //    emul_bmbt_on_bus_msg(src,dst,msg,msglen);
    //    break;
    //}

    // ----------------------------
    // special message treatment for OpenBM related messages
    // ----------------------------
    if (dst == IBUS_DEV_BMBT && msg[0] == IBUS_MSG_OPENBM_TO)
    {
        if (msg[1] == IBUS_MSG_OPENBM_GET_TICKS)
        {
            uint8_t data[6] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_TICKS, 0, 0, 0, 0};
            data[2] = (g_tickNumber & 0xFF000000) >> 24;
            data[3] = (g_tickNumber & 0x00FF0000) >> 16;
            data[4] = (g_tickNumber & 0x0000FF00) >> 8;
            data[5] = (g_tickNumber & 0x000000FF) >> 0;
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 6, 3);
            
        }else if (msg[1] == IBUS_MSG_OPENBM_GET_PHOTO)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, g_photoSensor};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, 3);

        }else if (msg[1] == IBUS_MSG_OPENBM_GET_DIMMER)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_DIMMER, g_backLightDimmer};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, 3);

        // reset command F0 07 .. FA BA AD FE ED ..
        }else if (msglen == 5 && msg[1] == 0xBA && msg[2] == 0xAD && msg[3] == 0xFE && msg[4] == 0xED)
        {
            resetCPU();
        }
        
    }
}

//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{    
    // init hardware
    tick_init();
    led_init();
    i2c_init();
    display_init();
    button_init();
    adc_init();
    emul_mid_init();
    ibus_init();

    sei();

    start_bootloader();
    
    ibus_setMessageCallback(ibus_MessageCallback);
}

//------------------------------------------------------------------------------
// Set FUSES correctrly ( http://www.engbedded.com/fusecalc/):
// avrdude -U lfuse:w:0xff:m -U hfuse:w:0xc9:m  (ext crystal, disable jtag, ckopt)
//------------------------------------------------------------------------------
int main(void)
{
    // init hardware and do small delay after init
    initHardware();

    //uint8_t welcommsg1[16] = {IBUS_MSG_UPDATE_MID_TOP, 0x40, 0x20, 'W', 'E', 'L', 'C', 'O', 'M', 'E', 0x20 , 0x20 , 0x20 , 0x20, 0x20, 0x20};
    //uint8_t welcommsg2[16] = {IBUS_MSG_UPDATE_MID_TOP, 0x40, 0x20, 'O', 'P', 'E', 'N', 'B', 'M', '_', 'E', '3'  , '9'  , '!'  , 0x20, 0x20};
    //ibus_sendMessage(IBUS_DEV_RAD, IBUS_DEV_ANZV, data, 16, IBUS_TRANSMIT_TRIES);
    
    //led_red_set(0b11110000);
    
    for(;;)
    {
        // update current encoder states, need this here to eventually reset
        // the encoder rotary state to predefined value
        button_tick_encoder();
        
        // ------------ Task timer has shooted, so perfrom repeated task -------
        if (tick_event())
        {
            // update current state of buttons
            button_tick();

            // update mid emulator
            emul_mid_tick();

            // update flashing LEDs
            led_tick();

            // display buttons
            display_updateState();
            led_green_immediate_set(display_getInputState() == 1);

            // perform global tick
            tick();
        }

        // ----------- Do full reset if user liked so --------------------------
        if (button_released(BUTTON_MENU_LR) && button(BUTTON_SELECT) && button(BUTTON_EJECT))
            resetCPU();

        /*if (button_pressed(BUTTON_EJECT))
        {
            static uint8_t show = 1;
            if (show)
                ibus_sendMessage(IBUS_DEV_RAD, IBUS_DEV_MID, welcommsg1, 16, IBUS_TRANSMIT_TRIES);
            else
                ibus_sendMessage(IBUS_DEV_RAD, IBUS_DEV_MID, welcommsg2, 16, IBUS_TRANSMIT_TRIES);
            show = !show;
        }*/

        // depending on emulation mode, execute corresponding task
        //switch(g_emulationMode)
        //{
        //case OPENBM:
        //    emul_openbm_tick();
        //    break;
        //case MID:
            emul_mid_encoder_tick();
        //    break;
        //case BMBT:
        //    emul_bmbt_tick();
        //    break;
        //}


        // Switch Emulation mode
        //if (button_down(BUTTON_MENU_LR) && button_down(BUTTON_SELECT) && button_down(BUTTON_MODE) && button_released(BUTTON_RADIO_KNOB))
        //{
        //    if (g_emulationMode == OPENBM) g_emulationMode = MID;
        //    else if (g_emulationMode == MID) g_emulationMode = BMBT;
        //    else if (g_emulationMode == BMBT) g_emulationMode = OPENBM;
        //}

        // perform ibus tasks
        ibus_tick();
    }

    return 0;
}

//------------------------------------------------------------------------------
// React on ADC interrupts, so analog value is readed
// afterwards conversion is started again and channel is switched
//------------------------------------------------------------------------------
ISR(ADC_vect)
{
    BEGIN_ATOMAR;
    {
        if (g_adcCurrentChannel == 0)
        {
            g_photoSensor = ADCH;

            // switch channel (read on ADC0 next)
            g_adcCurrentChannel = 1;
            ADMUX &= ~(1 << MUX4) & ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
        }else
        {
            g_backLightDimmer = ADCH;

            // switch channel (read on ADC7 next)
            g_adcCurrentChannel = 0;
            ADMUX &= ~(1 << MUX4) & ~(1 << MUX3);
            ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2);
        }
        ADCSRA |= (1 << ADSC);
    }
    END_ATOMAR;
}

