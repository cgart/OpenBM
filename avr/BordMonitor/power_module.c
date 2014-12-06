/*
* Copyright 2010-2013 Art Tevs <art@tevs.eu>
* This file is part of OpenBM (firmware).
*
* OpenBM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenBM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/


#include "base.h"
#include "buttons.h"
#include "ibus.h"
#include "display.h"
#include "config.h"
#include "leds.h"
#include "power_module.h"
#include "obm_special.h"
#include "emul_mid.h"
#include "include/base.h"
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/sleep.h>

RunningMode _runningMode = INIT;

typedef struct _PowerSettings
{
    uint8_t  shutdownTimer;
}PowerSettings;


PowerSettings power_Settings;
PowerSettings power_SettingsEEPROM EEMEM;
uint8_t      power_SettingsInit EEMEM;

static uint8_t _lastKeyPressed = 0;
static ticks_t _nextIbusTick;
static uint8_t _hwIgnitionState = 0;
static ticks_t _shutdownTimer = 0;

//------------------------------------------------------------------------------
void power_setHWIgnitionState(uint8_t state)
{
    _hwIgnitionState = state;
}

//------------------------------------------------------------------------------
uint8_t power_getHWIgnitionState(void)
{
    return _hwIgnitionState;
}

//------------------------------------------------------------------------------
// Reset cpu by watchdog, full reset is performed
//------------------------------------------------------------------------------
void power_reset_cpu(void)
{
    cli();
    wdt_enable(WDTO_15MS);
    while(1){};
}


//------------------------------------------------------------------------------
// Full shut down of the main board (MOSFET turn off)
//------------------------------------------------------------------------------
void shutDown(void)
{
    // shut down main mosfet for the PC
    PORTB &= ~(1 << 0);    
    nop();
    DDRD &= ~(1 << 0);
    
    cli();
    led_fan_immediate_set(0);
    led_green_immediate_set(0);
    led_yellow_immediate_set(0);
    led_red_immediate_set(0);
    led_radio_immediate_set(0);
    _delay_ms(500);
    display_turnOff(); // turn display off without storing the power state

    // shut down main board (HACK because of broken TH3122??? need to set TX to low also)
    DDRC |= (1 << DDC3);
    nop();
    PORTC &= ~(1 << 3);
    nop();
    bit_clear(IBUS_TX_PORT, IBUS_TX_PIN);
    nop();
    IBUS_TX_DISOUT();    
    nop();

    _delay_ms(100);

    led_fan_immediate_set(1);
    //led_green_immediate_set(1);
    //led_yellow_immediate_set(1);
    //led_red_immediate_set(1);
    //led_radio_immediate_set(1);
    
    // debug
    set_sleep_mode(SLEEP_MODE_IDLE);
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    power_reset_cpu();
}

//------------------------------------------------------------------------------
void power_prepare_shutdown(void)
{
    _runningMode = PREPARE_SHUTDOWN;
    _nextIbusTick = tick_get() + TICKS_PER_TWO_SECONDS;
}

//------------------------------------------------------------------------------
RunningMode power_get_running_mode(void)
{
    return _runningMode;
}

//------------------------------------------------------------------------------
void power_set_running_mode(RunningMode mode)
{
    _runningMode = mode;
    if (mode == PREPARE_SHUTDOWN)
        power_prepare_shutdown();
}

//------------------------------------------------------------------------------
void power_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if ((_runningMode & PREPARE_SHUTDOWN) != PREPARE_SHUTDOWN)
    {
        _nextIbusTick = tick_get() + _shutdownTimer;
    }
    
    // stop be active on ibus when ignition completly off
    //if (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO && msglen == 2 && msg[0] == IBUS_MSG_IGNITION && msg[1] == 0)
    //    g_runningMode = STOP_IBUS;

    // close key
    // 00,BF,72,12 down  -  00,BF,72,02 released

    // open key
    // 00,BF,72,22 down  -  00,BF,72,02 released

    // trunk key
    // 00,BF,72,42 down  -  00,BF,72,02 released

    // key EWS
    // 44,BF,74,00,01 out
    // 44,BF,74,04,01 in

    // close car completely
    // 44,bf,74,00,ff -> EWS de/activated???

    // 00,BF,76,00 -> car locked ???
    // 00,BF,76,02 -> car unlocked ???

    // CLOSE Session:
    // 00,BF,72,12  - key down
    // 00,BF,76,00
    // 00,BF,72,02  - key release
    // 00,BF,7A,32,01 - state of Doors and other stuff
    // 44,BF,74,00,FF

    // OPEN Session:
    // 00,BF,72,22
    // 00,BF,76,02
    // 00,BF,7A,53,21 - state of doors
    // 00,BF,76,00
    // 00,BF,72,02

    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO && msglen >= 2 && msg[0] == IBUS_MSG_GM_KEY_BUTTON)
    {
        if ((msg[1] & 0xF0) == 0)
        {
            if (_lastKeyPressed == 0x10)
            {
                power_stop();

                led_red_set(0b1000000);
                display_turnOff();

                obms_stop();
                mid_stop();

            // go into running mode, if key opened the car
            }else if (_lastKeyPressed == 0x20)
            {
                power_resume();
                obms_resume();
                mid_resume();

                led_init();
            }
            _lastKeyPressed = 0;
        }
        _lastKeyPressed = msg[1] & 0xF0;

    }

    // still in running mode
    if (_runningMode == RUN)
    {
        // door was opened or ignition is not 0
        if  ((src == IBUS_DEV_GM && dst == IBUS_DEV_GLO && msglen >= 2 && msg[0] == IBUS_MSG_GM_STATE && (msg[1] & 0x0F))
         || (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO && msglen >= 2 && msg[0] == IBUS_MSG_IGNITION && msg[1] != 0))
        {
            display_tryTurnOn();    
            
        // ignition turned off
        }else if (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO && msglen >= 2 && msg[0] == IBUS_MSG_IGNITION && msg[1] == 0)
        {
            led_init();
        }
    }

    
    // Setup settings if asked so over IBus
    if (dst == IBUS_DEV_BMBT  && msglen >= 3 && msg[0] == IBUS_MSG_OPENBM_TO && msg[1] == IBUS_MSG_OPENBM_SET_POWER)
    {
        // every such message will be responded, default response is 0
        uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_SET_POWER, msg[2]};
        uint8_t ok = 0;
        
        if (msglen == 4 && msg[2] == 0x01)
        {
            eeprom_update_byte(&power_SettingsEEPROM.shutdownTimer, msg[3]);
            power_Settings.shutdownTimer = msg[3];
            _shutdownTimer = (((uint32_t)TICKS_PER_X_SECONDS(power_Settings.shutdownTimer)) << 3);
            ok = 1;
        }else
        {
            data[2] = 0;
        }
        if (ok)
          ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);
    }
    
}


//------------------------------------------------------------------------------
void power_tick(void)
{
    // go into full sleep mode if there is no action on the ibus
    // happens during the last X seconds (full shut down!!!)
    if (tick_get() > _nextIbusTick)
        shutDown();
}

//------------------------------------------------------------------------------
void power_stop(void)
{
    _runningMode = STOP_IBUS;
}

//------------------------------------------------------------------------------
void power_resume(void)
{
    power_init();
    _runningMode = RUN;
}

//------------------------------------------------------------------------------
void power_init(void)
{
    DDRB |= (1 << 0);
    
    _nextIbusTick =  tick_get() + TICKS_PER_X_SECONDS(30);
    _lastKeyPressed = 0;
    //UPDATE_SLEEP_COUNTER();
    
    PORTB |= (1 << 0);    
    
    
    if (eeprom_read_byte(&power_SettingsInit) != EE_CHECK_BYTE)
    {
        eeprom_write_byte(&power_SettingsInit, EE_CHECK_BYTE);

        eeprom_update_byte(&power_SettingsEEPROM.shutdownTimer, 35);
    }
    power_Settings.shutdownTimer = eeprom_read_byte(&power_SettingsEEPROM.shutdownTimer);
    
    _shutdownTimer = (((uint32_t)TICKS_PER_X_SECONDS(power_Settings.shutdownTimer)) << 3);
}

//------------------------------------------------------------------------------
// On any button action this interrupt will be called.
// We use this interrupt to wake up our main CPU and let button handler know
// that there is something happening with the buttons
//------------------------------------------------------------------------------
ISR(INT1_vect)
{    
    //UPDATE_SLEEP_COUNTER();
    _nextIbusTick = tick_get() + _shutdownTimer;
    button_isr();
}
