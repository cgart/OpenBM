/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include "config.h"
#include "ibus.h"
#include "display.h"
#include "buttons.h"
#include "leds.h"
#include "emul_mid.h"
#include "uart.h"
#include "obm_special.h"
#include "photo_sensor.h"
#include "power_module.h"
#include <i2cmaster.h>
#include <avr/boot.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "bootloader/bootloader.h"
#include "include/leds.h"
#include "include/power_module.h"
#include "include/photo_sensor.h"
#include "include/ibus.h"

// global tick counter
ticks_t g_tickNumber;
uint8_t g_tickEventHappened;
uint8_t g_setupMode;

// current adc value
volatile uint8_t g_backLightDimmer;
volatile int8_t  g_temperatureSensor;
volatile uint8_t g_adcCurrentChannel;

bootldrinfo_t bootLdrInfo __attribute__((section(".bootloadercfg")));
static bootldrinfo_t g_bootldrinfo;

//! Device settings used
DeviceSettings g_deviceSettings;
DeviceSettings g_deviceSettingsEEPROM EEMEM;


//------------------------------------------------------------------------------
// Read device settings from eeprom
//------------------------------------------------------------------------------
void settings_readAndSetup(void)
{
    //BEGIN_ATOMAR;

    #ifdef BOOTLOADER_ACTIVE
    // dummy commands to prevent compiler to optimize bootloader away
    uint8_t dummy = eeprom_read_byte(&bin_data[0]);
    eeprom_update_byte(&bin_data[0], dummy);
    #endif

    // setup default settings if not in eeprom
    if (eeprom_read_byte(&g_deviceSettingsEEPROM.initSeed) != 'L')
    {
        eeprom_update_byte(&g_deviceSettingsEEPROM.initSeed, 'L');

        // --------------------
        // IO-s
        // --------------------
        eeprom_update_word((void*)&g_deviceSettingsEEPROM.io_assignment[0], BUTTON_EJECT);
        eeprom_update_word((void*)&g_deviceSettingsEEPROM.io_assignment[1], BUTTON_TONE);
        eeprom_update_word((void*)&g_deviceSettingsEEPROM.io_assignment[2], BUTTON_TEL);

        // --------------------
        // Display
        // --------------------
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_idleVoltage, DEVICE_DISP_IDLE);
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_SwitchKey, DEVICE_DISP_SWITCH);
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_PowerKey, DEVICE_DISP_POWER);

        // --------------------
        // BMW settings
        // --------------------
        eeprom_update_byte(&g_deviceSettingsEEPROM.device_Settings1, DEVICE_CODING1);
        eeprom_update_byte(&g_deviceSettingsEEPROM.device_Settings2, DEVICE_CODING2);
    }

    // load current display state from the eeprom
    g_deviceSettings.dac_idleVoltage  = eeprom_read_word(&g_deviceSettingsEEPROM.dac_idleVoltage);
    g_deviceSettings.dac_PowerKey  = eeprom_read_word(&g_deviceSettingsEEPROM.dac_PowerKey);
    g_deviceSettings.dac_SwitchKey = eeprom_read_word(&g_deviceSettingsEEPROM.dac_SwitchKey);

    g_deviceSettings.device_Settings1 = eeprom_read_byte(&g_deviceSettingsEEPROM.device_Settings1);
    g_deviceSettings.device_Settings2 = eeprom_read_byte(&g_deviceSettingsEEPROM.device_Settings2);

    g_deviceSettings.io_assignment[0] = eeprom_read_word((void*)&g_deviceSettingsEEPROM.io_assignment[0]);
    g_deviceSettings.io_assignment[1] = eeprom_read_word((void*)&g_deviceSettingsEEPROM.io_assignment[1]);
    g_deviceSettings.io_assignment[2] = eeprom_read_word((void*)&g_deviceSettingsEEPROM.io_assignment[2]);

    //END_ATOMAR;
}


//------------------------------------------------------------------------------
// Initialize analog hardware (read photo sensor, read bg light)
//------------------------------------------------------------------------------
void adc_init(void)
{
    g_setupMode = 0;
    g_temperatureSensor = 0;
    g_backLightDimmer = 0;
    g_adcCurrentChannel = 0;

    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 109KHz sample rate @ 14MHz
    ADMUX = (1 << REFS0); // Set ADC reference to AVCC with external capacitor
    ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
    ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2); // enable reading on ADC7
    //ADCSRB = (1 << ADTS1) | (1 << ADTS0);  // Set ADC to start on Timer0 compare match
    ADCSRB = (1 << ADTS2) | (1 << ADTS0);  // Set ADC to start on Timer1 compare match B
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADATE); // Enable ADC to start on trigger (Timer0 compare, tick())
    
    DIDR0 = 0xFF; // disable digital inputs on the complete ADC port/ A.port

    // start reading from ADC
    ADCSRA |= (1 << ADSC);
}

#if 0
//------------------------------------------------------------------------------
// disable currently the ADC (saves power)
//------------------------------------------------------------------------------
void adc_sleep(void)
{
    ADCSRA &= ~(1 << ADIE);
    ADCSRA &= ~(1 << ADEN);
    PRR |= (1 << PRADC);
}

//------------------------------------------------------------------------------
// Enable ADC (must be initialized before)
//------------------------------------------------------------------------------
void adc_resume(void)
{
    PRR &= ~(1 << PRADC);
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADSC);
}
#endif

//------------------------------------------------------------------------------
// This will put CPU into sleep mode. Call this in the main loop in order to save power
//------------------------------------------------------------------------------
void sleepCPU(void)
{
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
}


//------------------------------------------------------------------------------
// This method will be called by IBus stack when new message recieved
//------------------------------------------------------------------------------
void ibus_MessageCallback(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    power_on_bus_msg(src, dst, msg, msglen);

    // do not proceed while not running
    if (power_get_running_mode() != RUN) return;

    // only send messages if not in setup mode
    if (!g_setupMode)
    {
        mid_on_bus_msg(src,dst,msg,msglen);
        obms_on_bus_msg(src, dst, msg, msglen);
        photo_on_bus_msg(src, dst, msg, msglen);
    }

    // ----------------------------
    // special message treatment for OpenBM related messages
    // ----------------------------
    if (dst == IBUS_DEV_BMBT && msg[0] == IBUS_MSG_OPENBM_TO)
    {
        if (msg[1] == IBUS_MSG_OPENBM_GET_VERSION)
        {
            uint8_t data[4] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_VERSION, g_bootldrinfo.app_version>>8, g_bootldrinfo.app_version&0xFF};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 4, IBUS_TRANSMIT_TRIES);
            
        } else if (msg[1] == IBUS_MSG_OPENBM_GET_TICKS)
        {
            uint8_t data[6] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_TICKS, 0, 0, 0, 0};
            data[2] = (g_tickNumber & 0xFF000000) >> 24;
            data[3] = (g_tickNumber & 0x00FF0000) >> 16;
            data[4] = (g_tickNumber & 0x0000FF00) >> 8;
            data[5] = (g_tickNumber & 0x000000FF) >> 0;
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 6, IBUS_TRANSMIT_TRIES);
            
        }else if (msg[1] == IBUS_MSG_OPENBM_GET_DIMMER)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_DIMMER, g_backLightDimmer};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);

        }else if (msg[1] == IBUS_MSG_OPENBM_GET_TEMP)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_TEMP, g_temperatureSensor};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);

        // reset command .. 07 F0 FA BA AD FE ED ..
        }else if (msglen == 5 && msg[1] == 0xBA && msg[2] == 0xAD && msg[3] == 0xFE && msg[4] == 0xED)
        {
            power_reset_cpu();
        }else if (msglen == 3 && msg[1] == IBUS_MSG_OPENBM_SET_DISPLAY)
        {
            // get power and input states from the second byte
            uint8_t pwr = msg[2] & 0xF0;
            uint8_t inp = msg[2] & 0x0F;

            if (pwr == 0b01100000) display_powerOff();
            else if (pwr == 0b11110000) display_powerOn();
            display_setInputState(inp);
        }
    }
}

// ----------------------------------------------------------------------------
// Make sure the watchdog is disabled as soon as possible
// MCU doesn't disable the WDT after reset!
// ----------------------------------------------------------------------------
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
    // enable main MOSFET to control hardware power
    DDRC  |= (1 << DDC3);// | (1 << DDC4);
    PORTC |= (1 << 3);

    // disable wdt
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}


//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{
    power_set_running_mode(INIT);

    // disable not needed-hardware
    PRR = 0;                  // enable all hardware
    PRR |= (1 << PRUSART1);   // disable USART-1
    PRR |= (1 << PRSPI);      // disable SPI

    // init ibus as soon as possible, so that messages don't get lost while initialization procedure
    ibus_init();
    sei();
    
    // set full I/O clock speed regardles of the fuse
    cli();
    CLKPR = (1 << CLKPCE);
    CLKPR = 0;
    sei();

    // read this firmware version
    memcpy_P(&g_bootldrinfo, &bootLdrInfo, sizeof(bootldrinfo_t));//(PGM_VOID_P)(BOOTLOADERSTARTADR - SPM_PAGESIZE), sizeof(bootldrinfo_t));

    // read settings and setup hardware to the settings
    settings_readAndSetup();

    // init hardware
    tick_init();
    power_init();
    led_init();
    i2c_init();
    display_init();
    button_init();
    adc_init();
    photo_init();
    //ibus_init();

    // do small animation to indicate start
    /*{
        led_green_immediate_set(1);
        _delay_ms(50);
        led_green_immediate_set(0);
        led_red_immediate_set(1);
        _delay_ms(50);
        led_red_immediate_set(0);
        led_yellow_immediate_set(1);
        _delay_ms(50);
        led_yellow_immediate_set(0);
        led_fan_immediate_set(1);
        _delay_ms(50);
        led_fan_immediate_set(0);
        led_radio_immediate_set(1);
        _delay_ms(50);
        led_radio_immediate_set(0);
        _delay_ms(50);
    }*/

    //sei();
    
    ibus_setMessageCallback(ibus_MessageCallback);
    //display_setBackgroundLight(photo_get_max_value());
    //display_tryTurnOn();

    // init software modules
    mid_init();
    obms_init();
}

// -----------------------------------------------------------------------------
// Setup backcup camera input. if a = 0 then no backcup camera
// -----------------------------------------------------------------------------
void setBackCamInput(uint8_t a)
{
    g_deviceSettings.device_Settings1 &= 0xFC;
    g_deviceSettings.device_Settings1 |= (0x03 & a);
    eeprom_update_byte(&g_deviceSettingsEEPROM.device_Settings1, g_deviceSettings.device_Settings1);
}

// -----------------------------------------------------------------------------
// Executed when setup mode is activated. Buttons might be used to change settings
// -----------------------------------------------------------------------------
void checkSetupMode(void)
{
    // change to setup mode, when these three buttons are pressed long
    if (button_down_long(BUTTON_MODE))
    {
        if ((!g_setupMode && button_down(BUTTON_MENU_LR) && button_down(BUTTON_SELECT)) || g_setupMode)
        {
            g_setupMode = !g_setupMode;
            led_fan_immediate_set(g_setupMode);
            display_setBackgroundLight(0xFF);

            return;
        }
    }

    if (!g_setupMode) return;
    
    // if in setup mode, we can go to different settings menu by pressing different buttons
    static enum Settings
    {
        NO,
        PHOTO, PHOTO_MIN, PHOTO_MAX, PHOTO_CALIB_MAX, PHOTO_CALIB_MIN,
        BACKCAM,
        IO
    }_settings = NO;

    if (_settings != IO) // if not in the IO menu
    {
        if (button_pressed(BUTTON_1))
        {
            _settings = PHOTO;
            led_radioBlinkLock(1);
        }else if (button_pressed(BUTTON_2))
        {
            _settings = BACKCAM;
            led_radioBlinkLock(2);
        }else if (button_pressed(BUTTON_3))
        {
            _settings = IO;
            led_radioBlinkLock(3);
        }else if (button_down_long(BUTTON_RADIO_KNOB) && button_down(BUTTON_MENU_LR))
        {
            // Blink Once with LEDs
            led_radio_immediate_set(1);
            led_green_immediate_set(1);
            led_yellow_immediate_set(1);
            led_red_immediate_set(1);
            _delay_ms(500);
            led_radio_immediate_set(0);
            led_green_immediate_set(0);
            led_yellow_immediate_set(0);
            led_red_immediate_set(0);
            _delay_ms(500);
            led_radio_immediate_set(1);
            led_green_immediate_set(1);
            led_yellow_immediate_set(1);
            led_red_immediate_set(1);

            // erase EEPROM
            uint8_t i;
            uint16_t addr;
            uint8_t clear[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            uint8_t data[8];

            BEGIN_ATOMAR;

            // Write page by page using the block writing method
            for(addr = 0; addr < E2END; addr += 8)
            {
                eeprom_read_block((void *)&data[0], (const void *)addr, 8);
                for(i = 0; i < 8; i++)
                    if(data[i] != 0xFF)
                    {
                        eeprom_write_block((void*)&clear[0], (void*)addr, 8);
                        break;
                    }
            }
            END_ATOMAR;

            // turn LEDs off
            led_radio_immediate_set(0);
            led_green_immediate_set(0);
            led_yellow_immediate_set(0);
            led_red_immediate_set(0);

            g_setupMode = 0;

            power_reset_cpu();
        }
    }

    switch(_settings)
    {
    //change IO-settings:
    case IO:
    {
        static uint8_t selio = 0;

        // while selection is active check button press
        if (selio != 0)
        {
            buttonIndex_t but = button_whichDown();
            if (but != BUTTON_NUM_BUTTONS)
            {
                g_deviceSettings.io_assignment[selio] = but;
                eeprom_update_byte(&g_deviceSettingsEEPROM.io_assignment[selio], but);
                selio = 0;
                led_radioBlinkLock(2);
                _settings = NO;
            }
        }

        if (button_down_long(BUTTON_MENU_LR))
        {
            if (button_down(BUTTON_1))
            {
                selio = 0;
                led_greenBlinkLock(1);
            }
            else if (button_down(BUTTON_2))
            {
                selio = 1;
                led_greenBlinkLock(2);
            }
            else if (button_down(BUTTON_3))
            {
                selio = 2;
                led_greenBlinkLock(3);
            }else
                selio = 0;
        }

        led_red_immediate_set(selio != 0);

        break;
    }
    // change Photo-Sensor Settings
    case PHOTO:
    case PHOTO_MIN:
    case PHOTO_MAX:
    case PHOTO_CALIB_MIN:
    case PHOTO_CALIB_MAX:
    {
        static uint8_t savePhoto = 0;

        // --------- Toggle Use of photo sensor -------------
        if (button_pressed(BUTTON_PRG))
        {
            if (photo_is_enabled())
            {
                led_radioBlinkLock(1);
                photo_enable(false);
                display_setBackgroundLight(photo_get_max_value());
            }else
            {
                led_radioBlinkLock(2);
                photo_enable(true);
            }
        }
        
        // --------- Change minimum brightness of the background --------
        if (button_pressed(BUTTON_REW))
        {
            _settings = PHOTO_MIN;
            photo_enable(false);
            savePhoto = photo_get_min_value();
        }else if (button_pressed(BUTTON_FF))
        {
            _settings = PHOTO_MAX;
            photo_enable(false);
            savePhoto = photo_get_max_value();
        }else if (button_pressed(BUTTON_SELECT))
        {
            if (_settings == PHOTO_CALIB_MIN || _settings == PHOTO_CALIB_MAX)
            {
                _settings = PHOTO;
                //photo_setup_calibration();

            }else
                _settings = PHOTO_CALIB_MAX;

        }else if (button_pressed(BUTTON_BMBT_KNOB))
        {
            if (_settings == PHOTO_CALIB_MAX)
            {
                photo_set_max_calib_value(photo_get_adc_raw_value());
                _settings = PHOTO_CALIB_MIN;
            }else if (_settings == PHOTO_CALIB_MIN)
            {
                photo_set_min_calib_value(photo_get_adc_raw_value());
                _settings = PHOTO;
                //photo_setup_calibration();

            }else{
                photo_enable(true);
                if (_settings == PHOTO_MIN)
                    photo_set_min_value(savePhoto);
                else
                    photo_set_min_value(savePhoto);

                //photo_setup_calibration();
                _settings = PHOTO;
            }
            led_radioBlinkLock(2);
        }

        if (_settings == PHOTO_MIN || _settings == PHOTO_MAX)
        {
            int8_t enc = button_encoder(ENC_BMBT);
            int16_t val = (int16_t)savePhoto + (int16_t)enc;
            if (val < 0) val = 0;
            if (val > 0xFF) val = 0xFF;
            savePhoto = val & 0xFF;
            display_setBackgroundLight(savePhoto);
        }
        led_yellow_immediate_set(_settings == PHOTO_MIN || _settings == PHOTO_MAX);
        led_red_immediate_set(_settings == PHOTO_CALIB_MIN || _settings == PHOTO_CALIB_MAX);

        break;

    }
    case BACKCAM:
        // --------- Toggle Use of backup camera-------------
        if (button_pressed(BUTTON_PRG))
        {
            if (BACKCAM_INPUT())
            {
                led_radioBlinkLock(1);
                setBackCamInput(0);
            }else
            {
                led_radioBlinkLock(2);
                setBackCamInput(1);
            }
        }

        // --------------- Switch camera input -------------------
        if (button_pressed(BUTTON_REW))
        {
            led_greenBlinkLock(1);
            setBackCamInput(1);
        }else if (button_pressed(BUTTON_FF))
        {
            led_greenBlinkLock(2);
            setBackCamInput(2);
        }

        break;

    case NO:
    default:
        break;
    }

}
//------------------------------------------------------------------------------
// Set FUSES correctrly ( http://www.engbedded.com/fusecalc/):
// avrdude -c usbasp -p atmega324p -F -U lfuse:w:0xff:m -U hfuse:w:0xd8:m  (ext crystal, disable jtag, ckopt, boot reset)
//------------------------------------------------------------------------------
int main(void)
{
    initHardware();

    power_set_running_mode(RUN);

    for(;;)
    {
        // goto sleep only if ibus queue is empty
        if (ibus_isQueueFree())
            sleepCPU();

        // update current encoder states, need this here to eventually reset
        // the encoder rotary state to predefined value
        button_tick_encoder();

        // ------------ Task timer has shooted, so perfrom repeated task -------
        if (tick_event())
        {
            // update software modules which are independent of setup and running modes
            power_tick();
            button_tick();
            photo_tick();
            display_updateState();

            // update software modules
            if (!g_setupMode)
            {
                led_tick();
                display_setBackgroundLight(photo_get_value());
                if (power_get_running_mode() == RUN)
                {
                    mid_tick();
                    obms_tick();
                }
            }
            led_green_immediate_set(BACKCAM_INPUT() && display_getInputState() == BACKCAM_INPUT() && !g_setupMode);

            // Enable Setup-Mode, i.e. settable settings per buttons
            checkSetupMode();

            // perform global tick
            button_after_tick();
            tick();
        }

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
    // values specific to the DC electric coefficients of the temperature sensor
    #define TEMP_ADC_ZERO 25
    #define TEMP_INV_DC 2
    
    BEGIN_ATOMAR;
    {
        if (g_adcCurrentChannel == 0)  // we were converting PhotoSensor values
        {
            photo_set_adc_raw_value(ADCH);

            // switch channel (read on ADC0 next)
            g_adcCurrentChannel = 1;
            ADMUX &= ~(1 << MUX4) & ~(1 << MUX3) & ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
        }else if (g_adcCurrentChannel == 1)  // we were converting Backlight Dimmer values
        {
            g_backLightDimmer = ADCH;

            // switch channel (read on ADC1 next)
            g_adcCurrentChannel = 2;
            ADMUX &= ~(1 << MUX4) & ~(1 << MUX3);
            ADMUX |= (1 << MUX0);
        }else  // we were converting Temperature Sensor values
        {
            // convert voltage into human readable temperature (-128, +128)
            g_temperatureSensor = (int8_t)((ADCH - TEMP_ADC_ZERO) * TEMP_INV_DC);

            // switch channel (read on ADC7 next)
            g_adcCurrentChannel = 0;
            ADMUX &= ~(1 << MUX4) & ~(1 << MUX3);
            ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2);
        }
        //ADCSRA |= (1 << ADSC);
    }
    END_ATOMAR;
}


//------------------------------------------------------------------------------
// Timer interrupt on ticks
//------------------------------------------------------------------------------
tick_interrupt()
{
    g_tickEventHappened = 1;
}
