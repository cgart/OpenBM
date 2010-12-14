/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include "config.h"
#include "ibus.h"
#include <uart.h>
#include <i2cmaster.h>
#include <avr/wdt.h>
#include "display.h"
#include "buttons.h"
#include "leds.h"
#include "emul_mid.h"
#include <avr/boot.h>
//#include "bootloader/bootloader.h"

// global tick counter
ticks_t g_tickNumber = 0;
uint8_t g_tickEventHappened = 0;
volatile ticks_t g_nextIbusTick = 0;

#define UPDATE_SLEEP_COUNTER() { g_nextIbusTick = tick_get() + TICKS_PER_SECOND() * 300L; }

// current adc value
volatile uint8_t g_backLightDimmer = 0;
volatile int8_t g_temperatureSensor = 0;
volatile uint8_t g_adcCurrentChannel = 0;

static bootldrinfo_t g_bootldrinfo;

//! Device settings used
DeviceSettings g_deviceSettings;
DeviceSettings g_deviceSettingsEEPROM EEMEM;

//! Photo sensor ring buffer
#define PHOTO_NUM_SAMPLES_EXP 5
#define PHOTO_NUM_SAMPLES (1 << PHOTO_NUM_SAMPLES_EXP)
uint8_t photoSensorValues[PHOTO_NUM_SAMPLES];
uint8_t photoSensorIndex = 0;
uint16_t photoSensorSum = 0;
volatile uint8_t photoSensorLast = 0;
volatile uint8_t photoSensorChanged = 0;

//------------------------------------------------------------------------------
// Read device settings from eeprom
//------------------------------------------------------------------------------
void settings_readAndSetup(void)
{
    #ifdef BOOTLOADER_ACTIVE
    // dummy commands to prevent compiler to optimize bootloader away
    uint8_t dummy = eeprom_read_byte(&bin_data[0]);
    eeprom_update_byte(&bin_data[0], dummy);
    #endif

    // setup default settings if not in eeprom
    if (eeprom_read_byte(&g_deviceSettingsEEPROM.initSeed) != 'K')
    {
        eeprom_update_byte(&g_deviceSettingsEEPROM.initSeed, 'K');

        // --------------------
        // Display
        // --------------------
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_idleVoltage, DEVICE_DISP_IDLE);
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_SwitchKey, DEVICE_DISP_SWITCH);
        eeprom_update_word(&g_deviceSettingsEEPROM.dac_PowerKey, DEVICE_DISP_POWER);

        // --------------------
        // Hardware settings
        // --------------------
        eeprom_update_byte(&g_deviceSettingsEEPROM.photo_minValue, 50);
        eeprom_update_byte(&g_deviceSettingsEEPROM.photo_maxValue, 0xFF);

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

    g_deviceSettings.photo_minValue  = eeprom_read_byte(&g_deviceSettingsEEPROM.photo_minValue);
    g_deviceSettings.photo_maxValue = eeprom_read_byte(&g_deviceSettingsEEPROM.photo_maxValue);

    g_deviceSettings.device_Settings1 = eeprom_read_byte(&g_deviceSettingsEEPROM.device_Settings1);
    g_deviceSettings.device_Settings2 = eeprom_read_byte(&g_deviceSettingsEEPROM.device_Settings2);
}

//------------------------------------------------------------------------------
// Update average photo sensor values (average filter)
//------------------------------------------------------------------------------
inline uint8_t getPhotoSensor(void)
{
    return (photoSensorSum >> PHOTO_NUM_SAMPLES_EXP);
}
uint8_t updatePhotoSensor(uint8_t val)
{
    // remove last element of the ring buffer and add new value to the sum
    photoSensorSum -= photoSensorValues[(photoSensorIndex + 1) & (PHOTO_NUM_SAMPLES-1)];
    photoSensorSum += val;
    
    // add new value into the ring buffer
    photoSensorValues[photoSensorIndex++] = val;
    photoSensorIndex = photoSensorIndex & (PHOTO_NUM_SAMPLES-1);

    return getPhotoSensor();
}

//------------------------------------------------------------------------------
// Initialize analog hardware (read photo sensor, read bg light)
//------------------------------------------------------------------------------
void adc_init(void)
{
    g_temperatureSensor = 0;
    g_backLightDimmer = 0;
    g_adcCurrentChannel = 0;
    
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 109KHz sample rate @ 14MHz
    ADMUX = (1 << REFS0); // Set ADC reference to AVCC with external capacitor
    ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
    ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2); // enable reading on ADC7
    //ADCSRB &= ~(1 << ADTS2) & ~(1 << ADTS1) & ~(1 << ADTS0);  // Set ADC to Free-Running Mode
    ADCSRB = (1 << ADTS1) | (1 << ADTS0);  // Set ADC to start on Timer0 compare match
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    ADCSRA |= (1 << ADEN);  // Enable ADC

    DIDR0 = 0xFF; // disable digital inputs on the complete ADC port/ A.port

    // start reading from ADC
    ADCSRA |= (1 << ADSC);
}

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
}

#if 0
// ----------------------------------------------------------------------------
/* Make sure the watchdog is disabled as soon as possible    */
/* MCU doesn't disable the WDT after reset!                  */
// ----------------------------------------------------------------------------
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));

void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
#endif

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
    UPDATE_SLEEP_COUNTER();
    
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
        if (msg[1] == IBUS_MSG_OPENBM_GET_VERSION)
        {
            uint8_t data[4] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_VERSION, g_bootldrinfo.app_version>>8, g_bootldrinfo.app_version&0xFF};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 4, 3);
            
        } else if (msg[1] == IBUS_MSG_OPENBM_GET_TICKS)
        {
            uint8_t data[6] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_TICKS, 0, 0, 0, 0};
            data[2] = (g_tickNumber & 0xFF000000) >> 24;
            data[3] = (g_tickNumber & 0x00FF0000) >> 16;
            data[4] = (g_tickNumber & 0x0000FF00) >> 8;
            data[5] = (g_tickNumber & 0x000000FF) >> 0;
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 6, 3);
            
        }else if (msg[1] == IBUS_MSG_OPENBM_GET_PHOTO)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, getPhotoSensor()};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, 3);

        }else if (msg[1] == IBUS_MSG_OPENBM_GET_DIMMER)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_DIMMER, g_backLightDimmer};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, 3);

        }else if (msg[1] == IBUS_MSG_OPENBM_GET_TEMP)
        {
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_TEMP, g_temperatureSensor};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, 3);

        // reset command F0 07 .. FA BA AD FE ED ..
        }else if (msglen == 5 && msg[1] == 0xBA && msg[2] == 0xAD && msg[3] == 0xFE && msg[4] == 0xED)
        {
            resetCPU();
        }
        
    }
}

//------------------------------------------------------------------------------
// Full shut down of the main board (MOSFET turn off signale)
//------------------------------------------------------------------------------
void shutDown(void)
{
    display_shutDown();         // disbale display
    PORTC &= ~(1 << 3);         // shut down main board
}

//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{
    // enable main MOSFET to control hardware power
    DDRC |= (1 << DDC3);
    PORTC |= (1 << 3);

    // disable not needed-hardware
    PRR = 0;
    PRR |= (1 << PRUSART1);   // disable USART-1
    PRR |= (1 << PRSPI);      // disable SPI

    // prepare averaging of photo sensor values
    memset(&photoSensorValues[0], 0, PHOTO_NUM_SAMPLES);

    // read this firmware version
    memcpy_P(&g_bootldrinfo, (PGM_VOID_P)(BOOTLOADERSTARTADR - SPM_PAGESIZE), sizeof(bootldrinfo_t));

    // read settings and setup hardware to the settings
    settings_readAndSetup();

    // init hardware
    tick_init();
    led_init();
    i2c_init();
    display_init();
    button_init();
    adc_init();
    emul_mid_init();
    ibus_init();
    
    // do small animation to indicate start
    {
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
    }

    sei();
    
    ibus_setMessageCallback(ibus_MessageCallback);

    UPDATE_SLEEP_COUNTER();
}


//------------------------------------------------------------------------------
// Set FUSES correctrly ( http://www.engbedded.com/fusecalc/):
// avrdude -U lfuse:w:0xff:m -U hfuse:w:0xc9:m  (ext crystal, disable jtag, ckopt)
//------------------------------------------------------------------------------
int main(void)
{
    // init hardware and do small delay after init
    initHardware();
    
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
            led_green_immediate_set(display_getInputState() == BACKCAM_INPUT());

            // go into full sleep mode if there is no action on the ibus
            // happens during the last X seconds (full shut down!!!)
            if (tick_get() > g_nextIbusTick)
                shutDown();

            // based on ambient light we setup display's brightness
            display_setBackgroundLight(getPhotoSensor());

            // perform global tick
            tick();
        }
        
        // ----------- Do full reset if user liked so --------------------------
        if (button_released(BUTTON_MENU_LR) && button(BUTTON_SELECT) && button(BUTTON_EJECT))
            resetCPU();

        // update photo sensor if value has changed
        if (photoSensorChanged)
        {
            if (photoSensorLast > g_deviceSettings.photo_maxValue) photoSensorLast = g_deviceSettings.photo_maxValue;
            if (photoSensorLast < g_deviceSettings.photo_minValue) photoSensorLast = g_deviceSettings.photo_minValue;
            updatePhotoSensor(photoSensorLast);
            photoSensorChanged = 0;
        }

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
    // values specific to the DC electric coefficients of the temperature sensor
    #define TEMP_ADC_ZERO 25
    #define TEMP_INV_DC 2
    #define PHOTO_MIN 50
    
    BEGIN_ATOMAR;
    {
        if (g_adcCurrentChannel == 0)  // we were converting PhotoSensor values
        {
            uint8_t val = ADCH & 0xFC;

            if (photoSensorChanged == 0 && val != photoSensorLast)
            {
                photoSensorChanged = 1;
                photoSensorLast = val;
            }

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
        ADCSRA |= (1 << ADSC);
    }
    END_ATOMAR;
}

//------------------------------------------------------------------------------
// On any button action this interrupt will be called.
// We use this interrupt to wake up our main CPU and let button handler know
// that there is something happening with the buttons
//------------------------------------------------------------------------------
ISR(INT1_vect)
{
    UPDATE_SLEEP_COUNTER();
    button_isr();
}


//------------------------------------------------------------------------------
// Timer interrupt on ticks
//------------------------------------------------------------------------------
ISR(TIMER0_COMPA_vect)
{
    g_tickEventHappened = 1;
}
