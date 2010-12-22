#include "display.h"
#include "leds.h"
#include "buttons.h"
#include "config.h"
#include "base.h"
#include <avr/eeprom.h>
#include "i2cmaster.h"

//------------------------------------------------------------------------------
// Current display state, (on/off, input)
//------------------------------------------------------------------------------
typedef struct _DisplayState
{
    uint8_t display_Power;
    uint8_t display_Input; // (0=vga, 1=av1, 2=av2)
}DisplayState;

// current hardware state
DisplayState g_DisplayState;
DisplayState g_eeprom_DisplayState EEMEM;
uint8_t      g_eeprom_DisplayDataInit EEMEM;

uint16_t     max_dacVoltage;
uint16_t     dac_currentVoltage;

ticks_t g_display_NextResponseTime;
uint8_t g_displayError;

static DeviceSettings* deviceSettings;
static DeviceSettings* deviceSettingsEEPROM;

#define DISP_MOSFET_SETUP {DDRB |= (1 << DDB4); PORTB &= ~(1 << 4);}
#define DISP_MOSFET_OFF {PORTB &= ~(1 << 4);}
#define DISP_MOSFET_ON  {PORTB |= (1 << 4);}

//------------------------------------------------------------------------------
uint8_t display_getPowerState()
{
    return g_DisplayState.display_Power;
}

//------------------------------------------------------------------------------
uint8_t display_getInputState()
{
    return g_DisplayState.display_Input;
}

//------------------------------------------------------------------------------
void display_dac_sleep(void)
{
    // send dac into sleep mode
    uint8_t cmd1 = (3 << 4) | (uint8_t)((dac_currentVoltage >> 8) & 0x0F);
    uint8_t cmd2 = dac_currentVoltage & 0xFF;

    // program DAC
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 10)) == 0)
    {
        i2c_write(cmd1);
        i2c_write(cmd2);
        i2c_write(cmd1);   // repeat due to the specification of MCP4725
        i2c_write(cmd2);   // repeat due to the specification of MCP4725
        i2c_stop();
    }
}

//------------------------------------------------------------------------------
void display_dac_setVoltage_fast(uint16_t value)
{
    // truncate given voltage level
    if (value > max_dacVoltage) value = max_dacVoltage;
    dac_currentVoltage = value;

    // create both command bytes to be sent
    uint8_t cmd1 = (uint8_t)((value >> 8) & 0x0F);
    uint8_t cmd2 = value & 0xFF;

    // program DAC
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 10)) == 0)
    {
        i2c_write(cmd1);
        i2c_write(cmd2);
        i2c_write(cmd1);   // repeat due to the specification of MCP4725
        i2c_write(cmd2);   // repeat due to the specification of MCP4725
        i2c_stop();
    }
}

//------------------------------------------------------------------------------
void display_dac_setVoltage(uint16_t value)
{
    // truncate given voltage level
    if (value > max_dacVoltage) value = max_dacVoltage;
    dac_currentVoltage = value;

    // create both command bytes to be sent
    uint8_t cmd1 = 0b01100000;    // write DAC+EEPROM and no power down mode
    uint8_t cmd2 = (uint8_t)((value >> 4) & 0xFF);
    uint8_t cmd3 = (value & 0x0F) << 4;;

    // program DAC
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 10)) == 0)
    {
        i2c_write(cmd1);
        i2c_write(cmd2);
        i2c_write(cmd3);
        i2c_write(cmd1);   // repeat due to the specification of MCP4725
        i2c_write(cmd2);   // repeat due to the specification of MCP4725
        i2c_write(cmd3);   // repeat due to the specification of MCP4725
        i2c_stop();
    }
}

//------------------------------------------------------------------------------
void display_setVoltagePower(uint16_t data)
{
    if (data > max_dacVoltage) data = max_dacVoltage;
    deviceSettings->dac_PowerKey = data;
    eeprom_update_word(&deviceSettingsEEPROM->dac_PowerKey, data);
}

//------------------------------------------------------------------------------
uint16_t display_getVoltagePower(void)
{
    return deviceSettings->dac_PowerKey;
}

//------------------------------------------------------------------------------
void display_setVoltageSwitch(uint16_t data)
{
    if (data > max_dacVoltage) data = max_dacVoltage;
    deviceSettings->dac_SwitchKey = data;
    eeprom_update_word(&deviceSettingsEEPROM->dac_SwitchKey, data);
}

//------------------------------------------------------------------------------
uint16_t display_getVoltageSwitch(void)
{
    return deviceSettings->dac_SwitchKey;
}

//------------------------------------------------------------------------------
void display_init(void)
{
    // based on the jumper settings we enable either 3.3V or 5V as idle voltage
    DDRB &= ~(1 << DDB3); // set Jumper pin to input mode
    PORTB |= (1 << 3);    // enable pull-up

    if (bit_is_set(PINB,3)) // bit is 1, so pull-up, so 5V as maximum
        max_dacVoltage = 0xFFF; // nominal output voltage is 5V
    else
        max_dacVoltage = 0xA8F; // Data = 3.3V * 4096 / 5V;

    // set variables
    deviceSettings = &g_deviceSettings;
    deviceSettingsEEPROM = &g_deviceSettingsEEPROM;

    // disable DAC output and switch display mosfet off
    dac_currentVoltage = deviceSettings->dac_idleVoltage;
    display_dac_sleep();

    DISP_MOSFET_SETUP;
    DISP_MOSFET_OFF;

    // if run for the first time, then write default data to eeprom
    if (eeprom_read_byte(&g_eeprom_DisplayDataInit) != 'K')
    {
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, 1);
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, 0);
        eeprom_write_byte(&g_eeprom_DisplayDataInit, 'K');
    }

    // load current display state from the eeprom
    g_DisplayState.display_Input = eeprom_read_byte(&g_eeprom_DisplayState.display_Input);
    g_DisplayState.display_Power = eeprom_read_byte(&g_eeprom_DisplayState.display_Power);
    g_display_NextResponseTime = 0;
    g_displayError = 0;

    // setup PWM for the bglight
    DDRD |= (1 << DDD7);
    TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS21) | (1 << CS20);
    PORTD |= (1 << 7);
    TCNT2 = 0;
    OCR2A = 0xFF;//g_DisplayState.bglight_maxDuty;

    // if display was previously on, then enable power state
    display_dac_setVoltage(deviceSettings->dac_idleVoltage);
    DISP_MOSFET_ON;
}

//------------------------------------------------------------------------------
void display_TogglePower(uint8_t writeToEeprom)
{
    // emulate key
    display_dac_setVoltage_fast(deviceSettings->dac_PowerKey);
    _delay_ms(100);
    display_dac_setVoltage_fast(deviceSettings->dac_idleVoltage);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Power = !g_DisplayState.display_Power;
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
    }
    _delay_ms(500);

    //g_display_NextResponseTime = tick_get() + TICKS_PER_QUARTERSECOND();
    g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND() - TICKS_PER_QUARTERSECOND();
}

//------------------------------------------------------------------------------
void display_ToggleInput(uint8_t writeToEeprom)
{
    // emulate key
    display_dac_setVoltage_fast(deviceSettings->dac_SwitchKey);
    _delay_ms(100);
    display_dac_setVoltage_fast(deviceSettings->dac_idleVoltage);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Input = (g_DisplayState.display_Input + 1) % 3;
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
    }
    _delay_ms(500);
    
    //g_display_NextResponseTime = tick_get() + TICKS_PER_QUARTERSECOND();
    g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND() - TICKS_PER_QUARTERSECOND();
}

//------------------------------------------------------------------------------
void display_savePowerState(uint8_t state)
{
    g_DisplayState.display_Power = state;
    eeprom_update_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
}


//------------------------------------------------------------------------------
void display_saveInputState(uint8_t state)
{
    g_DisplayState.display_Input = state;
    eeprom_update_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
}

//------------------------------------------------------------------------------
void display_setPowerState(uint8_t state)
{
    if (g_DisplayState.display_Power == state) return;
    display_TogglePower(1);
}

//------------------------------------------------------------------------------
void display_setInputState(uint8_t state)
{
    if (g_DisplayState.display_Input == state) return;

    uint8_t oldState = g_DisplayState.display_Input;
    if ((oldState == 0 && state == 1)
      ||(oldState == 1 && state == 2)
      ||(oldState == 2 && state == 0))
    {
        display_ToggleInput(1);
    }else if (
        (oldState == 0 && state == 2)
      ||(oldState == 1 && state == 0)
      ||(oldState == 2 && state == 1)
            )
    {
        display_ToggleInput(1);
        display_ToggleInput(1);
    }

    display_saveInputState(state);
}

//------------------------------------------------------------------------------
void display_setBackgroundLight(uint8_t duty)
{
    OCR2A = duty;
}

//------------------------------------------------------------------------------
uint8_t display_getBackgroundLight(void)
{
    return OCR2A;
}

//------------------------------------------------------------------------------
void display_powerOn(void)
{
    display_dac_setVoltage(deviceSettings->dac_idleVoltage);
    DISP_MOSFET_ON;
}

//------------------------------------------------------------------------------
void display_shutDown(void)
{
    // save display settings
    display_savePowerState(g_DisplayState.display_Power);
    display_saveInputState(g_DisplayState.display_Input);

    // put DAC to sleep
    display_dac_sleep();

    // diable MOSFET
    DISP_MOSFET_OFF;
}

//------------------------------------------------------------------------------
void display_updateState(void)
{
    static uint8_t ignoreButtons = 0;
    if (tick_get() < g_display_NextResponseTime) return;

    #if 0
    //if (button_down(BUTTON_DISP) && button_down(BUTTON_MODE))
    {
        register uint8_t light = display_getBackgroundLight();

        if (button_down(BUTTON_FF) && light < 0xFF)
        {
            light++;
            display_setBackgroundLight(light);
        }else if (button_down(BUTTON_REW) && light > 0)
        {
            light--;
            display_setBackgroundLight(light);
        }
    }
    #endif

    // if holding button longer than certain time, then turn off screen if it is on
    //else
    if (button_down_long(BUTTON_DISP))// && ignoreButtons == 0)
    {
        // if time elapsed and also SELECT and MENU button was hold, then set display
        // state to predefined value (on, vga). this will make sure that display state
        // is synchronized with the display itself
        if (button_down(BUTTON_MENU_LR) && button_down(BUTTON_SELECT))
        {
            // ok reset the state to a default value (vga, power on)
            display_savePowerState(1);
            display_saveInputState(0);

            // indicate by LEDs
            led_radio_immediate_set(1);
            _delay_ms(500);
            led_radio_immediate_set(0);
            _delay_ms(150);
            led_radio_immediate_set(1);
            _delay_ms(300);
            led_radio_immediate_set(0);
            _delay_ms(150);
            led_radio_immediate_set(1);
            _delay_ms(300);
            led_radio_immediate_set(0);
            _delay_ms(150);
            led_radio_immediate_set(1);
            _delay_ms(300);
            led_radio_immediate_set(0);

        }else if (g_DisplayState.display_Power && ignoreButtons == 0)
            display_setPowerState(0);

        // ignore all next down times
        ignoreButtons = 1;

    }else if (button_released(BUTTON_DISP))
    {
        if (ignoreButtons)
        {
            ignoreButtons = 0;
            return;
        }

        // if display was off, then turn it on
        if (!g_DisplayState.display_Power)
            display_setPowerState(1);

        // if display was on, then just switch inputs
        else
        {
            display_ToggleInput(1);
        }
    }

}

