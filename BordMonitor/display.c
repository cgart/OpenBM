#include "display.h"
#include "leds.h"
#include "buttons.h"
#include "config.h"
#include "base.h"
#include "i2cmaster.h"
#include "include/base.h"
#include "include/leds.h"
#include <avr/eeprom.h>

//------------------------------------------------------------------------------
// Current display state, (on/off, input)
//------------------------------------------------------------------------------
typedef struct _DisplayState
{
    uint8_t display_Power;
    uint8_t display_Input; // (0=vga, 1=av1, 2=av2)

    // --------------------------------------
    // Voltage settings
    // --------------------------------------
    uint16_t dac_maxVoltage;  // in DAC value 12bit (either 3.3V or 5V key)
    uint16_t dac_idleVoltage;   // voltage used when idle
    uint16_t dac_PowerKey;      // voltage when Power-Button pressed
    uint16_t dac_SwitchKey;     // voltage when button to switch input pressed
    uint16_t dac_MenuKey;       // voltage when button menu pressed
    uint16_t dac_IncKey;        // voltage when button inc pressed
    uint16_t dac_DecKey;        // voltage when button dec pressed

}DisplayState;

// current hardware state
static DisplayState g_DisplayState;
static DisplayState g_eeprom_DisplayState EEMEM;
static uint8_t      g_eeprom_DisplayDataInit EEMEM;

static uint16_t     dac_currentVoltage;

static ticks_t g_display_NextResponseTime;
static uint8_t g_displayError;

static ticks_t _nextStopOSD;

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
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 100)) == 0)
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
    if (value > g_DisplayState.dac_maxVoltage) value = g_DisplayState.dac_maxVoltage;
    dac_currentVoltage = value;

    // create both command bytes to be sent
    uint8_t cmd1 = (uint8_t)((value >> 8) & 0x0F);
    uint8_t cmd2 = value & 0xFF;

    // program DAC
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 100)) == 0)
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
    if (value > g_DisplayState.dac_maxVoltage) value = g_DisplayState.dac_maxVoltage;
    dac_currentVoltage = value;

    // create both command bytes to be sent
    uint8_t cmd1 = 0b01100000;    // write DAC+EEPROM and no power down mode
    uint8_t cmd2 = (uint8_t)((value >> 4) & 0xFF);
    uint8_t cmd3 = (value & 0x0F) << 4;;

    // program DAC
    if ((g_displayError = i2c_start_wait(DAC_I2C_ADDRESS + I2C_WRITE, 100)) == 0)
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
    if (data > g_DisplayState.dac_maxVoltage) data = g_DisplayState.dac_maxVoltage;
    g_DisplayState.dac_PowerKey = data;
    eeprom_update_word(&g_eeprom_DisplayState.dac_PowerKey, data);
}

//------------------------------------------------------------------------------
uint16_t display_getVoltagePower(void)
{
    return g_DisplayState.dac_PowerKey;
}

//------------------------------------------------------------------------------
void display_setVoltageSwitch(uint16_t data)
{
    if (data > g_DisplayState.dac_maxVoltage) data = g_DisplayState.dac_maxVoltage;
    g_DisplayState.dac_SwitchKey = data;
    eeprom_update_word(&g_eeprom_DisplayState.dac_SwitchKey, data);
}

//------------------------------------------------------------------------------
uint16_t display_getVoltageSwitch(void)
{
    return g_DisplayState.dac_SwitchKey;
}

//------------------------------------------------------------------------------
void display_init(void)
{
    // based on the jumper settings we enable either 3.3V or 5V as idle voltage
    DDRB &= ~(1 << DDB3); // set Jumper pin to input mode
    PORTB |= (1 << 3);    // enable pull-up, will refer to this later, so that we don't get spurious data here

    DISP_MOSFET_SETUP;
    DISP_MOSFET_OFF;

    // disable display per default and read max voltage levels
    if (bit_is_set(PINB,3)) // bit is 1, so pull-up, so 5V as maximum
        g_DisplayState.dac_maxVoltage = 0xFFF; // nominal output voltage is 5V
    else
        g_DisplayState.dac_maxVoltage = 0xA8F; // Data = 3.3V * 4096 / 5V;

    // if run for the first time, then write default data to eeprom
    if (eeprom_read_byte(&g_eeprom_DisplayDataInit) != 'E' || eeprom_read_word(&g_eeprom_DisplayState.dac_maxVoltage) != g_DisplayState.dac_maxVoltage)
    {
        eeprom_write_byte(&g_eeprom_DisplayDataInit, 'E');
        
        // default state
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, 1);
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, 0);

        #define DEVICE_DISP_SWITCH   (((DEVID_2 << 4L) | ((DEVID_3 & 0xF0) >> 4L)) & 0xFFF)
        #define DEVICE_DISP_POWER    ((((DEVID_3 & 0x0F) << 8L) | (DEVID_4 & 0xFF)) & 0xFFF)
        #define DEVICE_DISP_MENU     (((DEVID_5 << 4L) | ((DEVID_6 & 0xF0) >> 4L)) & 0xFFF)
        #define DEVICE_DISP_INC      ((((DEVID_6 & 0x0F) << 8L) | (DEVID_7 & 0xFF)) & 0xFFF)
        #define DEVICE_DISP_DEC      (((DEVID_8 << 4L) | ((DEVID_9 & 0xF0) >> 4L)) & 0xFFF)
        #define DEVICE_DISP_IDLE     ((((DEVID_9 & 0x0F) << 8L) | (DEVID_10 & 0xFF)) & 0xFFF)

        eeprom_update_word(&g_eeprom_DisplayState.dac_SwitchKey, DEVICE_DISP_SWITCH);
        eeprom_update_word(&g_eeprom_DisplayState.dac_PowerKey, DEVICE_DISP_POWER);
        eeprom_update_word(&g_eeprom_DisplayState.dac_MenuKey, DEVICE_DISP_MENU);
        eeprom_update_word(&g_eeprom_DisplayState.dac_IncKey, DEVICE_DISP_INC);
        eeprom_update_word(&g_eeprom_DisplayState.dac_DecKey, DEVICE_DISP_DEC);
        eeprom_update_word(&g_eeprom_DisplayState.dac_idleVoltage, DEVICE_DISP_IDLE);
        
    }

    // load current display state from the eeprom
    g_DisplayState.display_Input = eeprom_read_byte(&g_eeprom_DisplayState.display_Input);
    g_DisplayState.display_Power = eeprom_read_byte(&g_eeprom_DisplayState.display_Power);

    g_DisplayState.dac_idleVoltage  = eeprom_read_word(&g_eeprom_DisplayState.dac_idleVoltage);
    g_DisplayState.dac_PowerKey  = eeprom_read_word(&g_eeprom_DisplayState.dac_PowerKey);
    g_DisplayState.dac_SwitchKey = eeprom_read_word(&g_eeprom_DisplayState.dac_SwitchKey);
    g_DisplayState.dac_MenuKey  = eeprom_read_word(&g_eeprom_DisplayState.dac_MenuKey);
    g_DisplayState.dac_IncKey  = eeprom_read_word(&g_eeprom_DisplayState.dac_IncKey);
    g_DisplayState.dac_DecKey = eeprom_read_word(&g_eeprom_DisplayState.dac_DecKey);

    g_display_NextResponseTime = 0;
    g_displayError = 0;
    _nextStopOSD = 0;

    // setup PWM for the bglight
    DDRD |= (1 << DDD7);
    TCCR2A = (1 << COM2A1) | (1 << COM2A0) /*| (1 << WGM21) |*/ |  (1 << WGM20);
    TCCR2B = (1 << CS22)  | (0 << CS21) | (0 << CS20);
    //TCCR2B = (0 << CS22)  | (1 << CS21) | (0 << CS20);
    TCNT2 = 0;
    OCR2A = 0x80;//g_DisplayState.bglight_maxDuty;
    PORTD |= (1 << 7);


    // set variables
    //display_dac_setVoltage(g_DisplayState.dac_idleVoltage);

    // disable DAC output and switch display mosfet off
    display_dac_sleep();

    // if display was previously off, then don't enable it again
    //if (g_DisplayState.display_Power)
    //    display_powerOn();
}

//------------------------------------------------------------------------------
void display_ToggleKey(uint16_t keyVoltage)
{
    display_dac_setVoltage_fast(keyVoltage);
    _delay_ms(200);
    display_dac_setVoltage_fast(g_DisplayState.dac_idleVoltage);
}

//------------------------------------------------------------------------------
void display_enableBackupCameraInput(uint8_t en)
{
    uint8_t val = 0xFF;
    if (en) val &= ~(1 << 4);
    if (i2c_start(PORT_EXPANDER_ENC_BMBT + I2C_WRITE) == 0)
    {
        i2c_write(val);
        i2c_stop();
    }
}

#if 1
//------------------------------------------------------------------------------
void display_TogglePower(uint8_t writeToEeprom)
{
    // emulate key
    display_ToggleKey(g_DisplayState.dac_PowerKey);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Power = !g_DisplayState.display_Power;
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
    }
    _delay_ms(500); // TODO look how to remove that!!!

    g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND - TICKS_PER_QUARTERSECOND;
}
#endif

//------------------------------------------------------------------------------
void display_ToggleInput(uint8_t writeToEeprom)
{
    // emulate key
    display_ToggleKey(g_DisplayState.dac_SwitchKey);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Input = (g_DisplayState.display_Input + 1) % 3;
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
    }
    _delay_ms(500); // TODO look how to remove that!!!
    
    //g_display_NextResponseTime = tick_get() + TICKS_PER_QUARTERSECOND();
    g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND - TICKS_PER_QUARTERSECOND;
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


#if 0
//------------------------------------------------------------------------------
void display_setPowerState(uint8_t state, bool saveEEPROM)
{
    if (g_DisplayState.display_Power == state) return;
    display_TogglePower(saveEEPROM);
}
#endif

//------------------------------------------------------------------------------
void display_updateInputState(uint8_t state)
{
    g_DisplayState.display_Input = state;
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

    //display_saveInputState(state);
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
    g_DisplayState.display_Power = 1;
    display_dac_setVoltage(g_DisplayState.dac_idleVoltage);
    DISP_MOSFET_ON;
    eeprom_update_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
    g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND - TICKS_PER_QUARTERSECOND;
}

//------------------------------------------------------------------------------
void display_powerOff(void)
{
    g_DisplayState.display_Power = 0;

    // save display settings
    display_savePowerState(g_DisplayState.display_Power);
    display_saveInputState(g_DisplayState.display_Input);

    display_dac_sleep();
    DISP_MOSFET_OFF;
}

//------------------------------------------------------------------------------
void display_turnOff(void)
{
    display_dac_sleep();
    DISP_MOSFET_OFF;
}

//------------------------------------------------------------------------------
void display_tryTurnOn(void)
{
    if (eeprom_read_byte(&g_eeprom_DisplayState.display_Power))
    {
        display_dac_setVoltage(g_DisplayState.dac_idleVoltage);
        DISP_MOSFET_ON;
        g_display_NextResponseTime = tick_get() + TICKS_PER_SECOND - TICKS_PER_QUARTERSECOND;
    }
}

//------------------------------------------------------------------------------
void display_updateState(void)
{
    //led_yellow_immediate_set(tick_get() < _nextStopOSD);

    // check if we have switched to the OSD menu of the main TFT
    if (tick_get() < _nextStopOSD)
    {
        bool prolong = false;

        int8_t bmbt = button_encoder(ENC_BMBT);

        // emulate keys to browse through OSD menu
        if (button_released(BUTTON_DISP))
        {
            display_ToggleKey(g_DisplayState.dac_MenuKey);
            prolong = true;
        }else if (button_released(BUTTON_INFO_R))
        {
            display_ToggleKey(g_DisplayState.dac_SwitchKey);
            prolong = true;
        }
        if (bmbt < 0)
        {
            display_ToggleKey(g_DisplayState.dac_DecKey);
            prolong = true;
        }else if (bmbt > 0)
        {
            display_ToggleKey(g_DisplayState.dac_IncKey);
            prolong = true;
        }

        if (prolong)
            _nextStopOSD = tick_get() + TICKS_PER_X_SECONDS(2);

        return;

    // check if user want to switch to the OSD menu (only when not already there)
    }else if (button_released(BUTTON_DISP) && button_down(BUTTON_MENU_LR))
    {
        display_ToggleKey(g_DisplayState.dac_MenuKey);
        _nextStopOSD = tick_get() + TICKS_PER_X_SECONDS(2);

        return;
    }

    static uint8_t ignoreButtons = 0;
    if (tick_get() < g_display_NextResponseTime) return;

    if (button_down_long(BUTTON_INFO_R))
        display_TogglePower(false);

    // if holding button longer than certain time, then turn off screen if it is on
    if (button_down_long(BUTTON_DISP) && ignoreButtons == 0)
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
            led_radioBlinkLock(3);

        }else if (g_DisplayState.display_Power && ignoreButtons == 0)
            display_powerOff();

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
            display_powerOn();

        // if display was on, then just switch inputs
        else
        {
            display_ToggleInput(1);
        }
    }

}

