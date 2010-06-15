#include "display.h"
#include "leds.h"
#include "buttons.h"
#include "base.h"
#include <avr/eeprom.h>

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
ticks_t g_display_NextResponseTime = 0;

//------------------------------------------------------------------------------
void display_init(void)
{
    g_display_NextResponseTime = 0;
    
    // display power switch
    DDRC |= (1 << DDC3); PORTC |= (1 << 3);

    // display input switch
    DDRC |= (1 << DDC4); PORTC |= (1 << 4);

    // load current display state from the eeprom
    eeprom_busy_wait();
    g_DisplayState.display_Input = eeprom_read_byte(&g_eeprom_DisplayState.display_Input);
    g_DisplayState.display_Power = eeprom_read_byte(&g_eeprom_DisplayState.display_Power);
}

//------------------------------------------------------------------------------
void display_TogglePower(uint8_t writeToEeprom)
{
    // emulate key
    PORTC &= ~(1 << 3);
    _delay_ms(50);
    PORTC |= (1 << 3);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Power = !g_DisplayState.display_Power;
        eeprom_busy_wait();
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
    }
    //_delay_ms(1000);

    g_display_NextResponseTime = tick_get() + TICKS_PER_ONE_AND_A_HALF_SECONDS();
}

//------------------------------------------------------------------------------
void display_ToggleInput(uint8_t writeToEeprom)
{
    // emulate key
    PORTC &= ~(1 << 4);
    _delay_ms(50);
    PORTC |= (1 << 4);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Input = (g_DisplayState.display_Input + 1) % 3;
        eeprom_busy_wait();
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
    }
    //_delay_ms(1000);
    
    g_display_NextResponseTime = tick_get() + TICKS_PER_ONE_AND_A_HALF_SECONDS();
}

//------------------------------------------------------------------------------
void display_savePowerState(uint8_t state)
{
    g_DisplayState.display_Power = state;
    eeprom_busy_wait();
    eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
}


//------------------------------------------------------------------------------
void display_saveInputState(uint8_t state)
{
    g_DisplayState.display_Input = state;
    eeprom_busy_wait();
    eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
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
    int8_t diff = 0;
    diff = state - g_DisplayState.display_Input;
    diff &= 3;
    for (; diff >=0; diff--)
        display_ToggleInput(0);

    display_saveInputState(g_DisplayState.display_Input);
}

//------------------------------------------------------------------------------
void display_updateState(void)
{
    static uint8_t ignoreButtons = 0;
    if (tick_get() < g_display_NextResponseTime) return;

    // if holding button longer than certain time, then turn off screen if it is on
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

