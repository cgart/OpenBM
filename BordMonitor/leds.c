#include "leds.h"
#include <string.h>

#define R_LED_ON() {PORTC |= (1 << 2);}
#define R_LED_OFF() {PORTC &= ~(1 << 2);}
#define R_LED(a) { if(a){ R_LED_ON(); } else{ R_LED_OFF();} }

#define G_LED_ON() {PORTD |= (1 << 4);}
#define G_LED_OFF() {PORTD &= ~(1 << 4);}
#define G_LED(a) {if(a){ G_LED_ON(); }else{ G_LED_OFF();}}

#define Y_LED_ON() {PORTD |= (1 << 5);}
#define Y_LED_OFF() {PORTD &= ~(1 << 5);}
#define Y_LED(a) {if(a){ Y_LED_ON();} else{ Y_LED_OFF();}}

#define FAN_LED_ON() {PORTD |= (1 << 6);}
#define FAN_LED_OFF() {PORTD &= ~(1 << 6);}
#define FAN_LED(a) {if(a){ FAN_LED_ON();} else{ FAN_LED_OFF();}}

#define RADIO_LED_ON() {PORTC |= (1 << 5);}
#define RADIO_LED_OFF() {PORTC &= ~(1 << 5);}
#define RADIO_LED(a) {if(a){ RADIO_LED_ON(); }else{ RADIO_LED_OFF();}}

//------------------------------------------------------------------------------
// Current state of the LEDs
// LED state (flashing pattern, i.e. 0=off, 0b11110000 long on, long off, ... )
//------------------------------------------------------------------------------
typedef struct _LedState
{
    unsigned char Red;
    unsigned char Green;
    unsigned char Yellow;
    unsigned char Fan;
    unsigned char Radio;
}LedState;

LedState g_LedState;


//------------------------------------------------------------------------------
void led_init(void)
{
    memset(&g_LedState, 0, sizeof(LedState));

    // RED
    DDRC |= (1 << DDC2); PORTC &= ~(1 << 2);

    // GREEN
    DDRD |= (1 << DDD4); PORTD &= ~(1 << 4);

    // YELLOW
    DDRD |= (1 << DDD5); PORTD &= ~(1 << 5);

    // FAN
    DDRD |= (1 << DDD6); PORTD &= ~(1 << 6);

    // RADIO
    DDRC |= (1 << DDC5); PORTC &= ~(1 << 5);

}

//------------------------------------------------------------------------------
void led_tick(void)
{
    // react only every 1/8 second
    static uint8_t ticks = 0;
    if (ticks++ < TICKS_PER_ONE_EIGHTH_SECOND()) return;
    ticks = 0;

    // this is our current tick counter
    static unsigned char tickCounter = 0;

    if (g_LedState.Red) R_LED(g_LedState.Red & (1 << tickCounter));
    if (g_LedState.Green) G_LED(g_LedState.Green & (1 << tickCounter));
    if (g_LedState.Yellow) Y_LED(g_LedState.Yellow & (1 << tickCounter));
    if (g_LedState.Fan) FAN_LED(g_LedState.Fan & (1 << tickCounter));
    if (g_LedState.Radio) RADIO_LED(g_LedState.Radio & (1 << tickCounter));

    // incrementcounter
    tickCounter++;
    tickCounter = tickCounter & 7;
}

//------------------------------------------------------------------------------
void led_red_set(uint8_t state)
{
    g_LedState.Red = state;
    if (state == 0) led_red_immediate_set(0);
}
//------------------------------------------------------------------------------
void led_green_set(uint8_t state)
{
    g_LedState.Green = state;
    if (state == 0) led_green_immediate_set(0);
}
//------------------------------------------------------------------------------
void led_yellow_set(uint8_t state)
{
    g_LedState.Yellow = state;
    if (state == 0) led_yellow_immediate_set(0);
}
//------------------------------------------------------------------------------
void led_fan_set(uint8_t state)
{
    g_LedState.Fan = state;
    if (state == 0) led_fan_immediate_set(0);
}
//------------------------------------------------------------------------------
void led_radio_set(uint8_t state)
{
    g_LedState.Radio = state;
    if (state == 0) led_radio_immediate_set(0);
}

//------------------------------------------------------------------------------
void led_red_immediate_set(uint8_t state)
{
    R_LED(state);
}
//------------------------------------------------------------------------------
void led_green_immediate_set(uint8_t state)
{
    G_LED(state);
}
//------------------------------------------------------------------------------
void led_yellow_immediate_set(uint8_t state)
{
    Y_LED(state);
}
//------------------------------------------------------------------------------
void led_fan_immediate_set(uint8_t state)
{
    FAN_LED(state);
}
//------------------------------------------------------------------------------
void led_radio_immediate_set(uint8_t state)
{
    RADIO_LED(state);
}

//------------------------------------------------------------------------------
void led_radioBlinkLock(uint8_t times)
{
    uint8_t t = times;
    for (; t > 0; t--)
    {
        RADIO_LED(1);
        _delay_ms(500);
        RADIO_LED(0);
        _delay_ms(500);
    }
}

//------------------------------------------------------------------------------
void led_greenBlinkLock(uint8_t times)
{
    uint8_t t = times;
    for (; t > 0; t--)
    {
        G_LED(1);
        _delay_ms(500);
        G_LED(0);
        _delay_ms(500);
    }
}

