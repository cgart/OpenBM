#include "buttons.h"
#include "i2cmaster.h"

#define EJECT_BUTTON_STATE() bit_is_clear(PIND,4)
#define ENABLE_RADIO_BUTTON_LEFT_PART() {PORTC &= ~(1 << 6); PORTC |= (1 << 7);}
#define ENABLE_RADIO_BUTTON_RIGHT_PART() {PORTC &= ~(1 << 7); PORTC |= (1 << 6);}

buttonGlobalState_t g_buttons = 0;
buttonGlobalState_t g_buttonsDown = 0;
buttonGlobalState_t g_buttonsPressed = 0;
buttonGlobalState_t g_buttonsRelease = 0;

buttonGlobalState_t g_buttons_last = 0;
buttonGlobalState_t g_buttonsDown_last = 0;
buttonGlobalState_t g_buttonsPressed_last = 0;
buttonGlobalState_t g_buttonsRelease_last = 0;

//------------------------------------------------------------------------------
buttonBool_t button(buttonIndex_t id) { return is_bit_set(g_buttons,id); }
buttonBool_t button_down(buttonIndex_t id) { return is_bit_set(g_buttonsDown,id); }
buttonBool_t button_pressed(buttonIndex_t id) { return is_bit_set(g_buttonsPressed,id); }
buttonBool_t button_released(buttonIndex_t id) { return is_bit_set(g_buttonsRelease,id); }

//------------------------------------------------------------------------------
buttonGlobalState_t button_global(void) { return g_buttons; }
buttonGlobalState_t button_global_down(void) { return g_buttonsDown; }
buttonGlobalState_t button_global_pressed(void) { return g_buttonsPressed; }
buttonGlobalState_t button_global_released(void) { return g_buttonsRelease; }

//------------------------------------------------------------------------------
void button_init(void)
{
    //memset(g_oldButtonState, 0, sizeof(ButtonState));
    //memset(g_ButtonState, 0, sizeof(ButtonState));

    g_buttons = 0;
    g_buttonsDown = 0;
    g_buttonsPressed = 0;
    g_buttonsRelease = 0;

    g_buttons_last = 0;
    g_buttonsDown_last = 0;
    g_buttonsPressed_last = 0;
    g_buttonsRelease_last = 0;

    // eject button
    DDRD &= ~(1 << 4); PORTD |= (1 << 4);

    // button switcher on radio PCB
    DDRC |= (1 << 6) | (1 << 7);
}

//------------------------------------------------------------------------------
void button_tick(void)
{
    // ----- First read the Radio Part -----
    if (i2c_start(PORT_EXPANDER_RADIO_ADD + I2C_READ) == 0)
    {
        unsigned char state = ~(i2c_readNak());
        i2c_stop();

        // first two bits ignored
        state >>= 2;

        //g_buttons &= ~(BUTTON_RIGHT_STATE_MASK);
        //g_buttons |= (g_buttons_last & BUTTON_RIGHT_STATE_MASK);

        // setup buttons according to the state
        if (is_bit_set(g_buttons_last, BUTTON_ACTIVE_RIGHT_PART))
        {
            ENABLE_RADIO_BUTTON_RIGHT_PART();

            g_buttons &= ~(BUTTON_RIGHT_STATE_MASK_1);
            g_buttons |= ((uint32_t)state << 0);

            g_buttons &= ~(1L << BUTTON_ACTIVE_RIGHT_PART);
        }else
        {
            ENABLE_RADIO_BUTTON_LEFT_PART();

            g_buttons &= ~(BUTTON_RIGHT_STATE_MASK_2);
            g_buttons |= ((uint32_t)state << 6);

            g_buttons |= (1L << BUTTON_ACTIVE_RIGHT_PART);
        }
    }

    // ----- Second read the BMBT Buttons -----
    if (i2c_start(PORT_EXPANDER_BMBT_ADD + I2C_READ) == 0)
    {
        unsigned char state = ~(i2c_readNak());
        i2c_stop();

        g_buttons &= ~(BUTTON_LEFT_STATE_MASK);
        g_buttons |= ((uint32_t)state << 12);

    }

    // ----- Third ask both encoders -----
    if (i2c_start(PORT_EXPANDER_ENCODER_ADD + I2C_READ) == 0)
    {
        unsigned char state = ~(i2c_readNak());
        i2c_stop();

        // we ignore bits 3 and 4, since they are not connected
        state &= ~(1 << 3) & ~(1 << 4);

        // set acquired states
        g_buttons &= ~(BUTTON_KNOBS_STATE_MASK);
        g_buttons |= ((uint32_t)state << 20);
        

        // check BMBT-Encoders
        {
            register char oldEncoder1 = is_bit_set(g_buttons_last, BUTTON_BMBT_ENCODER_OUT1);
            register char oldEncoder2 = is_bit_set(g_buttons_last, BUTTON_BMBT_ENCODER_OUT2);
            register char clearBit1 = state & (1 << 1);
            register char clearBit2 = state & (1 << 2);
            register char setBit1 = !clearBit1;
            register char setBit2 = !clearBit2;

            if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                || (!oldEncoder1 && oldEncoder2 && setBit2)
                || (oldEncoder2 && oldEncoder1 && setBit1)
                || (!oldEncoder2 && !oldEncoder1 && clearBit1))
            {
                g_buttons |= (1L << BUTTON_BMBT_CCW);
                g_buttons &= ~(1L << BUTTON_BMBT_CW);
            }else
            if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                  || (oldEncoder1 && oldEncoder2 && setBit2)
                  || (!oldEncoder2 && oldEncoder1 && setBit1)
                  || (oldEncoder2 && !oldEncoder1 && clearBit1))
            {
                g_buttons &= ~(1L << BUTTON_BMBT_CCW);
                g_buttons |= (1L << BUTTON_BMBT_CW);
            }else
            {
                g_buttons &= ~(1L << BUTTON_BMBT_CCW);
                g_buttons &= ~(1L << BUTTON_BMBT_CW);
            }
        }

        // check RADIO-Encoders
        {
            register char oldEncoder1 = is_bit_set(g_buttons_last, BUTTON_RADIO_ENCODER_OUT1);
            register char oldEncoder2 = is_bit_set(g_buttons_last, BUTTON_RADIO_ENCODER_OUT2);
            register char clearBit1 = state & (1 << 5);
            register char clearBit2 = state & (1 << 6);
            register char setBit1 = !clearBit1;
            register char setBit2 = !clearBit2;

            if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                || (!oldEncoder1 && oldEncoder2 && setBit2)
                || (oldEncoder2 && oldEncoder1 && setBit1)
                || (!oldEncoder2 && !oldEncoder1 && clearBit1))
            {
                g_buttons &= ~(1L << BUTTON_RADIO_CCW);
                g_buttons |= (1L << BUTTON_RADIO_CW);
            }else
            if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                  || (oldEncoder1 && oldEncoder2 && setBit2)
                  || (!oldEncoder2 && oldEncoder1 && setBit1)
                  || (oldEncoder2 && !oldEncoder1 && clearBit1))
            {
                g_buttons &= ~(1L << BUTTON_RADIO_CW);
                g_buttons |= (1L << BUTTON_RADIO_CCW);
            }else
            {
                g_buttons &= ~(1L << BUTTON_RADIO_CW);
                g_buttons &= ~(1L << BUTTON_RADIO_CCW);
            }
        }
    }

    // ----- Read Eject Button -----
    if (EJECT_BUTTON_STATE())
        g_buttons |= (1L << BUTTON_EJECT);
    else
        g_buttons &= ~(1L << BUTTON_EJECT);


    // compute down, pressed and released states
    g_buttonsDown       = g_buttons_last    & g_buttons;
    g_buttonsPressed    = (~g_buttons_last) & g_buttons;
    g_buttonsRelease    = g_buttons_last    & (~g_buttons);

    // swap current with the old state
    g_buttonsDown_last = g_buttonsDown;
    g_buttonsPressed_last = g_buttonsPressed;
    g_buttonsRelease_last = g_buttonsRelease;
    g_buttons_last = g_buttons;
}
