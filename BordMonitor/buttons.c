#include "buttons.h"
#include "i2cmaster.h"
#include "base.h"

#define EJECT_BUTTON_STATE() bit_is_clear(PIND,4)
#define ENABLE_RADIO_BUTTON_LEFT_PART() {PORTC &= ~(1 << 6); PORTC |= (1 << 7);}
#define ENABLE_RADIO_BUTTON_RIGHT_PART() {PORTC &= ~(1 << 7); PORTC |= (1 << 6);}

buttonGlobalState_t g_buttons = 0;
buttonGlobalState_t g_buttonsDown = 0;
buttonGlobalState_t g_buttonsPressed = 0;
buttonGlobalState_t g_buttonsRelease = 0;

buttonGlobalState_t g_buttons_last = 0;

// delay in ticks time steps for each button
uint8_t g_button_delay[BUTTON_NUM_BUTTONS];

// direction states of both encoders
volatile uint8_t g_encoder_flag = 0;
volatile int8_t g_enc_delta[2];
int8_t g_enc_last[2];
#define ENC_BMBT_KNOB (1 << 0)
#define ENC_BMBT_OUT1 (1 << 1)
#define ENC_BMBT_OUT2 (1 << 2)
#define ENC_BMBT 0
#define ENC_RADIO_OUT1 (1 << 6)
#define ENC_RADIO_OUT2 (1 << 5)
#define ENC_RADIO_KNOB (1 << 7)
#define ENC_RADIO 1

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
uint8_t button_down_time(buttonIndex_t id)
{
    return g_button_delay[id];
}

//------------------------------------------------------------------------------
void button_init(void)
{
    // get current state of encoders so that we have a start value for them
    if (i2c_start(PORT_EXPANDER_ENCODER_ADD + I2C_READ) == 0)
    {
        unsigned char state = ~(i2c_readNak());
        i2c_stop();

        // power-on state of bmbt encoder
        int8_t new = 0;
        if( state & ENC_BMBT_OUT1 ) new  = 0b11;
        if( state & ENC_BMBT_OUT1 ) new ^= 0b01;
        g_enc_last[ENC_BMBT] = new;
        g_enc_delta[ENC_BMBT] = 0;

        // power-on state of radio encoder
        new = 0;
        if( state & ENC_RADIO_OUT1 ) new  = 0b11;
        if( state & ENC_RADIO_OUT1 ) new ^= 0b01;
        g_enc_last[ENC_RADIO] = new;
        g_enc_delta[ENC_RADIO] = 0;
    }

    int i;
    for (i=0; i < BUTTON_NUM_BUTTONS; i++) g_button_delay[i] = 0;
    
    g_buttons = 0;
    g_buttonsDown = 0;
    g_buttonsPressed = 0;
    g_buttonsRelease = 0;

    g_buttons_last = 0;

    g_encoder_flag = 0;

    // eject button
    DDRD &= ~(1 << 4); PORTD |= (1 << 4);

    // button switcher on radio PCB
    DDRC |= (1 << 6) | (1 << 7);

    // interrupt for the rotary encoders' port expander
    MCUCR |= (1 << ISC11);
    MCUCR &= ~(1 << ISC10);
    GICR |= (1 << INT1);
}

//------------------------------------------------------------------------------
void button_tick_encoder(void)
{
    // clear pressed/release states
    g_buttonsPressed = 0;
    g_buttonsRelease = 0;
    
    // clear rotary states of both encoders, this will make sure
    // that if flag get active, this will happen only for the duration until next
    // call of this routine
    g_buttons &= ~(BUTTON_ROTARY_STATE_MASK);

    // if no event happened, then don't do anything
    if (g_encoder_flag == 0) return;
    g_encoder_flag = 0;
    
    // ----- Get value of both encoders -----
    if (i2c_start(PORT_EXPANDER_ENCODER_ADD + I2C_READ) == 0)
    {
        unsigned char state = ~(i2c_readNak());
        i2c_stop();

        // get current button states
        if (state & ENC_BMBT_KNOB) g_buttons |= (1L << BUTTON_BMBT_KNOB);
        else g_buttons &= ~(1L << BUTTON_BMBT_KNOB);
        if (state & ENC_RADIO_KNOB) g_buttons |= (1L << BUTTON_RADIO_KNOB);
        else g_buttons &= ~(1L << BUTTON_RADIO_KNOB);

        int8_t new, diff;

        // get value for bmbt encoder
        new = 0;
        if( (state & ENC_BMBT_OUT1) ) new  = 0b11;
        if( (state & ENC_BMBT_OUT2) ) new ^= 0b01;
        diff = g_enc_last[ENC_BMBT] - new;
        if( diff & 1 )
        {
            g_enc_last[ENC_BMBT] = new;
            g_enc_delta[ENC_BMBT] += (diff & 2) - 1;		// bit 1 = direction (+/-)
        }
        if (g_enc_delta[ENC_BMBT] < 0)
        {
            g_buttons |= (1L << BUTTON_BMBT_CCW);
            g_buttons &= ~(1L << BUTTON_BMBT_CW);
        }else if (g_enc_delta[ENC_BMBT] > 0)
        {
            g_buttons |= (1L << BUTTON_BMBT_CW);
            g_buttons &= ~(1L << BUTTON_BMBT_CCW);
        }
        g_enc_delta[ENC_BMBT] = 0;

        // get value for radio encoder
        new = 0;
        if( (state & ENC_RADIO_OUT1) ) new  = 0b11;
        if( (state & ENC_RADIO_OUT2) ) new ^= 0b01;
        diff = g_enc_last[ENC_RADIO] - new;
        if( diff & 1 )
        {
            g_enc_last[ENC_RADIO] = new;
            g_enc_delta[ENC_RADIO] += (diff & 2) - 1;		// bit 1 = direction (+/-)
        }
        if (g_enc_delta[ENC_RADIO] < 0)
        {
            g_buttons |= (1L << BUTTON_RADIO_CCW);
            g_buttons &= ~(1L << BUTTON_RADIO_CW);
        }else if (g_enc_delta[ENC_RADIO] > 0)
        {
            g_buttons |= (1L << BUTTON_RADIO_CW);
            g_buttons &= ~(1L << BUTTON_RADIO_CCW);
        }
        g_enc_delta[ENC_RADIO] = 0;
    }

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
    g_buttons_last = g_buttons;


    // increment hold counter for all buttons, which are currently down
    uint8_t i;
    buttonGlobalState_t tempDownState = g_buttonsDown;
    for (i=0; i < BUTTON_NUM_BUTTONS; i++)
    {
        if (tempDownState & 1) g_button_delay[i]++;
        else g_button_delay[i] = 0;

        tempDownState >>= 1L;

        if (g_button_delay[i] > BUTTON_DELAY_LONG) g_button_delay[i] = 0;
    }
}


//------------------------------------------------------------------------------
ISR(INT1_vect)
{
    g_encoder_flag = 1;
}