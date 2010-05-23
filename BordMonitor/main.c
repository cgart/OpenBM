/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include <i2cmaster.h>

typedef struct _ButtonState
{
    //right part
    char bDISP;
    char bMODE;
    char bAM;
    char bFM;
    char b6;
    char b3;
    char b5;
    char b2;
    char b4;
    char b1;
    char bINFO_R;
    char bINFO_L;
    char bRadioLeftPart;
    
    // left part
    char bUHR;
    char bTEL;
    char bFF;
    char bREW;
    char bSELECT;
    char bMENU_LR;
    char bTONE;
    char bPRG;
    char bEJECT;

    // BMBT-Encoder
    char bmbt_bButton;
    char bmbt_bCW;
    char bmbt_bCCW;
    char bmbt_encoderOut1;
    char bmbt_encoderOut2;

    // Radio-Encoder
    char radio_bButton;
    char radio_bRotate;
    char radio_bCW;
    char radio_bCCW;
    char radio_encoderOut1;
    char radio_encoderOut2;
}ButtonState;

// button states
volatile ButtonState g_ButtonState;
ButtonState g_oldButtonState;

// current photo sensor value
volatile unsigned char g_PhotoSensorValue;


#define PORT_EXPANDER_ENCODER_ADD 0b01000000
#define PORT_EXPANDER_RADIO_ADD   0b01000010
#define PORT_EXPANDER_BMBT_ADD    0b01000100

//#define SETUP_BG_LED(a) {DDRA |= (1 << DDA0); PORTA &= ~(1 << 0); }
#define SETUP_R_LED(a)  {DDRC |= (1 << DDC2); PORTC &= ~(1 << 2); }
#define SETUP_G_LED(a)  {DDRD |= (1 << DDD7); PORTD &= ~(1 << 7); }
#define SETUP_Y_LED(a) {DDRD |= (1 << DDD5); PORTD &= ~(1 << 5); }
#define SETUP_FAN_LED(a) {DDRD |= (1 << DDD6); PORTD &= ~(1 << 6); }
#define SETUP_RADIO_LED(a) {DDRC |= (1 << DDC5); PORTC &= ~(1 << 5); }
#define SETUP_DISPLAY_POWER(a) {DDRC |= (1 << DDC3); PORTC |= (1 << 3); }
#define SETUP_DISPLAY_TOGGLE(a) {DDRC |= (1 << DDC4); PORTC |= (1 << 4); }
#define SETUP_EJECT_BUTTON(a) {DDRD &= ~(1 << 4); PORTD |= (1 << 4);}
#define SETUP_RADIO_BUTTON_SWITCH(a) {DDRC |= (1 << 6) | (1 << 7); }

//#define BG_LED_ON() {PORTA |= (1 << 0);}
//#define BG_LED_OFF() {PORTA &= ~(1 << 0);}
//#define BG_LED(a) {if(a){ BG_LED_ON(); }else{ BG_LED_OFF();}}

#define R_LED_ON() {PORTC |= (1 << 2);}
#define R_LED_OFF() {PORTC &= ~(1 << 2);}
#define R_LED(a) { if(a){ R_LED_ON(); } else{ R_LED_OFF();} }

#define G_LED_ON() {PORTD |= (1 << 7);}
#define G_LED_OFF() {PORTD &= ~(1 << 7);}
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

#define DISPLAY_POWER_TOGGLE() {PORTC &= ~(1 << 3); _delay_ms(50);  PORTC |= (1 << 3);}
#define DISPLAY_INPUT_TOGGLE() {PORTC &= ~(1 << 4); _delay_ms(50);  PORTC |= (1 << 4);}

#define EJECT_BUTTON_STATE() bit_is_clear(PIND,4)

#define BUTTON_STATE_CHANGED(a) (g_oldButtonState.a != g_ButtonState.a)

#define ENABLE_RADIO_BUTTON_LEFT_PART() {PORTC &= ~(1 << 6); PORTC |= (1 << 7);}
#define ENABLE_RADIO_BUTTON_RIGHT_PART() {PORTC &= ~(1 << 7); PORTC |= (1 << 6);}

#define PHOTOSENSOR_START_READ() { ADCSRA |= (1 << ADSC); }
#define PHOTOSENSOR_VALUE ADCH

//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{
    // init I2C, port expander
    i2c_init();
    //MCUCR |= (1 << ISC01); // react on falling edge
    //MCUCR &= ~(1 << ISC00);
    //GICR |= (1 << INT1);

    // enable PhotoSensor ADC reader
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 109KHz sample rate @ 14MHz
    ADMUX = (1 << REFS0); // Set ADC reference to AVCC
    ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
    ADMUX |= (1 << MUX0) | (1 << MUX1) | (1 << MUX2); // enable reading on ADC7
    SFIOR &= ~(1 << ADTS2) & ~(1 << ADTS1) & ~(1 << ADTS0);  // Set ADC to Free-Running Mode
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
    ADCSRA |= (1 << ADEN);  // Enable ADC

    // setup all ports to some default value
    DDRA = 0;
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;

    // setup LEDs
    SETUP_R_LED();
    SETUP_G_LED();
    SETUP_Y_LED();
    SETUP_FAN_LED();
    SETUP_RADIO_LED();
    SETUP_DISPLAY_POWER();
    SETUP_DISPLAY_TOGGLE();
    SETUP_EJECT_BUTTON();
    SETUP_RADIO_BUTTON_SWITCH();

    // switch display mode 2 times, because after the start of avr
    // display goes on and swith one time the mode
    DISPLAY_INPUT_TOGGLE();
    DISPLAY_INPUT_TOGGLE();
    
    // enable interrupts
    sei();
}

//------------------------------------------------------------------------------
// Set FUSES correctrly ( http://www.engbedded.com/fusecalc/):
// avrdude -U lfuse:w:0xff:m -U hfuse:w:0xc9:m  (ext crystal, disable jtag, ckopt)
//------------------------------------------------------------------------------
int main(void)
{
    // init hardware and do small delay after init
    initHardware();
    _delay_ms(100);

    // init default values
    g_ButtonState.b1 = 0;
    g_ButtonState.b2 = 0;
    g_ButtonState.b3 = 0;
    g_ButtonState.b4 = 0;
    g_ButtonState.b5 = 0;
    g_ButtonState.b6 = 0;
    g_ButtonState.bAM = 0;
    g_ButtonState.bDISP = 0;
    g_ButtonState.bFM = 0;
    g_ButtonState.bINFO_L = 0;
    g_ButtonState.bINFO_R = 0;
    g_ButtonState.bMODE = 0;
    g_ButtonState.bUHR = 0;
    g_ButtonState.bTEL = 0;
    g_ButtonState.bFF = 0;
    g_ButtonState.bREW = 0;
    g_ButtonState.bSELECT = 0;
    g_ButtonState.bMENU_LR = 0;
    g_ButtonState.bTONE = 0;
    g_ButtonState.bPRG = 0;
    g_ButtonState.bEJECT = 0;
    g_ButtonState.bmbt_bButton = 0;
    g_ButtonState.bmbt_bCW = 0;
    g_ButtonState.bmbt_bCCW = 0;
    g_ButtonState.bmbt_encoderOut1 = 0;
    g_ButtonState.bmbt_encoderOut2 = 0;
    g_ButtonState.radio_bButton = 0;
    g_ButtonState.radio_bRotate = 0;
    g_ButtonState.radio_bCW = 0;
    g_ButtonState.radio_bCCW = 0;
    g_ButtonState.radio_encoderOut1 = 0;
    g_ButtonState.radio_encoderOut2 = 0;
    g_ButtonState.bRadioLeftPart = 0;

    PHOTOSENSOR_START_READ();
    
    for(;;)
    {
        // set old state
        g_oldButtonState = g_ButtonState;

        // ----- First read the Radio Part -----
        if (i2c_start(PORT_EXPANDER_RADIO_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            // setup buttons according to the state
            if (g_ButtonState.bRadioLeftPart)
            {
                g_ButtonState.bINFO_R = bit_is_clear(state, 7);
                g_ButtonState.b4      = bit_is_clear(state, 4);
                g_ButtonState.b5      = bit_is_clear(state, 6);
                g_ButtonState.b6      = bit_is_clear(state, 2);
                g_ButtonState.bAM     = bit_is_clear(state, 5);
                g_ButtonState.bDISP   = bit_is_clear(state, 3);

                ENABLE_RADIO_BUTTON_RIGHT_PART();
                g_ButtonState.bRadioLeftPart = 0;
            }else
            {
                g_ButtonState.bINFO_L = bit_is_clear(state, 7);
                g_ButtonState.b1      = bit_is_clear(state, 4);
                g_ButtonState.b2      = bit_is_clear(state, 6);
                g_ButtonState.b3      = bit_is_clear(state, 2);
                g_ButtonState.bFM     = bit_is_clear(state, 5);
                g_ButtonState.bMODE   = bit_is_clear(state, 3);

                ENABLE_RADIO_BUTTON_LEFT_PART();
                g_ButtonState.bRadioLeftPart = 1;
            }
        }

        // ----- Second read the BMBT Buttons -----
        if (i2c_start(PORT_EXPANDER_BMBT_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            g_ButtonState.bTEL     = bit_is_clear(state, 0);
            g_ButtonState.bSELECT  = bit_is_clear(state, 1);
            g_ButtonState.bFF      = bit_is_clear(state, 2);
            g_ButtonState.bUHR     = bit_is_clear(state, 3);
            g_ButtonState.bMENU_LR = bit_is_clear(state, 4);
            g_ButtonState.bREW     = bit_is_clear(state, 5);
            g_ButtonState.bTONE    = bit_is_clear(state, 6);
            g_ButtonState.bPRG     = bit_is_clear(state, 7);
        }

        // ----- Thirs ask both encoders -----
        if (i2c_start(PORT_EXPANDER_ENCODER_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            g_ButtonState.bmbt_bButton = bit_is_clear(state, 0);
            g_ButtonState.radio_bButton = bit_is_clear(state, 7);

            // check BMBT-Encoders
            {
                register char oldEncoder1 = g_ButtonState.bmbt_encoderOut1;
                register char oldEncoder2 = g_ButtonState.bmbt_encoderOut2;
                register char clearBit1 = bit_is_clear(state, 1);
                register char clearBit2 = bit_is_clear(state, 2);
                register char setBit1 = bit_is_set(state, 1);
                register char setBit2 = bit_is_set(state, 2);

                if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                    || (!oldEncoder1 && oldEncoder2 && setBit2)
                    || (oldEncoder2 && oldEncoder1 && setBit1)
                    || (!oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState.bmbt_bCCW = 1;
                    g_ButtonState.bmbt_bCW = 0;
                }else
                if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                      || (oldEncoder1 && oldEncoder2 && setBit2)
                      || (!oldEncoder2 && oldEncoder1 && setBit1)
                      || (oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState.bmbt_bCCW = 0;
                    g_ButtonState.bmbt_bCW = 1;
                }else
                {
                    g_ButtonState.bmbt_bCCW = 0;
                    g_ButtonState.bmbt_bCW = 0;
                }
            }

            // check RADIO-Encoders
            {
                register char oldEncoder1 = g_ButtonState.radio_encoderOut1;
                register char oldEncoder2 = g_ButtonState.radio_encoderOut2;
                register char clearBit1 = bit_is_clear(state, 5);
                register char clearBit2 = bit_is_clear(state, 6);
                register char setBit1 = bit_is_set(state, 5);
                register char setBit2 = bit_is_set(state, 6);

                if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                    || (!oldEncoder1 && oldEncoder2 && setBit2)
                    || (oldEncoder2 && oldEncoder1 && setBit1)
                    || (!oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState.radio_bCCW = 1;
                    g_ButtonState.radio_bCW = 0;
                }else
                if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                      || (oldEncoder1 && oldEncoder2 && setBit2)
                      || (!oldEncoder2 && oldEncoder1 && setBit1)
                      || (oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState.radio_bCCW = 0;
                    g_ButtonState.radio_bCW = 1;
                }else
                {
                    g_ButtonState.radio_bCCW = 0;
                    g_ButtonState.radio_bCW = 0;
                }
            }
            
            // set current state
            g_ButtonState.bmbt_encoderOut1 = bit_is_clear(state, 1);
            g_ButtonState.bmbt_encoderOut2 = bit_is_clear(state, 2);
            g_ButtonState.radio_encoderOut1 = bit_is_clear(state, 5);
            g_ButtonState.radio_encoderOut2 = bit_is_clear(state, 6);
        }

        // ----- Read Eject Button -----
        g_ButtonState.bEJECT = EJECT_BUTTON_STATE();

        // switch display on/off and switch the input mode
        if (BUTTON_STATE_CHANGED(bDISP)) DISPLAY_POWER_TOGGLE();
        if (BUTTON_STATE_CHANGED(bMODE)) DISPLAY_INPUT_TOGGLE();

        // debugging
        {
            R_LED(g_ButtonState.bmbt_bButton);
            G_LED(g_ButtonState.radio_bCCW);
            Y_LED(g_ButtonState.radio_bCW);
            FAN_LED(g_ButtonState.radio_bButton);
            RADIO_LED(g_ButtonState.bMENU_LR);
        }

    }

    cli();
    
    return 0;
}

//------------------------------------------------------------------------------
// React on ADC interrupts, so PhotoSensor value is readed
// and conversion is started again
//------------------------------------------------------------------------------
ISR(ADC_vect)
{
    g_PhotoSensorValue = PHOTOSENSOR_VALUE;
    PHOTOSENSOR_START_READ();
}
