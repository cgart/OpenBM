/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on December 10, 2009, 12:24 AM
 */

#include "base.h"
#include "ibus.h"
#include <uart.h>
#include <i2cmaster.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

//------------------------------------------------------------------------------
// Helping structure for button states
//------------------------------------------------------------------------------
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
    char bRadioRightPart;
    
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

//------------------------------------------------------------------------------
// Current display state, (on/off, input)
//------------------------------------------------------------------------------
typedef struct _DisplayState
{
    uint8_t display_Power;
    uint8_t display_Input; // (0=vga, 1=av1, 2=av2)
}DisplayState;

// button states are stored as ping-pong structure
ButtonState  g_buttonState[2];
ButtonState* g_ButtonState = &g_buttonState[0];
ButtonState* g_oldButtonState = &g_buttonState[1];

// current hardware state
LedState g_LedState;
DisplayState g_DisplayState;
DisplayState g_eeprom_DisplayState EEMEM;


// current photo sensor value
volatile unsigned char g_PhotoSensorValue;

// variable telling if task timer reached its end
volatile char g_TaskTimerShooted = 1;


#define SETUP_R_LED(a)  {DDRC |= (1 << DDC2); PORTC &= ~(1 << 2); }
#define SETUP_G_LED(a)  {DDRD |= (1 << DDD7); PORTD &= ~(1 << 7); }
#define SETUP_Y_LED(a) {DDRD |= (1 << DDD5); PORTD &= ~(1 << 5); }
#define SETUP_FAN_LED(a) {DDRD |= (1 << DDD6); PORTD &= ~(1 << 6); }
#define SETUP_RADIO_LED(a) {DDRC |= (1 << DDC5); PORTC &= ~(1 << 5); }
#define SETUP_DISPLAY_POWER(a) {DDRC |= (1 << DDC3); PORTC |= (1 << 3); }
#define SETUP_DISPLAY_TOGGLE(a) {DDRC |= (1 << DDC4); PORTC |= (1 << 4); }
#define SETUP_EJECT_BUTTON(a) {DDRD &= ~(1 << 4); PORTD |= (1 << 4);}
#define SETUP_RADIO_BUTTON_SWITCH(a) {DDRC |= (1 << 6) | (1 << 7); }

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

#define EJECT_BUTTON_STATE() bit_is_clear(PIND,4)

#define BUTTON_STATE_CHANGED(a) (g_oldButtonState->a != g_ButtonState->a)
#define BUTTON_UP(a) (g_oldButtonState->a && !g_ButtonState->a)
#define BUTTON_DOWN(a) (!g_oldButtonState->a && g_ButtonState->a)
#define BUTTON(a) g_ButtonState->a
#define OLD_BUTTON(a) g_oldButtonState->a

#define ENABLE_RADIO_BUTTON_LEFT_PART() {PORTC &= ~(1 << 6); PORTC |= (1 << 7);}
#define ENABLE_RADIO_BUTTON_RIGHT_PART() {PORTC &= ~(1 << 7); PORTC |= (1 << 6);}

#define PHOTOSENSOR_START_READ() { ADCSRA |= (1 << ADSC); }
#define PHOTOSENSOR_VALUE ADCH

#define START_TASK_TIMER() { TCNT2 = 76; } // shoot timer event every 1/80s by 14MHz
//#define START_TASK_TIMER() { TCNT2 = 0; } // shoot timer event every 1/80s by 14MHz

//------------------------------------------------------------------------------
// Toggle display power
//------------------------------------------------------------------------------
void display_TogglePower(uint8_t writeToEeprom)
{
    // emulate key
    PORTC &= ~(1 << 3);
    _delay_ms(100);
    PORTC |= (1 << 3);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Power = !g_DisplayState.display_Power;
        eeprom_busy_wait();
        eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
    }
    //_delay_ms(1000);
}

//------------------------------------------------------------------------------
// Set eeprom value to the given state (use this, when reseting the state)
//------------------------------------------------------------------------------
void display_setPowerState(uint8_t state)
{
    g_DisplayState.display_Power = state;
    eeprom_busy_wait();
    eeprom_write_byte(&g_eeprom_DisplayState.display_Power, g_DisplayState.display_Power);
}

//------------------------------------------------------------------------------
// Toggle display input
//------------------------------------------------------------------------------
void display_ToggleInput(uint8_t writeToEeprom)
{
    // emulate key
    PORTC &= ~(1 << 4);
    _delay_ms(100);
    PORTC |= (1 << 4);

    // update current state and write into eeprom
    if (writeToEeprom)
    {
        g_DisplayState.display_Input = (g_DisplayState.display_Input + 1) % 3;
        eeprom_busy_wait();
        eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
    }
    //_delay_ms(1000);
}

//------------------------------------------------------------------------------
// Set input state (use this when reset the state)
//------------------------------------------------------------------------------
void display_setInputState(uint8_t state)
{
    g_DisplayState.display_Input = state;
    eeprom_busy_wait();
    eeprom_write_byte(&g_eeprom_DisplayState.display_Input, g_DisplayState.display_Input);
}

//------------------------------------------------------------------------------
// update display buttons and status LEDs
//------------------------------------------------------------------------------
void display_updateState(void)
{
    // switch display on/off and switch the input mode (do not store to eeprom
    // if we are currently resetting these values)
    if (BUTTON_DOWN(bDISP)) display_TogglePower(!BUTTON(bMENU_LR) || !BUTTON(bSELECT));
    if (BUTTON_DOWN(bMODE) && g_DisplayState.display_Power) display_ToggleInput(!BUTTON(bMENU_LR) || !BUTTON(bSELECT));

    //RADIO_LED(g_DisplayState.display_Power);

    // if user would like to reset the display state, then do so
    // otherwise just handle the button state changes
    if ((!BUTTON(bMENU_LR) || !BUTTON(bSELECT))
      && (OLD_BUTTON(bMENU_LR) && OLD_BUTTON(bSELECT)) )
    {
        // ok reset the state to a default value (vga, power on)
        display_setPowerState(1);
        display_setInputState(0);

        // indicate by LEDs
        RADIO_LED_ON();
        _delay_ms(500);
        RADIO_LED_OFF();
        _delay_ms(150);
        RADIO_LED_ON();
        _delay_ms(300);
        RADIO_LED_OFF();
        _delay_ms(150);
        RADIO_LED_ON();
        _delay_ms(300);
        RADIO_LED_OFF();
        _delay_ms(150);
        RADIO_LED_ON();
        _delay_ms(300);
        RADIO_LED_OFF();
    }
}

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
// Update LEDs if they has to flash
//------------------------------------------------------------------------------
void flashLEDsTick(void)
{
    // react only every 1/8 second
    static unsigned char ticks = 0;
    if (ticks++ < 10) return;
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

    // debug photo sensor
    //RADIO_LED(g_PhotoSensorValue > 0);
    //G_LED(g_PhotoSensorValue > 50);
    //R_LED(g_PhotoSensorValue > 100);
    //Y_LED(g_PhotoSensorValue > 150);
    //FAN_LED(g_PhotoSensorValue > 200);
}

//------------------------------------------------------------------------------
// This method will be called by IBus stack when new message recieved
//------------------------------------------------------------------------------
void ibus_MessageCallback(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
}


//------------------------------------------------------------------------------
// Init all hardware parts needed for the project
//------------------------------------------------------------------------------
void initHardware(void)
{
    // set default variables
    g_TaskTimerShooted = 1;
    g_PhotoSensorValue = 0;
    
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

    // enable timer with a prescaler of 1024, so giving around 14.4kHz @ 14MHz
    TIMSK |= (1 << TOIE2);
    TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);
    
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

    // load current display state from the eeprom
    eeprom_busy_wait();
    g_DisplayState.display_Input = eeprom_read_byte(&g_eeprom_DisplayState.display_Input);
    g_DisplayState.display_Power = eeprom_read_byte(&g_eeprom_DisplayState.display_Power);

    // setup IBus hardware
    ibus_init();
    ibus_setMessageCallback(ibus_MessageCallback);

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

    uint8_t data[16] = {0x23, 0x40, 0x20 , 'W', 'E' , 'L' , 'C', 'O', 'M', 'E', ' ' , 'E' , '3' , '9', 0x20, 0x20};
    ibus_sendMessage(0x68, 0xE7, data, 16, IBUS_TRANSMIT_TRIES);

    // do small animation to indicate start
    {
        G_LED_ON();
        _delay_ms(500);
        G_LED_OFF();
        R_LED_ON();
        _delay_ms(500);
        R_LED_OFF();
        Y_LED_ON();
        _delay_ms(500);
        Y_LED_OFF();
        FAN_LED_ON();
        _delay_ms(500);
        FAN_LED_OFF();
        RADIO_LED_ON();
        _delay_ms(500);
        RADIO_LED_OFF();
        _delay_ms(500);
    }
    
    // clear both structures
    memset(g_ButtonState, 0, sizeof(ButtonState));
    memset(g_oldButtonState, 0, sizeof(ButtonState));
    memset(&g_LedState, 0, sizeof(LedState));
    
    //g_LedState.Red =   0b11110000;
    
    for(;;)
    {
        // ----- First read the Radio Part -----
        if (i2c_start(PORT_EXPANDER_RADIO_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            // setup buttons according to the state
            if (g_oldButtonState->bRadioRightPart)
            {
                ENABLE_RADIO_BUTTON_RIGHT_PART();

                g_ButtonState->bINFO_R = bit_is_clear(state, 7);
                g_ButtonState->b4      = bit_is_clear(state, 4);
                g_ButtonState->b5      = bit_is_clear(state, 6);
                g_ButtonState->b6      = bit_is_clear(state, 2);
                g_ButtonState->bAM     = bit_is_clear(state, 5);
                g_ButtonState->bDISP   = bit_is_clear(state, 3);

                g_ButtonState->bINFO_L = g_oldButtonState->bINFO_L;
                g_ButtonState->b1      = g_oldButtonState->b1;
                g_ButtonState->b2      = g_oldButtonState->b2;
                g_ButtonState->b3      = g_oldButtonState->b3;
                g_ButtonState->bFM     = g_oldButtonState->bFM;
                g_ButtonState->bMODE   = g_oldButtonState->bMODE;

                g_ButtonState->bRadioRightPart = 0;
            }else
            {
                ENABLE_RADIO_BUTTON_LEFT_PART();

                g_ButtonState->bINFO_L = bit_is_clear(state, 7);
                g_ButtonState->b1      = bit_is_clear(state, 4);
                g_ButtonState->b2      = bit_is_clear(state, 6);
                g_ButtonState->b3      = bit_is_clear(state, 2);
                g_ButtonState->bFM     = bit_is_clear(state, 5);
                g_ButtonState->bMODE   = bit_is_clear(state, 3);

                g_ButtonState->bINFO_R = g_oldButtonState->bINFO_R;
                g_ButtonState->b4      = g_oldButtonState->b4;
                g_ButtonState->b5      = g_oldButtonState->b5;
                g_ButtonState->b6      = g_oldButtonState->b6;
                g_ButtonState->bAM     = g_oldButtonState->bAM;
                g_ButtonState->bDISP   = g_oldButtonState->bDISP;

                g_ButtonState->bRadioRightPart = 1;
            }
        }

        // ----- Second read the BMBT Buttons -----
        if (i2c_start(PORT_EXPANDER_BMBT_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            g_ButtonState->bTEL     = bit_is_clear(state, 0);
            g_ButtonState->bSELECT  = bit_is_clear(state, 1);
            g_ButtonState->bFF      = bit_is_clear(state, 2);
            g_ButtonState->bUHR     = bit_is_clear(state, 3);
            g_ButtonState->bMENU_LR = bit_is_clear(state, 4);
            g_ButtonState->bREW     = bit_is_clear(state, 5);
            g_ButtonState->bTONE    = bit_is_clear(state, 6);
            g_ButtonState->bPRG     = bit_is_clear(state, 7);
        }

        // ----- Third ask both encoders -----
        if (i2c_start(PORT_EXPANDER_ENCODER_ADD + I2C_READ) == 0)
        {
            unsigned char state = i2c_readNak();
            i2c_stop();

            g_ButtonState->bmbt_bButton = bit_is_clear(state, 0);
            g_ButtonState->radio_bButton = bit_is_clear(state, 7);

            // check BMBT-Encoders
            {
                register char oldEncoder1 = g_oldButtonState->bmbt_encoderOut1;
                register char oldEncoder2 = g_oldButtonState->bmbt_encoderOut2;
                register char clearBit1 = bit_is_clear(state, 1);
                register char clearBit2 = bit_is_clear(state, 2);
                register char setBit1 = bit_is_set(state, 1);
                register char setBit2 = bit_is_set(state, 2);

                if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                    || (!oldEncoder1 && oldEncoder2 && setBit2)
                    || (oldEncoder2 && oldEncoder1 && setBit1)
                    || (!oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState->bmbt_bCCW = 1;
                    g_ButtonState->bmbt_bCW = 0;
                }else
                if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                      || (oldEncoder1 && oldEncoder2 && setBit2)
                      || (!oldEncoder2 && oldEncoder1 && setBit1)
                      || (oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState->bmbt_bCCW = 0;
                    g_ButtonState->bmbt_bCW = 1;
                }else
                {
                    g_ButtonState->bmbt_bCCW = 0;
                    g_ButtonState->bmbt_bCW = 0;
                }
            }

            // check RADIO-Encoders
            {
                register char oldEncoder1 = g_oldButtonState->radio_encoderOut1;
                register char oldEncoder2 = g_oldButtonState->radio_encoderOut2;
                register char clearBit1 = bit_is_clear(state, 5);
                register char clearBit2 = bit_is_clear(state, 6);
                register char setBit1 = bit_is_set(state, 5);
                register char setBit2 = bit_is_set(state, 6);

                if ((oldEncoder1 && !oldEncoder2 && clearBit2)
                    || (!oldEncoder1 && oldEncoder2 && setBit2)
                    || (oldEncoder2 && oldEncoder1 && setBit1)
                    || (!oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState->radio_bCCW = 0;
                    g_ButtonState->radio_bCW = 1;
                }else
                if ((!oldEncoder1 && !oldEncoder2 && clearBit2)
                      || (oldEncoder1 && oldEncoder2 && setBit2)
                      || (!oldEncoder2 && oldEncoder1 && setBit1)
                      || (oldEncoder2 && !oldEncoder1 && clearBit1))
                {
                    g_ButtonState->radio_bCCW = 1;
                    g_ButtonState->radio_bCW = 0;
                }else
                {
                    g_ButtonState->radio_bCCW = 0;
                    g_ButtonState->radio_bCW = 0;
                }
            }
            
            // set current state
            g_ButtonState->bmbt_encoderOut1 = bit_is_clear(state, 1);
            g_ButtonState->bmbt_encoderOut2 = bit_is_clear(state, 2);
            g_ButtonState->radio_encoderOut1 = bit_is_clear(state, 5);
            g_ButtonState->radio_encoderOut2 = bit_is_clear(state, 6);
        }

        // ----- Read Eject Button -----
        g_ButtonState->bEJECT = EJECT_BUTTON_STATE();

        // ----------- Do full reset if user liked so --------------------------
        if (BUTTON_UP(bMENU_LR) && BUTTON(bSELECT) && BUTTON(bEJECT))
            resetCPU();

        if (BUTTON_UP(bFM))
        {
            uint8_t data[160];// = {0x23, 0x40, 0x20 , 'W', 'E' , 'L' , 'C', 'O', 'M', 'E', ' ' , 'E' , '3' , '9', 0x20, 0x20};
            ibus_sendMessage(0x68, 0xE7, data, 160, IBUS_TRANSMIT_TRIES);
        }
        
        // ----- Display Buttons  ------
        display_updateState();
        
        // debugging
        {
            //R_LED(g_ButtonState->bmbt_bButton);
            //Y_LED(g_ButtonState->radio_bCCW);
            //G_LED(g_ButtonState->radio_bCW);
            //FAN_LED(g_ButtonState->bmbt_bCW);
            //G_LED(g_ButtonState->bEJECT);
        }

        // ------------ Task timer has shooted, so perfrom repeated task -------
        if (g_TaskTimerShooted)
        {
            // clear task shoot bit and restart timer
            g_TaskTimerShooted = 0;
            START_TASK_TIMER();

            // update flashing LEDs
            flashLEDsTick();
            
            // perform ibus tasks
            ibus_tick();
        }


        // swap current with the old state
        ButtonState* temp = g_oldButtonState;
        g_oldButtonState = g_ButtonState;
        g_ButtonState = temp;
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
    BEGIN_ATOMAR;

    g_PhotoSensorValue = PHOTOSENSOR_VALUE;

    PHOTOSENSOR_START_READ();

    END_ATOMAR;
}


//------------------------------------------------------------------------------
// Timer interrupt telling us, that task timer has reached its end
// task timer is restarted and global variable indicating task switch is set
//------------------------------------------------------------------------------
ISR(TIMER2_OVF_vect)
{
    BEGIN_ATOMAR;

    g_TaskTimerShooted = 1;
    
    END_ATOMAR;
}
