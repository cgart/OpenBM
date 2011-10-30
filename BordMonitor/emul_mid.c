#include "base.h"
#include "emul_mid.h"
#include <avr/pgmspace.h>
#include "buttons.h"
#include "include/ibus.h"
#include "display.h"
#include "config.h"
#include "include/leds.h"
#include "include/config.h"
#include "include/obm_special.h"

//------------------------------------------------------------------------------
static uint8_t _active_mode = 0; // 0x01 init, 0x40 radio, 0xC0=cd, 0x80=tape, 0x60=tone, etc.. this come from Radio
static int8_t _ignitionState = -1;
static ticks_t _reqIgnitionState = 0;

typedef enum _DevicePollState
{
    DEV_FIRST = 0,
    DEV_RADIO = DEV_FIRST,
    DEV_IKE = 1,
    DEV_DSP = 2,
    DEV_TEL = 3,
    //DEV_LAST = DEV_TEL,
    DEV_NUM = 4
}DevicePollState;
static DevicePollState _pollState = DEV_FIRST;

static bool _pollStateRecieved[DEV_NUM];
static uint8_t _ackToSrc = 0;
static uint8_t _ackAs = 0;
static uint8_t _ackTextUpdateWithNeg = 0;
static ticks_t _pingTicks = 0;

static uint8_t _ackBmbtTape[2] = {0,0};
static uint8_t _ackBmbtPingSrc = 0;

// mapping from button states to MID buttons
#define MID_MAP_SIZE 15
uint8_t _button_mapping_buis[MID_MAP_SIZE] PROGMEM = {
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_5,
    BUTTON_6,

    BUTTON_FM,
    BUTTON_AM,
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
    
    BUTTON_INFO_L,
    BUTTON_MODE,
    BUTTON_REW,
    BUTTON_FF,
    BUTTON_NUM_BUTTONS
};

uint8_t _button_mapping_prof[MID_MAP_SIZE] PROGMEM = {
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_5,
    BUTTON_6,

    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
    BUTTON_AM,
    BUTTON_FM,

    BUTTON_INFO_L,
    BUTTON_MODE,
    BUTTON_REW,
    BUTTON_FF,
    BUTTON_NUM_BUTTONS
};



uint8_t _button_mapping_dbyte1_mask[MID_MAP_SIZE] PROGMEM = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0x00,
    0x00,
    0x00
};

#if 0
    #if ((DEVICE_CODING2 & REW_FF_ONMID) == REW_FF_ONMID)
        #define BMBT_RADIO_BUTTONS 9
        #define BMBT_MAP_SIZE   18
    #else
        #define BMBT_RADIO_BUTTONS 11
        #define BMBT_MAP_SIZE   20
    #endif
#endif

// original BMBT codes
#define BMBT_MAP_SIZE 20
uint8_t _bmbt_button_mapping[BMBT_MAP_SIZE][3] PROGMEM =
{
    {BUTTON_INFO_R,0xFF,0x03},
    {BUTTON_TONE,0x68,0x04},
    {BUTTON_BMBT_KNOB,0x3B,0x05},
    {BUTTON_UHR,0xFF,0x07},
    {BUTTON_TEL,0xFF,0x08},
    {BUTTON_PRG,0x68,0x14},
    {BUTTON_SELECT,0x68,0x20},
    {BUTTON_EJECT,0x68,0x24},
    {BUTTON_MENU_LR,0xFF,0x34},

// REW and FF buttons send codes as usual
//#if ((DEVICE_CODING2 & REW_FF_ONMID) != REW_FF_ONMID)
    {BUTTON_FF,0x68,0x00},
    {BUTTON_REW,0x68,0x10},
//#endif

    // Left side
    {BUTTON_INFO_L,0x68,0x07},
    {BUTTON_1,0x68,0x11},
    {BUTTON_2,0x68,0x01},
    {BUTTON_3,0x68,0x12},
    {BUTTON_4,0x68,0x02},
    {BUTTON_5,0x68,0x13},
    {BUTTON_6,0x68,0x03},
    {BUTTON_AM,0x68,0x21},
    {BUTTON_FM,0x68,0x31}
//    {BUTTON_MODE,0x68,0x23}
};

static ticks_t _nextSendMFLSignalTick;
static bool _nextSendMFLSignal;

#define MID_RADIO     0
#define MID_CDCHANGER 1
#define MID_TAPE      2

//------------------------------------------------------------------------------
uint8_t mid_active_mode(void)
{
    if (_active_mode == 0x40) return MID_RADIO; // Radio
    if (_active_mode == 0xC0) return MID_CDCHANGER; // CD-Changer
    if (_active_mode == 0x80) return MID_TAPE; // Tape
    return 0xFF;
}

//------------------------------------------------------------------------------
void mid_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    // based on the src decide if this device does exists
    // if the device hasn't been polled before, then allow up-to 5 polls
    if (msg[0] == IBUS_MSG_DEV_READY && _ignitionState > 0)
    {        
        if (src == IBUS_DEV_RAD)
        {
            _pollStateRecieved[DEV_RADIO]++;
            if (_active_mode == 0) _active_mode = 1; // need to reinitialize the radio
            _pollStateRecieved[DEV_DSP]++;
        }
        /*if (src == IBUS_DEV_DSP)
        {
            _pollStateRecieved[DEV_DSP]++;
            if (_active_mode == 0) _active_mode = 1; // need to reinitialize the radio
        }*/
        if (src == IBUS_DEV_TEL)
        {
            _pollStateRecieved[DEV_TEL]++;
        }
        if (src == IBUS_DEV_IKE)
        {
            _pollStateRecieved[DEV_IKE]++;
        }
    }

    // answer to request messages C8 03 E7 01 2D -> C0 04 C8 02 00 0E
    // as soon as any request has been recieved
    if (dst == IBUS_DEV_ANZV || dst == IBUS_DEV_MID || dst == IBUS_DEV_GT || dst == IBUS_DEV_BMBT)
    {
        // if we need to poll the answer, then do so
        if (msg[0] == IBUS_MSG_DEV_POLL)
        {
            _ackAs = dst;
            _ackToSrc = src;
        }

        if (src == IBUS_DEV_IKE && !_pollStateRecieved[DEV_IKE])
            _pollStateRecieved[DEV_IKE] = 5;
        else if (src == IBUS_DEV_TEL && !_pollStateRecieved[DEV_TEL])
            _pollStateRecieved[DEV_TEL] = 5;
        else if (src == IBUS_DEV_RAD && !_pollStateRecieved[DEV_RADIO] && _active_mode)
            _pollStateRecieved[DEV_RADIO] = 5;
        else if (src == IBUS_DEV_DSP && !_pollStateRecieved[DEV_DSP] && _active_mode)
            _pollStateRecieved[DEV_DSP] = 5;
    }


    // if ignition is off, then stop request on IKE and TEL
    if (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO && msglen >= 2 && msg[0] == IBUS_MSG_IGNITION)
    {
        _reqIgnitionState = 0;

        if (msg[1] == 0)
        {
            _active_mode = 0;

            _pollStateRecieved[DEV_IKE] = 0;
            _pollStateRecieved[DEV_TEL] = 0;
            _pollStateRecieved[DEV_DSP] = 0;
            _pollStateRecieved[DEV_RADIO] = 0;
        }else
        {
            _pollStateRecieved[DEV_IKE] = 5;
            _pollStateRecieved[DEV_TEL] = 5;
        }

        // if ignition state was unknown before, and has been recieved as active
        // then we should ping for the radio
        if (_ignitionState < 0 && msg[1] > 0)
        {
            _pollStateRecieved[DEV_RADIO] = 5;
            _pollStateRecieved[DEV_IKE] = 5;
            _pollStateRecieved[DEV_DSP] = 5;
            _pollStateRecieved[DEV_TEL] = 5;

            _active_mode = 1;
            _pingTicks = 0xFFFFFF;
        }

        _ignitionState = msg[1];

        return;
    }

    // if ignore MFL ff/rew modus is activated, then send resend this event
    if (src == IBUS_DEV_MFL && dst == IBUS_DEV_RAD && msg[0] == IBUS_MSG_MFL_BUTTON && msglen >= 2 && mid_active_mode() == CARPC_INPUT() && mid_active_mode() == MID_TAPE)
    {
        _nextSendMFLSignal = 0;
        
        if (msg[1] == 0x21)
            _nextSendMFLSignal = 0x01;
        else if (msg[1] == 0x28)
            _nextSendMFLSignal = 0x08;

        if (_nextSendMFLSignal)
            _nextSendMFLSignalTick = tick_get() + TICKS_PER_TWO_SECONDS;
    }
    
    // ok, radio will give us a hint, which current mode is active
    // this can be seen on text messages sent from radio to ANZV
    // so react only on text messages
    if (dst == IBUS_DEV_ANZV || dst == IBUS_DEV_MID)// || dst == IBUS_DEV_LOC)
    {
        // take into account only if for bottom text update
        if (src == IBUS_DEV_RAD && msg[0] == IBUS_MSG_UPDATE_MID_BOTTOM && msglen >= 2)
        {
            _ackTextUpdateWithNeg = ~(0x01);

            // BMW Business with DSP ( 68 06 C0 21 00 00 20 AF -> ACK differently)
            if (msglen == 4 && msg[1] == 0 && msg[2] == 0 && msg[3] == 0x20)
            {
                _ackTextUpdateWithNeg = ~(0x00);
            }else if (msglen >= 2)
            {
                _ackTextUpdateWithNeg = ~(0x02);
            }

            // if mode is 0, then we have turned off, so stop polling the radio and DSP
            if (msg[1] == 0)
            {
                _pollStateRecieved[DEV_RADIO] = 0;
                _pollStateRecieved[DEV_DSP] = 0;
            }

            _active_mode = msg[1];
        }

        // React on LED message
        else if (msg[0] == IBUS_MSG_LED && USE_BM_LEDS() && msglen >= 2)
        {
            if (msg[1] & (1 << 0)) led_red_set(0b11111111); else led_red_set(0);
            if (msg[1] & (1 << 1)) led_red_set(0b11110000);

            if (msg[1] & (1 << 2)) led_yellow_set(0b11111111); else led_yellow_set(0);
            if (msg[1] & (1 << 3)) led_yellow_set(0b11110000);

            if (msg[1] & (1 << 4)) led_green_set(0b11111111); else led_green_set(0);
            if (msg[1] & (1 << 5)) led_green_set(0b11110000);

            if (msg[1] & (1 << 6)) led_fan_set(0b11111111); else led_fan_set(0);
            if (msg[1] & (1 << 7)) led_fan_set(0b11110000);
        }

        return;
    }

    // special treatment for messages defined for BMBT/OpenBM
    if (dst == IBUS_DEV_BMBT)
    {
        // LED messages
        if (msg[0] == IBUS_MSG_LED && msglen >= 2)
        {
            if (msg[1] & (1 << 0)) led_red_set(0b11111111); else led_red_set(0);
            if (msg[1] & (1 << 1)) led_red_set(0b11110000);

            if (msg[1] & (1 << 2)) led_yellow_set(0b11111111); else led_yellow_set(0);
            if (msg[1] & (1 << 3)) led_yellow_set(0b11110000);

            if (msg[1] & (1 << 4)) led_green_set(0b11111111); else led_green_set(0);
            if (msg[1] & (1 << 5)) led_green_set(0b11110000);

            if (msg[1] & (1 << 6)) led_fan_set(0b11111111); else led_fan_set(0);
            if (msg[1] & (1 << 7)) led_fan_set(0b11110000);

        }else if (msg[0] == IBUS_MSG_LED_SPECIAL && msglen >= 2)
        {
            if (msg[1] & (1 << 0))
            {
                led_red_set(0);
                led_green_set(0);
                led_yellow_set(0);
                led_fan_set(0);
                led_radio_set(0);
            }
            if (msg[1] & (1 << 1)) led_red_set(msg[2]);
            if (msg[1] & (1 << 2)) led_yellow_set(msg[2]);
            if (msg[1] & (1 << 3)) led_green_set(msg[2]);
            if (msg[1] & (1 << 4)) led_fan_set(msg[2]);
            if (msg[1] & (1 << 5)) led_radio_set(msg[2]);
        }

        // if BMBT emulation is on
        if (obms_does_emulate_bordmonitor())
        {
            // we were asked for the state of the tape
            if (msg[0] == IBUS_MSG_BMBT_TAPE_STATE && msglen >= 2)
            {
                // if we are in the tape mode and carpc is on tape, then say to radio that tape is playing
                // this would ensure that radio doesn't turn off the audio source
                if (CARPC_INPUT() == MID_TAPE && mid_active_mode() == CARPC_INPUT())
                {
                    _ackBmbtTape[0] = 0x06;
                    _ackBmbtTape[1] = 0x11;
                }else
                {
                    _ackBmbtTape[0] = 0x05;
                    _ackBmbtTape[1] = 0x00;
                }
            }

            // answer ping messages
            if (msg[0] == IBUS_MSG_DEV_POLL)
                _ackBmbtPingSrc = src;
        }
    }

}

//------------------------------------------------------------------------------
void mid_ping_tick(void)
{
    // every 10 seconds we perform a ping on different hardware to check if they exists
    _pingTicks ++;

    if (_pingTicks > TICKS_PER_X_SECONDS(10) && _ignitionState > 0)
    {
        uint8_t data[1] = {IBUS_MSG_DEV_POLL};

        if (_pollState == DEV_RADIO)
        {
            if (_pollStateRecieved[DEV_RADIO] > 0)
            {
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 1, IBUS_TRANSMIT_TRIES);

                if (obms_does_emulate_bordmonitor())
                    ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_RAD, data, 1, IBUS_TRANSMIT_TRIES);

                _pollStateRecieved[DEV_RADIO]--;
            }

            _pollState = DEV_IKE;

        }else if (_pollState == DEV_IKE)
        {
            if (_pollStateRecieved[DEV_IKE] > 0)
            {
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_IKE, data, 1, IBUS_TRANSMIT_TRIES);
                _pollStateRecieved[DEV_IKE]--;
            }

            _pollState = DEV_DSP;

        }else if (_pollState == DEV_DSP)
        {
            if (_pollStateRecieved[DEV_DSP] > 0)
            {
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_DSP, data, 1, IBUS_TRANSMIT_TRIES);
                _pollStateRecieved[DEV_DSP]--;
            }

            _pollState = DEV_TEL;

        }else if (_pollState == DEV_TEL)
        {
            if (_pollStateRecieved[DEV_TEL] > 0)
            {
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_TEL, data, 1, IBUS_TRANSMIT_TRIES);
                _pollStateRecieved[DEV_TEL]--;
            }

            _pollState = DEV_FIRST;

            _pingTicks = 0;
        }
    }

    // if radio has been detected however haven't been asked for its state, do this
    if (_active_mode == 1)
    {
        // mein BMW Prof, ohne DSP C0 06 FF 20 20 B0 00 89
        // Alex BMW Buis,  mit DSP C0 06 FF 20 00 B3 20 8A

        //uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB2, 0x00 /* DSP 0x20 */};
        uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x00, 0xB3, 0x20};

        //if (g_deviceSettings.device_Settings2 & RADIO_PROFESSIONAL)
        //    data[2] |= 0xB2;

        ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_LOC, data, 4, IBUS_TRANSMIT_TRIES);
        _active_mode = 0;
    }
}

//------------------------------------------------------------------------------
void mid_tick(void)
{
    mid_ping_tick();
    
    // turn ON, radio LED if in RADIO mode
    led_radio_immediate_set(_active_mode == 0x40);

    // send ACK message if MID has been requested
    if (_ackToSrc)
    {
        uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x00};
        ibus_sendMessage(_ackAs, _ackToSrc, data, 2, IBUS_TRANSMIT_TRIES);
        _ackToSrc = 0;
    }

    // acknowledge every text update message sent to MID
    if (_ackTextUpdateWithNeg)
    {
        // acknowledge msg
        uint8_t data[2] = {IBUS_MSG_MID_ACK_TEXT, ~_ackTextUpdateWithNeg};
        ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 2, IBUS_TRANSMIT_TRIES);
        _ackTextUpdateWithNeg = 0;
    }

    // should we send bmbt tape messages
    if (_ackBmbtTape[0])
    {
        uint8_t data[3] = {IBUS_MSG_BMBT_TAPE_RESP, _ackBmbtTape[0], _ackBmbtTape[1]};
        uint8_t len = 3;
        if (_ackBmbtTape[1] == 0) len = 2;
        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_RAD, data, len, IBUS_TRANSMIT_TRIES);
        _ackBmbtTape[0] = 0;
        _ackBmbtTape[1] = 0;
    }

    // acknowledge the poll
    if (_ackBmbtPingSrc)
    {
        uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x30};
        ibus_sendMessage(IBUS_DEV_BMBT, _ackBmbtPingSrc, data, 2, IBUS_TRANSMIT_TRIES);
        _ackBmbtPingSrc = 0;
    }

    // special treatment for the Radio knob button
    // If radio knob is pressed shortly, then on release a message will be sent
    // the same as it was pressed on the MID (turn off message)
    // if radio knob is hold long, then another type of the message will be sent
    // and the release event will be ignored.
    {
        static uint8_t ignoreReleaseEvent = 0;

        if (button_down_long(BUTTON_RADIO_KNOB))
        {
            ignoreReleaseEvent = 1;
            uint8_t data[2] = {IBUS_MSG_BMBT_BUTTON, 0x46};
            ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_RAD, data, 2, IBUS_TRANSMIT_TRIES);
        }else if (button_released(BUTTON_RADIO_KNOB) && _ignitionState >= 1)
        {
            if (ignoreReleaseEvent == 0)
            {
                if (_active_mode)
                {
                    _pollStateRecieved[DEV_RADIO] = 0;
                    _pollStateRecieved[DEV_DSP] = 0;
                    _active_mode = 0;

                    display_powerOff();
                }else
                {
                    _pollStateRecieved[DEV_RADIO] = 5;
                    _pollStateRecieved[DEV_IKE] = 5;
                    _pollStateRecieved[DEV_DSP] = 5;
                    _pollStateRecieved[DEV_TEL] = 5;

                    _active_mode = 1;
                    _pingTicks = 0xFFFFFF;

                    display_powerOn();
                }

                //uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB0, 0x00};
                uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB2, 0x00};

                // if DSP, then include DSP bit to switch DSP on/off
                if (g_deviceSettings.device_Settings2 & DSP_AMPLIFIER)
                    data[3] |= 0x20;

                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_LOC, data, 4, IBUS_TRANSMIT_TRIES);
            }
            ignoreReleaseEvent = 0;
        }
    }

    {
        uint8_t i;
        for (i=0; i < MID_MAP_SIZE; i++)
        {
            void* addr = NULL;
            if (g_deviceSettings.device_Settings2 & RADIO_PROFESSIONAL)
                addr = &(_button_mapping_prof[i]);
            else
                addr = &(_button_mapping_buis[i]);

            // read to which button this one maps to and go to next, if no mapping
            uint8_t bmap = pgm_read_byte(addr);
            if (bmap >= BUTTON_NUM_BUTTONS) continue;

            // send MID buttons, however only if not in CarPC mode (MODE send always)
            if (mid_active_mode() == CARPC_INPUT() && (bmap != BUTTON_MODE && bmap != BUTTON_REW && bmap != BUTTON_FF) ) continue;

            // REW and FF send only as MID button, if this has been set in our device settings
            else if ((bmap == BUTTON_REW || bmap == BUTTON_FF)
              //&& mid_active_mode() != CARPC_INPUT()
              && (g_deviceSettings.device_Settings2 & REW_FF_ONMID) != REW_FF_ONMID) continue;            

            else if ((bmap == BUTTON_REW || bmap == BUTTON_FF)
              && mid_active_mode() == CARPC_INPUT()
              && (g_deviceSettings.device_Settings2 & REW_FF_ONMID) == REW_FF_ONMID) continue;

            uint8_t bmask = pgm_read_byte(&(_button_mapping_dbyte1_mask[i]));

            // prepare ibus message for this button
            uint8_t data[4] = {IBUS_MSG_BUTTON, _active_mode & bmask, 0x00, i};
            uint8_t sendmsg = button_pressed(bmap);
            if (button_released(bmap))
            {
                data[3] += 0x40;
                sendmsg = 1;
            }else if (button_down_long(bmap))
            {
                data[3] += 0x20;
                sendmsg = 1;
            }

            // send message if one of the buttons is set
            if (sendmsg)
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 4, IBUS_TRANSMIT_TRIES);
        }
    }

    // check BMBT buttons and transmit message
    {
        uint8_t i;
        for (i=0; i < BMBT_MAP_SIZE; i++)
        {
            // find out the button index from which we map the buttons to MID instead of BMBT
            unsigned midButtonsFrom = 11;
            if ((g_deviceSettings.device_Settings2 & REW_FF_ONMID) == REW_FF_ONMID)
                midButtonsFrom = 9;

            // if we are in the CarPC mode then send other codes for the MID buttons as usual
            if (mid_active_mode() != CARPC_INPUT() && i >= midButtonsFrom) break;
            //if (mid_active_mode() != CARPC_INPUT() && i >= BMBT_RADIO_BUTTONS) break;

            // read to which button this one maps to and go to next, if no mapping
            uint8_t bmap = pgm_read_byte(&(_bmbt_button_mapping[i][0]));
            uint8_t bdst = pgm_read_byte(&(_bmbt_button_mapping[i][1]));
            uint8_t bcod = pgm_read_byte(&(_bmbt_button_mapping[i][2]));

            /*if ((bmap == BUTTON_REW || bmap == BUTTON_FF)
              //&& mid_active_mode() != CARPC_INPUT()
              && (g_deviceSettings.device_Settings2 & REW_FF_ONMID) != REW_FF_ONMID) continue;

            else if ((bmap == BUTTON_REW || bmap == BUTTON_FF)
              && mid_active_mode() == CARPC_INPUT()
              && (g_deviceSettings.device_Settings2 & REW_FF_ONMID) == REW_FF_ONMID) continue;
            */
            if ((bmap == BUTTON_REW || bmap == BUTTON_FF))
            {
              if (mid_active_mode() != CARPC_INPUT() && (g_deviceSettings.device_Settings2 & REW_FF_ONMID) == REW_FF_ONMID) continue;
            }

            // prepare ibus message for this button
            uint8_t data[2] = {IBUS_MSG_BMBT_BUTTON, bcod};
            uint8_t sendmsg = button_pressed(bmap);
            if (button_released(bmap))
            {
                data[1] += 0x80;
                sendmsg = 1;
            }else if (button_down_long(bmap))
            {
                data[1] += 0x40;
                sendmsg = 1;
            }

            // send message if one of the buttons is set
            if (sendmsg)
                ibus_sendMessage(IBUS_DEV_BMBT, bdst, data, 2, IBUS_TRANSMIT_TRIES);
        }
    }

    // Bordmonitor knob
    int8_t bmbt = button_encoder(ENC_BMBT);
    if (bmbt)
    {
        uint8_t data[2] = {IBUS_MSG_BMBT_ENCODER, 0x00};

        if (bmbt < 0)
            data[1] = -bmbt;
        else
            data[1] = 0x80 + bmbt;

        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_GT, data, 2, 1);
    }

    // radio knob
    int encradio = button_encoder(ENC_RADIO);
    if (encradio)
    {
        uint8_t data[2] = {IBUS_MSG_RADIO_ENCODER, 0x00};

        if (encradio < 0)
            data[1] = ((-encradio) << 4);
        else
            data[1] = (encradio << 4) + 0x01;

        ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 2, 1);
    }


    // MFL emulation if active
    if (_nextSendMFLSignal && tick_get() > _nextSendMFLSignalTick)
    {
        uint8_t data[2] = {IBUS_MSG_MFL_BUTTON, _nextSendMFLSignal};
        ibus_sendMessage(IBUS_DEV_MFL,IBUS_DEV_RAD, data, 2, IBUS_TRANSMIT_TRIES);
        data[1] |= 0x20;
        ibus_sendMessage(IBUS_DEV_MFL,IBUS_DEV_RAD, data, 2, IBUS_TRANSMIT_TRIES);

        data[0] = IBUS_MSG_BMBT_BUTTON;
        if (_nextSendMFLSignal == 0x01)
            data[1] = 0x00;
        else
            data[1] = 0x10;
        ibus_sendMessage(IBUS_DEV_BMBT,IBUS_DEV_RAD, data, 2, IBUS_TRANSMIT_TRIES);
        _nextSendMFLSignal = 0;
    }

    // if we are allowed to request the status of a device, then do so
    if (_ignitionState > -10 && _ignitionState < 0 && _reqIgnitionState > 0 && tick_get() > _reqIgnitionState)
    {
        _ignitionState--;

        uint8_t data[2] = {IBUS_MSG_IGNITION_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_IKE, data, 2, IBUS_TRANSMIT_TRIES);

        _reqIgnitionState = tick_get() + TICKS_PER_X_SECONDS(2);
    }

}

//------------------------------------------------------------------------------
void mid_stop()
{
    _ignitionState = 0;
    _active_mode = 0;
    for (int8_t i=0; i < DEV_NUM; i++)
        _pollStateRecieved[i] = 0;
}

//------------------------------------------------------------------------------
void mid_resume()
{
    mid_init();
}

//------------------------------------------------------------------------------
void mid_init(void)
{
    _active_mode = 0; // 0x40 radio, 0xC0=cd, 0x80=tape, etc.. this comes from Radio
    _pollState = DEV_FIRST;
    _ackToSrc = 0;
    _ackAs = IBUS_DEV_MID;
    _ackTextUpdateWithNeg = 0;
    _pingTicks = 0;
    _ignitionState = -1;
    _reqIgnitionState = 1;

    for (int8_t i=0; i < DEV_NUM; i++)
        _pollStateRecieved[i] = 0;
}
