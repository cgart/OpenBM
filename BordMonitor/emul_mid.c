#include "base.h"
#include "emul_mid.h"
#include <avr/pgmspace.h>
#include "buttons.h"
#include "ibus.h"
#include "include/ibus.h"
#include "leds.h"
#include "display.h"
#include "config.h"

//------------------------------------------------------------------------------
uint8_t mid_active_mode = 0; // 0x40 radio, 0xC0=cd, 0x80=tape, etc.. this come from Radio
#if 1
typedef enum _BackCamState
{
    CAM_IDLE = 0,
    CAM_PREPARE_TO_SWITCH = 1 << 1,
    CAM_SWITCHING = 1 << 2,
    CAM_ON = 1 << 3
}BackCamState;
BackCamState mid_cam_state = CAM_IDLE;
uint8_t mid_cam_oldstate = 0;
#endif

typedef enum _DevicePollState
{
    DEV_IDLE,
    DEV_RADIO,
    DEV_IKE,
    DEV_DSP,
    DEV_TEL,
    DEV_LAST = DEV_TEL
}DevicePollState;
DevicePollState mid_pollState = DEV_IDLE;


// mapping from button states to MID buttons
#define MID_MAP_SIZE 15
uint8_t mid_button_mapping[MID_MAP_SIZE] PROGMEM = {
    BUTTON_1,
    BUTTON_2,
    BUTTON_3,
    BUTTON_4,
    BUTTON_5,
    BUTTON_6,
#if ((DEVICE_CODING2 & RADIO_PROFESSIONAL) == RADIO_PROFESSIONAL)
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
    BUTTON_AM,
    BUTTON_FM,
#else
    BUTTON_FM,
    BUTTON_AM,
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
#endif
    BUTTON_INFO_L,
    BUTTON_MODE,
#if ((DEVICE_CODING2 & REW_FF_ONMID) == REW_FF_ONMID)
    BUTTON_REW,
    BUTTON_FF,
#else
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
#endif
    BUTTON_NUM_BUTTONS
};

uint8_t mid_button_mapping_dbyte1_mask[MID_MAP_SIZE] PROGMEM = {
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

#if ((DEVICE_CODING2 & REW_FF_ONMID) == REW_FF_ONMID)
//    #define OPENBM_MAP_SIZE 9
    #define BMBT_MAP_SIZE   9
#else
//    #define OPENBM_MAP_SIZE 11
    #define BMBT_MAP_SIZE   11
#endif

#if 0
uint8_t openbm_button_mapping[OPENBM_MAP_SIZE] PROGMEM = {
    BUTTON_EJECT,
    BUTTON_TEL,
    BUTTON_PRG,
    BUTTON_UHR,
    BUTTON_TONE,
    BUTTON_SELECT,
#if ((DEVICE_CODING2 & REW_FF_ONMID) != REW_FF_ONMID)
    BUTTON_REW,
    BUTTON_FF,
#endif
    BUTTON_MENU_LR,
    BUTTON_BMBT_KNOB,
    BUTTON_INFO_R
};
#endif

// original BMBT codes
uint8_t bmbt_button_mapping[BMBT_MAP_SIZE][3] PROGMEM =
{
#if ((DEVICE_CODING2 & REW_FF_ONMID) != REW_FF_ONMID)
    {BUTTON_FF,0x68,0x00},
    {BUTTON_REW,0x68,0x10},
#endif
    {BUTTON_INFO_R,0xFF,0x03},
    {BUTTON_TONE,0x68,0x04},
    {BUTTON_BMBT_KNOB,0x3B,0x05},
    {BUTTON_UHR,0xFF,0x07},
    {BUTTON_TEL,0xFF,0x08},
    {BUTTON_PRG,0x68,0x14},
    {BUTTON_SELECT,0x68,0x20},
    {BUTTON_EJECT,0x68,0x24},
    {BUTTON_MENU_LR,0xFF,0x34}
};

//------------------------------------------------------------------------------
void emul_mid_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    // answer to request messages C8 03 E7 01 2D -> C0 04 C8 02 00 0E
    if (msg[0] == IBUS_MSG_DEV_POLL && (dst == IBUS_DEV_ANZV || dst == IBUS_DEV_MID || dst == IBUS_DEV_GT || dst == IBUS_DEV_BMBT))
    {
        uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x00};
        ibus_sendMessage(dst, src, data, 2, 3);
    }

    // ok, radio will give us a hint, which current mode is active
    // this can be seen on text messages sent from radio to ANZV
    // so react only on text messages
    if (dst == IBUS_DEV_ANZV || dst == IBUS_DEV_MID)// || dst == IBUS_DEV_LOC)
    {
        // take into account only if for bottom text update
        if (msg[0] == IBUS_MSG_UPDATE_MID_BOTTOM)
        {
            if (src == IBUS_DEV_RAD)
            {
                // acknowledge msg
                uint8_t data[2] = {IBUS_MSG_MID_ACK_TEXT, 0x01};

                // BMW Business with DSP ( 68 06 C0 21 00 00 20 AF -> ACK differently)
                if (msglen == 4 && msg[1] == 0 && msg[2] == 0 && msg[3] == 0x20)
                {
                    data[1] = 0;
                }else if (msglen >= 2)
                {
                    mid_active_mode = msg[1];
                    data[1] = 0x02;
                }

                // let receiver know, that we accepted the change
                ibus_sendMessage(IBUS_DEV_MID, src, data, 2, 3);
            }
        }

        // React on LED message
        if (USE_BM_LEDS() && msg[0] == IBUS_MSG_LED)
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
    }

    // special treatment for messages defined for BMBT/OpenBM
    if (dst == IBUS_DEV_BMBT)
    {
        if (msg[0] == IBUS_MSG_LED)
        {
            if (msg[1] & (1 << 0)) led_red_set(0b11111111); else led_red_set(0);
            if (msg[1] & (1 << 1)) led_red_set(0b11110000);

            if (msg[1] & (1 << 2)) led_yellow_set(0b11111111); else led_yellow_set(0);
            if (msg[1] & (1 << 3)) led_yellow_set(0b11110000);

            if (msg[1] & (1 << 4)) led_green_set(0b11111111); else led_green_set(0);
            if (msg[1] & (1 << 5)) led_green_set(0b11110000);

            if (msg[1] & (1 << 6)) led_fan_set(0b11111111); else led_fan_set(0);
            if (msg[1] & (1 << 7)) led_fan_set(0b11110000);

        }else if (msg[0] == IBUS_MSG_LED_SPECIAL)
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
    }

    // change display to back camera if "R" is received
    // 80 09 BF 13 00/02 11/10 00 00 00 00 xx
    if (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO)
    {
        if (msglen == 7 && msg[0] == IBUS_MSG_IKE_STATE && display_getPowerState())
        {
            // REVERSE gear selected
            if ((msg[2] & 0xF0) == 0x10)
            {
                // switch only if in idle mode and not set to AV already
                if (mid_cam_state == CAM_IDLE && display_getInputState() != 1)
                {
                    mid_cam_state |= CAM_PREPARE_TO_SWITCH;
                    mid_cam_state |= CAM_ON;
                }

            // other gear, change back to previous mode
            }else
            {
                mid_cam_state |= CAM_PREPARE_TO_SWITCH;
                mid_cam_state &= ~CAM_ON;
            }
        }
    }

}

//------------------------------------------------------------------------------
void emul_mid_ping_tick(void)
{
    // on very first start, ask for state update
    static uint8_t firstTicks = 0;
    if (firstTicks < TICKS_PER_SECOND())
        firstTicks++;
    else if (firstTicks == TICKS_PER_SECOND())
    {
        firstTicks++;

        uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x00, 0x01, 0x00};

        if (g_deviceSettings.device_Settings2 & RADIO_PROFESSIONAL)
            data[2] |= 0xB2;

        ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_LOC, data, 4, 5);
    }

    // every 10 seconds we perform a ping on different hardware to check if this exists
    static ticks_t ticks = 0;
    if (mid_pollState == DEV_IDLE) ticks++;

    if (ticks > TICKS_PER_SECOND() * 10)
    {
        // go to next ping state and ping
        if (mid_pollState == DEV_IDLE)
        {
            mid_pollState = DEV_RADIO;

            //C0 03 68 01 AA
            uint8_t data[1] = {IBUS_MSG_DEV_POLL};
            ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 1, 5);

        }else if (mid_pollState == DEV_RADIO)
        {
            mid_pollState = DEV_IKE;

            //C0 03 80 01 42
            uint8_t data[1] = {IBUS_MSG_DEV_POLL};
            ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_IKE, data, 1, 5);

        }else if (mid_pollState == DEV_IKE)
        {
            mid_pollState = DEV_DSP;

            //C0 03 6A 01 A8
            if (g_deviceSettings.device_Settings2 & DSP_AMPLIFIER)
            {
                uint8_t data[1] = {IBUS_MSG_DEV_POLL};
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_DSP, data, 1, 5);
            }

        }else if (mid_pollState == DEV_DSP)
        {
            mid_pollState = DEV_TEL;

            //C0 03 C8 01 0A
            uint8_t data[1] = {IBUS_MSG_DEV_POLL};
            ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_TEL, data, 1, 5);

        }else if (mid_pollState == DEV_LAST)
        {
            mid_pollState = DEV_IDLE;
            ticks = 0;
        }
    }
}

//------------------------------------------------------------------------------
void emul_mid_backcam_tick(void)
{
    static ticks_t ticks = 0;

    // if we are in the mode prepared to switch, then switch after certain amount of ticks
    if (mid_cam_state & CAM_PREPARE_TO_SWITCH)
    {
        ticks++;

        // react after 500ms
        if (ticks > TICKS_PER_HALFSECOND())
        {
            // switch camera according to the output we like to have
            if (mid_cam_state & CAM_ON)
            {
                mid_cam_state = CAM_SWITCHING;
                mid_cam_oldstate = display_getInputState();
                display_setInputState(BACKCAM_INPUT());
            }else
            {
                mid_cam_state = CAM_SWITCHING;
                display_setInputState(mid_cam_oldstate);
            }

            // while camera was swithcing, we might have received new messages
            // so whenever camera state is still swithcing, we are safe to go into idle
            // otherwise, on the next tick we will reset the output again
            BEGIN_ATOMAR;
                if (mid_cam_state == CAM_SWITCHING)
                    mid_cam_state = CAM_IDLE;
            END_ATOMAR;

            ticks = 0;
        }
    }else
        ticks = 0;
}

//------------------------------------------------------------------------------
void emul_mid_tick(void)
{
    emul_mid_ping_tick();
    emul_mid_backcam_tick();
    
    // turn ON, radio LED if in RADIO mode
    led_radio_immediate_set(mid_active_mode == 0x40);

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
            ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_RAD, data, 2, 1);
        }else if (button_released(BUTTON_RADIO_KNOB))
        {
            if (ignoreReleaseEvent == 0)
            {
                if (mid_active_mode)
                {
                    display_setPowerState(0);
                    mid_active_mode = 0;
                }else
                {
                    display_setPowerState(1);
                }

                //uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB0, 0x00};
                uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB2, 0x00};

                // if DSP, then include DSP bit to switch DSP on/off
                if (g_deviceSettings.device_Settings2 & DSP_AMPLIFIER)
                    data[3] |= 0x20;

                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_LOC, data, 4, 1);
            }
            ignoreReleaseEvent = 0;
        }
    }

    // prepare message to send, when a button is down/pressed/up
    // check for every button for which we have mapping defined
    {
        uint8_t i;
        for (i=0; i < MID_MAP_SIZE; i++)
        {
            // read to which button this one maps to and go to next, if no mapping
            uint8_t bmap = pgm_read_byte(&(mid_button_mapping[i]));
            if (bmap >= BUTTON_NUM_BUTTONS) continue;

            uint8_t bmask = pgm_read_byte(&(mid_button_mapping_dbyte1_mask[i]));

            // prepare ibus message for this button
            uint8_t data[4] = {IBUS_MSG_BUTTON, mid_active_mode & bmask, 0x00, i};
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
            // read to which button this one maps to and go to next, if no mapping
            uint8_t bmap = pgm_read_byte(&(bmbt_button_mapping[i][0]));
            uint8_t bdst = pgm_read_byte(&(bmbt_button_mapping[i][1]));
            uint8_t bcod = pgm_read_byte(&(bmbt_button_mapping[i][2]));
            
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

}

//------------------------------------------------------------------------------
void emul_mid_init(void)
{
    mid_cam_oldstate = display_getInputState();
    
    //if (display_getInputState() == 1) mid_cam_state = CAM_ON | CAM_SWITCHED;
    //else mid_cam_state = CAM_IDLE;
}

#if 0
//------------------------------------------------------------------------------
void emul_openbm_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{

}

//------------------------------------------------------------------------------
void emul_openbm_tick(void)
{

}

//------------------------------------------------------------------------------
void emul_bmbt_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{

}

//------------------------------------------------------------------------------
void emul_bmbt_tick(void)
{

}
#endif 

