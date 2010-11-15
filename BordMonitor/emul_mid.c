#include "base.h"
#include "emul_mid.h"
#include <avr/pgmspace.h>
#include "buttons.h"
#include "ibus.h"
#include "include/ibus.h"
#include "leds.h"
#include "display.h"

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
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
    BUTTON_AM,
    BUTTON_FM,
    BUTTON_INFO_L,
    BUTTON_MODE,
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS,
    BUTTON_NUM_BUTTONS
};

uint8_t mid_button_mapping_dbyte1_mask[MID_MAP_SIZE] PROGMEM = {
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0x00,
    0x00,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0x00,
    0x00,
    0x00
};

#define OPENBM_MAP_SIZE 11
uint8_t openbm_button_mapping[OPENBM_MAP_SIZE] PROGMEM = {
    BUTTON_EJECT,
    BUTTON_TEL,
    BUTTON_PRG,
    BUTTON_UHR,
    BUTTON_TONE,
    BUTTON_SELECT,
    BUTTON_REW,
    BUTTON_FF, 
    BUTTON_MENU_LR,
    BUTTON_BMBT_KNOB,
    BUTTON_INFO_R
};

//------------------------------------------------------------------------------
void emul_mid_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    // answer to request messages C8 03 E7 01 2D -> C0 04 C8 02 00 0E
    if (msg[0] == IBUS_MSG_DEV_POLL && (dst == IBUS_DEV_ANZV || dst == IBUS_DEV_GT || dst == IBUS_DEV_BMBT))
    {
        uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x00};
        ibus_sendMessage(dst, src, data, 2, 3);
    }

    // ok, radio will give us a hint, which current mode is active
    // this can be seen on text messages sent from radio to ANZV
    // so react only on text messages
    //if (dst != IBUS_DEV_ANZV || dst != IBUS_DEV_MID || dst != IBUS_DEV_LOC)
    {
        if (msg[0] == IBUS_MSG_UPDATE_MID_TOP || msg[0] == IBUS_MSG_UPDATE_MID_BOTTOM)
        {
            // acknowledge msg
            uint8_t data[2] = {IBUS_MSG_MID_ACK_TEXT, 0x01};

            if (src == IBUS_DEV_RAD && msglen >= 2)
            {
                mid_active_mode = msg[1];
                data[1] = 0x02;
            }

            // let receiver know, that we accepted the change
            ibus_sendMessage(IBUS_DEV_MID, src, data, 2, 3);
        }

        if (0 && msg[0] == IBUS_MSG_LED)
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

            /*
            if ((msg[2] & 0xF0) == 0x10 && display_getInputState() != 1)
            {
                mid_cam_state |= CAM_PREPARE_TO_SWITCH;
                mid_cam_state |= CAM_ON;
            }else if (msg[2] != 0x13 && msg[2] != 0x11)
            {
                mid_cam_state |= CAM_PREPARE_TO_SWITCH;
                mid_cam_state &= ~CAM_ON;
            }*/

            
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
        
        // send enable MID message C0 06 FF 20 00 B3 00
        uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x00, 0xB3, 0x00};
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
                display_setInputState(1);
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
    
    // turn ON, radio LED if not in TAPE mode
    led_radio_immediate_set(mid_active_mode && mid_active_mode != 0x80);

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
            uint8_t data[2] = {IBUS_MSG_BMBT_BUTTON, 0x4F};
            ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 2, 1);
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

                uint8_t data[4] = {IBUS_MSG_MID_STATE_BUTTONS, 0x20, 0xB0, 0x00};
                ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_LOC, data, 4, 1);
            }
            ignoreReleaseEvent = 0;
        }//else if (button_pressed(BUTTON_RADIO_KNOB))
        //{
        //    uint8_t data[2] = {IBUS_MSG_BMBT_BUTTON, 0x0F};
        //    ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 2, 1);
        //}
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
        for (i=0; i < OPENBM_MAP_SIZE; i++)
        {
            // read to which button this one maps to and go to next, if no mapping
            uint8_t bmap = pgm_read_byte(&(openbm_button_mapping[i]));
            if (bmap >= BUTTON_NUM_BUTTONS) continue;

            // prepare ibus message for this button
            uint8_t data[2] = {IBUS_MSG_BMBT_BUTTON, i};
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
                ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_LOC, data, 2, IBUS_TRANSMIT_TRIES);
        }
    }
}

//------------------------------------------------------------------------------
void emul_mid_encoder_tick(void)
{
    // radio knob
    if (button(BUTTON_RADIO_CW) || button(BUTTON_RADIO_CCW))
    {
        uint8_t data[2] = {IBUS_MSG_RADIO_ENCODER, 0x10};

        if (button(BUTTON_RADIO_CW))
            data[1] += 1;

        ibus_sendMessage(IBUS_DEV_MID, IBUS_DEV_RAD, data, 2, 1);
    }

    // Bordmonitor knob
    if (button(BUTTON_BMBT_CW) || button(BUTTON_BMBT_CCW))
    {
        uint8_t data[2] = {IBUS_MSG_BMBT_ENCODER, 0x01};

        if (button(BUTTON_BMBT_CW))
            data[1] += 0x80;

        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_GT, data, 2, 1);
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

