#include "base.h"
#include "obm_special.h"
#include "buttons.h"
#include "ibus.h"
#include "display.h"
#include "config.h"
#include "leds.h"

typedef enum _BackCamState
{
    CAM_IDLE = 0,
    CAM_PREPARE_TO_SWITCH = 1 << 1,
    CAM_SWITCHING = 1 << 2,
    CAM_ON = 1 << 3
}BackCamState;
BackCamState _cam_state;
uint8_t _cam_oldstate;


typedef enum _Gear
{
    UNKNOWN,
    REVERSE,
    DRIVE,
    NEUTRAL,
    PARK,
    G1,
    G2,
    G3,
    G4,
    G5,
    G6
}Gear;
Gear _selectedGear = UNKNOWN;


#define IDLE                     0

#define LEFT_TURNLIGHT           0
#define RIGHT_TURNLIGHT          1
#define FOGLIGHT_FRONT_LEFT      2
#define FOGLIGHT_FRONT_RIGHT     3

#define LOWBEAM_LEFT             4
#define LOWBEAM_RIGHT            5

#define PARKINGLIGHT_LEFT        6
#define PARKINGLIGHT_RIGHT       7

#define HIGHBEAM_LEFT            8
#define HIGHBEAM_RIGHT           9

#define BRAKE_LEFT               10
#define BRAKE_CENTER             11
#define BRAKE_RIGHT              12

#define FOGLIGHT_BACK_LEFT       13
#define FOGLIGHT_BACK_RIGHT      14

#define BACKLIGHT_LEFT           15
#define BACKLIGHT_RIGHT          16

#define REARGEAR_LEFT            17
#define REARGEAR_RIGHT           18

#define NUMBERPAD                19
#define EMERGENCY                20
#define LEFT_TURNLIGHT_FLASH     21
#define RIGHT_TURNLIGHT_FLASH    22
#define FOGLIGHT_TURN_LEFT       23
#define FOGLIGHT_TURN_RIGHT      24

typedef uint32_t lightState_t;

lightState_t _lightState = IDLE;
lightState_t _oldLightState = IDLE;

bool _centralLocked = false;
bool _parkingBrake = false;
uint8_t _carHalfSpeed = 0;
uint8_t _carRPM = 0;

bool _doToggleCentralLock = false;


#define DEV_IKE      (1 << 0)
#define DEV_IKE_DONE (1 << 1)

#define DEV_GM       (1 << 3)
#define DEV_GM_DONE  (1 << 4)

uint8_t _reqStatus = 0;

//ticks_t _nextSendMFLSignalTick;
//bool _nextSendMFLSignal;

//------------------------------------------------------------------------------
void obms_toggle_central_lock(void)
{
    uint8_t data[3] = {IBUS_MSG_VEHICLE_CTRL, 0x00, 0x0B};
    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);

    _centralLocked = !_centralLocked;
}

//------------------------------------------------------------------------------
void obms_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{

    // driver door open   00 05 BF 7A 11 02 xx
    // driver door open   00 05 BF 7A 51 02 xx
    // driver door closed 00 05 BF 7A 10 02 xx
    // driver door closed 00 05 BF 7A 50 02 xx

    // passanger door     00 05 BF 7A 13 02 xx
    // passanger door     00 05 BF 7A 53 02 xx

    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO)
    {
        if (msg[0] == IBUS_MSG_GM_STATE)
        {
            _reqStatus |= DEV_GM_DONE;

            // central unlocked   00 05 BF 7A 10 02 xx
            // central locked     00 05 BF 7A 20 02 xx
            if (msg[1] & 0x20)
                _centralLocked = 1;
            else
                _centralLocked = 0;
        }
        if (!(_reqStatus & DEV_GM_DONE))
            _reqStatus |= DEV_GM;
    }

    // IKE state
    // 80 09 BF 13 00/02 11/10 00 00 00 00 xx
    if (src == IBUS_DEV_IKE && dst == IBUS_DEV_GLO)
    {
        if (msglen == 3 && msg[0] == IBUS_MSG_IKE_SPEED)
        {
            _carHalfSpeed = msg[1];
            _carRPM = msg[2];
        }
        if (msglen == 7 && msg[0] == IBUS_MSG_IKE_STATE)
        {
            _reqStatus |= DEV_IKE_DONE;

            // hand brake
            _parkingBrake = msg[1] & 1;

            // REVERSE gear selected
            if ((msg[2] & 0xF0) == 0x10)
                _selectedGear = REVERSE;
            else if ((msg[2] & 0xF0) == 0xB0)
                _selectedGear = PARK;
            else
                _selectedGear = UNKNOWN;


            // if display is enabled, then process with backup camera
            if (display_getPowerState() == 1)
            {
                if (_selectedGear == REVERSE)
                {
                    if (_cam_state == CAM_IDLE && display_getInputState() != BACKCAM_INPUT())
                    {
                        _cam_state |= CAM_PREPARE_TO_SWITCH | CAM_ON;
                        _cam_state |= CAM_ON;
                    }
                }else{
                    _cam_state |= CAM_PREPARE_TO_SWITCH;
                    _cam_state &= ~CAM_ON;
                }
            }
        }
        if (!(_reqStatus & DEV_IKE_DONE))
            _reqStatus |= DEV_IKE;

        // if ignition is switched off and central is locked, then unlock it
        if (msglen ==2 && msg[0] == IBUS_MSG_IGNITION && msg[1] != 0x03 && _centralLocked)
            _doToggleCentralLock = true;
    }

    // update of the lamp state
    if (src == IBUS_DEV_LCM && dst == IBUS_DEV_GLO && msglen == 5 && msg[0] == IBUS_MSG_LAMP_STATE)
    {
        _oldLightState = _lightState;
        _lightState = IDLE;
        if (msg[1] & 0x01) _lightState |= (1L << PARKINGLIGHT_LEFT) | (1L << PARKINGLIGHT_RIGHT);
        if (msg[1] & 0x02) _lightState |= (1L << LOWBEAM_LEFT) | (1L << LOWBEAM_RIGHT);
        if (msg[1] & 0x04) _lightState |= (1L << HIGHBEAM_LEFT) | (1L << HIGHBEAM_RIGHT);
        if (msg[1] & 0x08) _lightState |= (1L << FOGLIGHT_FRONT_LEFT) | (1L << FOGLIGHT_FRONT_RIGHT);
        if (msg[1] & 0x10) _lightState |= (1L << FOGLIGHT_BACK_LEFT) | (1L << FOGLIGHT_BACK_RIGHT);
        if (msg[1] & 0x20)
        {
            _lightState |= (1L << LEFT_TURNLIGHT);
            if (msg[3] & 0x04) _lightState |= (1L << LEFT_TURNLIGHT_FLASH);
        }
        if (msg[1] & 0x40)
        {
            _lightState |= (1L << RIGHT_TURNLIGHT);
            if (msg[3] & 0x04) _lightState |= (1L << RIGHT_TURNLIGHT_FLASH);
        }
        if (_selectedGear == REVERSE)
            _lightState |= (1L << REARGEAR_LEFT) | (1L << REARGEAR_RIGHT);
    }



}

//------------------------------------------------------------------------------
void obms_backcam_tick(void)
{
    static ticks_t ticks = 0;

    // if we are in the mode prepared to switch, then switch after certain amount of ticks
    if (_cam_state & CAM_PREPARE_TO_SWITCH)
    {
        ticks++;

        // react after 500ms
        if (ticks > TICKS_PER_HALFSECOND())
        {
            _cam_state = CAM_SWITCHING;

            // switch camera according to the output we like to have
            if (_cam_state & CAM_ON)
            {
                _cam_oldstate = display_getInputState();
                display_setInputState(BACKCAM_INPUT());
            }else
            {
                display_setInputState(_cam_oldstate);
            }

            // while camera was switching, we might have received new messages
            // so whenever camera state is still switching, we are safe to go into idle
            // otherwise, on the next tick we will reset the output again
            //BEGIN_ATOMAR;
                if (_cam_state == CAM_SWITCHING)
                    _cam_state = CAM_IDLE;
            //END_ATOMAR;

            ticks = 0;
        }
    }else
        ticks = 0;
}

//------------------------------------------------------------------------------
void obms_tick(void)
{
    obms_backcam_tick();

    // if Central Lock feature is enabled and car is running
    if (g_deviceSettings.obms_centralLock)
    {
        // if we just need to toggle the central lock
        if (_doToggleCentralLock

        // if the central is locked and hand brake was activated, then open the door
        // open also the door if locked and P-gear was set
          || (_centralLocked && (_parkingBrake || _selectedGear == PARK))

        // if unlocked and car speed is above certain limit then lock the car
          || (!_centralLocked && _carHalfSpeed > g_deviceSettings.obms_centralLock))
        {
            obms_toggle_central_lock();
            _doToggleCentralLock = false;
        }
    }

    // if we are allowed to request the status of a device, then do so
    if (_reqStatus & DEV_IKE)
    {
        uint8_t data[3] = {IBUS_MSG_IKE_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_IKE, data, 3, IBUS_TRANSMIT_TRIES);

        _reqStatus &= ~DEV_IKE;
        _reqStatus |= DEV_IKE_DONE;
    }
    if (_reqStatus & DEV_GM)
    {
        uint8_t data[3] = {IBUS_MSG_GM_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);

        _reqStatus &= ~DEV_GM;
        _reqStatus |= DEV_GM_DONE;
    }
}


//------------------------------------------------------------------------------
void obms_init(void)
{
    _centralLocked = 0;
    _parkingBrake = 0;
    _carHalfSpeed = 0;
    _carRPM = 0;
    _cam_state = CAM_IDLE;
    _cam_oldstate = display_getInputState();
    _reqStatus = 0;
}
