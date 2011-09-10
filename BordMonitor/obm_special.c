#include "base.h"
#include "obm_special.h"
#include "buttons.h"
#include "ibus.h"
#include "display.h"
#include "photo_sensor.h"
#include "config.h"
#include "leds.h"
#include "power_module.h"
#include "include/ibus.h"
#include <avr/eeprom.h>


typedef uint32_t lightState_t;


typedef struct _OBMSSettings
{
    uint8_t  automaticCentralLock;

    uint8_t  lockCarUnused;

    uint8_t  comfortTurnLight;

    uint8_t  leavingHome;
    uint32_t leavingHomeLights;
}ObmsSettings;


ObmsSettings obms_Settings;
ObmsSettings obms_SettingsEEPROM EEMEM;
uint8_t      obms_SettingsInit EEMEM;


typedef enum _BackCamState
{
    CAM_IDLE = 0,
    CAM_PREPARE_TO_SWITCH = 1 << 1,
    CAM_SWITCHING = 1 << 2,
    CAM_ON = 1 << 3
}BackCamState;
static BackCamState _cam_state;
//uint8_t _cam_oldstate;


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
static Gear _selectedGear = UNKNOWN;


#define IDLE                     0

#define LEFT_TURNLIGHT           (1L << 1)
#define RIGHT_TURNLIGHT          (1L << 2)

#define LEFT_TURNLIGHT_BACK      (1L << 3)
#define RIGHT_TURNLIGHT_BACK     (1L << 4)

#define FOGLIGHT_FRONT_LEFT      (1L << 5)
#define FOGLIGHT_FRONT_RIGHT     (1L << 6)

#define FOGLIGHT_BACK_LEFT       (1L << 7)
#define FOGLIGHT_BACK_RIGHT      (1L << 8)

#define LOWBEAM_LEFT             (1L << 9)
#define LOWBEAM_RIGHT            (1L << 10)

#define PARKINGLIGHT_LEFT        (1L << 11)
#define PARKINGLIGHT_RIGHT       (1L << 12)

#define HIGHBEAM_LEFT            (1L << 13)
#define HIGHBEAM_RIGHT           (1L << 14)

#define BRAKE_LEFT               (1L << 15)
#define BRAKE_CENTER             (1L << 16)
#define BRAKE_RIGHT              (1L << 17)

#define BACKLIGHT_LEFT           (1L << 18)
#define BACKLIGHT_RIGHT          (1L << 19)

#define REARGEAR_LEFT            (1L << 20)
#define REARGEAR_RIGHT           (1L << 21)

#define LICENSE_PLATE            (1L << 22)

#define LEFT_INDICATOR           (1L << 23)
#define RIGHT_INDICATOR          (1L << 24)


#define LIGHT_STATE_CHANGED      (1L << 31)

static lightState_t _lightState = IDLE;
//lightState_t _oldLightState = IDLE;

static bool _centralLocked = false;
static bool _parkingBrake = false;
static uint8_t _carHalfSpeed = 0;
static uint8_t _carRPM = 0;

static bool _doToggleCentralLock = false;

#define DEV_IKE      (1 << 0)
#define DEV_IKE_DONE (1 << 1)

#define DEV_GM       (1 << 3)
#define DEV_GM_DONE  (1 << 4)

static uint8_t _reqStatus = 0;

typedef enum _LeavingHome
{
    NO = 0,
    START = 1,
    HOLD = 2,
    STOP = 3,
    DISABLED = 4
}LeavingHome;
static LeavingHome _leavingHome = NO;

static uint8_t _tippBlinken = 0;
static uint8_t _tippBlinkenCounter = 0;
static uint8_t _dimmerState = 0;
static uint8_t _lwrState = 0;

static ticks_t _leavingHomeResendAfter = 0;
static ticks_t _leavingHomeStopAfter = 0;

static ticks_t _lockCarIfNoDoorOpened = 0;
static bool    _lockCarIfNoDoorOpenedActive = true;

//ticks_t _nextSendMFLSignalTick;
//bool _nextSendMFLSignal;

//------------------------------------------------------------------------------
void obms_toggle_central_lock(void)
{
    uint8_t data[3] = {IBUS_MSG_VEHICLE_CTRL, 0x00, 0x0B};
    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);
}

//------------------------------------------------------------------------------
void obms_req_light_state(void)
{
    uint8_t data[1] = {IBUS_MSG_VEHICLE_CTRL_REQ};
    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_LCM, data, 1, IBUS_TRANSMIT_TRIES);
}

//------------------------------------------------------------------------------
void obms_set_light_state(void)
{
    if (!(_lightState & LIGHT_STATE_CHANGED)) return;
    
    uint8_t data[13] = {IBUS_MSG_VEHICLE_CTRL, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, _dimmerState, _lwrState, 0x00};

    // prepare light message according to the current state
    if (_lightState & LEFT_TURNLIGHT)  data[8] |= 0x40;
    if (_lightState & RIGHT_TURNLIGHT) data[7] |= 0x40;
    if (_lightState & LEFT_TURNLIGHT_BACK) data[7] |= 0x80;
    if (_lightState & RIGHT_TURNLIGHT_BACK) data[8] |= 0x02;

    if (_lightState & LEFT_INDICATOR) data[3] |= 0x80;
    if (_lightState & RIGHT_INDICATOR) data[3] |= 0x40;

    if (_lightState & REARGEAR_RIGHT) data[8] |= 0x80;
    if (_lightState & REARGEAR_LEFT) data[6] |= 0x08;

    if (_lightState & BACKLIGHT_RIGHT) data[8] |= 0x10;
    if (_lightState & BACKLIGHT_LEFT) data[7] |= 0x08;

    if (_lightState & FOGLIGHT_BACK_LEFT) data[8] |= 0x04;
    //if (_lightState & FOGLIGHT_BACK_RIGHT) data[8] |= 0x04; not connected in the car, hence unknown

    if (_lightState & LICENSE_PLATE) data[7] |= 0x04;

    if (_lightState & PARKINGLIGHT_LEFT) data[6] |= 0x01;
    if (_lightState & PARKINGLIGHT_RIGHT) data[7] |= 0x20;

    if (_lightState & LOWBEAM_LEFT) data[6] |= 0x10;
    if (_lightState & LOWBEAM_RIGHT) data[6] |= 0x20;

    if (_lightState & HIGHBEAM_LEFT) data[5] |= 0x40;
    if (_lightState & HIGHBEAM_RIGHT) data[5] |= 0x20;

    if (_lightState & FOGLIGHT_FRONT_LEFT) data[6] |= 0x04;
    if (_lightState & FOGLIGHT_FRONT_RIGHT) data[6] |= 0x40;

    if (_lightState & BRAKE_LEFT) data[5] |= 0x08;
    if (_lightState & BRAKE_RIGHT) data[5] |= 0x10;
    if (_lightState & BRAKE_CENTER) data[7] |= 0x10;

    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_LCM, data, 13, IBUS_TRANSMIT_TRIES);

    _lightState &= ~LIGHT_STATE_CHANGED;
}

//------------------------------------------------------------------------------
void obms_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO)
    {
        // if GM has not been seen yet, then ping it on next tick
        if (!(_reqStatus & DEV_GM_DONE))
            _reqStatus |= DEV_GM;

        if (msg[0] == IBUS_MSG_GM_STATE)
        {
            // GM has been seen now
            _reqStatus |= DEV_GM_DONE;

            // central unlocked   00 05 BF 7A [dd1=10] [dd2=00] xx // dd1 = bit8 - trunk, bit7 = kl50, bit6=locked, bit5=?, bit4-bit1=door, dd2 = bit6 - trunk opened, bit4-1 windows
            // central locked     00 05 BF 7A 20 00 xx
            if (msg[1] & 0x20)
                _centralLocked = 1;
            else
                _centralLocked = 0;

            if ((msg[1] & 0xF0) && _lockCarIfNoDoorOpenedActive)
                _lockCarIfNoDoorOpened = tick_get() + TICKS_PER_X_SECONDS(obms_Settings.lockCarUnused);

            // just any door was opened
            if (msg[1] & 0x0F)
            {
                // deactivate leaving home
                if (_leavingHome == HOLD)
                    _leavingHome = STOP;

                // stop lock door counter, if this one is running
                _lockCarIfNoDoorOpened = 0;
                _lockCarIfNoDoorOpenedActive = false;

            // light inside turns on (KL50), however door wasn't opened, so just unlocked the car
            }else if ((msg[1] & 0x40) && _leavingHome == NO && !photo_is_bright_enough())
                _leavingHome = START;
        }
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
            // ok, IKE has been seen now
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
                        _cam_state = CAM_PREPARE_TO_SWITCH | CAM_ON;
                }else
                    _cam_state = CAM_PREPARE_TO_SWITCH;
            }
        }

        // if IKE has not been seen till now, then request its state on next tick
        if (!(_reqStatus & DEV_IKE_DONE))
            _reqStatus |= DEV_IKE;

        // if ignition is switched off and central is locked, then unlock it
        if (msglen ==2 && msg[0] == IBUS_MSG_IGNITION && msg[1] && msg[1] != 0x03 && _centralLocked)
            _doToggleCentralLock = true;
    }

    // Status Message from LCM, response to 3F,D0,0B
    // D0 23 3F A0 C1 C0 00 00 00 00 00 00 00 9B 00 00 80 00 00 59 33 00 08 00 00 00 00 00 00 00 00 00 00 FF FF FE EA
    if (src == IBUS_DEV_LCM && dst == IBUS_DEV_DIA && msglen > 20 && msg[0] == IBUS_MSG_DIA_ACK && msg[1] == 0xC1)
    {
        _dimmerState = msg[16];
        _lwrState = msg[17];
    }

    // update of the lamp state
    if (src == IBUS_DEV_LCM && dst == IBUS_DEV_GLO)
    {
        if (msg[0] == IBUS_MSG_DIMMER_STATE)
        {
            // delete both values, which will force to get the value on next tick
            _dimmerState = 0;
            _lwrState = 0;
        }else if (msglen == 5 && msg[0] == IBUS_MSG_LAMP_STATE)
        {
            // TippBlinken only active if not already running
            if ((_tippBlinken & 0x0F) == 0 && obms_Settings.comfortTurnLight > 0)
            {
                // turn flash light
                if (msg[3] == 0x04)
                {
                    // activated left
                    if (msg[1] & 0x20)
                    {
                        // if we have had right flash before, then reset the counter
                        if (_tippBlinken & 0x40) _tippBlinkenCounter = 0;
                        _tippBlinken = 0x80;

                    // activated right
                    }else if (msg[1] & 0x40)
                    {
                        // if we have had left flash before, then reset the counter
                        if (_tippBlinken & 0x80) _tippBlinkenCounter = 0;
                        _tippBlinken = 0x40;
                    }
                    _tippBlinkenCounter++;

                // if turn flash light is turned off
                }else if (msg[3] == 0)
                {
                    // activate tipp blinken for at most X flashes
                    if (_tippBlinkenCounter > 0 && _tippBlinkenCounter < obms_Settings.comfortTurnLight)
                        _tippBlinken |= (obms_Settings.comfortTurnLight - _tippBlinkenCounter);
                    _tippBlinkenCounter = 0;
                }
            }
        }
    }


    // Setup settings if asked so over IBus
    if (dst == IBUS_DEV_BMBT && msg[0] == IBUS_MSG_OPENBM_TO && msg[1] == IBUS_MSG_OPENBM_OBMS_SET)
    {
        // every such message will be responded, default response is 0
        uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_OBMS_SET, msg[2]};

        if (msglen == 4 && msg[2] == 0x01)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.automaticCentralLock, msg[3] >> 1);
            obms_Settings.automaticCentralLock = msg[3] >> 1;
        }else if (msglen == 4 && msg[2] == 0x02)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.lockCarUnused, msg[3]);
            obms_Settings.lockCarUnused = msg[3];
        }else if (msglen == 4 && msg[2] == 0x03)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.comfortTurnLight, msg[3]);
            obms_Settings.comfortTurnLight = msg[3];
        }else if (msglen == 4 && msg[2] == 0x04)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.leavingHome, msg[3]);
            obms_Settings.leavingHome = msg[3];
        }else if (msglen == 7 && msg[2] == 0x05)
        {
            union
            {
                struct
                {
                    uint8_t byte[4];
                };
                uint32_t lights;
            } light;
            light.byte[3] = msg[3];
            light.byte[2] = msg[4];
            light.byte[1] = msg[5];
            light.byte[0] = msg[6];
            eeprom_update_dword(&obms_SettingsEEPROM.leavingHomeLights, light.lights);
            obms_Settings.leavingHomeLights = light.lights;
        }else
        {
            data[2] = 0;
        }
        ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);
    }
}

//------------------------------------------------------------------------------
void obms_backcam_tick(void)
{
    // do not run this procedure if camera is not active
    if (!BACKCAM_INPUT()) return;

    static ticks_t ticks = 0;

    // if we are in the mode prepared to switch, then switch after certain amount of ticks
    uint8_t state = _cam_state;
    if (state & CAM_PREPARE_TO_SWITCH)
    {
        ticks++;

        // react after 500ms
        if (ticks > TICKS_PER_HALFSECOND)
        {
            _cam_state = CAM_SWITCHING;

            // switch camera according to the output we like to have
            if (state & CAM_ON)
            {
                //_cam_oldstate = display_getInputState();
                display_setInputState(BACKCAM_INPUT());
            }else
            {
                display_setInputState(0);//_cam_oldstate);
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

    // request state of the dimmer and lwr if unknown
    if (_dimmerState == 0 && _lwrState == 0)
    {
        obms_req_light_state();

        // just set to some state, so that we don't resend this message before recieved an answer
        _dimmerState = 0XFF;
        _lwrState = 0X33;
    }

    // Handle leaving home support if such is activated
    if (obms_Settings.leavingHome)
    {
        switch(_leavingHome)
        {
            // start leaving home, this is done by activating predefined lights
            case START:
            {
                _lightState = obms_Settings.leavingHomeLights;
                _lightState |= LIGHT_STATE_CHANGED;
                _leavingHome = HOLD;

                _leavingHomeStopAfter = tick_get() + TICKS_PER_X_SECONDS(obms_Settings.leavingHome);
                _leavingHomeResendAfter = 0;

                break;
            }

            // hold leaving home, this is done by resending the light message
            case HOLD:
            {
                _leavingHomeResendAfter++;

                if (_leavingHomeResendAfter > TICKS_PER_X_SECONDS(4))
                {
                    _leavingHomeResendAfter = 0;
                    _lightState |= LIGHT_STATE_CHANGED;
                }
                if (tick_get() > _leavingHomeStopAfter)
                {
                    _lightState = LIGHT_STATE_CHANGED;
                    _leavingHome = DISABLED;
                }
                break;
            }

            // stop leaving home, do this by turning off all lights
            case STOP:
            {
                _lightState = LIGHT_STATE_CHANGED;
                _leavingHome = DISABLED;
                break;
            }

        case DISABLED:
        case NO:
            default:
                break;
        }
    }

    // If comfort turn light is active
    if (obms_Settings.comfortTurnLight)
    {
        // 3f,d0,0c,0,0,80,0,0,0,0,0,0 - indicator left
        // 3f,d0,0c,0,0,40,0,0,0,0,0,0 - indicator right
        static bool flash = true;
        static ticks_t nextFlash = 0;

        uint8_t tippCounter = _tippBlinken & 0x0F;
        if (tippCounter > 0)
        {
            if (tick_get() > nextFlash)
            {
                nextFlash = 0;
                _lightState &= ~LEFT_INDICATOR & ~RIGHT_INDICATOR;

                if (flash)
                {
                    if (_tippBlinken & 0x80)
                        _lightState |= LEFT_INDICATOR;
                    else if (_tippBlinken & 0x40)
                        _lightState |= RIGHT_INDICATOR;
                }
                else
                {
                    _tippBlinken &= 0xF0;
                    _tippBlinken |= (tippCounter - 1);
                }
                _lightState |= LIGHT_STATE_CHANGED;
                flash = !flash;
            }
            if (nextFlash == 0)
                nextFlash = tick_get() + 9;
        }
    }

    // set state of light if changed
    obms_set_light_state();

    // if Central Lock feature is enabled
    if (obms_Settings.automaticCentralLock)
    {
        // if we just need to toggle the central lock
        if (_doToggleCentralLock

        // if the central is locked and hand brake was activated, then open the door
        // open also the door if locked and P-gear was set
          || (_centralLocked && (_parkingBrake || _selectedGear == PARK))

        // if unlocked and car speed is above certain limit then lock the car
          || (!_centralLocked && _carHalfSpeed > obms_Settings.automaticCentralLock))
        {
            obms_toggle_central_lock();
            _doToggleCentralLock = false;
        }
    }

    // if we are allowed to request the status of a device, then do so
    if (_reqStatus & DEV_IKE)
    {
        uint8_t data[2] = {IBUS_MSG_IKE_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_IKE, data, 2, IBUS_TRANSMIT_TRIES);

        data[0] = IBUS_MSG_IGNITION_REQ;
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_IKE, data, 2, IBUS_TRANSMIT_TRIES);

        _reqStatus &= ~DEV_IKE;
        _reqStatus |= DEV_IKE_DONE;
    }
    if (_reqStatus & DEV_GM)
    {
        uint8_t data[2] = {IBUS_MSG_GM_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 2, IBUS_TRANSMIT_TRIES);

        _reqStatus &= ~DEV_GM;
        _reqStatus |= DEV_GM_DONE;
    }

    // if we have a timer activated to close the central lock, then do so
    if (obms_Settings.lockCarUnused)
    {
        if (_lockCarIfNoDoorOpenedActive && _lockCarIfNoDoorOpened > 0 && tick_get() > _lockCarIfNoDoorOpened)
        {
            _lockCarIfNoDoorOpened = 0;
            _lockCarIfNoDoorOpenedActive = false;
            if (!_centralLocked)
                obms_toggle_central_lock();
            power_prepare_shutdown();
        }
    }
}

//------------------------------------------------------------------------------
void obms_stop(void)
{
    _lockCarIfNoDoorOpened = 0;
    _leavingHome = NO;
    _lockCarIfNoDoorOpenedActive = false;
}

//------------------------------------------------------------------------------
void obms_resume(void)
{
    obms_init();
}

//------------------------------------------------------------------------------
void obms_init(void)
{
    _centralLocked = 1;
    _parkingBrake = 0;
    _carHalfSpeed = 0;
    _carRPM = 0;
    _cam_state = CAM_IDLE;
    //_cam_oldstate = display_getInputState();
    _reqStatus = DEV_GM;
    _leavingHome = NO;
    _tippBlinken = 0;
    _tippBlinkenCounter = 0;
    _dimmerState = 0;
    _lwrState = 0;
    _lockCarIfNoDoorOpened = 0;
    _lockCarIfNoDoorOpenedActive = true;


    // if run for the first time, then write default data to eeprom
    if (eeprom_read_byte(&obms_SettingsInit) != 'O')
    {
        eeprom_update_byte(&obms_SettingsEEPROM.automaticCentralLock, 5);
        eeprom_update_byte(&obms_SettingsEEPROM.lockCarUnused, 60);

        eeprom_update_byte(&obms_SettingsEEPROM.comfortTurnLight, 3);

        eeprom_update_byte(&obms_SettingsEEPROM.leavingHome, 30);
        eeprom_update_dword(&obms_SettingsEEPROM.leavingHomeLights,
                         (BACKLIGHT_LEFT | BACKLIGHT_RIGHT |
                          FOGLIGHT_FRONT_LEFT | FOGLIGHT_FRONT_RIGHT |
                          PARKINGLIGHT_LEFT | PARKINGLIGHT_RIGHT |
                          BRAKE_LEFT | BRAKE_RIGHT |
                          LICENSE_PLATE));

        eeprom_write_byte(&obms_SettingsInit, 'O');
    }

    // read settings from EEPROM
    obms_Settings.automaticCentralLock = eeprom_read_byte(&obms_SettingsEEPROM.automaticCentralLock);

    obms_Settings.lockCarUnused = eeprom_read_byte(&obms_SettingsEEPROM.lockCarUnused);

    obms_Settings.comfortTurnLight = eeprom_read_byte(&obms_SettingsEEPROM.comfortTurnLight);

    obms_Settings.leavingHome = eeprom_read_byte(&obms_SettingsEEPROM.leavingHome);
    obms_Settings.leavingHomeLights = eeprom_read_dword(&obms_SettingsEEPROM.leavingHomeLights);
}
