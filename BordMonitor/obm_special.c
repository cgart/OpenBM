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
#include "include/photo_sensor.h"
#include "include/obm_special.h"
#include "include/leds.h"
#include <avr/eeprom.h>


typedef uint32_t lightState_t;


typedef struct _OBMSSettings
{
    uint8_t  automaticCentralLock;

    uint8_t  lockCarUnused;

    uint8_t  comfortTurnLight;

    uint8_t  comingHome;
    uint8_t  leavingHome;
    uint32_t leavingHomeLights;

    uint8_t emulateCDchanger;
    uint8_t emulateBordmonitor;
    uint8_t emulateMID;
    
    uint8_t  automaticMirrorFold;
    uint8_t  mirrorFoldDelay;
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

#define AUT_MIRROR_FOLD    (1 << 0)
#define AUT_MIRROR_UNFOLD  (1 << 1)
#define AUT_MIRROR_FLEFT   (1 << 2)
#define AUT_MIRROR_FRIGHT  (1 << 3)
#define AUT_MIRROR_ULEFT   (1 << 4)
#define AUT_MIRROR_URIGHT  (1 << 5)
#define AUT_MIRROR_RESTORE (1 << 6)
#define AUT_MIRROR_IGNORE  (1 << 7)
static int8_t  _nextMirrorFold = 0;
static ticks_t _nextMirrorFoldTick = 0;
static ticks_t _comfortCloseTimer = 0;

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

#define DIMMER_BACKGROUND        (1L << 25)

#define LIGHT_STATE_CHANGED      (1L << 31)

static lightState_t _lightState = IDLE;
//lightState_t _oldLightState = IDLE;

// IKE state
static bool _parkingBrake = false;
static uint8_t _carHalfSpeed = 0;
static uint8_t _carRPM = 0;

// central lock state
static ticks_t _centralLockWait = 0;
static ticks_t _reqGMState = 0;
static int8_t _centralLocked = -1;
static uint8_t _catchedKeyReleased = 0;

// ignition and EWS state
static int8_t _ignitionState = -1;
static int8_t _ewsState = -1;
static ticks_t _reqIgnitionState = 0;

typedef enum _LeavingHome
{
    NO = 0,
    START = 1,
    HOLD = 2,
    STOP = 3,
    DISABLED = 4,
    GOODBYE = 5
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

typedef enum _CDChangerState
{
    CDC_INIT = 0,
    CDC_IDLE = 1,
    CDC_RESPONSE = 2,
    CDC_SEND_START_PLAYING = 3,
    CDC_SEND_STATE_NOTPLAYING = 4
}CDChangerState;
static CDChangerState _cdcState = CDC_INIT;

#define BMBT             (1 << 0)
#define BMBT_LCD_OFF     (1 << 1)
#define BMBT_LCD_ON      (1 << 2)
#define BMBT_LCD_GT_1    (1 << 3)
#define BMBT_LCD_GT_2    (1 << 4)
#define BMBT_LCD_GT(a)   ((a & 0b00011000) >> 3)
#define BMBT_LCD_TV_1    (1 << 5)
#define BMBT_LCD_TV_2    (1 << 6)
#define BMBT_LCD_TV(a)   ((a & 0b01100000) >> 5)
#define BMBT_DIFF_KEYS   (1 << 7)

#define BMBT_INPUT_GT  2
#define BMBT_INPUT_TV  1

//------------------------------------------------------------------------------
uint8_t obms_does_emulate_bordmonitor(void)
{
    return (obms_Settings.emulateBordmonitor & BMBT);
}

//------------------------------------------------------------------------------
uint8_t obms_does_emulate_mid(void)
{
    return (obms_Settings.emulateMID) != 0;
}

//------------------------------------------------------------------------------
uint8_t obms_does_send_different_buttoncodes(void)
{
    return (obms_Settings.emulateBordmonitor & BMBT_DIFF_KEYS);    
}

//------------------------------------------------------------------------------
int8_t obms_ignition_state(void)
{
    return _ignitionState;
}

//------------------------------------------------------------------------------
void obms_set_mirror_fold(uint8_t fold, uint8_t which)
{
    //Fahrerspiegel ein
    //3F 05 00 0C 01 31 06

    //Beifahrerspiegel ein
    //3F 05 00 0C 02 31 05

    //Fahrerspiegel aus
    //3F 05 00 0C 01 30 07

    //Beifahrerspiegel aus
    //3F 05 00 0C 02 30 04

    if (which & 1)
    {
        uint8_t data[3] = {IBUS_MSG_VEHICLE_CTRL, 0x01, 0x30 | fold};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);
    }

    if (which & 2)
    {
        uint8_t data[3] = {IBUS_MSG_VEHICLE_CTRL, 0x02, 0x30 | fold};
        ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);
    }
}

//------------------------------------------------------------------------------
void obms_toggle_central_lock(void)
{
    uint8_t data[3] = {IBUS_MSG_VEHICLE_CTRL, 0x00, 0x0B};
    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_GM, data, 3, IBUS_TRANSMIT_TRIES);

    _centralLockWait = tick_get() + TICKS_PER_X_SECONDS(10);
    _centralLocked = -1; // make the current state unknown, so that we can check if GM accepted our message
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

    if (_lightState & DIMMER_BACKGROUND) data[9] |= 0x02;

    ibus_sendMessage(IBUS_DEV_DIA, IBUS_DEV_LCM, data, 13, IBUS_TRANSMIT_TRIES);

    _lightState &= ~LIGHT_STATE_CHANGED;
}

//------------------------------------------------------------------------------
void obms_lights_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    int8_t doorOpened = -1;
    int8_t kl50 = -1;
    bool brightEnough = false;

    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO && msg[0] == IBUS_MSG_GM_STATE && msglen >= 2)
    {
        // central unlocked   00 05 BF 7A [dd1=10] [dd2=00] xx // dd1 = bit8 - trunk, bit7 = kl50, bit6=locked, bit5=?, bit4-bit1=door, dd2 = bit6 - trunk opened, bit4-1 windows
        // central locked     00 05 BF 7A 20 00 xx
        if (msg[1] & 0x20)
            _centralLocked = 1;
        else
            _centralLocked = 0;
    }

    if (photo_use_state() != 0)
        brightEnough = photo_is_bright_enough();

    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO && msglen >= 2)
    {
        // turn leaving home if car was closed with the key
        if (msg[0] == IBUS_MSG_GM_KEY_BUTTON && (msg[1] & 0x10))
        {
            _lockCarIfNoDoorOpened = 0;
            _leavingHome = NO;
            _lockCarIfNoDoorOpenedActive = false;
            kl50 = 2;
        }

        // door opened and/or kl50 activated
        if (msg[0] == IBUS_MSG_GM_STATE)
        {
            if (msg[1] & 0x0F) doorOpened = 1;
            if (msg[1] & 0x40) kl50 = 1;
        }
    }
    
    if (doorOpened == 1 || kl50 >= 1)
    {
        if (doorOpened == 1)
        {
            if (_leavingHome == HOLD)
                _leavingHome = STOP;

        }else if (/*_centralLocked != -1 &&*/ _leavingHome == NO && !brightEnough)
        {
            if (_centralLocked == 0 && kl50 == 1) // greeting light
                _leavingHome = START;
            else if (/*_centralLocked == -1 &&*/ kl50 == 2) // goodbye light
                _leavingHome = GOODBYE;
        }
    }

    // Status Message from LCM, response to 3F,D0,0B
    // D0 23 3F A0 C1 C0 00 00 00 00 00 00 00 9B 00 00 80 00 00 59 33 00 08 00 00 00 00 00 00 00 00 00 00 FF FF FE EA
    if (src == IBUS_DEV_LCM && dst == IBUS_DEV_DIA && msglen > 20 && msg[0] == IBUS_MSG_DIA_ACK && msg[1] == 0xC1)
    {
        _dimmerState = msg[16];
        _lwrState = msg[17];
    }

    // if BMBT emulation is active and we want to use BMBT LCD messages
    if (dst == IBUS_DEV_BMBT && msg[0] == IBUS_MSG_BMBT_DISP_SET)
    {
        uint8_t on = (msg[1] & 0x10) == 0x10;
        if (!on && (obms_Settings.emulateBordmonitor & BMBT_LCD_OFF))
            display_turnOff();
        if (on && (obms_Settings.emulateBordmonitor & BMBT_LCD_ON))
            display_tryTurnOn();
        
        // switch to this input
        uint8_t input = msg[1] & 0x03;
        if (input != 0 && on)
        {
            uint8_t gtInput = BMBT_LCD_GT(obms_Settings.emulateBordmonitor);
            uint8_t tvInput = BMBT_LCD_TV(obms_Settings.emulateBordmonitor);
            if (input == BMBT_INPUT_GT && gtInput)
                display_setInputState(gtInput - 1);
            else if (input == BMBT_INPUT_TV && tvInput)
                display_setInputState(tvInput - 1);
        }
    }
}

//------------------------------------------------------------------------------
void obms_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    obms_lights_on_bus_msg(src, dst, msg, msglen);

    if (src == IBUS_DEV_GM && dst == IBUS_DEV_GLO)
    {
        // car was closed with the key
        if (msg[0] == IBUS_MSG_GM_KEY_BUTTON && msglen >= 2)
        {
            if (msg[1] & 0x10)
                _comfortCloseTimer = tick_get() + TICKS_PER_TWO_SECONDS;

            _catchedKeyReleased = (msg[1] & 0xF0) == 0;

            if (_catchedKeyReleased)
                _comfortCloseTimer = 0;
        }

        if (msg[0] == IBUS_MSG_GM_STATE && msglen >= 2)
        {
            _reqGMState = 0;

            if ((msg[1] & 0xF0) && _lockCarIfNoDoorOpenedActive)
                _lockCarIfNoDoorOpened = tick_get() + TICKS_PER_X_SECONDS(obms_Settings.lockCarUnused);

            // just any door was opened
            if (msg[1] & 0x0F)
            {
                // stop lock door counter, if this one is running
                _lockCarIfNoDoorOpened = 0;
                _lockCarIfNoDoorOpenedActive = false;
            }
        }
    }

    // EWS state
    if (src == IBUS_DEV_EWS && dst == IBUS_DEV_GLO && msglen >= 3 && msg[0] == IBUS_MSG_EWS_STATE)
    {
        _ewsState = msg[1];
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

        if (msglen >= 2 && msg[0] == IBUS_MSG_IGNITION)
        {
            _ignitionState = msg[1];
            _reqIgnitionState = 0;
        }

        if (msglen >= 7 && msg[0] == IBUS_MSG_IKE_STATE)
        {
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
                }else if (display_getInputState() == BACKCAM_INPUT() || (_cam_state & CAM_ON) != 0 )
                    _cam_state = CAM_PREPARE_TO_SWITCH;
            }
        }
    }

    // on valid ews state or valid ignition we stop the leaving home and stop counting door closer
    if (_ewsState > 1 || _ignitionState > 0)
    {
        _lockCarIfNoDoorOpened = 0;
        _lockCarIfNoDoorOpenedActive = false;
        if (_leavingHome == HOLD)
            _leavingHome = STOP;
        else
            _leavingHome = DISABLED;
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
            //if ((_tippBlinken & 0x0F) == 0 &&
            if (obms_Settings.comfortTurnLight > 0)
            {
                // turn flash light
                if (msg[3] == 0x04)
                {
                    // this counter will be passed to the new
                    uint8_t counter = (_tippBlinken & 0x0F);

                    // activated left
                    if (msg[1] & 0x20)
                    {
                        // if we have had right flash before, then reset the counter
                        if (_tippBlinken & 0x40) _tippBlinkenCounter = 0;
                        _tippBlinken = 0x80 | counter;

                    // activated right
                    }else if (msg[1] & 0x40)
                    {
                        // if we have had left flash before, then reset the counter
                        if (_tippBlinken & 0x80) _tippBlinkenCounter = 0;
                        _tippBlinken = 0x40 | counter;
                    }

                    // increase counter only if not running yet
                    if ((_tippBlinken & 0x0F) == 0)
                        _tippBlinkenCounter++;

                // if turn flash light is turned off
                }else if ((_tippBlinken & 0x0F) == 0 && msg[3] == 0)
                {
                    // activate tipp blinken for at most X flashes
                    if (_tippBlinkenCounter > 0 && _tippBlinkenCounter < obms_Settings.comfortTurnLight)
                        _tippBlinken |= ((obms_Settings.comfortTurnLight - _tippBlinkenCounter) & 0x0F);
                    _tippBlinkenCounter = 0;
                }
            }
        }
    }


    // Setup settings if asked so over IBus
    if (dst == IBUS_DEV_BMBT  && msglen >= 3 && msg[0] == IBUS_MSG_OPENBM_TO && msg[1] == IBUS_MSG_OPENBM_OBMS_SET)
    {
        // every such message will be responded, default response is 0
        uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_OBMS_SET, msg[2]};
        uint8_t ok = 0;
        
        if (msglen == 4 && msg[2] == 0x01)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.automaticCentralLock, msg[3] >> 1);
            obms_Settings.automaticCentralLock = msg[3] >> 1;
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x02)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.lockCarUnused, msg[3]);
            obms_Settings.lockCarUnused = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x03)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.comfortTurnLight, msg[3]);
            obms_Settings.comfortTurnLight = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x04)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.leavingHome, msg[3]);
            obms_Settings.leavingHome = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x05)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.comingHome, msg[3]);
            obms_Settings.comingHome = msg[3];
            ok = 1;
        }else if (msglen == 7 && msg[2] == 0x06)
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
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x07)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.emulateCDchanger, msg[3]);
            obms_Settings.emulateCDchanger = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x08)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.automaticMirrorFold, msg[3]);
            obms_Settings.automaticMirrorFold = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x09)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.emulateBordmonitor, msg[3]);
            obms_Settings.emulateBordmonitor = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x0A)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.mirrorFoldDelay, msg[3]);
            obms_Settings.mirrorFoldDelay = msg[3];
            ok = 1;
        }else if (msglen == 4 && msg[2] == 0x0B)
        {
            eeprom_update_byte(&obms_SettingsEEPROM.emulateMID, msg[3]);
            obms_Settings.emulateMID = msg[3];
            ok = 1;
        }else
        {
            data[2] = 0;
        }
        if (ok)
          ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);
    }

    // Emulate cd-changer
    if (obms_Settings.emulateCDchanger && dst == IBUS_DEV_CDC)
    {
        if (msg[0] == IBUS_MSG_DEV_POLL)
            _cdcState = CDC_RESPONSE;
        else if (msg[0] == IBUS_MSG_RADIO_CDC && msglen >= 2)
        {
            _cdcState = CDC_SEND_START_PLAYING;
            if (msg[1] == 0x01)
                _cdcState = CDC_SEND_STATE_NOTPLAYING;
        }
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
            if (HAS_BACKCAM_SWITCH && BACKCAM_INPUT() == 0x02)
            {
                static uint8_t oldState = 0;
                _cam_state = CAM_SWITCHING;
                if ((state & CAM_ON) == CAM_ON)
                {
                    oldState = display_getInputState();

                    display_enableBackupCameraInput(1);
                    display_updateInputState(BACKCAM_INPUT());
                }else
                {
                    display_enableBackupCameraInput(0);
                    display_updateInputState(oldState);
                }
                _cam_state = CAM_IDLE;
            }else
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
            }

            ticks = 0;
        }
    }else
        ticks = 0;
}

//------------------------------------------------------------------------------
void obms_lights_tick(void)
{
    // request state of the dimmer and lwr if unknown
    if (_dimmerState == 0 && _lwrState == 0)
    {
        obms_req_light_state();

        // just set to some state, so that we don't resend this message before recieved an answer
        _dimmerState = 0XFF;
        _lwrState = 0X33;
    }

    // Handle leaving home support if such is activated
    if (obms_Settings.leavingHome || obms_Settings.comingHome)
    {
        switch(_leavingHome)
        {
            // start leaving home, this is done by activating predefined lights
            case GOODBYE:
            case START:
            {
                if (_leavingHome == START)
                {
                    if (obms_Settings.leavingHome == 0)
                    {
                        _leavingHome = DISABLED;
                        break;
                    }
                    _leavingHomeStopAfter = tick_get() + TICKS_PER_X_SECONDS(obms_Settings.leavingHome);
                }else
                {
                    if (obms_Settings.comingHome == 0)
                    {
                        _leavingHome = DISABLED;
                        break;
                    }
                    _leavingHomeStopAfter = tick_get() + TICKS_PER_X_SECONDS(obms_Settings.comingHome);
                }

                _leavingHome = HOLD;
                _lightState = obms_Settings.leavingHomeLights;
                _lightState |= LIGHT_STATE_CHANGED;
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

    // set state of light if changed
    obms_set_light_state();

    // Mirror should fold and folding is activated
    if ((_nextMirrorFold & (AUT_MIRROR_FLEFT | AUT_MIRROR_FRIGHT)) && (obms_Settings.automaticMirrorFold & AUT_MIRROR_FOLD))
    {
        // if there is a timer active, then only fold after that timer passes
        if (!_nextMirrorFoldTick || (_nextMirrorFoldTick && _nextMirrorFoldTick < tick_get()))
        {
            obms_set_mirror_fold(1, (_nextMirrorFold & AUT_MIRROR_FLEFT) ? 1 : 2);

            _nextMirrorFoldTick = 0;
            _nextMirrorFold &= ~AUT_MIRROR_FLEFT;
            _nextMirrorFold &= ~AUT_MIRROR_FRIGHT;
        }
    }
    if ((_nextMirrorFold & (AUT_MIRROR_ULEFT | AUT_MIRROR_URIGHT)) && (obms_Settings.automaticMirrorFold & AUT_MIRROR_UNFOLD))
    {
        if (!_nextMirrorFoldTick || (_nextMirrorFoldTick && _nextMirrorFoldTick < tick_get()))
        {
            obms_set_mirror_fold(0, (_nextMirrorFold & AUT_MIRROR_ULEFT) ? 1 : 2);

            _nextMirrorFoldTick = 0;
            _nextMirrorFold &= ~AUT_MIRROR_ULEFT;
            _nextMirrorFold &= ~AUT_MIRROR_URIGHT;
        }
    }
}

//------------------------------------------------------------------------------
void obms_comfort_close(void)
{
    // full shutdown of the system if pressed longer than 7 = (5 + 2) seconds
    if (_comfortCloseTimer > 0 && tick_get() > _comfortCloseTimer + TICKS_PER_X_SECONDS(5))
    {
        led_radioBlinkLock(3);
        power_prepare_shutdown();
    }

    if (_comfortCloseTimer > 0 && tick_get() > _comfortCloseTimer && _nextMirrorFold == 0)
    {
        // set this to -2, so that we do not send close message again, if we are waiting longer for exampel to shutdown
        _nextMirrorFold = AUT_MIRROR_IGNORE;

        // ok if we have pressed the close key two times, then we can fold the mirror
        {
            if (_centralLocked == 1 &&
                (obms_Settings.automaticMirrorFold & AUT_MIRROR_FOLD) != 0)
            {
                eeprom_update_byte(&obms_SettingsEEPROM.automaticMirrorFold, obms_Settings.automaticMirrorFold | AUT_MIRROR_RESTORE);
                obms_Settings.automaticMirrorFold |= AUT_MIRROR_RESTORE;

                // fold left mirror and set timer to be executed to fold the right mirror
                obms_set_mirror_fold(1, 1);

                _nextMirrorFoldTick = tick_get() + obms_Settings.mirrorFoldDelay;
                _nextMirrorFold |= AUT_MIRROR_FRIGHT;
            }
        }
    }
}

//------------------------------------------------------------------------------
void obms_tick(void)
{
  
    obms_backcam_tick();

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
                    _tippBlinken |= ((tippCounter - 1) & 0x0F);
                }
                _lightState |= LIGHT_STATE_CHANGED;
                flash = !flash;
            }
            if (nextFlash == 0)
                nextFlash = tick_get() + 9;
        }
    }

    obms_lights_tick();

    // if Central Lock feature is enabled
    if (obms_Settings.automaticCentralLock && tick_get() > _centralLockWait)
    {
        // if ignition is switched off (or key removed) and central is locked, then unlock it
        if ((_centralLocked == 1 && ((_ignitionState > 0 && _ignitionState < 3) /*|| _ewsState > 0*/))// && _carHalfSpeed < obms_Settings.automaticCentralLock)

        // if the central is locked and hand brake was activated, then open the door
        // open also the door if locked and P-gear was set
          || (_centralLocked == 1 && _ignitionState > 0 && _carHalfSpeed < (obms_Settings.automaticCentralLock << 1) && (_parkingBrake || _selectedGear == PARK))

        // if unlocked and car speed is above certain limit then lock the car
          || (_centralLocked == 0 && _ignitionState > 0 && _carHalfSpeed > obms_Settings.automaticCentralLock))
        {
            obms_toggle_central_lock();
        }
    }

    // if we are allowed to request the status of a device, then do so
    if (_ignitionState > -10 && _ignitionState < 0 && _reqIgnitionState > 0 && tick_get() > _reqIgnitionState)
    {
        _ignitionState--;

        uint8_t data[2] = {IBUS_MSG_IKE_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_BMBT/*DIA*/, IBUS_DEV_IKE, data, 1 /*2*/, IBUS_TRANSMIT_TRIES);

        data[0] = IBUS_MSG_IGNITION_REQ;
        ibus_sendMessage(IBUS_DEV_BMBT /*DIA*/, IBUS_DEV_IKE, data, 1 /*2*/, IBUS_TRANSMIT_TRIES);

        _reqIgnitionState = tick_get() + TICKS_PER_X_SECONDS(2);
    }

    // if central state is unknown, then it means we should request the state from GM to get it
    if (_centralLocked > -10 && _centralLocked < 0 && _reqGMState > 0 && tick_get() > _reqGMState)
    {
        _centralLocked--;

        uint8_t data[2] = {IBUS_MSG_GM_STATE_REQ, 0x00};
        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_GM, data, 1 /*2*/, IBUS_TRANSMIT_TRIES);

        _reqGMState = tick_get() + TICKS_PER_X_SECONDS(2);
    }

    // if we have a timer activated to close the central lock, then do so
    if (obms_Settings.lockCarUnused)
    {
        if (_lockCarIfNoDoorOpenedActive)
        {
            //led_red_immediate_set(_lockCarIfNoDoorOpened > 0 && tick_get() <= _lockCarIfNoDoorOpened);
            if (_lockCarIfNoDoorOpened > 0 && tick_get() > _lockCarIfNoDoorOpened)
            {
                _lockCarIfNoDoorOpened = 0;
                _lockCarIfNoDoorOpenedActive = false;
                if (_ignitionState == 0)
                {
                    power_prepare_shutdown();
                    if (_centralLocked == 0 && _catchedKeyReleased == 1)
                        obms_toggle_central_lock();
                }
            }
        }
    }

    // emulate CD-changer
    if (obms_Settings.emulateCDchanger && _cdcState != CDC_INIT && _ignitionState > 0)
    {
        switch (_cdcState)
        {
            case CDC_INIT:
            {
                uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x01};
                ibus_sendMessage(IBUS_DEV_CDC, IBUS_DEV_LOC, data, 2, IBUS_TRANSMIT_TRIES);
            }
            case CDC_RESPONSE:
            {
                uint8_t data[2] = {IBUS_MSG_DEV_READY, 0x00};
                ibus_sendMessage(IBUS_DEV_CDC, IBUS_DEV_LOC, data, 2, IBUS_TRANSMIT_TRIES);
                break;
            }
            case CDC_SEND_STATE_NOTPLAYING:
            {
                uint8_t data[8] = {IBUS_MSG_CDC_STATE, 0x00, 0x02, 0x00, 0x3F, 0x00, 0x06, 0x99};
                ibus_sendMessage(IBUS_DEV_CDC, IBUS_DEV_RAD, data, 8, IBUS_TRANSMIT_TRIES);
                break;
            }
            case CDC_SEND_START_PLAYING:
            {
                uint8_t data[8] = {IBUS_MSG_CDC_STATE, 0x02, 0x09, 0x00, 0x20, 0x00, 0x06, 0x99};
                ibus_sendMessage(IBUS_DEV_CDC, IBUS_DEV_RAD, data, 8, IBUS_TRANSMIT_TRIES);
                break;
            }
            default:
                break;
        }
        _cdcState = CDC_INIT;
    }

    // if unfolding is enabled, then unfold the mirrors if they were previously folded
    // and we detected unlocked car as also catched the key up
    if (_centralLocked == 0 &&
        _catchedKeyReleased &&
        (obms_Settings.automaticMirrorFold & (AUT_MIRROR_UNFOLD | AUT_MIRROR_RESTORE)) == (AUT_MIRROR_UNFOLD | AUT_MIRROR_RESTORE))
    {
        obms_Settings.automaticMirrorFold &= ~(AUT_MIRROR_RESTORE);
        eeprom_update_byte(&obms_SettingsEEPROM.automaticMirrorFold, obms_Settings.automaticMirrorFold);
        obms_set_mirror_fold(0, 1);
        _nextMirrorFold = AUT_MIRROR_URIGHT;
        _nextMirrorFoldTick = tick_get() + obms_Settings.mirrorFoldDelay;
    }

    obms_comfort_close();
}


//------------------------------------------------------------------------------
void obms_stop(void)
{
    obms_comfort_close();
    //_lockCarIfNoDoorOpened = 0;
    //_leavingHome = NO;
    //_lockCarIfNoDoorOpenedActive = false;
}

//------------------------------------------------------------------------------
void obms_resume(void)
{
    obms_init();
}

//------------------------------------------------------------------------------
void obms_init(void)
{
    _cdcState = CDC_INIT;
    _centralLockWait = 0;
    _centralLocked = -1;
    _catchedKeyReleased = 0;
    _parkingBrake = 0;
    _carHalfSpeed = 0;
    _carRPM = 0;
    _cam_state = CAM_IDLE;
    //_cam_oldstate = display_getInputState();
    //_reqStatus = DEV_GM;
    _leavingHome = NO;
    _tippBlinken = 0;
    _tippBlinkenCounter = 0;
    _dimmerState = 0;
    _lwrState = 0;
    _lockCarIfNoDoorOpened = 0;
    _lockCarIfNoDoorOpenedActive = true;
    _ignitionState = -1;
    _reqIgnitionState = 1;
    _reqGMState = 1;
    _nextMirrorFold = 0;
    _comfortCloseTimer = 0;

    // if run for the first time, then write default data to eeprom
    if (eeprom_read_byte(&obms_SettingsInit) != EE_CHECK_BYTE)
    {
        eeprom_write_byte(&obms_SettingsInit, EE_CHECK_BYTE);

        eeprom_update_byte(&obms_SettingsEEPROM.automaticCentralLock, (DEVICE_CODING3 & OBMS_AUT_CENTRALLOCK) == OBMS_AUT_CENTRALLOCK ? 5 : 0);
        eeprom_update_byte(&obms_SettingsEEPROM.automaticMirrorFold, AUT_MIRROR_FOLD | AUT_MIRROR_UNFOLD);
        eeprom_update_byte(&obms_SettingsEEPROM.mirrorFoldDelay, 4);
        eeprom_update_byte(&obms_SettingsEEPROM.lockCarUnused, 160);

        eeprom_update_byte(&obms_SettingsEEPROM.comfortTurnLight, 3);

        eeprom_update_byte(&obms_SettingsEEPROM.leavingHome, 30);
        eeprom_update_byte(&obms_SettingsEEPROM.comingHome, 10);
        eeprom_update_dword(&obms_SettingsEEPROM.leavingHomeLights,
                         (BACKLIGHT_LEFT | BACKLIGHT_RIGHT |
                          FOGLIGHT_FRONT_LEFT | FOGLIGHT_FRONT_RIGHT |
                          PARKINGLIGHT_LEFT | PARKINGLIGHT_RIGHT |
                          //REARGEAR_LEFT | REARGEAR_RIGHT |
                          //BRAKE_LEFT | BRAKE_RIGHT |
                          DIMMER_BACKGROUND |
                          LICENSE_PLATE));

        eeprom_update_byte(&obms_SettingsEEPROM.emulateCDchanger, (DEVICE_CODING2 & EMULATE_CDCHANGER) == EMULATE_CDCHANGER);
        eeprom_update_byte(&obms_SettingsEEPROM.emulateMID, (DEVICE_CODING2 & EMULATE_MID) == EMULATE_MID);
        
        if ((DEVICE_CODING2 & EMULATE_BORDMONITOR) == EMULATE_BORDMONITOR)
            eeprom_update_byte(&obms_SettingsEEPROM.emulateBordmonitor, BMBT | BMBT_LCD_OFF | BMBT_LCD_ON /*| BMBT_LCD_GT_2 | BMBT_LCD_TV_1 | BMBT_LCD_TV_2*/ | BMBT_DIFF_KEYS);
        else
            eeprom_update_byte(&obms_SettingsEEPROM.emulateBordmonitor, 0);
        
    }

    // read settings from EEPROM
    obms_Settings.automaticCentralLock = eeprom_read_byte(&obms_SettingsEEPROM.automaticCentralLock);
    obms_Settings.automaticMirrorFold = eeprom_read_byte(&obms_SettingsEEPROM.automaticMirrorFold);
    obms_Settings.mirrorFoldDelay = eeprom_read_byte(&obms_SettingsEEPROM.mirrorFoldDelay);

    obms_Settings.lockCarUnused = eeprom_read_byte(&obms_SettingsEEPROM.lockCarUnused);

    obms_Settings.comfortTurnLight = eeprom_read_byte(&obms_SettingsEEPROM.comfortTurnLight);

    obms_Settings.comingHome = eeprom_read_byte(&obms_SettingsEEPROM.comingHome);
    obms_Settings.leavingHome = eeprom_read_byte(&obms_SettingsEEPROM.leavingHome);
    obms_Settings.leavingHomeLights = eeprom_read_dword(&obms_SettingsEEPROM.leavingHomeLights);

    obms_Settings.emulateCDchanger = eeprom_read_byte(&obms_SettingsEEPROM.emulateCDchanger);
    obms_Settings.emulateBordmonitor = eeprom_read_byte(&obms_SettingsEEPROM.emulateBordmonitor);
    obms_Settings.emulateMID = eeprom_read_byte(&obms_SettingsEEPROM.emulateMID);
}
