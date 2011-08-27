#include "base.h"
#include "photo_sensor.h"
#include "ibus.h"
#include "display.h"
#include "avrfix.h"
#include "config.h"
#include <avr/eeprom.h>

typedef struct _PhotoSettings
{
    uint8_t photo_minValue;
    uint8_t photo_maxValue;
    uint8_t photo_minCalibValue;
    uint8_t photo_maxCalibValue;
    uint8_t photo_useSensor;
}PhotoSettings;

PhotoSettings photo_Settings;
PhotoSettings photo_SettingsEEPROM EEMEM;
uint8_t       photo_SettingsInit EEMEM;


#define PHOTO_NUM_SAMPLES_EXP 4
#define PHOTO_NUM_SAMPLES (1 << PHOTO_NUM_SAMPLES_EXP)
uint8_t photoSensorValues[PHOTO_NUM_SAMPLES];
uint8_t photoSensorIndex;
uint16_t photoSensorSum;
uint8_t photoSensorLast;
uint8_t photoSensorRawValue;
_aAccum photoSensorRangeFactor;

//------------------------------------------------------------------------------
bool photo_is_enabled(void)
{
    return photo_Settings.photo_useSensor;
}

//------------------------------------------------------------------------------
void photo_enable(bool flag)
{
    photo_Settings.photo_useSensor = flag;
    eeprom_update_byte(&photo_SettingsEEPROM.photo_useSensor, flag);
}

//------------------------------------------------------------------------------
uint8_t photo_get_max_value(void)
{
    return photo_Settings.photo_maxValue;
}

//------------------------------------------------------------------------------
uint8_t photo_get_min_value(void)
{
    return photo_Settings.photo_minValue;
}

//------------------------------------------------------------------------------
void photo_set_max_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_maxValue, val);
    photo_Settings.photo_maxValue = val;
}

//------------------------------------------------------------------------------
void photo_set_min_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, val);
    photo_Settings.photo_minValue = val;
}

//------------------------------------------------------------------------------
void photo_set_max_calib_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, val);
    photo_Settings.photo_maxCalibValue = val;
}

//------------------------------------------------------------------------------
void photo_set_min_calib_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, val);
    photo_Settings.photo_minCalibValue = val;
}

//------------------------------------------------------------------------------
void photo_set_adc_raw_value(uint8_t adcval)
{
    photoSensorRawValue = adcval;
}

//------------------------------------------------------------------------------
uint8_t photo_get_adc_raw_value(void)
{
    return photoSensorRawValue;
}

//------------------------------------------------------------------------------
void photo_setup_calibration()//uint8_t minCalib, uint8_t maxCalib, uint8_t minRange, uint8_t maxRange)
{
    uint8_t minCalib = photo_Settings.photo_minCalibValue;
    uint8_t maxCalib = photo_Settings.photo_maxCalibValue;
    uint8_t minRange = photo_Settings.photo_minValue;
    uint8_t maxRange = photo_Settings.photo_maxValue;

    //uint8_t dataph[6] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, minCalib, maxCalib, minRange, maxRange};
    //ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_BMBT, dataph, 6, 6);

    //photo_Settings.photo_minCalibValue = minCalib;
    //photo_Settings.photo_maxCalibValue = maxCalib;

    eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, minCalib);
    eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, maxCalib);

    //photo_Settings.photo_minValue = minRange;
    //photo_Settings.photo_maxValue = maxRange;

    eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, minRange);
    eeprom_update_byte(&photo_SettingsEEPROM.photo_maxValue, maxRange);

    _aAccum calibMax = itok(maxCalib);
    _aAccum calibMin = itok(minCalib);

    _aAccum photoMax = itok(maxRange);
    _aAccum photoMin = itok(minRange);

    photoSensorRangeFactor = divk(photoMax - photoMin, calibMax - calibMin);
}

//------------------------------------------------------------------------------
uint8_t photo_apply_calibration(uint8_t value)
{
    if (value > photo_Settings.photo_maxCalibValue) value = photo_Settings.photo_maxCalibValue;
    if (value < photo_Settings.photo_minCalibValue) value = photo_Settings.photo_minCalibValue;

    _aAccum v = itok(value - photo_Settings.photo_minCalibValue);
    _aAccum r = mulk(v, photoSensorRangeFactor);

    return ktoi(r) + photo_Settings.photo_minValue;
}

//------------------------------------------------------------------------------
// Get averaged photo sensor value
//------------------------------------------------------------------------------
uint8_t photo_get_value(void)
{
    return photoSensorLast;
}

//------------------------------------------------------------------------------
// Update average photo sensor values (average filter)
//------------------------------------------------------------------------------
uint8_t updatePhotoSensor(uint8_t val)
{
    val = photo_apply_calibration(val);

    // remove last element of the ring buffer and add new value to the sum
    photoSensorSum -= photoSensorValues[(photoSensorIndex + 1) & (PHOTO_NUM_SAMPLES-1)];
    photoSensorSum += val;

    // add new value into the ring buffer
    photoSensorValues[photoSensorIndex++] = val;
    photoSensorIndex = photoSensorIndex & (PHOTO_NUM_SAMPLES-1);

    return (photoSensorSum >> PHOTO_NUM_SAMPLES_EXP);
}

//------------------------------------------------------------------------------
void photo_init(void)
{
    // if run for the first time, then write default data to eeprom
    if (eeprom_read_byte(&photo_SettingsInit) != 'P')
    {
        eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, 0x28);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxValue, 0xFF);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, 0x3A);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, 0xFF);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_useSensor, 0xFF);

        eeprom_write_byte(&photo_SettingsInit, 'P');
    }

    photo_Settings.photo_minValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_minValue);
    photo_Settings.photo_maxValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_maxValue);
    photo_Settings.photo_minCalibValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_minCalibValue);
    photo_Settings.photo_maxCalibValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_maxCalibValue);
    photo_Settings.photo_useSensor = eeprom_read_byte(&photo_SettingsEEPROM.photo_useSensor);

    photo_setup_calibration();

    photoSensorIndex = 0;
    photoSensorSum = (uint16_t)photo_Settings.photo_maxValue << (uint16_t)PHOTO_NUM_SAMPLES_EXP;
    photoSensorLast = photo_Settings.photo_maxValue;
    memset(&photoSensorValues[0], photo_Settings.photo_maxValue, PHOTO_NUM_SAMPLES);
}

//------------------------------------------------------------------------------
void photo_tick(void)
{
    // update photo sensor if its low-pass filtered value has changed
    register uint8_t photoVal = updatePhotoSensor(photoSensorRawValue);

    if (photo_Settings.photo_useSensor && photoVal != photoSensorLast)
    {
        photoSensorLast = photoVal;
    }
}

//------------------------------------------------------------------------------
void photo_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if (dst == IBUS_DEV_BMBT && msglen >= 2 && msg[0] == IBUS_MSG_OPENBM_TO && msg[1] == IBUS_MSG_OPENBM_GET_PHOTO)
    {
        uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, photoSensorSum >> PHOTO_NUM_SAMPLES_EXP};
        ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);
    }
}
