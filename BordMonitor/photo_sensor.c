/*
* Copyright 2010-2013 Art Tevs <art@tevs.eu>
* This file is part of OpenBM (firmware).
*
* OpenBM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenBM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/


#include "base.h"
#include "photo_sensor.h"
#include "ibus.h"
#include "buttons.h"
#include "config.h"
#include "include/photo_sensor.h"
#include "include/config.h"
#include "include/leds.h"
#include <avr/eeprom.h>
#include <math.h>


typedef struct _PhotoSettings
{
    uint8_t photo_minValue;
    uint8_t photo_maxValue;
    uint8_t photo_minCalibValue;
    uint8_t photo_maxCalibValue;
    uint8_t photo_useSensor;
    uint8_t photo_calibChanged;
    uint8_t photo_sensorLast;
    float   photo_Gamma;
}PhotoSettings;

PhotoSettings photo_Settings;
PhotoSettings photo_SettingsEEPROM EEMEM;
uint8_t       photo_SettingsInit EEMEM;


#define PHOTO_NUM_SAMPLES_EXP 6
#define PHOTO_NUM_SAMPLES (1 << PHOTO_NUM_SAMPLES_EXP)
uint8_t photoSensorValues[PHOTO_NUM_SAMPLES];
uint8_t photoSensorIndex;
uint16_t photoSensorSum;
uint8_t photoSensorLast;
uint8_t photoSensorRawValue;
//_aAccum photoSensorRangeFactor;

static uint8_t areLightsOn = 0;

#define CACHE_RANGE 64
#define CACHE_FACTOR 6

//------------------------------------------------------------------------------
uint8_t photo_calibTableEEPROM[256] EEMEM;
uint8_t photo_calibCache[CACHE_RANGE];
uint8_t photo_calibCacheLine = 0xFF;

//------------------------------------------------------------------------------
uint8_t photo_get_cached(uint8_t value)
{
    // instead of always recomputing the current value into the calibration range
    // we cache the calibration values and just load the calibrated photo 
    // sensor value directly from the cache
    uint8_t cacheLine = value >> CACHE_FACTOR;
    uint8_t cacheStart = cacheLine << CACHE_FACTOR;

    // check if cache isn't loaded yet, then preload it
    if (photo_calibCacheLine != cacheLine)
    {
        photo_calibCacheLine = cacheLine;

        eeprom_read_block(photo_calibCache, &photo_calibTableEEPROM[cacheStart], CACHE_RANGE);
    }

    // the value we look for must be in the loaded cache line
    return photo_calibCache[value - cacheStart];
}


//------------------------------------------------------------------------------
void photo_setup_calibration()
{
    if (photo_Settings.photo_calibChanged)
    {
        uint8_t minCalib = photo_Settings.photo_minCalibValue;
        uint8_t maxCalib = photo_Settings.photo_maxCalibValue;
        uint8_t minRange = photo_Settings.photo_minValue;
        uint8_t maxRange = photo_Settings.photo_maxValue;

        eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, minCalib);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, maxCalib);

        eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, minRange);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxValue, maxRange);

        // setup look-up table
        for (uint8_t i=0 ; i <= minCalib; i++)
            eeprom_update_byte(&photo_calibTableEEPROM[i], minRange);

        for (uint8_t i=minCalib+1; i < maxCalib; i++)
        {
            float val = (float)(i - minCalib);
            val = val / (float)(maxCalib - minCalib);
            val = powf(val, photo_Settings.photo_Gamma);
            uint8_t res = (uint8_t)(val * (float)(maxRange - minRange)) + minRange;
            eeprom_update_byte(&photo_calibTableEEPROM[i], res);
        }

        for (uint8_t i=maxCalib ; i < 255; i++)
            eeprom_update_byte(&photo_calibTableEEPROM[i], maxRange);

        // preload look-up table into cache
        photo_get_cached(photoSensorRawValue);

        photo_Settings.photo_calibChanged = 0;
        eeprom_update_byte(&photo_SettingsEEPROM.photo_calibChanged, 0);
    }
}

//------------------------------------------------------------------------------
uint8_t photo_use_state(void)
{
    return photo_Settings.photo_useSensor;
}

//------------------------------------------------------------------------------
void photo_enable(uint8_t flag)
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
    photo_Settings.photo_calibChanged = 1;
}

//------------------------------------------------------------------------------
void photo_set_min_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, val);
    photo_Settings.photo_minValue = val;
    photo_Settings.photo_calibChanged = 1;
}

//------------------------------------------------------------------------------
void photo_set_max_calib_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, val);
    photo_Settings.photo_maxCalibValue = val;
    photo_Settings.photo_calibChanged = 1;
}

//------------------------------------------------------------------------------
void photo_set_min_calib_value(uint8_t val)
{
    eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, val);
    photo_Settings.photo_minCalibValue = val;
    photo_Settings.photo_calibChanged = 1;
}

//------------------------------------------------------------------------------
/*void photo_set_adc_raw_value(uint8_t adcval)
{
    photoSensorRawValue = adcval;
}*/

//------------------------------------------------------------------------------
uint8_t photo_get_adc_raw_value(void)
{
    return photoSensorRawValue;
}

//------------------------------------------------------------------------------
bool photo_is_bright_enough(void)
{
    // if the current value of the photo sensor is above the mid-value calibration value then 
    uint8_t hdiff = (photo_Settings.photo_maxCalibValue - photo_Settings.photo_minCalibValue) >> 1;
    return photoSensorRawValue >= (photo_Settings.photo_minCalibValue + hdiff);
}

//------------------------------------------------------------------------------
uint8_t photo_apply_calibration(uint8_t value)
{
    if (photo_Settings.photo_calibChanged)
        photo_setup_calibration();

    return photo_get_cached(value);
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
    if (eeprom_read_byte(&photo_SettingsInit) != EE_CHECK_BYTE)
    {
        eeprom_write_byte(&photo_SettingsInit, EE_CHECK_BYTE);
        
        eeprom_update_byte(&photo_SettingsEEPROM.photo_minValue, 0x35); // 0x90 if 3.5kHz for duty cycles
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxValue, 0xFF);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_minCalibValue, 0x06);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_maxCalibValue, 0x13);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_useSensor, USE_PHOTOSENSOR() ? 2  : 0);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_calibChanged, 1);
        eeprom_update_byte(&photo_SettingsEEPROM.photo_sensorLast, 0xFF);

        float gamma = 0.35f;
        eeprom_update_block(&gamma, &photo_SettingsEEPROM.photo_Gamma, sizeof(float));

    }

    photo_Settings.photo_minValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_minValue);
    photo_Settings.photo_maxValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_maxValue);
    photo_Settings.photo_minCalibValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_minCalibValue);
    photo_Settings.photo_maxCalibValue = eeprom_read_byte(&photo_SettingsEEPROM.photo_maxCalibValue);
    photo_Settings.photo_useSensor = eeprom_read_byte(&photo_SettingsEEPROM.photo_useSensor);
    photo_Settings.photo_calibChanged = eeprom_read_byte(&photo_SettingsEEPROM.photo_calibChanged);
    photo_Settings.photo_sensorLast = eeprom_read_byte(&photo_SettingsEEPROM.photo_sensorLast);

    eeprom_read_block(&photo_Settings.photo_Gamma, &photo_SettingsEEPROM.photo_Gamma, sizeof(float));

    photo_setup_calibration();

    photoSensorIndex = 0;
    photoSensorSum = (uint16_t)photo_Settings.photo_maxValue << (uint16_t)PHOTO_NUM_SAMPLES_EXP;
    photoSensorLast = photo_Settings.photo_sensorLast;
    memset(&photoSensorValues[0], photo_Settings.photo_maxValue, PHOTO_NUM_SAMPLES);
    areLightsOn = 0;
    
    // setup readout of the bg-led state
    DDRA &= ~(1 << 0);
    nop();
    PORTA &= ~(1 << 0);    
}


//------------------------------------------------------------------------------
void photo_tick(void)
{    
    // update photo sensor if its low-pass filtered value has changed
    if (photo_Settings.photo_useSensor == 2)
    {        
        photoSensorLast = updatePhotoSensor(photoSensorRawValue);
    }else if (photo_Settings.photo_useSensor == 1)
    {
        if (areLightsOn) photoSensorLast = photo_Settings.photo_minValue;
        else photoSensorLast = photo_Settings.photo_maxValue;
    }

#if 1
    if (button_down(BUTTON_TEL) && button_down(BUTTON_UHR))
    {
        uint8_t data[8] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, photoSensorRawValue, photoSensorSum >> PHOTO_NUM_SAMPLES_EXP, photo_Settings.photo_minValue, photo_Settings.photo_maxValue, photo_Settings.photo_minCalibValue, photo_Settings.photo_maxCalibValue};
        ibus_sendMessage(IBUS_DEV_BMBT, IBUS_DEV_GLO, data, 8, IBUS_TRANSMIT_TRIES);
    }
#endif
}

//------------------------------------------------------------------------------
void photo_on_bus_msg(uint8_t src, uint8_t dst, uint8_t* msg, uint8_t msglen)
{
    if (dst == IBUS_DEV_BMBT && msg[0] == IBUS_MSG_OPENBM_TO)
    {
        if (msglen == 2 && msg[1] == IBUS_MSG_OPENBM_GET_PHOTO)
        {
            uint8_t data[8] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_GET_PHOTO, photoSensorRawValue, photoSensorSum >> PHOTO_NUM_SAMPLES_EXP, photo_Settings.photo_minValue, photo_Settings.photo_maxValue, photo_Settings.photo_minCalibValue, photo_Settings.photo_maxCalibValue};
            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 8, IBUS_TRANSMIT_TRIES);
        }else if (msglen == 4 && msg[1] == IBUS_MSG_OPENBM_SET_DISPLAY_LIGHT)
        {
            photo_enable(msg[2] == 0xFF ? 2 : (msg[2] == 0x0F ? 1 : 0));
            photoSensorLast = msg[3];
            eeprom_update_byte(&photo_SettingsEEPROM.photo_sensorLast, msg[3]);
        }else if (msglen == 7 && msg[1] == IBUS_MSG_OPENBM_SET_PHOTO)
        {
            // every such message will be responded, default response is 0
            uint8_t data[3] = {IBUS_MSG_OPENBM_FROM, IBUS_MSG_OPENBM_SET_PHOTO, msg[2]};

            // setup gamma
            if (msg[2] == 0x01)
            {
                union
                {
                    struct
                    {
                        uint8_t byte[4];
                    };
                    float gamma;
                } gamma;
                gamma.byte[3] = msg[3];
                gamma.byte[2] = msg[4];
                gamma.byte[1] = msg[5];
                gamma.byte[0] = msg[6];

                eeprom_update_block(&gamma.gamma, &photo_SettingsEEPROM.photo_Gamma, sizeof(float));
                photo_Settings.photo_Gamma = gamma.gamma;
                photo_Settings.photo_calibChanged = 1;
                photo_calibCacheLine = 0xFF;

            // don't know what to do with this
            }else
                data[2] = 0;

            ibus_sendMessage(IBUS_DEV_BMBT, src, data, 3, IBUS_TRANSMIT_TRIES);
        }
    }

    // what about the bg leds
    if (OPENBM_HW_1)
    {
        if (src == IBUS_DEV_LCM && dst == IBUS_DEV_GLO && msglen > 4 && msg[0] == IBUS_MSG_LAMP_STATE && OPENBM_HW_1)
            areLightsOn = (msg[1] & 0x01);
    }else
        areLightsOn = bit_is_clear(PINA,0);
    
}
