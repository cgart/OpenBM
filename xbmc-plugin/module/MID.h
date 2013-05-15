/*
 *   (C) 2012 Art Tevs  <tevs 'email' mpi-inf 'dot' mpg 'dot' de> 
 * 
 *    This file is part of OpenBM-XBMC-Plugin.
 *
 *    OpenBM-XBMC-Plugin is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    OpenBM-XBMC-Plugin is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with OpenBM-XBMC-Plugin.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MID_H
#define	_MID_H

#include <vector>
#include <python2.7/Python.h>
#include "IPCClient.h"
#include <boost/shared_ptr.hpp>

enum MidMessageState
{
    OK = 0,
    TITLE = (1 << 0),
    BOTTOM = (1 << 1),
    STATE = (1 << 2)
};

enum MidRadioState
{
    RADIO_RADIO = 0,
    RADIO_CD = 1,
    RADIO_TAPE = 2,
    RADIO_TONE = 3
};

/**
 * Call when new message is received. The message will be parsed if MID emulation is active.
 * Through python we can the access emulated MID device.
 * @return: Following values are returned back. These can also be binary "or"ed together
 **/
int mid_onMessage(unsigned char src, unsigned char dst, const std::vector<unsigned char>& data);

const char* mid_getTitle();
const char* mid_getBottomField(unsigned char field);
int mid_getBottomActiveField();
unsigned mid_getNumBottomFields();
MidRadioState mid_getRadioState();

void mid_setIPCClient(boost::shared_ptr<IPCClient::Client> client);

/**
 * Enable emulating of MID device through python interface
 **/
PyObject* pybm_mid_enable_emulation(PyObject *self, PyObject *args);

/**
 * Get current Title field specified by radio.
 **/
PyObject* pybm_mid_get_title_field(PyObject *self, PyObject *args);

/**
 * Get content of the MID's bottom field [0:12]
 **/
PyObject* pybm_mid_get_bottom_field(PyObject *self, PyObject *args);

/**
 * Get current current radio state/mode
 **/
PyObject* pybm_mid_get_radio_state(PyObject *self, PyObject *args);

/**
 * Set current radio state 
 **/
PyObject* pybm_mid_set_radio_state(PyObject *self, PyObject *args);

/**
 * Send button press event
 **/
PyObject* pybm_mid_send_button_press(PyObject *self, PyObject *args);

/**
 * Send ibus code to request a field update
 **/
PyObject* pybm_mid_req_update_fields(PyObject *self, PyObject *args);

/**
 * Send ibus code to change LED state
 **/
PyObject* pybm_set_led_state(PyObject *self, PyObject *args);

/**
 * Get current state of equalizer.
 **/
PyObject* pybm_get_radio_bass_state(PyObject* self, PyObject* args);
PyObject* pybm_get_radio_treble_state(PyObject* self, PyObject* args);
PyObject* pybm_get_radio_fade_state(PyObject* self, PyObject* args);
PyObject* pybm_get_radio_balance_state(PyObject* self, PyObject* args);
PyObject* pybm_get_radio_tone_state(PyObject* self, PyObject* args);

PyObject* pybm_set_radio_bass_state(PyObject* self, PyObject* args);
PyObject* pybm_set_radio_treble_state(PyObject* self, PyObject* args);
PyObject* pybm_set_radio_fade_state(PyObject* self, PyObject* args);
PyObject* pybm_set_radio_balance_state(PyObject* self, PyObject* args);
PyObject* pybm_set_radio_tone_state(PyObject* self, PyObject* args);

/**
 * Set error callback
 **/
void mid_setErrorCallback(void (*cb)(const std::string& msg));

#if 0
/**
 * Set callbck to be called when radio state is changed manually
 * and you would like to be informed about that
 **/
void mid_setForceRadioStateChangeCallback(void (*cb)());
#endif 

#endif	/* _MID_H */

