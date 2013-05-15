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

#include "MID.h"
#include "IPCClient.h"
#include <string>

#define IBUS_DEV_RAD     0x68    // Radio
#define IBUS_DEV_IKE     0x80    // Instrument cluster electronics
#define IBUS_DEV_MID     0xC0    // Multi-info display
#define IBUS_DEV_TEL     0xC8    // Telephone
#define IBUS_DEV_ANZV    0xE7    // Front display
#define IBUS_DEV_LOC     0xFF    // Local
#define IBUS_DEV_BMBT    0xF0    // On-board monitor operating part

#define IBUS_MSG_LED                0x2B    // set status-LED state
#define IBUS_MSG_LED_SPECIAL        0x2D    // set status-LED state (special function, defining blink ratio)
#define IBUS_MSG_BUTTON             0x31    // MID's button state change

#define BMW_E39_PROFESSIONAL_RADIO

#define MODE_PRESS(a) {\
                std::vector<unsigned char> data(4);\
                data[0] = IBUS_MSG_BUTTON;\
                data[1] = 0x00;\
                data[2] = 0x00;\
                data[3] = 0x0B;\
                IPCClient::IBusMessage msg(IBUS_DEV_MID, IBUS_DEV_RAD, data);\
                g_ipcClient->send(msg);\
            }\

#define MODE_RELEASE(a) {\
                std::vector<unsigned char> data(4);\
                data[0] = IBUS_MSG_BUTTON;\
                data[1] = 0x00;\
                data[2] = 0x00;\
                data[3] = 0x4B;\
                IPCClient::IBusMessage msg(IBUS_DEV_MID, IBUS_DEV_RAD, data);\
                g_ipcClient->send(msg);\
            }\

#ifndef WIN32
#include <unistd.h>
#endif
static int my_sleep( unsigned long _t ) {
#ifdef WIN32
return sleep( _t );
#else
return usleep( 1000 * _t );
#endif
}

bool g_emulateMID = false;
boost::shared_ptr<IPCClient::Client> g_ipcClient;

enum MIDState
{
    UNDEFINED = 0,
    RADIO = (1 << 0),
    CD =    (1 << 1),
    TAPE =  (1 << 2),
    TONE =  (1 << 3),
    AUDIO = RADIO | CD | TAPE | TONE,
    BC = (1 << 4),
    PHONE = (1 << 5)
};
MIDState g_midState = UNDEFINED;



// next implementation is only valid for professional radio
#ifdef BMW_E39_PROFESSIONAL_RADIO

#define TITLE_SIZE 11
#define BOTTOM_ONE_FIELD 4
#define BOTTOM_NUM_FIELDS 12
#define BOTTOM_SIZE BOTTOM_ONE_FIELD * BOTTOM_NUM_FIELDS

// 11 character title field
char g_TitleField[TITLE_SIZE+1] = {0,0,0,0,0,0,0,0,0,0,0,0};
char g_BottomField[BOTTOM_NUM_FIELDS][BOTTOM_ONE_FIELD];
char g_tempBottom[BOTTOM_ONE_FIELD+1];
int g_BottomActiveField = -1;
int g_TitleFieldFrequency = 0;
int g_TitleFieldRDS = 0;
int g_radioBass = 0;
int g_radioTreble = 0;
int g_radioFade = 0;
int g_radioBalance = 0;
#endif

void (*g_midErrorCallback)(const std::string& msg) = NULL;
//void (*g_midForceRadioStateChangeCallback)() = NULL;

//------------------------------------------------------------------------------
void mid_setErrorCallback(void (*cb)(const std::string& msg))
{
    g_midErrorCallback = cb;
}

#if 0
//------------------------------------------------------------------------------
void mid_setForceRadioStateChangeCallback(void (*cb)())
{
    g_midForceRadioStateChangeCallback = cb;
}
#endif 

//------------------------------------------------------------------------------
void mid_setIPCClient(boost::shared_ptr<IPCClient::Client> client)
{
    g_ipcClient = client;
}

//------------------------------------------------------------------------------
MIDState mid_convertByteToState(unsigned char byte)
{
    if (byte == 0x40)
        return RADIO;
    else if (byte == 0xC0)
        return CD;
    else if (byte == 0x80)
        return TAPE;
    else if (byte == 0x60)
        return TONE;
        
    return UNDEFINED;
}

//------------------------------------------------------------------------------
unsigned char mid_convertStateToByte(MIDState state)
{
    if ((state & RADIO) == RADIO)
        return 0x40;
    else if ((state & CD) == CD)
        return 0xC0;
    else if ((state & TAPE) == TAPE)
        return 0x80;
    else if ((state & TONE) == TONE)
        return 0x60;

    return 0;
}

//------------------------------------------------------------------------------
MidRadioState mid_getRadioState()
{
    if (g_midState & RADIO) return RADIO_RADIO;
    if (g_midState & CD) return RADIO_CD;
    if (g_midState & TAPE) return RADIO_TAPE;
    if (g_midState & TONE) return RADIO_TONE;
    return RADIO_RADIO;
}

//------------------------------------------------------------------------------
int mid_getBottomActiveField()
{
    return g_BottomActiveField;
}

//------------------------------------------------------------------------------
unsigned mid_getNumBottomFields()
{
    return BOTTOM_NUM_FIELDS;
}

//------------------------------------------------------------------------------
const char* mid_getTitle()
{
    return &g_TitleField[0];
}

//------------------------------------------------------------------------------
const char* mid_getBottomField(unsigned char field)
{
    if (field >= BOTTOM_NUM_FIELDS) return NULL;

    memset(g_tempBottom, 0, sizeof(BOTTOM_ONE_FIELD));
    memcpy(g_tempBottom, &g_BottomField[field][0], BOTTOM_ONE_FIELD);
    return &g_tempBottom[0];
}

//------------------------------------------------------------------------------
int mid_onMessage(unsigned char src, unsigned char dst, const std::vector<unsigned char>& data)
{
    if (!g_emulateMID) return OK;
    if (dst != IBUS_DEV_ANZV && dst != IBUS_DEV_MID && dst != IBUS_DEV_LOC) return OK;

    MIDState oldState = g_midState;
    
    // if message is received from RADIO
    if (src == IBUS_DEV_RAD)
    {
        // we received a message indicating of new value for text field
        if (data.size() > 4 && data[0] == 0x23)
        {
            // currently implemented only for BMW Professional Radio
            #ifdef BMW_E39_PROFESSIONAL_RADIO

            // third byte must always be 0x20
            if (data[2] != 0x20) goto check024;

            // second byte of message must indicate the type of currently active radio part
            g_midState = mid_convertByteToState(data[1]);
            
            unsigned titlePtr = 0;
            for (unsigned i=3; i < data.size() && titlePtr < sizeof(g_TitleField); i++)
            {
                // react on commandos
                if (data[i] == 0x07)
                {
                    memset(&g_TitleField[0], 0, sizeof(g_TitleField));
                    titlePtr = 0;
                }else if (data[i] == 0x08)
                {
                    titlePtr = 0;
                }else if (data[i] == 0x03)
                    g_TitleFieldFrequency = titlePtr;
                else if (data[i] == 0x04)
                    g_TitleFieldRDS = titlePtr;
                else
                    g_TitleField[titlePtr++] = (char)data[i];
            }
            g_TitleField[sizeof(g_TitleField)-1] = '\0';

            return TITLE | (oldState != g_midState ? STATE : 0);
            
            #endif
        }

    check024:
        if (data.size() > 4 && data[0] == 0x24)
        {
            #ifdef BMW_E39_PROFESSIONAL_RADIO

            if (data[2] != 0x00) goto checkBottom;

            g_midState = mid_convertByteToState(data[1]);

            char tempBuf[12];
            memset(&tempBuf[0], 0, sizeof(tempBuf));

            int titlePtr = 0;
            for (unsigned i=3; i < data.size() && titlePtr < (int)sizeof(g_TitleField); i++)
            {
                tempBuf[titlePtr++] = (char)data[i];
            }

            memset(&g_TitleField[g_TitleFieldFrequency], 0, sizeof(g_TitleField) - g_TitleFieldFrequency);
            
            // copy content into global titel field, however right aligned
            for (int i=0; i < titlePtr; i++)
                g_TitleField[g_TitleFieldFrequency + i] = tempBuf[i];

            g_TitleField[sizeof(g_TitleField)-1] = '\0';

            return TITLE | (oldState != g_midState ? STATE : 0);

            #endif
        }

    checkBottom:

        // currently implemented only for BMW Professional Radio
        #ifdef BMW_E39_PROFESSIONAL_RADIO

        if (data.size() > 4 && data[0] == 0x21)
        {
            // second byte of message must indicate the type of currently active radio part
            g_midState = mid_convertByteToState(data[1]);

            // third byte must always be 0x00
            if (data[2] != 0x00) goto radioCheckDone;

            // parse first byte indicating where to write to as also a clean command
            int field = data[3] & 0x0F;
            int ch    = 0;

            char* pBottom = &g_BottomField[field][ch];

            if (data[3] & 0x60) memset(&g_BottomField[0][0], 0, sizeof(g_BottomField));
            if (data[3] & 0x40) memset(pBottom, 0, sizeof(char) * BOTTOM_ONE_FIELD);
            if (data[3] & 0x20) memset(pBottom, 0, sizeof(char) * (&g_BottomField[BOTTOM_NUM_FIELDS-1][BOTTOM_ONE_FIELD-1] - pBottom));
            memset(&g_BottomField[field][ch], 0, sizeof(char) * BOTTOM_ONE_FIELD);
            
            for (unsigned i=4; i < data.size() && ((field * BOTTOM_ONE_FIELD) + ch) < BOTTOM_SIZE; i++)
            {
                if (data[i] == 0x05)
                {
                    field += 1;
                    ch = 0;
                    memset(&g_BottomField[field][ch], 0, sizeof(char) * BOTTOM_ONE_FIELD);
                }else if (data[i] == 0x06)
                {
                    field += 2;
                    ch = 0;
                }else if (data[i] == 0x2A)
                    g_BottomActiveField = field;
                else if (data[i] > 0x10)
                    g_BottomField[field][ch++] = data[i];
            }

            return BOTTOM | (oldState != g_midState ? STATE : 0);
        }

        #endif

    radioCheckDone:
        int dummy = 3;
        dummy += 1;
    }


    // if state changed, then call callback indicating state change
    if (g_midState != oldState) return STATE;

    return OK;
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_enable_emulation(PyObject *self, PyObject *args)
{
    unsigned char en = true;
    if (!PyArg_ParseTuple(args, "|b", &en))
        return NULL;

    g_emulateMID = (bool)en;

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_get_title_field(PyObject *self, PyObject *args)
{
    return Py_BuildValue("s", mid_getTitle());
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_get_bottom_field(PyObject *self, PyObject *args)
{
    unsigned char field = 100;
    if (!PyArg_ParseTuple(args, "b", &field))
        return NULL;

    return Py_BuildValue("s", mid_getBottomField(field));
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_get_radio_state(PyObject *self, PyObject *args)
{
    return Py_BuildValue("i", mid_getRadioState());
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_set_radio_state(PyObject *self, PyObject *args)
{
    if (g_midState == UNDEFINED)
    {
        if (g_midErrorCallback) g_midErrorCallback("Current radio state is inknown !");
        return NULL;
    }

    // get state one would like to set hte radio to
    unsigned current = mid_getRadioState();
    unsigned state = current;
    //unsigned char notify = true;
    if (!PyArg_ParseTuple(args, "i", &state))
        return NULL;

    //if (state != current)
    {
        // depending on current state, we need to emulate MODE button press
        if ((current == RADIO_RADIO && state == RADIO_CD) ||
            (current == RADIO_CD    && state == RADIO_TAPE) ||
            (current == RADIO_TAPE  && state == RADIO_RADIO))
        {
            // send one MODE press event
            MODE_PRESS();
            MODE_RELEASE();

        }else if ((current == RADIO_RADIO && state == RADIO_TAPE) ||
                  (current == RADIO_CD    && state == RADIO_RADIO)||
                  (current == RADIO_TAPE  && state == RADIO_CD))
        {
            MODE_PRESS();
            MODE_RELEASE();
            my_sleep(50);
            MODE_PRESS();
            MODE_RELEASE();
        }else
        {
            MODE_PRESS();
            MODE_RELEASE();
            my_sleep(50);
            MODE_PRESS();
            MODE_RELEASE();
            my_sleep(50);
            MODE_PRESS();
            MODE_RELEASE();        
        }
    }

    // if notify is set, then call callbacks
    //if (notify && g_midForceRadioStateChangeCallback)
    //{
    //    g_midForceRadioStateChangeCallback();        
    //}
    
    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_req_update_fields(PyObject *self, PyObject *args)
{    
    if (g_ipcClient)
    {
        // C0 06 FF 20 00 B3 00
        std::vector<unsigned char> data(4);
        data[0] = 0x20;
        data[1] = 0x00;
        data[2] = 0xB3;
        data[3] = 0x00;
        IPCClient::IBusMessage msg(0xC0, 0xFF, data);

        g_ipcClient->send(msg);
    }

#if 0
    // C0 06 FF 20 02 8E 00 xx
    // C0 06 FF 20 01 B0 00 xx
    if (g_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = 0x20;
        data[1] = 0x02;
        data[2] = 0x8E;
        data[3] = 0x00;
        IPCClient::IBusMessage msg(0xC0, 0xFF, data);

        g_ipcClient->send(msg);
    }
    my_sleep(100);
    
    if (g_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = 0x20;
        data[1] = 0x01;
        data[2] = 0xB0;
        data[3] = 0x00;
        IPCClient::IBusMessage msg(0xC0, 0xFF, data);

        g_ipcClient->send(msg);
    }
#endif
    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_mid_send_button_press(PyObject *self, PyObject *args)
{
    unsigned char button = g_BottomActiveField;
    if (!PyArg_ParseTuple(args, "|b", &button))
        return NULL;
    
    if (g_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = IBUS_MSG_BUTTON;
        data[1] = mid_convertStateToByte(g_midState);
        data[2] = 0x00;
        data[3] = button;
        IPCClient::IBusMessage msg(IBUS_DEV_MID, IBUS_DEV_RAD, data);
        //DBus::dbus_sendMessage(&msg);
        g_ipcClient->send(msg);
    }

    if (g_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = IBUS_MSG_BUTTON;
        data[1] = mid_convertStateToByte(g_midState);
        data[2] = 0x00;
        data[3] = 0x40 + button;
        IPCClient::IBusMessage msg(IBUS_DEV_MID, IBUS_DEV_RAD, data);

        //DBus::dbus_sendMessage(&msg);
        g_ipcClient->send(msg);
    }

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_set_led_state(PyObject *self, PyObject *args)
{
    char led = 0;
    unsigned char seq = 0;
    if (!PyArg_ParseTuple(args, "bb", &led, &seq))
        return NULL;

    if (led == 'c') led = 0;
    else if (led == 'r') led = (1 << 1);
    else if (led == 'y') led = (1 << 2);
    else if (led == 'g') led = (1 << 3);
    else if (led == 'f') led = (1 << 4);
    else if (led == 'd') led = (1 << 5);
    else return NULL;

    if (g_ipcClient)
    {
        std::vector<unsigned char> data(3);
        data[0] = IBUS_MSG_LED_SPECIAL;
        data[1] = led;
        data[2] = seq;
        IPCClient::IBusMessage msg(IBUS_DEV_LOC, IBUS_DEV_BMBT, data);
        //DBus::dbus_sendMessage(&msg);
        g_ipcClient->send(msg);
    }

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_get_radio_bass_state(PyObject* self, PyObject* args)
{
    return Py_BuildValue("i", g_radioBass);
}

//------------------------------------------------------------------------------
PyObject* pybm_get_radio_treble_state(PyObject* self, PyObject* args)
{
    return Py_BuildValue("i", g_radioTreble);
}

//------------------------------------------------------------------------------
PyObject* pybm_get_radio_fade_state(PyObject* self, PyObject* args)
{
    return Py_BuildValue("i", g_radioFade);
}

//------------------------------------------------------------------------------
PyObject* pybm_get_radio_balance_state(PyObject* self, PyObject* args)
{
    return Py_BuildValue("i", g_radioBalance);
}

PyObject* pybm_get_radio_tone_state(PyObject* self, PyObject* args)
{
    unsigned char tone = 0;
    if (!PyArg_ParseTuple(args, "b", &tone))
        return NULL;

    if (tone == 0) return pybm_get_radio_bass_state(self, args);
    if (tone == 1) return pybm_get_radio_treble_state(self, args);
    if (tone == 2) return pybm_get_radio_fade_state(self, args);
    if (tone == 3) return pybm_get_radio_balance_state(self, args);

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* pybm_set_radio_bass_state(PyObject* self, PyObject* args)
{
    int inc = 0;
    if (!PyArg_ParseTuple(args, "i", &inc))
        return NULL;

    g_radioBass += inc;
    if (g_radioBass < -10) g_radioBass = -10;
    if (g_radioBass > 10) g_radioBass = 10;

    return Py_BuildValue("i", g_radioBass);
}

//------------------------------------------------------------------------------
PyObject* pybm_set_radio_treble_state(PyObject* self, PyObject* args)
{
    int inc = 0;
    if (!PyArg_ParseTuple(args, "i", &inc))
        return NULL;

    g_radioTreble += inc;
    if (g_radioTreble < -10) g_radioTreble = -10;
    if (g_radioTreble > 10) g_radioTreble = 10;

    return Py_BuildValue("i", g_radioTreble);
}

//------------------------------------------------------------------------------
PyObject* pybm_set_radio_fade_state(PyObject* self, PyObject* args)
{
    int  inc = 0;
    if (!PyArg_ParseTuple(args, "i", &inc))
        return NULL;

    g_radioFade += inc;
    if (g_radioFade < -10) g_radioFade = -10;
    if (g_radioFade > 10) g_radioFade = 10;

    return Py_BuildValue("i", g_radioFade);
}

//------------------------------------------------------------------------------
PyObject* pybm_set_radio_balance_state(PyObject* self, PyObject* args)
{
    int inc = 0;
    if (!PyArg_ParseTuple(args, "i", &inc))
        return NULL;

    g_radioBalance += inc;
    if (g_radioBalance < -10) g_radioBalance = -10;
    if (g_radioBalance > 10) g_radioBalance = 10;

    return Py_BuildValue("i", g_radioBalance);
}

//------------------------------------------------------------------------------
PyObject* pybm_set_radio_tone_state(PyObject* self, PyObject* args)
{
    int inc = 0;
    unsigned char tone = 0;
    if (!PyArg_ParseTuple(args, "bi", &tone, &inc))
        return NULL;
    
    if (tone == 0)
    {
        g_radioBass += inc;
        if (g_radioBass < -10) g_radioBass = -10;
        if (g_radioBass > 10) g_radioBass = 10;
        return Py_BuildValue("i", g_radioBass);
    }
    if (tone == 1)
    {
        g_radioTreble += inc;
        if (g_radioTreble < -10) g_radioTreble = -10;
        if (g_radioTreble > 10) g_radioTreble = 10;
        return Py_BuildValue("i", g_radioTreble);
    }
    if (tone == 2)
    {
        g_radioFade += inc;
        if (g_radioFade < -10) g_radioFade = -10;
        if (g_radioFade > 10) g_radioFade = 10;
        return Py_BuildValue("i", g_radioFade);
    }
    if (tone == 3)
    {
        g_radioBalance += inc;
        if (g_radioBalance < -10) g_radioBalance = -10;
        if (g_radioBalance > 10) g_radioBalance = 10;
        return Py_BuildValue("i", g_radioBalance);
    }
    
    // return void
    Py_INCREF(Py_None);
    return Py_None;
}
