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
#include "BC.h"
#include <ctime>

#define IBUS_DEV_IKE     0x80    // Instrument cluster electronics
#define IBUS_DEV_GT      0x3B    // Graphics driver (in navigation system)
#define IBUS_DEV_DIA     0x3F    // Diagnostic
#define IBUS_DEV_GLO     0xBF    // Global, broadcast address
#define IBUS_DEV_ANZV    0xE7    // Front display

#define IBUS_MSG_IGNITION_REQ       0x10    // Request ignition state
#define IBUS_MSG_IGNITION           0x11    // State of the ignition
#define IBUS_MSG_UPDATE_ANZV        0x24    // update anzv text
#define IBUS_MSG_OBC_STATE          0x2A    // update of the OBC state
#define IBUS_MSG_SET_OBC            0x40    // Set bord computer parameters
#define IBUS_MSG_REQ_OBC            0x41    // request obc data

boost::shared_ptr<BordComputer> BordComputer::s_BC;

//------------------------------------------------------------------------------
BordComputer* BordComputer::instance()
{
    if (!s_BC) s_BC.reset(new BordComputer());
    return s_BC.get();
}

//------------------------------------------------------------------------------
BordComputer::BordComputer()
{
    m_onOBCStateCallback = NULL;
    m_onIgnitionStateCallback = NULL;
    m_ignitionState = -1;
}

//------------------------------------------------------------------------------
BordComputer::~BordComputer()
{

}

//------------------------------------------------------------------------------
void BordComputer::onMessage(unsigned char src, unsigned char dst, const std::vector<unsigned char>& data)
{
    if (src != IBUS_DEV_IKE || data.size() < 2) return;

    // ignition state
    if (dst == IBUS_DEV_GLO && data[0] == IBUS_MSG_IGNITION)
    {
        if (m_onIgnitionStateCallback && m_ignitionState != (int)data[1])
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            PyObject* arglist = Py_BuildValue("(b)", data[1]);
            PyObject* result = PyObject_CallObject(m_onIgnitionStateCallback, arglist);
            if (arglist) Py_DECREF(arglist);
            if (result) Py_DECREF(result);
            PyGILState_Release(gstate);
        }
        m_ignitionState = data[1];
    }

    // if we get update of the OBC values
    if (dst == IBUS_DEV_ANZV && data[0] == IBUS_MSG_UPDATE_ANZV)
    {
        // get string sent by the OBC
        char buffer[256];
        memset(&buffer[0], 0, 256);
        for (unsigned i=3; i < data.size(); i++)
            buffer[i-3] = data[i];

        // this value is updated
        OBC obc = (OBC)data[1];

        // call callback method indicating the update
        std::map<OBC, PyObject*>::const_iterator it = m_onValueCallbackMap.find(obc);
        if (it != m_onValueCallbackMap.end())
        {
            PyGILState_STATE gstate = PyGILState_Ensure();
            PyObject* arglist = Py_BuildValue("(s)", &buffer[0]);
            PyObject* result = PyObject_CallObject(it->second, arglist);
            if (arglist) Py_DECREF(arglist);
            if (result) Py_DECREF(result);
            PyGILState_Release(gstate);
            
            //printf("Call update %s\n", &buffer[0]);
        }
    }else if (dst == IBUS_DEV_ANZV && data[0] == IBUS_MSG_OBC_STATE && m_onOBCStateCallback)
    {
        unsigned char limit = (data[1] & 0x02);
        unsigned char memo  = (data[1] & 0x20) >> 4;
        unsigned char auxheat=(data[2] & 0x20) >> 4;
        unsigned char stopwatch = 0;
        unsigned char code = 0;

        //printf("OBC data: limit=%d, memo=%d, auxheat=%d, stopwatch=%d, code=%d\n",limit,memo,auxheat,stopwatch,code);
                
        PyGILState_STATE gstate = PyGILState_Ensure();
        PyObject* arglist = Py_BuildValue("(bbbbb)", limit, memo, stopwatch, auxheat, code);
        PyObject* result = PyObject_CallObject(m_onOBCStateCallback, arglist);
        if (arglist) Py_DECREF(arglist);
        if (result) Py_DECREF(result);
        PyGILState_Release(gstate);
    }
}

//------------------------------------------------------------------------------
void BordComputer::sendBCRequest(unsigned char a, int c)
{
    if (m_ipcClient)
    {
        std::vector<unsigned char> data(3);
        if (c < 0) data.resize(2);
        data[0] = IBUS_MSG_REQ_OBC;
        data[1] = a;
        if (c >=0) data[2] = c;
        IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
        m_ipcClient->send(msg);
    }
}

//------------------------------------------------------------------------------
void BordComputer::sendSetBCValue(unsigned char a, unsigned short dd)
{
    if (m_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = IBUS_MSG_SET_OBC;
        data[1] = a;
        data[2] = (dd & 0xFF00) >> 8;
        data[3] = (dd & 0x00FF);
        IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
        m_ipcClient->send(msg);
    }
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setCallbackOnOBCValue(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;
    unsigned char obc;

    if (PyArg_ParseTuple(args, "bO", &obc, &temp))
    {
        if (!PyCallable_Check(temp))
        {
            //PyErr_SetString(pyBM_Error, "second parameter must be a callable");
            return NULL;
        }
        PyObject* callback = NULL;

        std::map<OBC, PyObject*>::const_iterator it = instance()->m_onValueCallbackMap.find((OBC)obc);
        if (it != instance()->m_onValueCallbackMap.end())
            callback = it->second;

        Py_XINCREF(temp);
        Py_XDECREF(callback);
        callback = temp;
        instance()->m_onValueCallbackMap[(OBC)obc] = callback;

        // return void
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setCallbackOnOBCState(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            //PyErr_SetString(pyBM_Error, "second parameter must be a callable");
            return NULL;
        }
        Py_XINCREF(temp);
        Py_XDECREF(instance()->m_onOBCStateCallback);
        instance()->m_onOBCStateCallback = temp;
        
        // return void
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setCallbackOnIgnitionState(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            //PyErr_SetString(pyBM_Error, "second parameter must be a callable");
            return NULL;
        }
        Py_XINCREF(temp);
        Py_XDECREF(instance()->m_onIgnitionStateCallback);
        instance()->m_onIgnitionStateCallback = temp;
        
        // return void
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}


//------------------------------------------------------------------------------
PyObject* BordComputer::py_setSpeedLimit(PyObject *self, PyObject *args)
{
    unsigned short limit;

    if (!PyArg_ParseTuple(args, "H", &limit))
        return NULL;

    instance()->setSpeedLimit(limit);
    
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
void BordComputer::setSpeedLimit(unsigned short limit)
{
    if (m_ipcClient)
    {
        if (limit == 0)
        {
            std::vector<unsigned char> data(3);
            data[0] = IBUS_MSG_REQ_OBC;
            data[1] = 0x09;
            data[2] = SWITCH_OFF;
            IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
            m_ipcClient->send(msg);
        }else
        {
            std::vector<unsigned char> data(4);
            data[0] = IBUS_MSG_SET_OBC;
            data[1] = 0x09;
            data[2] = (limit & 0xFF00) >> 8;
            data[3] = (limit & 0x00FF);
            IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
            m_ipcClient->send(msg);
        }
    }
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setDistance(PyObject *self, PyObject *args)
{
    unsigned short dist;

    if (!PyArg_ParseTuple(args, "H", &dist))
        return NULL;

    instance()->setDistance(dist);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
void BordComputer::setDistance(unsigned short distance)
{
    if (m_ipcClient)
    {
        if (distance == 0)
        {
            std::vector<unsigned char> data(3);
            data[0] = IBUS_MSG_REQ_OBC;
            data[1] = 0x07;
            data[2] = SWITCH_OFF;
            IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
            m_ipcClient->send(msg);
        }else
        {
            std::vector<unsigned char> data(4);
            data[0] = IBUS_MSG_SET_OBC;
            data[1] = 0x07;
            data[2] = (distance & 0xFF00) >> 8;
            data[3] = (distance & 0x00FF);
            IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
            m_ipcClient->send(msg);
        }
    }
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_getIgnitionState(PyObject *self, PyObject *args)
{
    return Py_BuildValue("i", instance()->m_ignitionState);
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_reqIgnitionState(PyObject *self, PyObject *args)
{
    if (instance()->m_ipcClient)
    {
        std::vector<unsigned char> data(1);
        data[0] = IBUS_MSG_IGNITION_REQ;
        IPCClient::IBusMessage msg(IBUS_DEV_DIA, IBUS_DEV_IKE, data);
        instance()->m_ipcClient->send(msg);
    }

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_reqOBCState(PyObject *self, PyObject *args)
{
    instance()->sendBCRequest(AuxHeatVent, GET_STATUS);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_resetOBCValue(PyObject *self, PyObject *args)
{
    unsigned char obc;

    if (!PyArg_ParseTuple(args, "b", &obc))
        return NULL;

    instance()->sendBCRequest(obc, RESET);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_reqOBCValue(PyObject *self, PyObject *args)
{
    unsigned char obc;

    if (!PyArg_ParseTuple(args, "b", &obc))
        return NULL;

    instance()->sendBCRequest(obc, GET_VALUE);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_enableOBCValue(PyObject *self, PyObject *args)
{
    unsigned char obc;

    if (!PyArg_ParseTuple(args, "b", &obc))
        return NULL;

    instance()->sendBCRequest(obc, SWITCH_ON);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_disableOBCValue(PyObject *self, PyObject *args)
{
    unsigned char obc;

    if (!PyArg_ParseTuple(args, "b", &obc))
        return NULL;

    instance()->sendBCRequest(obc, SWITCH_OFF);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setOBCValue(PyObject *self, PyObject *args)
{
    unsigned char obc;
    unsigned short data;

    if (!PyArg_ParseTuple(args, "bH", &obc, &data))
        return NULL;

    instance()->sendSetBCValue(obc, data);

    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
void BordComputer::setTimeFromSystem()
{
    time_t now = time(NULL);
    tm* tm = localtime(&now);

    if (m_ipcClient)
    {
        std::vector<unsigned char> data(4);
        data[0] = IBUS_MSG_SET_OBC;
        data[1] = 0x01;
        data[2] = tm->tm_hour;
        data[3] = tm->tm_min;
        IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
        m_ipcClient->send(msg);
    }
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setTimeFromSystem(PyObject *self, PyObject *args)
{
    instance()->setTimeFromSystem();
    Py_INCREF(Py_None);
    return Py_None;
}

//------------------------------------------------------------------------------
void BordComputer::setDateFromSystem()
{
    time_t now = time(NULL);
    tm* tm = localtime(&now);

    if (m_ipcClient)
    {
        std::vector<unsigned char> data(5);
        data[0] = IBUS_MSG_SET_OBC;
        data[1] = 0x02;
        data[2] = tm->tm_mday;
        data[3] = tm->tm_mon;
        data[4] = tm->tm_year % 100;
        IPCClient::IBusMessage msg(IBUS_DEV_GT, IBUS_DEV_IKE, data);
        m_ipcClient->send(msg);
    }
}

//------------------------------------------------------------------------------
PyObject* BordComputer::py_setDateFromSystem(PyObject *self, PyObject *args)
{
    instance()->setDateFromSystem();
    Py_INCREF(Py_None);
    return Py_None;
}
