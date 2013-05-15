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

#include <python2.7/Python.h>
#include <stdlib.h>
#include <string>
#include "MID.h"
#include "IPCClient.h"
#include "BC.h"

// Bordcomputer instance
BordComputer* bordComputer = NULL;

// python specific variables
PyObject* pyBM_Error = NULL;
PyObject* pyBM_Callback = NULL;
PyObject* pyBM_CallbackOnTitleChange = NULL;
PyObject* pyBM_CallbackOnRadioStateChange = NULL;
PyObject* pyBM_CallbackOnButtonFieldChange = NULL;
PyObject* pyBM_CallbackOnDisconnect = NULL;

PyObject* pybm_set_onDisconnect(PyObject *self, PyObject *args);
PyObject* pybm_set_onNewTitle(PyObject *self, PyObject *args);
PyObject* pybm_set_onRadioStateChange(PyObject *self, PyObject *args);
PyObject* pybm_set_signal_filter(PyObject *self, PyObject *args);
PyObject* pybm_start(PyObject *self, PyObject *args);
PyObject* pybm_stop(PyObject *self, PyObject *args);

boost::shared_ptr<IPCClient::Client> ipcClient;

/**
 * Run python callbacks when radio changed its state
 */
void onRadioStateChange()
{
    if (pyBM_CallbackOnRadioStateChange)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();
        PyObject* arglist = Py_BuildValue("(I)", mid_getRadioState());
        PyObject* result = PyObject_CallObject(pyBM_CallbackOnRadioStateChange, arglist);
        if (arglist) Py_DECREF(arglist);
        if (result) Py_DECREF(result);
        PyGILState_Release(gstate);
    }
}

/**
 * React on received messages over dbus/ibus
 **/
void onMessage(const IPCClient::IBusMessage& msg)
{
    unsigned char src = msg.src;
    unsigned char dst = msg.dst;
    const std::vector<unsigned char>& data = msg.data;

    // --------- Update BC  -----------------
    if (bordComputer)
    {
        bordComputer->onMessage(src, dst, data);
    }

    // --------- Update MID -----------------
    int ret = (int)mid_onMessage(src,dst,data);
    
    if ((ret & STATE))
    {
      onRadioStateChange();
    }
    
    if ((ret & TITLE) && pyBM_CallbackOnTitleChange)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();
        PyObject* arglist = Py_BuildValue("(s)", mid_getTitle());
        PyObject* result = PyObject_CallObject(pyBM_CallbackOnTitleChange, arglist);
        if (arglist) Py_DECREF(arglist);
        if (result) Py_DECREF(result);
        PyGILState_Release(gstate);
    }

    if ((ret & BOTTOM) && pyBM_CallbackOnButtonFieldChange)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();

        PyObject *lst = PyList_New(mid_getNumBottomFields());
        if (lst == NULL)
        {
            PyErr_SetString(pyBM_Error, "No Memory!");
            PyGILState_Release(gstate);
            return;
        }
        std::vector<std::string> fields(mid_getNumBottomFields());

        for (size_t i = 0; i < mid_getNumBottomFields(); i++)
        {
            fields[i] = std::string(mid_getBottomField(i));
            PyList_SetItem(lst, i, Py_BuildValue("s", fields[i].c_str()));
        }
        PyObject* arglist = Py_BuildValue("(OI)", lst, mid_getBottomActiveField());
        PyObject* result = PyObject_CallObject(pyBM_CallbackOnButtonFieldChange, arglist);
        if (lst) Py_DECREF(lst);
        if (arglist) Py_DECREF(arglist);
        if (result) Py_DECREF(result);
        PyGILState_Release(gstate);
    }

    // execute callback if specified
    if (pyBM_Callback)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();

        PyObject *lst = PyList_New(data.size());
        if (lst == NULL)
        {
            PyErr_SetString(pyBM_Error, "No Memory!");
            PyGILState_Release(gstate);
            return;
        }

        for (size_t i = 0; i < data.size(); i++)
            PyList_SET_ITEM(lst, i, Py_BuildValue("B", data[i]));

        PyObject* arglist = Py_BuildValue("(BBO)", src, dst, lst);
        PyObject* result = PyObject_CallObject(pyBM_Callback, arglist);
        if (lst) Py_DECREF(lst);
        if (arglist) Py_DECREF(arglist);
        if (result) Py_DECREF(result);

        PyGILState_Release(gstate);
    }

}


#if 1
//------------------------------------------------------------------------------
void onDisconnect()
{
    if (pyBM_CallbackOnDisconnect)
    {
        PyGILState_STATE gstate = PyGILState_Ensure();

        PyObject* result = PyObject_CallObject(pyBM_CallbackOnDisconnect, NULL);
        if (result) Py_DECREF(result);

        PyGILState_Release(gstate);
    }
}
#endif

/**
 * When error received set python error
 **/
void onReceiveError(IPCClient::ReceiveError error, const std::string& msg)
{
    if (pyBM_Error)
        PyErr_SetString(pyBM_Error, msg.c_str());
}
void onError(const std::string& msg)
{
    if (pyBM_Error)
        PyErr_SetString(pyBM_Error, msg.c_str());
}

//------------------------------------------------------------------------------
PyObject* pybm_setRadioState(PyObject *self, PyObject *args)
{
	PyObject* currentCallback = pyBM_CallbackOnRadioStateChange;
	pyBM_CallbackOnRadioStateChange = NULL;

	PyObject* res = pybm_mid_set_radio_state(self, args);

	pyBM_CallbackOnRadioStateChange = currentCallback;

	return res;
}

/*
 * Stop dbus thread
 **/
PyObject* pybm_stop(PyObject *self, PyObject *args)
{
    //DBus::dbus_stop();
    if (ipcClient)
    {
        ipcClient->stop();
        ipcClient.reset();
    }

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

/*
 * Start thread responsible for receiving signals
 **/
PyObject* pybm_start(PyObject *self, PyObject *args)
{
    pybm_stop(NULL,NULL);

    const char* address = "127.0.0.1";
    int port = 4287;

    if (!PyArg_ParseTuple(args, "|si", &address, &port))
        return NULL;

    ipcClient.reset(new IPCClient::TCPIPClient(address, port));

    ipcClient->setOnMessageCallback(onMessage);
    ipcClient->setOnReceiveErrorCallback(onReceiveError);
    ipcClient->setOnDisconnectCallback(onDisconnect);
    ipcClient->start();

    mid_setIPCClient(ipcClient);
    bordComputer->setIPCClient(ipcClient);
    
    if (!ipcClient->isConnected())
        return NULL;

    // return void
    Py_INCREF(Py_None);
    return Py_None;
}

/*
 * Setup callback function on Gateway signald ( set_signal_filter(handler, string) )
 **/
PyObject* pybm_set_signal_filter(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O:setOnMessage", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            PyErr_SetString(pyBM_Error, "parameter must be callable");
            return NULL;
        }
        Py_XINCREF(temp);         /* Add a reference to new callback */
        Py_XDECREF(pyBM_Callback);  /* Dispose of previous callback */
        pyBM_Callback = temp;       /* Remember new callback */
        /* Boilerplate to return "None" */
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

/*
 * Setup callback function on Gateway signald ( set_signal_filter(handler, string) )
 **/
PyObject* pybm_set_onNewTitle(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O:setOnTitleChange", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            PyErr_SetString(pyBM_Error, "parameter must be callable");
            return NULL;
        }
        Py_XINCREF(temp);         /* Add a reference to new callback */
        Py_XDECREF(pyBM_CallbackOnTitleChange);  /* Dispose of previous callback */
        pyBM_CallbackOnTitleChange = temp;       /* Remember new callback */
        /* Boilerplate to return "None" */
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

/*
 *
 **/
PyObject* pybm_set_onRadioStateChange(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O:setOnRadioStateChange", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            PyErr_SetString(pyBM_Error, "parameter must be callable");
            return NULL;
        }
        Py_XINCREF(temp);         /* Add a reference to new callback */
        Py_XDECREF(pyBM_CallbackOnRadioStateChange);  /* Dispose of previous callback */
        pyBM_CallbackOnRadioStateChange = temp;       /* Remember new callback */
        /* Boilerplate to return "None" */
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

/*
 *
 **/
PyObject* pybm_set_onButtonFieldChange(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O:setOnButtonFieldChange", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            PyErr_SetString(pyBM_Error, "parameter must be callable");
            return NULL;
        }
        Py_XINCREF(temp);         /* Add a reference to new callback */
        Py_XDECREF(pyBM_CallbackOnButtonFieldChange);  /* Dispose of previous callback */
        pyBM_CallbackOnButtonFieldChange = temp;       /* Remember new callback */
        /* Boilerplate to return "None" */
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}

#if 1
/*
 * Setup callback function on disconnect
 **/
PyObject* pybm_set_onDisconnect(PyObject *self, PyObject *args)
{
    PyObject *result = NULL;
    PyObject *temp;

    if (PyArg_ParseTuple(args, "O:setOnDisconnect", &temp))
    {
        if (!PyCallable_Check(temp))
        {
            PyErr_SetString(pyBM_Error, "parameter must be callable");
            return NULL;
        }
        Py_XINCREF(temp);         /* Add a reference to new callback */
        Py_XDECREF(pyBM_CallbackOnDisconnect);  /* Dispose of previous callback */
        pyBM_CallbackOnDisconnect = temp;       /* Remember new callback */
        /* Boilerplate to return "None" */
        Py_INCREF(Py_None);
        result = Py_None;
    }
    return result;
}
#endif

//! Exported functions
static PyMethodDef pyBM_Methods[] = {
    {"setOnMessage",  pybm_set_signal_filter, METH_VARARGS, "Set callback handler on new signals."},
    {"setOnTitleChange",  pybm_set_onNewTitle, METH_VARARGS, "Set callback handler to react on title changes."},
    {"setOnRadioStateChange",  pybm_set_onRadioStateChange, METH_VARARGS, "Define callback handler to react on Radio state changes (i.e. Mode switch)."},
    {"setOnButtonFieldChange",  pybm_set_onButtonFieldChange, METH_VARARGS, "Define callback handler to react when Radio forces to rewrite buttons."},
    {"setOnDisconnect",  pybm_set_onDisconnect, METH_VARARGS, "Set callback handler when connection get lost or disconnected."},
    {"start",  pybm_start, METH_VARARGS, "Start thread listening for signals."},
    {"stop",  pybm_stop, METH_NOARGS, "Stop thread."},

    // ------------ MID Emulation -------------
    {"midEnableEmulation", pybm_mid_enable_emulation, METH_VARARGS, "Enable emulating of MID by filtering messages for/to MID."},
    {"midGetTitleField", pybm_mid_get_title_field, METH_NOARGS, "Return parsed title field filled by the Radio"},
    {"midGetBottomField", pybm_mid_get_bottom_field, METH_VARARGS, "Return parsed button field dedicated to MID"},
    {"midGetRadioState", pybm_mid_get_radio_state, METH_NOARGS, "Return current state/mode of the radio"},
    {"midSetRadioState", pybm_setRadioState, METH_VARARGS, "Set current state/mode of the radio"},
    {"midSendButtonPress", pybm_mid_send_button_press, METH_VARARGS, "Send button code over D-Bus to IBus simulating press event = down+up"},
    {"midReqUpdateFields", pybm_mid_req_update_fields, METH_NOARGS, "Send code requesting to update Radio fields"},
    {"midSetLedState", pybm_set_led_state, METH_VARARGS, "Set OpenBM LED state. First param is color 'r', 'g', 'y', 'f', 'd' or 'c' to clean. Second param is flash sequence"},

    {"midGetRadioBassState", pybm_get_radio_bass_state, METH_NOARGS, "Return current/parsed bass state of the radio"},
    {"midGetRadioTrebleState", pybm_get_radio_treble_state, METH_NOARGS, "Return current/parsed treble state of the radio"},
    {"midGetRadioFadeState", pybm_get_radio_fade_state, METH_NOARGS, "Return current/parsed fade state of the radio"},
    {"midGetRadioBalanceState", pybm_get_radio_balance_state, METH_NOARGS, "Return current/parsed balance state of the radio"},
    {"midGetRadioToneState", pybm_get_radio_tone_state, METH_VARARGS, "Return current/parsed tone (0=bass, 1=treble, 2=fade, 3=balance) state"},

    {"midSetRadioBassState", pybm_set_radio_bass_state, METH_VARARGS, "Set (increment, decrement) and return current/parsed bass state of the radio"},
    {"midSetRadioTrebleState", pybm_set_radio_treble_state, METH_VARARGS, "Set (increment, decrement) and return current/parsed treble state of the radio"},
    {"midSetRadioFadeState", pybm_set_radio_fade_state, METH_VARARGS, "Set (increment, decrement) and return current/parsed fade state of the radio"},
    {"midSetRadioBalanceState", pybm_set_radio_balance_state, METH_VARARGS, "Set (increment, decrement) and return current/parsed balance state of the radio"},
    {"midSetRadioToneState", pybm_set_radio_tone_state, METH_VARARGS, "Set and return current/parsed tone (0=bass, 1=treble, 2=fade, 3=balance) state"},


    // ------------ BordComputer -------------
    {"bcSetTimeFromSystem", BordComputer::py_setTimeFromSystem, METH_NOARGS, "Set IKE/BC time from the computer's system time"},
    {"bcSetDateFromSystem", BordComputer::py_setDateFromSystem, METH_NOARGS, "Set IKE/BC date from the computer's system date"},
    {"bcSetOnValueCallback", BordComputer::py_setCallbackOnOBCValue, METH_VARARGS, "Set callback method to be called when IKE send new value for given obc type."},
    {"bcSetOnStateCallback", BordComputer::py_setCallbackOnOBCState, METH_VARARGS, "Set callback method to be called when IKE send update of OBC states, like limit, memo, ..."},
    {"bcResetOBCValue", BordComputer::py_resetOBCValue, METH_VARARGS, "Reset obc value. Given argument should match the obc type."},
    {"bcReqOBCState", BordComputer::py_reqOBCState, METH_NOARGS, "Request update of the obc states, i.e. limit, memo, ..."},
    {"bcReqOBCValue", BordComputer::py_reqOBCValue, METH_VARARGS, "Request value from obc. Given argument should match the obc type."},
    {"bcEnableOBCValue", BordComputer::py_enableOBCValue, METH_VARARGS, "Activate value in the obc. Given argument should match the obc type."},
    {"bcDisableOBCValue", BordComputer::py_disableOBCValue, METH_VARARGS, "Dectivate value in the obc. Given argument should match the obc type."},
    {"bcSetOBCValue", BordComputer::py_setOBCValue, METH_VARARGS, "Set value in the obc. Given argument should match the obc type and the data type (unsigned short)."},
    {"bcReqTime", BordComputer::py_reqTime, METH_NOARGS, "Request time value from obc."},
    {"bcReqDate", BordComputer::py_reqDate, METH_NOARGS, "Request date value from obc."},
    {"bcReqOutsideTemperature", BordComputer::py_reqOutsideTemperature, METH_NOARGS, "Request outside temperature value from obc."},
    {"bcReqConsumption1", BordComputer::py_reqConsumption1, METH_NOARGS, "Request consumption 1 value from obc."},
    {"bcReqConsumption2", BordComputer::py_reqConsumption2, METH_NOARGS, "Request consumption 2 value from obc."},
    {"bcReqRange", BordComputer::py_reqRange, METH_NOARGS, "Request range value from obc."},
    {"bcReqDistance", BordComputer::py_reqDistance, METH_NOARGS, "Request distance value from obc."},
    {"bcReqArrival", BordComputer::py_reqArrival, METH_NOARGS, "Request arrival value from obc."},
    {"bcReqSpeedLimit", BordComputer::py_reqSpeedLimit, METH_NOARGS, "Request speed limit value and state if speed limit is set from obc."},
    {"bcReqAverageSpeed", BordComputer::py_reqAverageSpeed, METH_NOARGS, "Request average speed value from obc."},
    {"bcReqIgnitionState", BordComputer::py_reqIgnitionState, METH_NOARGS, "Set callback method to be called when IKE send new value for given obc type."},
    {"bcSetOnIgnitionStateCallback", BordComputer::py_setCallbackOnIgnitionState, METH_VARARGS, "Set callback to be called, when ignition state changes."},
    {"bcGetIgnitionState", BordComputer::py_getIgnitionState, METH_NOARGS, "Get state of the ignition, recieved from IKE."},

    {"bcSetDistance", BordComputer::py_setDistance, METH_VARARGS, "Set distance to target value to obc. Set distance to 0, to deactivate."},
    {"bcSetSpeedLimit", BordComputer::py_setSpeedLimit, METH_VARARGS, "Set speed limit (or 0 to deactive) to OBC."},
    {"bcEnableSpeedLimit", BordComputer::py_activateSpeedLimit, METH_NOARGS, "Activate speed limit."},
    {"bcSetSpeedLimitToCurrent", BordComputer::py_setToCurrentSpeedLimit, METH_NOARGS, "Activate speed limit and set it to the current speed."},

    {NULL, NULL, 0, NULL}        /* Sentinel */
};


/**
 * Init this library
 **/
PyMODINIT_FUNC initopenbm(void)
{
    PyObject *m;

    // init all module methods
    m = Py_InitModule("openbm", pyBM_Methods);
    if (m == NULL)
        return;

    // initialize Error object
    char errorName[] = "openbm.error";
    pyBM_Error = PyErr_NewException(errorName, NULL, NULL);
    Py_INCREF(pyBM_Error);
    PyModule_AddObject(m, "error", pyBM_Error);

    mid_setErrorCallback(onError);

    bordComputer = BordComputer::instance();
}

#include <execinfo.h>

/**
 * Crash handler to cleanup everything before die
 **/
void handler_SEGV(int sig)
{
  if (ipcClient) ipcClient->stop();
  ipcClient.reset();

  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "pyBMClient - Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, 2);
}


/*
 * 
 */
int main(int argc, char** argv)
{
    signal(SIGSEGV, handler_SEGV);

    Py_SetProgramName(argv[0]);
    Py_Initialize();

    if (!PyEval_ThreadsInitialized())
    {
        PyEval_InitThreads();
        // PyEval_InitThreads() acquires the GIL on my behalf but
        // I don't want it at the moment.
        PyEval_ReleaseLock();
    }
    
    initopenbm();
    
    return (EXIT_SUCCESS);
}

