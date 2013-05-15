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

#ifndef _BC_H
#define	_BC_H

#include "IPCClient.h"
#include <boost/noncopyable.hpp>
#include <vector>
#include <python2.7/Python.h>


class BordComputer : public boost::noncopyable
{
public:
    static BordComputer* instance();
    
    BordComputer();
    ~BordComputer();

    enum OBC{
        Time = 0x01,
        Date = 0x02,
        OutsideTemperature = 0x03,
        Consumption1 = 0x04,
        Consumption2 = 0x05,
        Range = 0x06,
        Distance = 0x07,
        Arrival = 0x08,
        SpeedLimit = 0x09,
        AverageSpeed = 0x0A,
        Memo1 = 0x0C,
        Memo2 = 0x0D,
        Stopwatch = 0x0E,
        Timer1 = 0x0F,
        Timer2 = 0x10,
        AuxHeatingOff = 0x11,
        AuxHeatingOn = 0x12,
        AuxVentOff = 0x13,
        AuxVentOn = 0x14,
        EndStellmode = 0x15,
        EmergencyDisarm = 0x16,
        InterimTime = 0x1A,
        AuxHeatVent = 0x1B
    };

    enum Control
    {
        GET_VALUE  = 0x01,
        GET_STATUS = 0x02,
        SWITCH_ON  = 0x04,
        SWITCH_OFF = 0x08,
        RESET      = 0x10,
        TAKE_OVER  = 0x80
    };

    //! Call this method when new ibus message received, bc controller will filter out needed things
    void onMessage(unsigned char src, unsigned char dst, const std::vector<unsigned char>& data);

    //! Set client to be used to send/receive ibus messages
    void setIPCClient(boost::shared_ptr<IPCClient::Client> client) { m_ipcClient = client; }

    //! Set time in the bordcomputer based on the system time
    void setTimeFromSystem();

    //! Set date in the bordcomputer based on the system date
    void setDateFromSystem();

    //! Set speed limit to given value (limit = 0 -> deaktiviere)
    void setSpeedLimit(unsigned short limit);

    //! Set distance to target (distance = 0 -> deaktiviere)
    void setDistance(unsigned short distance);

    // -------------------- Python Wrapper ----------------------------
    static PyObject* py_setCallbackOnOBCState(PyObject *self, PyObject *args);
    static PyObject* py_setCallbackOnOBCValue(PyObject *self, PyObject *args);
    static PyObject* py_setCallbackOnIgnitionState(PyObject *self, PyObject *args);
    static PyObject* py_reqIgnitionState(PyObject *self, PyObject *args);
    static PyObject* py_getIgnitionState(PyObject *self, PyObject *args);
    static PyObject* py_setTimeFromSystem(PyObject *self, PyObject *args);
    static PyObject* py_setDateFromSystem(PyObject *self, PyObject *args);
    static PyObject* py_resetOBCValue(PyObject *self, PyObject *args);
    static PyObject* py_reqOBCValue(PyObject *self, PyObject *args);
    static PyObject* py_reqOBCState(PyObject *self, PyObject *args);
    static PyObject* py_reqTime(PyObject *self, PyObject *args) { instance()->sendBCRequest(Time, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqDate(PyObject *self, PyObject *args) { instance()->sendBCRequest(Date, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqOutsideTemperature(PyObject *self, PyObject *args) { instance()->sendBCRequest(OutsideTemperature, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqConsumption1(PyObject *self, PyObject *args) { instance()->sendBCRequest(Consumption1, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqConsumption2(PyObject *self, PyObject *args) { instance()->sendBCRequest(Consumption2, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqRange(PyObject *self, PyObject *args) { instance()->sendBCRequest(Range, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqDistance(PyObject *self, PyObject *args) { instance()->sendBCRequest(Distance, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqArrival(PyObject *self, PyObject *args) { instance()->sendBCRequest(Arrival, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqSpeedLimit(PyObject *self, PyObject *args) { instance()->sendBCRequest(SpeedLimit, GET_VALUE); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_reqAverageSpeed(PyObject *self, PyObject *args) { instance()->sendBCRequest(AverageSpeed, GET_VALUE); Py_INCREF(Py_None); return Py_None; }

    static PyObject* py_activateSpeedLimit(PyObject *self, PyObject *args) { instance()->sendBCRequest(SpeedLimit, SWITCH_ON); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_setToCurrentSpeedLimit(PyObject *self, PyObject *args) { instance()->sendBCRequest(SpeedLimit, TAKE_OVER); Py_INCREF(Py_None); return Py_None; }
    static PyObject* py_setSpeedLimit(PyObject *self, PyObject *args);

    static PyObject* py_setDistance(PyObject *self, PyObject *args);

    static PyObject* py_setOBCValue(PyObject *self, PyObject *args);
    static PyObject* py_enableOBCValue(PyObject *self, PyObject *args);
    static PyObject* py_disableOBCValue(PyObject *self, PyObject *args);

private:
    static boost::shared_ptr<BordComputer> s_BC;

    boost::shared_ptr<IPCClient::Client> m_ipcClient;

    std::map<OBC, PyObject*> m_onValueCallbackMap;

    PyObject* m_onOBCStateCallback;
    PyObject* m_onIgnitionStateCallback;
    char m_ignitionState;
    
    void sendBCRequest(unsigned char a, int c);
    void sendSetBCValue(unsigned char a, unsigned short data);
};

#endif	/* _BC_H */

