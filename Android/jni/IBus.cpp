/*
 *   Copyright 2010 Art Tevs <art@tevs.eu>
 *   This file is part of OpenBM-Gateway.
 *
 *   Foobar is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Foobar is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Log.h"
#include "IBus.h"
#include "Application.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

//------------------------------------------------------------------------------
IBus::IBus() : m_Timer(m_ioService), m_Serial(m_ioService), m_lastSendMessage(0,0,0)
{
    m_State = IDLE;
    m_lastID = NO_ID + 1;
    m_running = true;
    timeOut(boost::asio::error::fault);
    mCTSType = NO;
}

//------------------------------------------------------------------------------
IBus::~IBus()
{
    close();
}

//------------------------------------------------------------------------------
void IBus::close()
{
    // first stop if currently running
    if (m_running)
        stop();

    // now cloase port if still open
    if (m_Serial.is_open())
    {
        m_Serial.cancel();
        m_Serial.close();
        
        // close serial port and stop io service
        LOG_DEBUG("Close serial port");
    }
}

//------------------------------------------------------------------------------
IBus::MessageID IBus::send(Message* msg)
{
    // lock tx queue for write
    boost::mutex::scoped_lock(m_txQueueMutex);

    // add message to the tx queue
    msg->m_ID = m_lastID++;
    m_TxQueue.push(msg);

    // notify main loop, that there is action happened, however only if we
    // are currently in the idle state
    if (m_State == IDLE)
    {
        m_actionCond.notify_all();
    }

    return msg->m_ID;
}

//--------------------------------------------------------------------------
void IBus::start()
{
    // check if currently running, then stop
    if (m_running)
        stop();

    LOG_DEBUG("Start IBus service");

    // for debugging, they must be NULL here
    assert(!m_ioThread);
    assert(!m_iBusThread);

    // run io service for serial port and timers in separate thread
    m_ioThread.reset(new boost::thread(boost::bind(&IBus::run_ioservice, this)));

    // run ibus thread
    m_iBusThread.reset(new boost::thread(boost::bind(&IBus::run_ibus, this)));
    //struct sched_param param;
    //param.sched_priority = 90;

    //pthread_attr_setschedparam( m_iBusThread->native_handle(), &param);

}

//--------------------------------------------------------------------------
void IBus::stop()
{
    // if not running, then we have nothing to stop
    if (m_running == false || (m_iBusThread == NULL && m_ioThread == NULL)) return;

    LOG_DEBUG("Stop IBus service");

    // these must be defined here
    assert(m_iBusThread);
    assert(m_ioThread);

    // indicate stop to the both threads
    m_running = false;
    m_Serial.cancel();
    m_Timer.cancel();

    // notify that there is action (main loop will so react on stop)
    m_actionCond.notify_all();

    // wait until io service thread stops
    m_ioService.stop();
    m_ioThread->join();
    m_ioThread.reset();

    // wait until ibus thread stops
    m_iBusThread->join();
    m_iBusThread.reset();
}

//--------------------------------------------------------------------------
void IBus::prepareReceive(bool startTimer)
{
    m_State = IDLE;

    // start timeout timer, to execute if reading was unsuccessfull
    if (startTimer)
    {
        m_State = RECEIVE;

        m_Timer.cancel();
        m_Timer.expires_from_now(boost::posix_time::microseconds(IBUS_RX_TIMEOUT));
        m_Timer.async_wait(boost::bind(&IBus::timeOut, this, boost::asio::placeholders::error));
    }

    // start reading of bytes into the buffer of certain length
    boost::asio::async_read(m_Serial, boost::asio::buffer(boost::asio::buffer(m_buffer) + m_bufferMsgPos, 1),
            boost::bind(&IBus::onReceive, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
}

//--------------------------------------------------------------------------
void IBus::onReceive(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    // on cancel operation, do nothing
    if (error == boost::asio::error::operation_aborted || bytes_transferred == 0)
        return;
    else if (error)
    {
        LOG_ERROR(std::string("There was an error while receiving bytes: ") + boost::lexical_cast<std::string>(error));
        stop();
        return;
    }
    m_Timer.cancel();
    
    assert(bytes_transferred == 1);

    if (m_bufferMsgPos == 1)
        m_bufferMsgLength = static_cast<std::size_t>(m_buffer[1]);

    m_bufferMsgPos += bytes_transferred;

    // check if we have reached the end of ibus frame
    if (m_bufferMsgPos >= m_bufferMsgLength + 2)
    {
        Message* msg = NULL;
        try
        {
            msg = new Message(&m_buffer[0], m_bufferMsgLength + 2);
            // if this was echo, then ignore
            if (m_lastSendMessage != *msg)
            {
                if (Logger::instance()->getLevel() != Logger::RAW)
                    LOG_INFO(std::string("ibus: Received: ") + msg->rawToString());
                else
                    LOG_RAW(msg->rawToString());

                // check if there is an event to react on this message, execute it
                for (EventList::const_iterator it = m_systemEventList.begin(); it != m_systemEventList.end(); it++)
                    if (it->first == *msg)
                    {
                        int res = system(it->second.c_str());
                        if (res)
                            LOG_WARN(std::string("ibus: SystemEvent returned error code ") + boost::lexical_cast<std::string>(res));
                    }
            }
        }catch(IBus::Error& err)
        {
            LOG_WARN(std::string("ibus: Receive failed: ") + err.asString());

            if (msg) delete msg;
            msg = NULL;
        }

        m_bufferMsgLength = 0;
        m_bufferMsgPos = 0;

        // start receiving of message again (so that we do not miss anything)
        prepareReceive(false);

        // if received message is ok, then put message into the rx queue
        if (msg)
        {
            boost::mutex::scoped_lock(m_rxQueueMutex);
            m_RxQueue.push(msg);
            
            // indicate main thread, that something new happens
            m_actionCond.notify_all();
        }
        
    // go into receive mode, awaiting for new input
    }else
        prepareReceive(true);
}

//--------------------------------------------------------------------------
void IBus::run_ioservice()
{
    LOG_DEBUG("ibus: Start I/O-Service Thread (serial port communication");

    // execute while we are running
    while(m_running)
    {
        try{
            m_ioService.reset();
            m_ioService.run();
        }catch(...)
        {
            LOG_ERROR("Unhandled error from IO-Service !");
        }
    }

    LOG_DEBUG("ibus: Stop I/O-Service Thread");
}

//--------------------------------------------------------------------------
void IBus::run_ibus()
{
    boost::mutex actionCondMutex;

    // start into receive mode
    prepareReceive(false);

    LOG_DEBUG("ibus: Start main thread");
    
    // repeat until we stop it
    boost::mutex::scoped_lock lock(actionCondMutex);    
    while (m_running)
    {
        // wait as long as there is no action (do only if there is nothing in the receive buffer)
        if (m_RxQueue.size() == 0 && m_TxQueue.size() == 0)
            m_actionCond.wait(lock);

        // if we are currently idling and there are messages to submit, then do so
        if (m_State == IDLE && m_TxQueue.size())
        {
            // get message from queue before anything else, this reduce the delay between
            // getCTS check and send possible operation
            m_txQueueMutex.lock();
              Message* msg = m_TxQueue.top();
              m_TxQueue.pop();
            m_txQueueMutex.unlock();

            // if message is expired
            if (msg->getTrials() == 0)
            {
                LOG_ERROR(std::string("ibus: Transmit FAILED: id=") + boost::lexical_cast<std::string>(msg->getID())
                        + std::string(" [p=") + boost::lexical_cast<std::string>(msg->getPriority()) + std::string("]")
                        + std::string(" -> ") + msg->rawToString());

                delete msg;
                continue;
            }

            // if bus is busy, then wait
            if (!getCTS())
            {
                // we need to push the message back into the queue
                m_txQueueMutex.lock();
                  m_TxQueue.push(msg);
                m_txQueueMutex.unlock();

                m_State = WAIT;
                timeWait(IBUS_TIMEOUT_WAIT_FREE_BUS);
                continue;
            }

            // decrement trial counter
            msg->setTrials(msg->getTrials()-1);

            std::vector<unsigned char> buffer = msg->getRaw();

            // stop receiving
            m_Serial.cancel();
            m_Timer.cancel();
            //extserial::receive_mode receiveOption(false);
            //m_Serial.set_option(receiveOption);
            //receiveOption.set(true);

            m_State = TRANSMIT;

            // send bytewise and check if status of the control line changes
            // if changes, then stop transmitting and wait until it get free again
            try
            {
                if (!getCTS()) throw boost::asio::error::try_again;
                
                // perform block write of full buffer
                boost::asio::write(m_Serial, boost::asio::buffer(buffer, buffer.size()), boost::asio::transfer_all());

                if (!getCTS()) throw boost::asio::error::try_again;
                m_Serial.drain();
                m_Serial.flush(true,false);
                
                // reactivate receive mode
                //m_Serial.set_option(receiveOption);
                m_lastSendMessage = *msg;

                // start wait timer after successfull msg send, only if we have messages in the send queue
                // otherwise go immediately into receive mode
                if (m_TxQueue.size())
                    timeWait(IBUS_TIMEOUT_TX_AFTER_SEND);
                else
                    timeOut(boost::asio::error::eof);

                if (Logger::instance()->getLevel() != Logger::RAW)
                {
                    LOG_INFO(std::string("ibus: Transmitted: id=") + boost::lexical_cast<std::string>(msg->getID())
                            + std::string(" [p=") + boost::lexical_cast<std::string>(msg->getPriority()) + std::string("]")
                            + std::string(" -> ") + msg->rawToString());
                }else{
                    LOG_RAW(msg->rawToString());
                }

                // delete message to mark that everything went fine
                delete msg;
                msg = NULL;

            }catch(boost::asio::error::basic_errors& ec)
            {
                // wait random amount of time between specified bounds
                double rnd = double(rand()) / double(INT_MAX);
                unsigned wait = IBUS_TIMEOUT_TX_COLLISION_MIN + unsigned(rnd * double(IBUS_TIMEOUT_TX_COLLISION_MAX - IBUS_TIMEOUT_TX_COLLISION_MIN));
                timeWait(wait);
            }catch(boost::asio::error::misc_errors& ec)
            {
                // if we loose the connection, then inform
                if (ec == boost::asio::error::eof)
                {
                    LOG_ERROR(std::string("Connection was closed by the peer!"));
                    stop();
                }
            }

            // if message is still there, then something went wrong
            // we just put the message back into the queue then
            if (msg)
            {
                m_txQueueMutex.lock();
                m_TxQueue.push(msg);
                m_txQueueMutex.unlock();
            }
        }

        // pass received messages to the callback function (only in idle state)
        if(m_State == IDLE && m_RxQueue.size())
        {
            while (m_RxQueue.size())
            {
                m_rxQueueMutex.lock();
                Message* msg = m_RxQueue.front(); m_RxQueue.pop();
                m_rxQueueMutex.unlock();

                if (m_msgRxCallback) m_msgRxCallback(*msg);
                delete msg;
            }
        }

    }        
    LOG_DEBUG("ibus: Stop main thread");
    if (m_stopIBusThreadCallback) m_stopIBusThreadCallback();
}

//------------------------------------------------------------------------------
void IBus::timeWait(unsigned microseconds)
{
    m_State = WAIT;
    m_Timer.cancel();
    m_Timer.expires_from_now(boost::posix_time::microseconds(microseconds));
    m_Timer.async_wait(boost::bind(&IBus::timeOut, this, boost::asio::placeholders::error));
}

//------------------------------------------------------------------------------
bool IBus::getCTS()
{
    // no CTS then always indicate free bus
    if (mCTSType == NO) return true;
    
    // if hardware in use 
    if (mCTSType == HARDWARE)
    {
        extserial::cts_bit cts;
        m_Serial.get_bit(cts);
        return cts.value();
    }
    
    return true;
}

//------------------------------------------------------------------------------
void IBus::timeOut(const boost::system::error_code& error)
{
    // if timer was aborted, then do nothing
    if (error == boost::asio::error::operation_aborted)
        return;

    // on timeout we stop any serial operation (receiving, transmitting)
    if (m_Serial.is_open())
    {
        m_Serial.cancel();
    }

    // reset every value
    m_bufferMsgPos = 0;
    m_bufferMsgLength = 0;
    memset(m_buffer.data(), 0, sizeof(Data) * m_buffer.size());
    m_State = IDLE;

    // start into receive mode
    prepareReceive(false);

    // and notify thread's loop about the change
    m_actionCond.notify_all();
}

//------------------------------------------------------------------------------
bool IBus::open(const std::string& port, CTSType cts)
{
    try
    {
        LOG_DEBUG(std::string("Open serial port ") + port);
        
        m_Serial.open(port);

        if (!m_Serial.is_open()) throw boost::asio::error::connection_aborted;

        m_Serial.flush(true,true);

        boost::asio::serial_port_base::baud_rate baud(9600);
        boost::asio::serial_port_base::character_size csize(8);
        boost::asio::serial_port_base::parity parity(boost::asio::serial_port_base::parity::even);
        boost::asio::serial_port_base::stop_bits stop(boost::asio::serial_port_base::stop_bits::one);

        m_Serial.set_option(baud);
        m_Serial.set_option(csize);
        m_Serial.set_option(parity);
        m_Serial.set_option(stop);
 
        if (cts == HARDWARE)
        {
            boost::asio::serial_port_base::flow_control flow(boost::asio::serial_port_base::flow_control::hardware);
            m_Serial.set_option(flow);
        }
        
        mCTSType = cts;
        
    }catch(boost::system::error_code ec)
    {
        LOG_ERROR(std::string("Error when opening port: ") + boost::lexical_cast<std::string>(ec));
        return false;
    }catch(...)
    {
        LOG_ERROR(std::string("Error when opening port: unknown error"));
        return false;    
    }
    LOG_DEBUG(std::string("serial port ") + port + (" opened"));
    return true;
}


//------------------------------------------------------------------------------
// Message implementation
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
IBus::Message::Message(IBus::Data src, IBus::Data dst, IBus::Data data, int priority) :
        m_Trials(IBus::Message::defaultNumberOfTrials),
        m_Priority(priority), m_ID(IBus::NO_ID)
{
    m_raw.resize(5, 0);
    m_raw[0] = src;
    m_raw[1] = 3;
    m_raw[2] = dst;
    m_raw[3] = data;
    m_raw[4] = computeChecksum();
}

//------------------------------------------------------------------------------
IBus::Message::Message(const IBus::Message& msg) :
        m_raw(msg.m_raw),
        m_Trials(msg.m_Trials), m_Priority(msg.m_Priority), m_ID(msg.m_ID)
{
    
}

//------------------------------------------------------------------------------
IBus::Message& IBus::Message::operator=(const Message& msg)
{
    m_raw = msg.m_raw;
    m_Trials = msg.m_Trials;
    m_Priority = msg.m_Priority;
    m_ID = msg.m_ID;

    return *this;
}

//------------------------------------------------------------------------------
IBus::Message::Message(IBus::Data* buffer, unsigned size)
{
    // check if this is a valid message
    if (size < 5) throw MessageHasBadLength();
    if (unsigned(buffer[1]+2) != size) throw MessageNotWellFormed();

    m_raw.resize(size, 0);
    memcpy(&m_raw[0], &buffer[0], size);
    if (computeChecksum() != buffer[size-1]) throw MessageHasBadChecksum();

    // defaults
    m_Trials = defaultNumberOfTrials;
    m_Priority = defaultPriority;
    m_ID = IBus::NO_ID;
}

//------------------------------------------------------------------------------
std::vector<IBus::Data> IBus::Message::getData() const
{
    if (m_raw.size() < 5) throw MessageHasBadLength();
    
    std::vector<unsigned char> buffer(m_raw.size()-4, 0);
    memcpy(&buffer[0], &m_raw[3], buffer.size());

    return buffer;
}

//------------------------------------------------------------------------------
void IBus::Message::setData(const std::vector<IBus::Data>& data)
{
    if (data.size() == 0) throw MessageNotWellFormed();
    if (m_raw.size() < 5) throw MessageHasBadLength();

    std::vector<Data> buffer = m_raw;
    m_raw.resize(data.size() + 4, 0);

    m_raw[0] = buffer[0];
    m_raw[1] = m_raw.size() - 2;
    m_raw[2] = buffer[2];
    memcpy(&m_raw[3], &data[0], data.size());
    m_raw[m_raw.size()-1] = computeChecksum();
}

//------------------------------------------------------------------------------
std::string IBus::Message::rawToString()
{
    std::string str;
    std::stringstream stream(str);
    
    stream << std::hex << std::uppercase << std::setfill('0');
    for (unsigned i=0; i < m_raw.size(); i++)
        stream << std::setw(2) << (int)m_raw[i] << " ";

    return stream.str();
}

//------------------------------------------------------------------------------
IBus::Data IBus::Message::computeChecksum()
{
    unsigned char checksum = 0;

    for(unsigned i = 0; i < m_raw.size()-1; i++)
        checksum ^= m_raw[i];

    return (IBus::Data)checksum;
}

template <typename ElemT>
struct HexTo {
    ElemT value;
    operator ElemT() const {return static_cast<ElemT>(value);}
    friend std::istream& operator>>(std::istream& in, HexTo& out)
    {
        in >> std::hex >> out.value;
        return in;
    }
};

//------------------------------------------------------------------------------
IBus::Message* IBus::Message::parseFromSDMString(const std::string& str)
{
    // parse given message
    if (str.length())
    {
        std::vector<std::string> strs;
        boost::split(strs, str, boost::is_any_of("\t ,.|:;"));
        if (strs.size() < 3)
            return NULL;

        try {
            unsigned char src = boost::lexical_cast<HexTo<unsigned int> >(strs[0]);
            unsigned char dst = boost::lexical_cast<HexTo<unsigned int> >(strs[1]);
            std::vector<unsigned char> data;
            for (unsigned int i=2; i < strs.size(); i++)
                data.push_back(boost::lexical_cast<HexTo<unsigned int> >(strs[i]));

            Message* msg = new Message(src, dst, 0);
            msg->setData(data);

            return msg;
        } catch(boost::bad_lexical_cast &) {
            return NULL;
        }
    }

    return NULL;
}
