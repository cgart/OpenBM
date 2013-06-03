/*
 *   Copyright 2011 Art Tevs <art@tevs.eu>
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
#include "ApplicationClient.h"
#include <iostream>
#include <string>
#include <functional>
#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

using namespace std;
using namespace boost::asio;

//------------------------------------------------------------------------------
ClientConnection::ClientConnection(Application* parent, unsigned id, const std::string& address, unsigned short port, unsigned alive) :
    _receiveHeader(0,0,0,0),
    _Socket(_IOservice),
    _Acceptor(_IOservice, boost::asio::ip::tcp::endpoint( ip::address_v4::from_string(address), port)),
    _Timer(_IOservice),
    _Parent(parent),
    _ID(id),
    _AliveTimeout(alive),
    running(false),
    connected(false),
    port(port)
{
}

//------------------------------------------------------------------------------
ClientConnection::~ClientConnection()
{
    stop();
}

//------------------------------------------------------------------------------
void ClientConnection::start()
{
    LOG_DEBUG(std::string("Session ") + boost::lexical_cast<std::string>(getID()) + string(":") + boost::lexical_cast<string>(getPort()) + string(" awaiting connection"));
    
    _Thread.reset(new boost::thread(boost::bind(&ClientConnection::run, this)));
    running = true;
}

//------------------------------------------------------------------------------
void ClientConnection::handle_accept(const boost::system::error_code& error)
{
    if (error)
    {
        _Parent->closeClientConnection(this);
    }else
    {
        setAliveTimeout(_AliveTimeout);

        connected = true;
        
        LOG_DEBUG(std::string("Session ") + boost::lexical_cast<std::string>(getID()) + string(":") + boost::lexical_cast<string>(getPort()) + string(" client connected, start communication"));

        start_receive();
    }
}

//------------------------------------------------------------------------------
void ClientConnection::handle_accept_timeout(const boost::system::error_code& error)
{
    if (error == boost::asio::error::operation_aborted) return;

    LOG_WARN(std::string("Session ") + boost::lexical_cast<std::string>(getID()) + string(":") + boost::lexical_cast<string>(getPort()) + string(" accept timeout failed"));

    _Parent->closeClientConnection(this);
}

//------------------------------------------------------------------------------
void ClientConnection::run()
{
    // start acceptor and timer to react when connection is not accepted within certain time period
    _Acceptor.async_accept(_Socket, boost::bind(&ClientConnection::handle_accept, this, boost::asio::placeholders::error));

    _Timer.cancel();
    _Timer.expires_from_now(boost::posix_time::milliseconds(_AliveTimeout));
    _Timer.async_wait(boost::bind(&ClientConnection::handle_accept_timeout, this, placeholders::error));

    while(running)
    {
        try
        {
            _IOservice.reset();
            _IOservice.run();
        }catch(const boost::system::error_code& error)
        {
            LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": Error from IO-Service: ") + error.message());
        }catch( const std::exception &e)
        {
            LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": Error from IO-Service: ") + e.what());
        }catch(...)
        {
            LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": unknown error!"));
        }
    }

}

//------------------------------------------------------------------------------
void ClientConnection::start_receive()
{
    memset(&_receiveHeader, 0, sizeof(_receiveHeader));
    _receiveBuffer.clear();

    async_read(_Socket, buffer(&_receiveHeader, sizeof(_receiveHeader)),
                    boost::bind(&ClientConnection::handle_receive_header,
                    this, placeholders::error, placeholders::bytes_transferred));
}

//------------------------------------------------------------------------------
void ClientConnection::handle_receive_header(const boost::system::error_code& error, const size_t& received)
{
    boost::mutex::scoped_lock lock(_Mutex);

    if (error == boost::asio::error::operation_aborted || received == 0) return;

    if (error)
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": receive header error - ") + boost::lexical_cast<string>(error));
        start_receive();
        return;
    }
    
    // usual header
    if (received == sizeof(_receiveHeader))
    {        
        // we received ping message, then send ping back
        if (openbm::PingMessage::isPingMessage(_receiveHeader))
        {
            // echo ping back
            openbm::PingMessage msg;
            boost::system::error_code ec;
            write(_Socket, buffer(&msg, sizeof(msg)), transfer_all(), ec);
            if (ec)
                LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID())  + string(":") + boost::lexical_cast<string>(getPort()) + string(" error while echoing ping."));

            setAliveTimeout(_AliveTimeout);

            // restart receiver
            start_receive();
            return;
        }

        // we received disconnect message
        if (openbm::DisconnectMessage::isDisconnectMessage(_receiveHeader))
        {
            disconnect();
            _Parent->closeClientConnection(this);
            return;
        }

        // check for firmware version
        if (openbm::GetFirmwareVersionMessage::isGetFirmwareVersionMessage(_receiveHeader))
        {
            std::vector<unsigned char> data;
            data.push_back(IBUS_MSG_OPENBM_TO);
            data.push_back(IBUS_MSG_OPENBM_GET_VERSION);
            IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0, 0);
            imsg->setData(data);
            _Parent->sendIBusMessage(imsg);

            start_receive();
            return;
        }

        // initiate firmware upload TODO implement loading firmware from any client connection over TCP
        /*
        if (openbm::InitFirmwareUpdateMessage::isInitFirmwareUpdateMessage(_receiveHeader))
        {
            // parse password out
            unsigned char pwd[4] = {_receiveHeader.priority >> 8,
                                    _receiveHeader.priority & 0xFF,
                                    _receiveHeader.reserved2,
                                    _receiveHeader.reserved3};
            
            _Parent->initiateFirmwareUpload()

            start_receive();
            return;
        }*/

        // if other kind of control message, then do nothing
        if (openbm::ControlMessage::isControlMessage(_receiveHeader) || _receiveHeader.len == 0)
        {
            start_receive();
            return;
        }

        // based on the received length, receive now amount of data bytes
        _receiveBuffer.clear();
        _receiveBuffer.resize(_receiveHeader.len, 0);

        async_read(_Socket, buffer(&_receiveBuffer[0], _receiveHeader.len),
                        boost::bind(&ClientConnection::handle_receive_data,
                        this, placeholders::error, placeholders::bytes_transferred));
    }else
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": receive wrong header length."));
        start_receive();
    }
}

//------------------------------------------------------------------------------
void ClientConnection::handle_receive_data(const boost::system::error_code& error, const size_t& received)
{
    boost::mutex::scoped_lock lock(_Mutex);

    if (error == boost::asio::error::operation_aborted || received == 0) return;

    if (error)
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": receive data error - ") + boost::lexical_cast<string>(error));
        start_receive();
        return;
    }

    if (received == _receiveHeader.len)
    {        
        // convert received message to an ibus message and send it
        IBus::Message* imsg = new IBus::Message(_receiveHeader.src, _receiveHeader.dst, 0, _receiveHeader.priority);
        imsg->setData(_receiveBuffer);

        _Parent->sendIBusMessage(imsg);
        //delete imsg;

    }else
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID()) + string(": receive wrong data length."));
    }

    start_receive();
}

//------------------------------------------------------------------------------
void ClientConnection::disconnect()
{
    if (!connected) return;
    if (_Socket.is_open())
    {
        LOG_DEBUG(string("Disconnect session ") + boost::lexical_cast<string>(getID()) + string(":") + boost::lexical_cast<string>(getPort()));

        _IOservice.post(boost::bind(&boost::asio::ip::tcp::socket::shutdown, &_Socket, boost::asio::ip::tcp::socket::shutdown_both ));
        _IOservice.post(boost::bind(&boost::asio::ip::tcp::socket::close, &_Socket));
    }
    connected = false;
}

//------------------------------------------------------------------------------
void ClientConnection::stop()
{
    if (!running) return;

    boost::mutex::scoped_lock lock(_Mutex);
        
    LOG_DEBUG(string("Stop session ") + boost::lexical_cast<string>(getID()) + string(":") + boost::lexical_cast<string>(getPort()));
    
    disconnect();

    running = false;
    _IOservice.post(boost::bind(&boost::asio::io_service::stop, &_IOservice));

    if (_Thread) _Thread->join();
    _Thread.reset();
}


//------------------------------------------------------------------------------
void ClientConnection::onReceive(const IBus::Message& imsg)
{
    const std::vector<unsigned char>& data = imsg.getData();

    // non-controll message received
    openbm::MessageHeader header(imsg.getSource(), imsg.getDestination(), imsg.getPriority(), data.size());

    // first transmit header
    boost::system::error_code ec;
    write(_Socket, buffer(&header, sizeof(header)), transfer_all(), ec);
    if (ec)
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID())  + string(":") + boost::lexical_cast<string>(getPort()) + string(": error while transmit header - ") + ec.message());
        return;
    }
    write(_Socket, buffer(&data[0], data.size()), transfer_all(), ec);
    if (ec)
    {
        LOG_ERROR(string("Session ") + boost::lexical_cast<string>(getID())  + string(":") + boost::lexical_cast<string>(getPort()) + string(": error while transmit data - ") + ec.message());
        return;
    }
}

//------------------------------------------------------------------------------
void ClientConnection::handle_timeout(const boost::system::error_code& error)
{
    // if timer canceled or we haven't received until now anything
    if (error == boost::asio::error::operation_aborted) return;

    // inform main application about close of the connection
    _Parent->closeClientConnection(this);
}

//------------------------------------------------------------------------------
void ClientConnection::setAliveTimeout(unsigned timeout)
{
    _Timer.cancel();
    if (timeout > 0)
    {
        _Timer.expires_from_now(boost::posix_time::milliseconds(timeout));
        _Timer.async_wait(boost::bind(&ClientConnection::handle_timeout, this, placeholders::error));
    }
    _AliveTimeout = timeout;
}






//------------------------------------------------------------------------------
FirmwareUploadClient::FirmwareUploadClient(Application* parent, unsigned id, const std::string& address, unsigned short port, unsigned alive):
    ClientConnection(parent, id, address, port, alive),
    m_FirmwareTimer(_IOservice)
{
    m_FirmwareUploadState = NO;
    m_FirmwareNextChunk = 0;
    m_FirmwareNumChunks = 0;
    m_FirmwareChunkSize = 64;

}

FirmwareUploadClient::~FirmwareUploadClient()
{

}

//------------------------------------------------------------------------------
void FirmwareUploadClient::prepareFirmwareUpload(const std::vector<unsigned char>& data, unsigned char pwd[4])
{
    if (data.size() <= 10)
    {
        LOG_ERROR("FirmwareUploadClient: cannot upload firmware, because it contains no valid data.");
        return;
    }

    m_FirmwareChunkSize = 64;

    // get version information from ROM-File
    if (data[0] == 'O' && data[1] == 'B' && data[2] == 'M' && data[3] == 1)
    {
        memcpy(&m_FirmwareChunkSize, &data[4], sizeof(unsigned short));
        m_FirmwareNewVersion[0] = data[6];
        m_FirmwareNewVersion[1] = data[7];
        m_FirmwareNumChunks = (data.size() - 10) / m_FirmwareChunkSize;
        m_FirmwareNewChecksum[0] = data[data.size()-2];
        m_FirmwareNewChecksum[1] = data[data.size()-1];
        m_Firmware.resize(data.size() - 10, 0xFF);
        memcpy(&m_Firmware[0], &data[8], m_Firmware.size());
    }else
    {
        LOG_ERROR("FirmwareUploadClient: No valid/supported firmware file given.");
        return;
    }

    m_FirmwareNextChunk = 0;
    m_FirmwarePwd[0] = pwd[0];
    m_FirmwarePwd[1] = pwd[1];
    m_FirmwarePwd[2] = pwd[2];
    m_FirmwarePwd[3] = pwd[3];

    m_FirmwareUploadState = PREPARE;

    LOG_INFO(std::string("FirmwareUploadClient: Upload firmware: v") + boost::lexical_cast<string>((int)m_FirmwareNewVersion[0])
            +string(".") + boost::lexical_cast<string>((int)m_FirmwareNewVersion[1])
            +string(" crc16=")+ boost::lexical_cast<string>((int)m_FirmwareNewChecksum[0])
            +string(":")+ boost::lexical_cast<string>((int)m_FirmwareNewChecksum[1])
            +string(" chunk=") + boost::lexical_cast<string>((int) m_FirmwareNumChunks)
            +string("[") + boost::lexical_cast<string>((int) m_FirmwareChunkSize) + string("]"));


    // reset OpenBM unit, so that we can start uploading soon
    {
        // reset OpenBM unit
        std::vector<unsigned char> dd;
        dd.push_back(IBUS_MSG_OPENBM_TO);
        dd.push_back(0xBA);
        dd.push_back(0xAD);
        dd.push_back(0xFE);
        dd.push_back(0xED);
        IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0, 0);
        imsg->setData(dd);
        _Parent->sendIBusMessage(imsg);
    }

}

//------------------------------------------------------------------------------
void FirmwareUploadClient::handle_firmwareupdate_timeout(const boost::system::error_code& error)
{
    if (error == boost::asio::error::operation_aborted) return;

    m_FirmwareTimer.cancel();

    // if we are waiting for acceptance of the password and timed-out, then
    // we assume that bootloader didn't get our message, so start procedure from
    // the beginning
    if (m_FirmwareUploadState == SEND_PWD)
    {
        LOG_DEBUG("FirmwareUploadClient: bootloader timed out when requesting on password");
        m_FirmwareUploadState = INITIATED;

        // emulate new message, so that we start our protocol accordingly
        std::vector<unsigned char> dd(9,0);
        dd[0] = IBUS_MSG_OPENBM_FROM;
        dd[1] = IBUS_MSG_OPENBM_SPECIAL_REQ;
        dd[2] = m_FirmwareCurrentVersion[0];
        dd[3] = m_FirmwareCurrentVersion[1];
        dd[4] = m_FirmwareMaxChunkSize;
        IBus::Message imsg(IBUS_DEV_BMBT, IBUS_DEV_LOC, 0);
        imsg.setData(dd);
        onReceive(imsg);
    }
    // if we are about to send chunks and we timed out, then we either
    // not received acknowledge or something else went wrong
    // retransmit last chunk then again
    else if (m_FirmwareUploadState == SEND_INIT || m_FirmwareUploadState == UPLOADING)
    {
        LOG_DEBUG("FirmwareUploadClient: bootloader timed out when acknwoledging a chunk, emulate force to resend last chunk");
        // emulate receive of wrong seed, this will enforce the protocol to run properly
        std::vector<unsigned char> dd(2,0);
        dd[0] = IBUS_MSG_OPENBM_FROM;
        dd[1] = ~m_FirmwareLastXoredSeed;
        IBus::Message imsg(IBUS_DEV_BMBT, IBUS_DEV_LOC, 0);
        imsg.setData(dd);
        onReceive(imsg);
    }
    // in all other cases just restart the update protocol
    else
    {
        m_FirmwareUploadState = PREPARE;
    }
}

//------------------------------------------------------------------------------
void FirmwareUploadClient::onReceive(const IBus::Message& imsg)
{
    //boost::mutex::scoped_lock lock(_Mutex);

    // -------------------------------------------------------------------------
    // Special treatment for server side IBus connection
    // -------------------------------------------------------------------------
    const std::vector<unsigned char>& data = imsg.getData();

    // if this are responses from the OpenBM device to our special messages then react appropriately
    if (imsg.getSource() == IBUS_DEV_BMBT && data.size() > 1 && data[0] == IBUS_MSG_OPENBM_FROM)
    {
        //openbm::MessageHeader* header = NULL;

        // if we receive some error code message from bootloader, then stop
        if (data.size() == 3 && data[1] > 0)
        {
            LOG_ERROR(std::string("Server: bootloader send error code EH=") + boost::lexical_cast<string>((int)data[1])
                    +std::string(", EL=") + boost::lexical_cast<string>((int)data[2]));
            if (m_Firmware.size() > 0)
            {
                m_FirmwareNextChunk = 0;
                m_FirmwareUploadState = PREPARE;
            }else
                m_FirmwareUploadState = NO;
        }
        // when bootloader is activated it sends info about used firmware
        // if we have previously initiate a firmware update, then catch this info and prepare for update
        else if ((m_FirmwareUploadState == INITIATED || m_FirmwareUploadState == PREPARE)  && data[1] == IBUS_MSG_OPENBM_SPECIAL_REQ && data.size() >= 9)
        {
            LOG_INFO("FirmwareUploadClient: received bootloader indicator message, send firmware password");

            m_FirmwareCurrentVersion[0] = data[2];
            m_FirmwareCurrentVersion[1] = data[3];
            m_FirmwareMaxChunkSize = data[4];
            if (data[4] < m_FirmwareChunkSize)
            {
                LOG_ERROR("FirmwareUploadClient: the hardware does not support firmware's chunk size, so cannot upload");\
                return;
            }

            // set bootloader into firmware update mode
            std::vector<unsigned char> dd(6);
            dd[0] = IBUS_MSG_OPENBM_TO;
            dd[1] = IBUS_MSG_OPENBM_SPECIAL_REQ;
            dd[2] = m_FirmwarePwd[0];
            dd[3] = m_FirmwarePwd[1];
            dd[4] = m_FirmwarePwd[2];
            dd[5] = m_FirmwarePwd[3];
            IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0);
            imsg->setTrials(255);
            imsg->setData(dd);

            _Parent->sendIBusMessage(imsg);
            m_FirmwareUploadState = SEND_PWD;

            // we should get answer from bootloader in less than one second
            m_FirmwareTimer.cancel();
            m_FirmwareTimer.expires_from_now(boost::posix_time::milliseconds(1000));
            m_FirmwareTimer.async_wait(boost::bind(&FirmwareUploadClient::handle_firmwareupdate_timeout, this, placeholders::error));

        // when send password we should receive its xored sum back, we check if this correct
        // and if yes we go into firmware transmit mode
        }else if (m_FirmwareUploadState == SEND_PWD && data[1] == 0 && data.size() == 3)
        {
            srand ( time(NULL) );

            unsigned char xored = m_FirmwarePwd[0];
            xored ^= m_FirmwarePwd[1];
            xored ^= m_FirmwarePwd[2];
            xored ^= m_FirmwarePwd[3];

            if (xored != data[2])
            {
                LOG_ERROR(string("FirmwareUploadClient: initiating firmware update went wrong, received xored sum of password from OpenBM is incorrect."));
                m_FirmwareUploadState = NO;
                return;
            }
            LOG_INFO("FirmwareUploadClient: received bootloader prepare upload message, so start uploading firmware");

            // send now the init chunk, which has the overall information about following chunks
            std::vector<unsigned char> dd(7);
            dd[0] = IBUS_MSG_OPENBM_TO;
            dd[1] = IBUS_MSG_OPENBM_SPECIAL_REQ;
            dd[2] = m_FirmwareNewVersion[0];   // app version major
            dd[3] = m_FirmwareNewVersion[1];   // app version minor
            dd[4] = m_FirmwareNewChecksum[0];   // crc16 high
            dd[5] = m_FirmwareNewChecksum[1];   // crc16 low
            dd[6] = (unsigned char)(rand() & 0xFF);    // random seed
            IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0);
            imsg->setData(dd);
            imsg->setTrials(255);
            _Parent->sendIBusMessage(imsg);

            m_FirmwareSeed = dd[6];
            m_FirmwareLastXoredSeed = m_FirmwareSeed;
            m_FirmwareUploadState = SEND_INIT;
            m_FirmwareLastWrongChunkNumber = 0;

            // we should get answer from bootloader in less than one second
            m_FirmwareTimer.cancel();
            m_FirmwareTimer.expires_from_now(boost::posix_time::milliseconds(1000));
            m_FirmwareTimer.async_wait(boost::bind(&FirmwareUploadClient::handle_firmwareupdate_timeout, this, placeholders::error));

        }

        // if we have send an init chunk, we should receive the seed back
        else if ((m_FirmwareUploadState == SEND_INIT || m_FirmwareUploadState == UPLOADING) && data.size() == 2)
        {
            unsigned char chunkIndicator = 0x10;

            // returned seed is wrong
            if (data[1] != m_FirmwareLastXoredSeed)
            {
                // if this was init chunk, then just error message
                if (m_FirmwareUploadState == SEND_INIT)
                {
                    LOG_ERROR(string("FirmwareUploadClient: bootloader returned wrong seed number of the init chunk, received=") + boost::lexical_cast<string>((int)data[1])
                            + string(", shouldbe=") + boost::lexical_cast<string>((int)m_FirmwareLastXoredSeed));
                    m_FirmwareNextChunk = 0;
                    m_FirmwareUploadState = PREPARE;
                    return;

                // this was a regular chunk, then resend last chunk
                }else
                {
                    m_FirmwareLastWrongChunkNumber++;
                    if (m_FirmwareLastWrongChunkNumber > 20)
                    {
                        LOG_ERROR("FirmwareUploadClient: bootloader continuously acknwoledge the chunk wrong. I give up!!! Restart the uploading process!!!");
                        m_FirmwareNextChunk = 0;
                        m_FirmwareUploadState = PREPARE;
                        return;
                    }

                    LOG_DEBUG(string("FirmwareUploadClient: bootloader wrong seed, received=") + boost::lexical_cast<string>((int)data[1])
                            + string(", shouldbe=") + boost::lexical_cast<string>((int)m_FirmwareLastXoredSeed));
                    chunkIndicator = 0;
                }
            }else if (m_FirmwareUploadState == UPLOADING)
            {
                m_FirmwareLastWrongChunkNumber = 0;
                m_FirmwareNextChunk++;

                // if there are no more chunks, then send last chunk indicator
                if (m_FirmwareNextChunk >= m_FirmwareNumChunks)
                {
                    std::vector<unsigned char> dd(3);
                    dd[0] = IBUS_MSG_OPENBM_TO;
                    dd[1] = IBUS_MSG_OPENBM_SPECIAL_REQ;
                    dd[2] = 0xFF;      // special chunk indicator
                    IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0);
                    imsg->setData(dd);
                    imsg->setTrials(255);
                    _Parent->sendIBusMessage(imsg);

                    m_FirmwareUploadState = NO;
                    return;
                }
            }

            if (m_FirmwareUploadState == SEND_INIT)
                LOG_INFO("FirmwareUploadClient: init chunk was accepted, send now the firmware code");

            m_FirmwareUploadState = UPLOADING;

            // upload chunk
            std::vector<unsigned char> dd(m_FirmwareChunkSize+3);
            dd[0] = IBUS_MSG_OPENBM_TO;
            dd[1] = IBUS_MSG_OPENBM_SPECIAL_REQ;
            dd[2] = chunkIndicator;      // special chunk indicator
            m_FirmwareLastXoredSeed = m_FirmwareSeed;
            for (unsigned i=0; i < m_FirmwareChunkSize; i++)
            {
                dd[i+3] = m_Firmware[m_FirmwareNextChunk * m_FirmwareChunkSize + i];
                m_FirmwareLastXoredSeed ^= dd[i+3];
            }
            IBus::Message* imsg = new IBus::Message(IBUS_DEV_LOC, IBUS_DEV_BMBT, 0);
            imsg->setData(dd);
            imsg->setTrials(255);
            _Parent->sendIBusMessage(imsg);

            // we should get answer from bootloader in less than one second
            m_FirmwareTimer.cancel();
            m_FirmwareTimer.expires_from_now(boost::posix_time::milliseconds(1000));
            m_FirmwareTimer.async_wait(boost::bind(&FirmwareUploadClient::handle_firmwareupdate_timeout, this, placeholders::error));

        }

    }
}

//------------------------------------------------------------------------------
void FirmwareUploadClient::handle_timeout(const boost::system::error_code& error)
{
    if (error == boost::asio::error::operation_aborted) return;

    m_FirmwareUploadState = NO;

    ClientConnection::handle_timeout(error);
}
