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

#ifndef _APPLICATION_CLIENT_H
#define	_APPLICATION_CLIENT_H

#include "Application.h"


/**
 * Class responsible for communicating of one client with the oepnbm-gateway
 **/
class ClientConnection
{
protected:
    openbm::MessageHeader _receiveHeader;
    std::vector<unsigned char> _receiveBuffer;

    void start_accept();
    void handle_accept(const boost::system::error_code& error);
    void handle_accept_timeout(const boost::system::error_code& error);

    void start_receive();
    void handle_receive_header(const boost::system::error_code& error, const size_t& received);
    void handle_receive_data(const boost::system::error_code& error, const size_t& received);
    virtual void handle_timeout(const boost::system::error_code& error);

    void start_upload();

    void disconnect();
    void run();

    boost::asio::io_service _IOservice;
    boost::asio::ip::tcp::socket _Socket;
    boost::asio::ip::tcp::acceptor _Acceptor;
    boost::shared_ptr<boost::thread> _Thread;
    boost::mutex _Mutex;
    boost::asio::deadline_timer _Timer;

    Application* _Parent;
    unsigned _ID;
    unsigned _AliveTimeout;
    bool running;
    bool connected;
    unsigned short port;

public:

    ClientConnection(Application* parent, unsigned id, const std::string& address, unsigned short port, unsigned alive);
    virtual ~ClientConnection();

    //! Start connection to client (this method starts a thread where connection is handled)
    virtual void start();

    //! Force stop of the connection to client (connection thread must be stopped/killed here)
    virtual void stop();

    //! Transfer received ibus message with the underlying protocol
    virtual void onReceive(const IBus::Message& imsg);

    //! Get client id
    unsigned getID() const { return _ID; }

    //! Get clients port
    unsigned short getPort() const { return port; }

    //! Is current client connected
    bool isConnected() const { return connected; }

    //! Set amount of ms to check for reaction of connected client (if nothing new, then close connection)
    void setAliveTimeout(unsigned timeout);
};
    

/**
 * This special client can upload the firmare to an OpenBM device.
 **/
class FirmwareUploadClient : public ClientConnection
{
private:
    typedef enum _FirmwareUploadState
    {
        NO,
        PREPARE,
        INITIATED,
        SEND_PWD,
        SEND_INIT,
        UPLOADING
    }FirmwareUploadState;
    FirmwareUploadState m_FirmwareUploadState;

    std::vector<unsigned char> m_Firmware;
    unsigned char m_FirmwarePwd[4];
    unsigned m_FirmwareNextChunk;
    unsigned m_FirmwareNumChunks;
    unsigned m_FirmwareChunkSize;
    unsigned char m_FirmwareLastXoredSeed;
    boost::asio::deadline_timer m_FirmwareTimer;
    unsigned m_FirmwareLastWrongChunkNumber;
    unsigned char m_FirmwareCurrentVersion[2];
    unsigned m_FirmwareMaxChunkSize;
    unsigned char m_FirmwareSeed;
    unsigned char m_FirmwareNewVersion[2];
    unsigned char m_FirmwareNewChecksum[2];

    virtual void handle_timeout(const boost::system::error_code& error);
    virtual void handle_firmwareupdate_timeout(const boost::system::error_code& error);

public:

    FirmwareUploadClient(Application* parent, unsigned id, const std::string& address, unsigned short port, unsigned alive);
    virtual ~FirmwareUploadClient();

    void prepareFirmwareUpload(const std::vector<unsigned char>& data, unsigned char pwd[4]);

    virtual void onReceive(const IBus::Message& imsg);

};

#endif	/* _APPLICATION_CLIENT_H */

