#include "IPCClient.h"
#include <iostream>
#include <boost/lexical_cast.hpp>

namespace IPCClient
{
    using namespace std;
    using namespace boost::asio;
    using namespace boost::asio::ip;

    //------------------------------------------------------------------------------
    TCPIPClient::TCPIPClient(const std::string& _address, unsigned short _port, unsigned pingTime, unsigned pingRetries):
        _receiveHeader(0,0,0,0),
        running(false),
        _Socket(_IOservice),
        _Timer(_IOservice),
        address(_address),
        port(_port),
        pingTimeout(pingTime),
        pingRetries(pingRetries)
    {
        sentPings = 0;
    }

    //------------------------------------------------------------------------------
    TCPIPClient::~TCPIPClient()
    {
        stop();
    }

    //------------------------------------------------------------------------------
    void set_result(boost::optional<boost::system::error_code>* a, boost::system::error_code b)
    {
        a->reset(b);
    }

    //------------------------------------------------------------------------------
    template <typename MutableBufferSequence>
    void read_with_timeout(tcp::socket& sock, const MutableBufferSequence& buffers, unsigned timeoutInMs)
    {
        boost::optional<boost::system::error_code> timer_result;
        boost::asio::deadline_timer timer(sock.get_io_service());
        timer.expires_from_now(boost::posix_time::milliseconds(timeoutInMs));
        timer.async_wait(boost::bind(set_result, &timer_result, _1));

        boost::optional<boost::system::error_code> read_result;
        async_read(sock, buffers, boost::bind(set_result, &read_result, _1));

        sock.get_io_service().reset();
        while (sock.get_io_service().run_one())
        {
          if (read_result)
            timer.cancel();
          else if (timer_result)
            sock.cancel();
        }

        if (*read_result)
          throw boost::system::system_error(*read_result);
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::start()
    {
        boost::mutex::scoped_lock lock(_Mutex);

        if (running) return;
        
        connected = false;

        tcp::resolver resolver(_IOservice);
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(tcp::resolver::query(address, boost::lexical_cast<std::string>(port)));
        tcp::resolver::iterator end;

        boost::system::error_code error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end)
        {
            _Socket.close();
            _Socket.connect(*endpoint_iterator++, error);
        }
        if (error && onReceiveErrorCb)
        {
            _Socket.close();
            onReceiveErrorCb(CONNECTION_ERROR, std::string("Cannot find server at ") + address + string(":") + boost::lexical_cast<string>(port));
            return;
        }

        // now we would like to ask server to give us port number to communicate on
        // so send hello message and we expect to receive port number or disconnect message
        {
            openbm::HelloMessage msg;
            port = 0;

            try
            {
                write(_Socket, buffer(&msg, sizeof(msg)), transfer_all(), error);
                if (error && onReceiveErrorCb)
                {
                    _Socket.close();
                    onReceiveErrorCb(CONNECTION_ERROR, std::string("Error while transmitting HELLO message"));
                    return;
                }

                read_with_timeout(_Socket, buffer(&_receiveHeader, sizeof(_receiveHeader)), pingTimeout);
                port = openbm::ConnectToMessage::isConnectToMessage(_receiveHeader);
            }catch(...)
            {
                _Socket.close();
                onReceiveErrorCb(CONNECTION_ERROR, std::string("Error while receiving Connect message"));
                return;
            }

            if (!port || openbm::DisconnectMessage::isDisconnectMessage(_receiveHeader))
            {
                _Socket.close();
                if (onReceiveErrorCb) onReceiveErrorCb(CONNECTION_ERROR, std::string("Server is busy!"));
                return;
            }
        }

        // reconnect socket on the assigned port
        endpoint_iterator = resolver.resolve(tcp::resolver::query(address, boost::lexical_cast<std::string>(port)));
        error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end)
        {
            _Socket.close();
            _Socket.connect(*endpoint_iterator++, error);
        }
        if (error && onReceiveErrorCb)
        {
            _Socket.close();
            onReceiveErrorCb(CONNECTION_ERROR, std::string("Cannot connect to server at ") + address + string(":") + boost::lexical_cast<string>(port));
            return;
        }

        connected = true;

        _Thread.reset(new boost::thread(boost::bind(&TCPIPClient::run, this)));
        running = true;
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::run()
    {
        start_receive();
        send_ping();
        
        while(running)
        {
            try
            {
                _IOservice.reset();
                _IOservice.run();
            }catch(...)
            {
                _IOservice.post(boost::bind(&TCPIPClient::handle_onDisconnect, this));
            }
        }

    }

    //------------------------------------------------------------------------------
    void TCPIPClient::stop()
    {
        boost::mutex::scoped_lock lock(_Mutex);
        
        if (running == false) return;
        running = false;

        if (_Socket.is_open())
        {
            send_close_connection();
            _IOservice.post(boost::bind(&boost::asio::ip::tcp::socket::shutdown, &_Socket, boost::asio::ip::tcp::socket::shutdown_both ));
            _IOservice.post(boost::bind(&boost::asio::ip::tcp::socket::close, &_Socket));
        }
        connected = false;

        _IOservice.post(boost::bind(&boost::asio::io_service::stop, &_IOservice));
        _Thread->join();
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::start_receive()
    {
        memset(&_receiveHeader, 0, sizeof(_receiveHeader));
        _receiveBuffer.clear();

        async_read(_Socket, buffer(&_receiveHeader, sizeof(_receiveHeader)),
                        boost::bind(&TCPIPClient::handle_receive_header,
                        this, placeholders::error, placeholders::bytes_transferred));
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::handle_onDisconnect()
    {
        if (onDisconnectCb)
            onDisconnectCb();

        stop();
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::handle_receive_header(const boost::system::error_code& error, const size_t& received)
    {
        boost::mutex::scoped_lock lock(_Mutex);

        if (error == boost::asio::error::operation_aborted || received == 0) return;

        if (error)
        {
            if (onReceiveErrorCb) onReceiveErrorCb(RECEIVE_ERROR, std::string("Error while receiving header: ") + boost::lexical_cast<string>(error));
            start_receive();
            return;
        }

        if (received == sizeof(_receiveHeader))
        {
            // ping ok, then restart ping timer
            if (openbm::PingMessage::isPingMessage(_receiveHeader))
            {
                send_ping();
                start_receive();
                return;
            }

            // disconnect message, ok then close connection
            if (openbm::DisconnectMessage::isDisconnectMessage(_receiveHeader))
            {
                _IOservice.post(boost::bind(&TCPIPClient::handle_onDisconnect, this));
                return;
            }

            // ignore other kind of control messages
            if (openbm::ControlMessage::isControlMessage(_receiveHeader) || _receiveHeader.len == 0)
            {
                start_receive();
                return;
            }

            // based on the received length, receive now amount of data bytes
            _receiveBuffer.clear();
            _receiveBuffer.resize(_receiveHeader.len, 0);

            async_read(_Socket, buffer(&_receiveBuffer[0], _receiveBuffer.size()),
                            boost::bind(&TCPIPClient::handle_receive_data,
                            this, placeholders::error, placeholders::bytes_transferred));
        }else
        {
            if (onReceiveErrorCb) onReceiveErrorCb(RECEIVE_ERROR, std::string("Received wrong amount of header bytesr: ") + boost::lexical_cast<string>(received));
            start_receive();
        }
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::handle_receive_data(const boost::system::error_code& error, const size_t& received)
    {
        boost::mutex::scoped_lock lock(_Mutex);
        
        if (error == boost::asio::error::operation_aborted || received == 0) return;

        if (error)
        {
            if (onReceiveErrorCb) onReceiveErrorCb(RECEIVE_ERROR, std::string("Error while receiving data: ") + boost::lexical_cast<string>(error));
            start_receive();
            return;
        }

        if (received == _receiveHeader.len)
        {
            if (onMessageCb)
                onMessageCb(IBusMessage(_receiveHeader.src, _receiveHeader.dst, _receiveBuffer));
        }else
        {
            if (onReceiveErrorCb)
                onReceiveErrorCb(RECEIVE_ERROR, std::string("Wrong amount of data bytes received: ") + boost::lexical_cast<string>(received));
        }
        start_receive();
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::send_close_connection()
    {
        if (!connected) return;

        openbm::DisconnectMessage header;
        boost::system::error_code ec;
        write(_Socket, buffer(&header, sizeof(header)), transfer_all(), ec);
        if (ec)
            std::cerr << string("TCP-Client ") + string(": Error on write: ") + ec.message() << std::endl;
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::send_ping()
    {
        if (!connected) return;

        // start receive timeout
        sentPings = 0;
        _Timer.cancel();
        _Timer.expires_from_now(boost::posix_time::milliseconds(pingTimeout));
        _Timer.async_wait(boost::bind(&TCPIPClient::handle_ping_timeout, this, placeholders::error));
    }

    //------------------------------------------------------------------------------
    void TCPIPClient::handle_ping_timeout(const boost::system::error_code& error)
    {
        if (error == boost::asio::error::operation_aborted || !connected) return;
        
        // increment amount of fail retries, if reached already to much then queue for disconnect
        if (++sentPings > (int)pingRetries)
        {
            //if (onReceiveErrorCb)
            //    onReceiveErrorCb(CONNECTION_ERROR, std::string("Lost connection to gateway server"));

            _IOservice.post(boost::bind(&TCPIPClient::handle_onDisconnect, this));
            return;
        }

        // send ping message again
        try
        {
            boost::system::error_code ec;
            openbm::PingMessage header;
            write(_Socket, buffer(&header, sizeof(header)), transfer_all(), ec);
        }catch(...){}

        // start timer again
        _Timer.expires_from_now(boost::posix_time::milliseconds(pingTimeout));
        _Timer.async_wait(boost::bind(&TCPIPClient::handle_ping_timeout, this, placeholders::error));
    }

    //------------------------------------------------------------------------------
    bool TCPIPClient::send(const IBusMessage& imsg)
    {
        if (!connected)
        {
            if (onReceiveErrorCb)
                onReceiveErrorCb(RECEIVE_ERROR, std::string("Cannot send, not connected!"));
            return false;
        }

        try
        {
            openbm::MessageHeader header(imsg.src, imsg.dst, 100, imsg.data.size());

            boost::system::error_code ec;
            write(_Socket, buffer(&header, sizeof(header)), transfer_all(), ec);
            if (ec)
            {
                if (onReceiveErrorCb)
                    onReceiveErrorCb(TRANSMIT_ERROR, std::string("Error while sending header"));
                return false;
            }
            write(_Socket, buffer(&imsg.data[0], imsg.data.size()), transfer_all(), ec);
            if (ec)
            {
                if (onReceiveErrorCb)
                    onReceiveErrorCb(TRANSMIT_ERROR, std::string("Error while sending data bytes"));
                return false;
            }

        }catch(...)
        {
            if (onReceiveErrorCb)
                onReceiveErrorCb(UNKNOWN_ERROR, std::string("Unknown error received while transmitting data"));
            return false;
        }

        return true;

    }


};

