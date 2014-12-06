/* 
 * File:   IPCClient.h
 * Author: tevs
 *
 * Created on August 26, 2010, 10:20 AM
 */

#ifndef _IPCCLIENT_H
#define	_IPCCLIENT_H

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <string>

#include "GatewayProtocol.h"

namespace IPCClient
{
    struct IBusMessage
    {
        IBusMessage() : src(0), dst(0), data() {}
        IBusMessage(unsigned char s, unsigned char d, const std::vector<unsigned char>& dt) : src(s), dst(d), data(dt) {}
        unsigned char src;
        unsigned char dst;
        std::vector<unsigned char> data;
    };

    enum ReceiveError
    {
        OK = 0,
        NOT_WELL_FORMED,
        DATA_MISSING,
        LENGTH_WRONG,
        WRONG_TYPE,
        NO_MEMORY,
        CONNECTION_ERROR,
        UNKNOWN_ERROR,
        RECEIVE_ERROR,
        TRANSMIT_ERROR
    };

    /**
     * Interface class for ipc communication with the ibus server
     **/
    class Client
    {
    public:
        typedef void (*OnMessageCallback)(const IBusMessage&);
        typedef void (*OnReceiveErrorCallback)(ReceiveError, const std::string&);
        typedef void (*OnDisconnectCallback)();

        Client() : onMessageCb(NULL), onReceiveErrorCb(NULL), onDisconnectCb(NULL) {};
        virtual ~Client() {};

        inline void setOnMessageCallback(OnMessageCallback cb) { onMessageCb = cb; }
        inline void setOnReceiveErrorCallback(OnReceiveErrorCallback cb) { onReceiveErrorCb = cb; }
        inline void setOnDisconnectCallback(OnDisconnectCallback cb) { onDisconnectCb = cb; }
        
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual bool send(const IBusMessage& msg) = 0;

        inline bool isConnected() const { return connected; }
        
    protected:
        bool connected;

        OnMessageCallback onMessageCb;
        OnReceiveErrorCallback onReceiveErrorCb;
        OnDisconnectCallback onDisconnectCb;
    };

    /**
     * Implementation of ibus client based on shared memory
     **/
    class TCPIPClient : public Client
    {
    public:
        TCPIPClient(const std::string& address = "127.0.0.1", unsigned short port = 4287, unsigned pingTime = 3000, unsigned pingRetries = 1);
        ~TCPIPClient();

        void start();
        void stop();
        bool send(const IBusMessage& msg);

    private:
        void run();

        void handle_receive_header(const boost::system::error_code& error, const size_t& received);
        void handle_receive_data(const boost::system::error_code& error, const size_t& received);
        void send_close_connection();
        void start_receive();
        void handle_onDisconnect();
        void send_ping();
        void handle_ping_timeout(const boost::system::error_code& error);
        
        openbm::MessageHeader _receiveHeader;
        std::vector<unsigned char> _receiveBuffer;

        bool running;
        
        boost::asio::io_service _IOservice;
        boost::asio::ip::tcp::socket _Socket;
        boost::shared_ptr<boost::thread> _Thread;
        boost::mutex _Mutex;
        boost::asio::deadline_timer _Timer;

        const std::string address;
        unsigned short port;
        const unsigned pingTimeout;
        const unsigned pingRetries;
        int sentPings;
    };

}; // enbd namespace



#endif	/* _IPCCLIENT_H */

