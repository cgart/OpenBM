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

#ifndef _APPLICATION_H
#define	_APPLICATION_H

#include "IBus.h"
#include "GatewayProtocol.h"
#include "mongoose.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>

class ClientConnection;

#define MAX_USER_LEN  32
#define MAX_SESSIONS 2
#define SESSION_TTL 600
#define WEB_IBUS_TIMEOUT 5000

/**
 * Main application class for handling TCP/IP connections
 **/
class Application
{
public:
    typedef void (*ReceiveMessageCallback)(const IBus::Message&);
    typedef void (*IBusThreadStopCallback)(void);
    
private:
    
    boost::asio::io_service _IOservice;
    boost::asio::ip::tcp::endpoint _Endpoint;
    boost::asio::ip::tcp::socket _Socket;
    boost::shared_ptr<boost::asio::ip::tcp::acceptor> _Acceptor;
    boost::asio::deadline_timer _Timer;
    boost::mutex _Mutex;
    boost::condition_variable _Cond;

    /**
     * Message filters which can be used to run callback method
     * for ibus messages matching the filter.
     **/
    class MsgFilter
    {
    public:
        MsgFilter() : src(-1), dst(-1), msg(-1), userdata(NULL), callback(NULL), remove(false){}
        short src;
        short dst;
        short msg;
        void* userdata;
        bool (*callback)(const IBus::Message&, MsgFilter*);
        bool remove;
    };
    boost::mutex _FilterMutex;
    MsgFilter m_msgFilter;
    //std::vector<MsgFilter*> m_msgFilters;

    std::vector<boost::shared_ptr<ClientConnection> > m_Client;
    std::vector<ClientConnection*> m_ClientToClose;
    std::map<unsigned short, bool> m_FreePort;
    
    unsigned short m_Port;
    const std::string m_Address;
    unsigned m_AliveTimeout;
    bool m_Running;
    bool m_useTCPClients;    
        
    IBusThreadStopCallback m_stopIBusThreadCallback;
    ReceiveMessageCallback m_ReceiveCallback;
    boost::shared_ptr<IBus> m_IBus;

    openbm::MessageHeader m_ReceiveHeader;

    void onStopIBusThread(void);
    void onReceiveIBusMessage(const IBus::Message& imsg);
    void runIOservice();
    void runLoop();

    void start_acceptor();
    void handle_accept(const boost::system::error_code& error);
    void handle_accept_read(const boost::system::error_code& error, const size_t& received);
    void handle_accept_timeout(const boost::system::error_code& error);
    void handle_firmwareupdate_timeout(const boost::system::error_code& error);

    // ---- web server -----
    // Describes web session.
    struct WebSession {
        char session_id[33];      // Session ID, must be unique
        char random[20];          // Random data used for extra user validation
        char user[MAX_USER_LEN];  // Authenticated user
        time_t expire;            // Expiration timestamp, UTC
    };

    bool web_useServer;
    std::vector<std::pair<unsigned int, IBus::Message> > web_ibusQueue;
    unsigned int web_queueStart;
    unsigned int web_queueEnd;

    boost::condition_variable web_reqCond;
    const char* web_loginUrl;
    const char* web_authorizeUrl;
    boost::mutex webRWlock;
    WebSession webSession[MAX_SESSIONS];
    mg_context* m_webCtx;
    unsigned short m_webPort;
    std::string m_webHTMLRoot;
    std::string m_webUser;
    std::string m_webPassword;
    bool m_webRunning;
    static void* web_handleEvents(mg_event event, mg_connection *conn, const mg_request_info* request_info);
    WebSession* web_getSession(const mg_connection *conn);
    WebSession* web_newSession();
    void web_generateSessionID(char *buf, const char *random, const char *user);
    bool web_isAuthorized(const mg_connection *conn, const mg_request_info *request_info);
    void web_authorize(mg_connection *conn, const mg_request_info *request_info);
    void web_getQvar(const mg_request_info *request_info, const char *name, char *dst, size_t dst_len);
    void web_redirectToLogin(mg_connection *conn, const mg_request_info* request_info);
    void* web_api_sendAndWait(mg_connection *conn, const mg_request_info* req);
    void* web_api_get(mg_connection *conn, const mg_request_info* req);
    bool web_beginHandleJSON(mg_connection *conn);
    void web_endHandleJSON(mg_connection *conn);
    static bool web_ibus_onRecieve(const IBus::Message& imsg, Application::MsgFilter* filter);

public:

    static Application* g_app;

    //! Create application and start connection acceptor at the given address and port
    Application(const std::string& address = openbm::Address, unsigned short port = openbm::Port, unsigned maxNumClients = openbm::ServerMaxNumClients, unsigned aliveTimeout = openbm::ClientAliveTimeout);
    ~Application();
    
    //! Connect to IBus
    bool connectIBus(const std::string& ibusPort, IBus::CTSType cts = IBus::HARDWARE);
    void disconnectIBus();
    
    //! Stop application
    void stop();

    //! Start main application
    void execute();

    //! Get ibus class used in the implementation
    inline IBus* getIBus() { return m_IBus.get(); }

    //! Get address used for the connection
    const std::string& getAddress() const { return m_Address; }

    //! Get port used for the connection
    const unsigned short getPort() const { return m_Port; }

    //! Give me firmware data and corresponding password to upload
    void uploadFirmware(const std::vector<unsigned char>& data, unsigned char pwd[4]);

    //! Send IBus message
    void sendIBusMessage(IBus::Message* msg, bool echo = true);

    //! Close a connection with the given client
    void closeClientConnection(ClientConnection* );

    //! Set webport which is used for the builtin webserver
    void setWebport(unsigned short port);

    //! Set HTML root directory
    void setHTMLRoot(const std::string& dir);

    //! Set credentials used to login to the webserver
    void setWebCredentials(const std::string& user, const std::string& password);

    //! Change the size of the internal ibus message queue for the web
    void setWebQueueSize(unsigned int size);

    void setUseWebserver(bool use) { web_useServer = use; }

    void setOnReceiveCallback(ReceiveMessageCallback cb) { m_ReceiveCallback = cb; }
    void setOnStopIBusThreadCallback(IBusThreadStopCallback cb) { m_stopIBusThreadCallback = cb; }
};


#endif	/* _APPLICATION_H */

