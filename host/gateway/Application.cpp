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

// global variable declaration
Application* Application::g_app = NULL;

//------------------------------------------------------------------------------
Application::Application(const std::string& ibusPort, const std::string& address, unsigned short port, unsigned numClients, unsigned alive) :
    _Endpoint(ip::address_v4::from_string(address), port),
    _Socket(_IOservice),
    _Timer(_IOservice),
    m_Port(port),
    m_Address(address),
    m_AliveTimeout(alive),
    m_ReceiveHeader(0,0,0,0),
    web_loginUrl("/login.html"),
    web_authorizeUrl("/auth"),
    m_webCtx(NULL),
    m_webPort(8080),
    m_webRunning(false)
{
    setWebQueueSize(100);

    // open IBus connection
    m_IBus.reset(new IBus());
    m_IBus->setMessageReceiveCallback(boost::bind(&Application::onReceiveIBusMessage, this, _1));
    m_IBus->open(ibusPort);

    // create list of free ports
    for (unsigned i=0; i < numClients; i++)
        m_FreePort[port+i+1] = true;
}

//------------------------------------------------------------------------------
Application::~Application()
{
}

//------------------------------------------------------------------------------
void Application::setWebport(unsigned short port)
{
    m_webPort = port;
    m_webRunning = false;
    _Cond.notify_one();
}

//------------------------------------------------------------------------------
void Application::setHTMLRoot(const std::string& dir)
{
    m_webHTMLRoot = dir;
    m_webRunning = false;
    _Cond.notify_one();
}

//------------------------------------------------------------------------------
void Application::setWebCredentials(const std::string& user, const std::string& password)
{
    m_webUser = user;
    m_webPassword = password;
}

//------------------------------------------------------------------------------
void Application::setWebQueueSize(unsigned int size)
{
    if (size > (1 << 16))
    {
        LOG_ERROR("Server:setWebQueueSize asks for a queue which is too large, I do not perform any change");
        return;
    }


    web_ibusQueue.resize(size, pair<unsigned,IBus::Message>(0, IBus::Message(0,0,0)));
    web_queueStart = 0;
    web_queueEnd = 0;
}

//------------------------------------------------------------------------------
void Application::uploadFirmware(const std::vector<unsigned char>& data, unsigned char pwd[4])
{
    // TODO implement start of an upload client
}

//------------------------------------------------------------------------------
void Application::stop()
{
    if (!m_Running) return;
    m_Running = false;

    LOG_DEBUG("Shutdown TPC/IP <-> IBus gateway ... (please wait)");

    _Cond.notify_one();
    web_reqCond.notify_one();
}

//------------------------------------------------------------------------------
void Application::start_acceptor()
{
    if (_Acceptor)
    {
        _Acceptor->cancel();
        _Acceptor->close();
    }

    _Timer.cancel();
    if (_Socket.is_open())
    {
        _Socket.cancel();
        _Socket.close();
    }

    _Acceptor.reset(new boost::asio::ip::tcp::acceptor(_IOservice, _Endpoint, true));
    _Acceptor->async_accept(_Socket, boost::bind(&Application::handle_accept, this, boost::asio::placeholders::error));
}

//------------------------------------------------------------------------------
void Application::handle_accept_read(const boost::system::error_code& error, const size_t& received)
{
    if (error == boost::asio::error::operation_aborted) return;

    _Timer.cancel();
    _Socket.cancel();
    
    if (!openbm::HelloMessage::isHelloMessage(m_ReceiveHeader) ||  received != sizeof(m_ReceiveHeader))
    {
        LOG_WARN(string("Nice people first say 'hi'!"));
        start_acceptor();
        return;
    }

    // look if there is a free port, to which we can connect
    unsigned short port = 0;
    for (std::map<unsigned short,bool>::const_iterator it = m_FreePort.begin(); it != m_FreePort.end() && port == 0; it++)
        if (it->second == true)
            port = it->first;
    
    boost::system::error_code ec;

    if (port == 0)
    {
        LOG_WARN("Cannot find free connection slots, discard connection attempt!");

        openbm::DisconnectMessage msg;
        write(_Socket, buffer(&msg, sizeof(msg)), transfer_all(), ec);

    }else
    {
        boost::mutex::scoped_lock lock(_Mutex);

        // create new connection session
        boost::shared_ptr<ClientConnection> conn(new ClientConnection(this, m_Client.size(), m_Address, port, m_AliveTimeout));
        LOG_DEBUG(string("Connection slot found, connect to session ") + boost::lexical_cast<string>(conn->getID()) + string(":") + boost::lexical_cast<string>(conn->getPort()));

        m_FreePort[port] = false;
        m_Client.push_back(conn);
        conn->start();

        openbm::ConnectToMessage msg(conn->getPort());
        write(_Socket, buffer(&msg, sizeof(msg)), transfer_all(), ec);
    }

    start_acceptor();
}

//------------------------------------------------------------------------------
void Application::handle_accept_timeout(const boost::system::error_code& error)
{
    if (error == boost::asio::error::operation_aborted) return;

    _Socket.cancel();
    _Timer.cancel();

    LOG_WARN(string("Timeout, nice people first say 'hi'!"));

    start_acceptor();
}

//------------------------------------------------------------------------------
void Application::handle_accept(const boost::system::error_code& error)
{
    if (error == boost::asio::error::operation_aborted) return;

    if (error)
    {
        LOG_ERROR(string("acceptor error: ") + boost::lexical_cast<string>(error));
        start_acceptor();
        return;
    };

    LOG_DEBUG(string("Detected new attempt to connect, awaiting 'hi' message for ") + boost::lexical_cast<string>(m_AliveTimeout) + string(" ms"));
        
    _Timer.expires_from_now(boost::posix_time::milliseconds(m_AliveTimeout));
    _Timer.async_wait(boost::bind(&Application::handle_accept_timeout, this, placeholders::error));

    memset(&m_ReceiveHeader, 0, sizeof(m_ReceiveHeader));
    async_read(_Socket, buffer(&m_ReceiveHeader, sizeof(m_ReceiveHeader)), boost::bind(&Application::handle_accept_read, this, placeholders::error, placeholders::bytes_transferred));
}

//------------------------------------------------------------------------------
void Application::closeClientConnection(ClientConnection* conn)
{
    if (std::find(m_ClientToClose.begin(), m_ClientToClose.end(), conn) != m_ClientToClose.end())
        return;

    m_ClientToClose.push_back(conn);
    _Cond.notify_one();
}

//------------------------------------------------------------------------------
void Application::runIOservice()
{
    LOG_DEBUG(string("Start client acceptor on ") + m_Address + string(":") + boost::lexical_cast<string>(m_Port));
    start_acceptor();

    // create webserver
    while(m_Running)
    {
        try
        {
            _IOservice.reset();
            _IOservice.run();
        }catch(const boost::system::error_code& error)
        {
            LOG_ERROR(string("Server error from IO-Service: ") + boost::lexical_cast<string>(error));
        }catch(...)
        {
            LOG_ERROR(string("Server unknown error!"));
        }
   }

   _Acceptor.reset();
   LOG_DEBUG(string("Stop client acceptor."));
}

//------------------------------------------------------------------------------
void Application::runLoop()
{
    LOG_DEBUG("Start TCP/IP <-> IBus gateway");

    // run our own acceptor, to wait for connect wishes from clients
    boost::thread ioServiceThread(boost::bind(&Application::runIOservice, this));
    
    m_Running = true;
    {
        try
        {            
            m_IBus->start();
            while (m_Running)
            {
                // if webserver is not running, then start it
                if (!m_webRunning)
                {
                    const char* options[] = {
                      "document_root", "html",
                      "listening_ports", NULL,
                      "num_threads", "1",
                      NULL
                    };
                    std::string port = boost::lexical_cast<std::string>(m_webPort);
                    options[3] = port.c_str();
                    options[1] = m_webHTMLRoot.c_str();
                    
                    if (m_webCtx)
                    {
                        mg_stop(m_webCtx);
                        LOG_DEBUG(string("Restart web-server at ") + m_Address + string(":") + port);
                    }else
                        LOG_DEBUG(string("Start web-server at ") + m_Address + string(":") + port);

                    m_webCtx = mg_start(web_handleEvents, NULL, options);
                    if (m_webCtx == NULL)
                        LOG_ERROR(string("Cannot start webserver! Check your settings."));
                    m_webRunning = m_webCtx != NULL;
                }

                // if we have connections to close, then do so
                for (std::vector<ClientConnection*>::iterator it = m_ClientToClose.begin(); it != m_ClientToClose.end(); )
                {
                    for (std::vector<boost::shared_ptr<ClientConnection> >::iterator jt=m_Client.begin(); jt != m_Client.end(); )
                    {
                        if (jt->get() == *it)
                        {
                            m_FreePort[(*jt)->getPort()] = true;
                            (*jt)->stop();
                            jt->reset();
                            jt = m_Client.erase(jt);
                        }else
                            jt++;
                    }
                    it = m_ClientToClose.erase(it);
                }

                boost::mutex::scoped_lock lock(_Mutex);
                _Cond.wait(lock);
            }
            m_IBus->stop();

            for (unsigned i=0; i < m_Client.size(); i++)
            {
                m_FreePort[m_Client[i]->getPort()] = true;
                m_Client[i]->stop();
                m_Client[i].reset();
            }

            if (m_webCtx)
            {
                LOG_DEBUG(string("Stop web-server"));
                mg_stop(m_webCtx);
            }
        }catch(const boost::system::error_code& error)
        {
            LOG_ERROR(string("Error from IO-Service: ") + boost::lexical_cast<string>(error));
        }catch(...)
        {
            LOG_ERROR(std::string("Unhandled error!"));
        }

    }
    m_Running = false;

    LOG_DEBUG(string("Stop acceptor and threads"));
    
    // stop acceptor
    _IOservice.post(boost::bind(&boost::asio::io_service::stop, &_IOservice));
    ioServiceThread.join();
}

//------------------------------------------------------------------------------
void Application::sendIBusMessage(IBus::Message* msg)
{
    //boost::mutex::scoped_lock lock(_Mutex);

    m_IBus->send(msg);
    onReceiveIBusMessage(*msg);
}

//------------------------------------------------------------------------------
void Application::onReceiveIBusMessage(const IBus::Message& imsg)
{
    boost::mutex::scoped_lock lock(_Mutex);

    // put the message into the webqueue
    const pair<unsigned, IBus::Message>& lastmsg = web_ibusQueue[web_queueStart];
    web_queueStart = (web_queueStart + 1) % web_ibusQueue.size();
    web_ibusQueue[web_queueStart] = pair<unsigned, IBus::Message>(lastmsg.first+1, imsg);
    
    // deliver messages to all clients
    for (unsigned i=0; i < m_Client.size(); i++)
    {
        if (m_Client[i])
            m_Client[i]->onReceive(imsg);
    }

    _FilterMutex.lock();

    // deliver message to all registered filters, however only if filter matches
    // the filters are applied as AND
    // filters where callback return true, will be removed
    //for (vector<MsgFilter*>::iterator it = m_msgFilters.begin(); it != m_msgFilters.end(); it++)
    MsgFilter* it = &m_msgFilter;
    if (!it->remove)
    {
        try{
            bool matched = true;

            if ((it)->src > 0 && imsg.getSource() != (it)->src) matched = false;
            if ((it)->dst > 0 && imsg.getDestination() != (it)->dst) matched = false;
            if ((it)->msg > 0 && imsg.getFirstDataByte() != (it)->msg) matched = false;

            if (matched && (it)->callback && !(it)->remove)
                (it)->remove = (it)->callback(imsg, it);
        }catch(...)
        {
            (it)->remove = true;
            LOG_DEBUG(string("Installed filter is invalid it will be removed from the queue"));
        }

    }

    // remove all marked filters
    /*for (vector<MsgFilter*>::iterator it = m_msgFilters.begin(); it != m_msgFilters.end();)
    {
        if ((*it)->remove)
        {
            delete *it;
            it = m_msgFilters.erase(it);
        }
        else it++;
    }*/

    _FilterMutex.unlock();
    web_reqCond.notify_one();
}

//------------------------------------------------------------------------------
void Application::execute()
{
    boost::thread appThread (boost::bind(&Application::runLoop, this));
    appThread.join();
}


//------------------------------------------------------------------------------
void Application::web_generateSessionID(char *buf, const char *random, const char *user)
{
    mg_md5(buf, random, user, NULL);
}


//------------------------------------------------------------------------------
Application::WebSession* Application::web_getSession(const struct mg_connection *conn)
{
    int i;
    char session_id[33];
    time_t now = time(NULL);
    mg_get_cookie(conn, "session", session_id, sizeof(session_id));
    for (i = 0; i < MAX_SESSIONS; i++)
    {
        if (webSession[i].expire != 0 && webSession[i].expire > now && strcmp(webSession[i].session_id, session_id) == 0)
            break;
    }
    return i == MAX_SESSIONS ? NULL : &(webSession[i]);
}

//------------------------------------------------------------------------------
void Application::web_getQvar(const mg_request_info *request_info, const char *name, char *dst, size_t dst_len)
{
    const char *qs = request_info->query_string;
    mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

//------------------------------------------------------------------------------
bool Application::web_isAuthorized(const mg_connection *conn, const mg_request_info *request_info)
{
    WebSession* session;
    char valid_id[33];
    bool authorized = false;

    // Always authorize accesses to login page and to authorize URI
    if (!strcmp(request_info->uri, web_loginUrl) || !strcmp(request_info->uri, web_authorizeUrl))
        return true;

    webRWlock.lock();
    {
        if ((session = web_getSession(conn)) != NULL)
        {
            web_generateSessionID(valid_id, session->random, session->user);
            if (strcmp(valid_id, session->session_id) == 0)
            {
                session->expire = time(0) + SESSION_TTL;
                authorized = true;
            }
        }
    }
    webRWlock.unlock();
    
    return authorized;
}

//------------------------------------------------------------------------------
Application::WebSession* Application::web_newSession()
{
    int i;
    time_t now = time(NULL);
    webRWlock.lock();
    {
        for (i = 0; i < MAX_SESSIONS; i++)
            if (webSession[i].expire == 0 || webSession[i].expire < now)
            {
                webSession[i].expire = time(0) + SESSION_TTL;
                break;
            }
    }
    webRWlock.unlock();
    return i == MAX_SESSIONS ? NULL : &webSession[i];
}

//------------------------------------------------------------------------------
void Application::web_authorize(mg_connection *conn, const mg_request_info *request_info)
{
    char user[MAX_USER_LEN], password[MAX_USER_LEN];
    WebSession *session;

    // Fetch user name and password.
    web_getQvar(request_info, "username", user, sizeof(user));
    web_getQvar(request_info, "password", password, sizeof(password));

    // check if password matches
    bool passwordMatches = false;
    if (strlen(user) && strlen(password) && strcmp(user, m_webUser.c_str())==0 && strcmp(password, m_webPassword.c_str())==0)
        passwordMatches = true;

    if (passwordMatches)
    {
        session = web_newSession();
        if (session)
        {
            snprintf(session->user, sizeof(session->user), "%s", user);
            snprintf(session->random, sizeof(session->random), "%d", rand());
            web_generateSessionID(session->session_id, session->random, session->user);
            mg_printf(conn, "HTTP/1.1 302 Found\r\n"
                            "Set-Cookie: session=%s; max-age=3600; http-only\r\n"  // Session ID
                            "Set-Cookie: user=%s\r\n"  // Set user, needed by Javascript code
                            "Set-Cookie: original_url=/; max-age=0\r\n"  // Delete original_url
                            "Location: /\r\n\r\n",
                            session->session_id, session->user);
            return;
        }
    }

    // if here, then nothign went right, so redirect to login page
    web_redirectToLogin(conn, request_info);
}

//------------------------------------------------------------------------------
void Application::web_redirectToLogin(mg_connection *conn, const mg_request_info* request_info)
{
    mg_printf(conn, "HTTP/1.1 302 Found\r\n"
                    "Set-Cookie: original_url=%s\r\n"
                    "Location: %s\r\n\r\n",
                    request_info->uri, web_loginUrl);
}

//------------------------------------------------------------------------------
void* Application::web_handleEvents(mg_event event, mg_connection *conn, const mg_request_info* request_info)
{
    // on error
    if (event == MG_EVENT_LOG)
    {
        LOG_WARN(string("[") + string(request_info->request_method) + string("] ") + string(request_info->uri) + string("   ") + string(request_info->log_message));
    }else if (event == MG_NEW_REQUEST)
    {
        LOG_DEBUG(string("[") + string(request_info->request_method) + string("] ") + string(request_info->uri));
        
        // static built-in routing maps
        if (strspn("/static", request_info->uri) == 7) return NULL;
        if (strcmp("/favicon.ico", request_info->uri) == 0) return NULL;

        // if currently not authorized then redirect to login page
        if (!g_app->web_isAuthorized(conn, request_info))
        {
            g_app->web_redirectToLogin(conn, request_info);
            return (void*)conn;

        // trying to authorize, perform autorization
        } else if (strcmp(request_info->uri, g_app->web_authorizeUrl) == 0)
        {
            g_app->web_authorize(conn, request_info);
            return (void*)conn;
        
        // send ajax request (return JSON)
        } else if (strcmp(request_info->uri, "/api/sendAndWait") == 0)
            return g_app->web_api_sendAndWait(conn, request_info);

        // send ajax request (return JSON)
        else if (strcmp(request_info->uri, "/api/get") == 0)
            return g_app->web_api_get(conn, request_info);
    }

    return NULL;
}

//------------------------------------------------------------------------------
bool Application::web_beginHandleJSON(mg_connection *conn)
{
    const char *ajax_reply_start =
          "HTTP/1.1 200 OK\r\n"
          "Cache: no-cache\r\n"
          "Content-Type: application/json\r\n"
          "\r\n";

    mg_printf(conn, "%s", ajax_reply_start);
    
    return true;
}

//------------------------------------------------------------------------------
void Application::web_endHandleJSON(mg_connection *conn)
{
}

//------------------------------------------------------------------------------
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
bool Application::web_ibus_onRecieve(const IBus::Message& imsg, Application::MsgFilter* filter)
{
    // filter stores in the userdata section the connection to the client
    mg_connection* conn = static_cast<mg_connection*>(filter->userdata);
    if (!conn) return true;

    g_app->web_beginHandleJSON(conn);

    // return the recieved message to the client
    ostringstream ss;
    ss << "{";
    ss << "\"s\":" << ((unsigned int)imsg.getSource());
    ss << ",\"d\":" << ((unsigned int)imsg.getDestination());
    ss << ",\"m\":[";
    vector<unsigned char> data = imsg.getData();
    ss << "" << ((unsigned int)data[0]) << "";
    for (unsigned int i=1; i < data.size(); i++)
        ss << "," << ((unsigned int)data[i]) << "";
    ss << "]";
    ss << ",\"c\":" << ((unsigned int)imsg.getChecksum());
    ss << "}";

    mg_printf(conn, ss.str().c_str());

    g_app->web_endHandleJSON(conn);

    return true;
}

//------------------------------------------------------------------------------
void* Application::web_api_sendAndWait(mg_connection *conn, const mg_request_info* req)
{
    char msg[300];
    char fsrc[4];
    char fdst[4];
    char fmsg[4];
    char tout[8];

    // get parameters, expecting msg, [fsrc], [fdst], [fmsg]
    web_getQvar(req, "msg",  msg, sizeof(msg));
    web_getQvar(req, "fsrc", fsrc, sizeof(fsrc));
    web_getQvar(req, "fdst", fdst, sizeof(fdst));
    web_getQvar(req, "fmsg", fmsg, sizeof(fmsg));
    web_getQvar(req, "time", tout, sizeof(tout));

    // if message was given, then parse it in order to send later
    IBus::Message* message = NULL;
    if (strlen(msg) != 0)
    {
        message = IBus::Message::parseFromSDMString(msg);
        if (!message)
        {
            LOG_ERROR(string("/api/sendAndWait - cannot parse the given message"));
            return NULL;
        }
    }

    // create filter callback
    MsgFilter* filter = &m_msgFilter;//new MsgFilter();
    {
        boost::mutex::scoped_lock lock(_FilterMutex);

        filter->src = -1;
        filter->dst = -1;
        filter->msg = -1;
        filter->remove = false;
        filter->userdata = static_cast<void*>(conn);
        filter->callback = web_ibus_onRecieve;
        try {
            if (strlen(fsrc))
                filter->src = boost::lexical_cast<HexTo<unsigned int> >(fsrc);
            if (strlen(fdst))
                filter->dst = boost::lexical_cast<HexTo<unsigned int> >(fdst);
            if (strlen(fmsg))
                filter->msg = boost::lexical_cast<HexTo<unsigned int> >(fmsg);
        } catch(boost::bad_lexical_cast &) {
            LOG_ERROR(string("/api/sendAndWait - cannot parse the given filters"));
            return NULL;
        }
    }

    // specify timeout how long to wait for the resulting message (only if filtered)
    unsigned timeout = 0;
    if (filter->src > 0 || filter->dst > 0 || filter->msg > 0)
        timeout = WEB_IBUS_TIMEOUT;

    if (strlen(tout) > 0)
    {
        try{
            timeout = boost::lexical_cast<unsigned>(tout);
        } catch(boost::bad_lexical_cast &) {
            LOG_ERROR(string("/api/sendAndWait - cannot parse the timeout value"));
            return NULL;
        }
    }

    // put the filter into the queue of filters and send ibus message
    //m_msgFilters.push_back(filter);
    if (message)
        sendIBusMessage(message);

    // now we wait until either the filter is processed or filter timeout occurs
    if (timeout)
    {
        boost::mutex::scoped_lock lock(_FilterMutex);
        web_reqCond.timed_wait(lock, boost::posix_time::milliseconds(timeout));
    }

    // mark filter as not processable anymore
    _FilterMutex.lock();
    filter->remove = true;
    _FilterMutex.unlock();

    return (void*)conn;
}

//------------------------------------------------------------------------------
void* Application::web_api_get(mg_connection *conn, const mg_request_info* req)
{
    char tid[16] = {0};
    char tout[8] = {0};

    web_getQvar(req, "tid", tid, sizeof(tid));
    web_getQvar(req, "time", tout, sizeof(tout));

    const pair<unsigned, IBus::Message>& lastmsg = web_ibusQueue[web_queueStart];

    unsigned int lastid = lastmsg.first;
    try {
        if (strlen(tid))
            lastid = boost::lexical_cast<unsigned int>(tid);
    } catch(boost::bad_lexical_cast &) {
        LOG_ERROR(string("/api/get - cannot parse the tid"));
        return NULL;
    }

    // specify timeout how long to wait for the resulting message (only if filtered)
    unsigned timeout = WEB_IBUS_TIMEOUT;
    if (strlen(tout) > 0)
    {
        try{
            timeout = boost::lexical_cast<unsigned>(tout);
        } catch(boost::bad_lexical_cast &) {
            LOG_ERROR(string("/api/get - cannot parse the timeout value"));
            return NULL;
        }
    }

    // if timeout was specified then wait for the given time for a new message
    if (timeout)
    {
        boost::mutex::scoped_lock lock(_FilterMutex);
        web_reqCond.timed_wait(lock, boost::posix_time::milliseconds(timeout));
    }

    // first we find for a position in the queue, where there is a start of the first new message
    // TODO implement binary search
    unsigned start = 0;
    unsigned previous = lastid;
    for (unsigned i=1; i <= web_ibusQueue.size(); i++)
    {
        start = (web_queueStart + i) % web_ibusQueue.size();
        const pair<unsigned, IBus::Message>& msg = web_ibusQueue[start];
        if (msg.first >= lastid) break;
    }

    // now we go through the queue and return all elements wiht id bigger then the given one
    ostringstream ss;
    string comma = ",\r\n";
    ss << "[";
    for (unsigned i=1; i <= web_ibusQueue.size(); i++)
    {
        unsigned index = (start + i) % web_ibusQueue.size();
        const pair<unsigned, IBus::Message>& msg = web_ibusQueue[index];
        if (msg.first < previous) break;
        previous = msg.first;

        if (i > 1)
            ss << comma;
        const IBus::Message& imsg = msg.second;
        ss << "{";
        ss << "\"s\":" << ((unsigned int)imsg.getSource());
        ss << ",\"d\":" << ((unsigned int)imsg.getDestination());
        ss << ",\"m\":[";
        vector<unsigned char> data = imsg.getData();
        ss << "" << ((unsigned int)data[0]) << "";
        for (unsigned int i=1; i < data.size(); i++)
            ss << "," << ((unsigned int)data[i]) << "";
        ss << "]";
        ss << ",\"c\":" << ((unsigned int)imsg.getChecksum());
        ss << ",\"tid\":" << msg.first;
        ss << "}";

    }
    ss << "]";

    g_app->web_beginHandleJSON(conn);
    mg_printf(conn, ss.str().c_str());
    g_app->web_endHandleJSON(conn);

    return (void*)conn;
}