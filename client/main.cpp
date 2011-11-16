/* 
 * File:   main.c
 * Author: tevs
 *
 * Created on August 14, 2011, 4:25 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include "IPCClient.h"

boost::shared_ptr<IPCClient::Client> ipcClient;
bool running = true;

/**
 * React on received messages over dbus/ibus
 **/
void onMessage(const IPCClient::IBusMessage& msg)
{
    unsigned char src = msg.src;
    unsigned char dst = msg.dst;
    const std::vector<unsigned char>& data = msg.data;
}

/**
 * Connection to the OpenBM-Gateway server is lost
 **/
void onDisconnect()
{
    std::cerr << "Connection to the server is lost or has been closed." << std::endl;
    running = false;
}

/**
 * When error received set python error
 **/
void onReceiveError(IPCClient::ReceiveError error, const std::string& msg)
{
    std::cerr << "Error recieved: " << msg << std::endl;
}

template <typename ElemT>
struct HexTo {
    ElemT value;
    operator ElemT() const {return static_cast<ElemT>(value);}
    friend std::istream& operator>>(std::istream& in, HexTo& out) {
        in >> std::hex >> out.value;
        return in;
    }
};

void runthread()
{
    std::cerr << "blah " << std::endl;
}

/*
 * 
 */
int main(int argc, char** argv)
{    
    // --------------------------------------------------
    // Get programm options
    // --------------------------------------------------
    using namespace boost::program_options;
    options_description argDesc("OpenBM-Client accepts IBus messages on standard input. \n"
    "The message will be send to the OpenBM-Gateway which pass it to the IBus.\n"
    "The format is: 3F,BF,11,22,33. Here 3F is the source, BF is the destionation and 11,22,33 are the data bytes.\n"
    "Allowed options");
    unsigned short tcpport;
    std::string address;
    std::string msg;

    argDesc.add_options()
        ("help,h", "print this help message")
        ("port,p", value<unsigned short>(&tcpport)->default_value(openbm::Port), "TCP/IP port used to bind the gateway to. Clients connects to consecutive ports.")
        ("address,i", value<std::string>(&address)->default_value(openbm::Address), "TCP/IP address where to bind the gateway to")
        ("msg,m", value<std::string>(&msg),"Message of the format: 3F,BF,11,22,33")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, argDesc), vm);
    notify(vm);

    if (vm.count("help"))
    {
        std::cout << argDesc << std::endl;
        return EXIT_SUCCESS;
    }

    // parse given message
    IPCClient::IBusMessage ibusMsg;
    if (msg.length())
    {
        std::vector<std::string> strs;
        boost::split(strs, msg, boost::is_any_of("\t ,"));
        if (strs.size() < 3)
        {
            std::cerr << "Given message is of wrong format. There must be at least 3 valid bytes: src, dst, data" << std::endl;
            return -1;
        }
        try {
            ibusMsg.src = boost::lexical_cast<HexTo<unsigned int> >(strs[0]);
            ibusMsg.dst = boost::lexical_cast<HexTo<unsigned int> >(strs[1]);
            for (int i=2; i < strs.size(); i++)
                ibusMsg.data.push_back(boost::lexical_cast<HexTo<unsigned int> >(strs[i]));
        } catch(boost::bad_lexical_cast &) {
            std::cerr << "Cannot parse the given message. Please make sure that it has the right format: " << std::endl;
            return -1;
        }
    }else
    {
        std::cerr << "No message was specified." << std::endl;
        return(EXIT_SUCCESS);
    }

    // --------------------------------------------------
    // connect to the IBUS server
    // --------------------------------------------------

    ipcClient.reset(new IPCClient::TCPIPClient(address, tcpport));

    ipcClient->setOnMessageCallback(onMessage);
    ipcClient->setOnReceiveErrorCallback(onReceiveError);
    ipcClient->setOnDisconnectCallback(onDisconnect);
    ipcClient->start();

    if (!ipcClient->isConnected())
    {
        std::cerr << "Cannot connect to the OpenBM-Gateway server. Execution stopped" << std::endl;
        return -1;
    }

    ipcClient->send(ibusMsg);
    
    // --------------------------------------------------
    // Disconnect from the server
    // --------------------------------------------------
    ipcClient->stop();
    //while(running){};
    ipcClient.reset();


    return (EXIT_SUCCESS);
}

