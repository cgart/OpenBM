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

#include <stdlib.h>
#include "Application.h"
#include "Log.h"
#include "GatewayProtocol.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <execinfo.h>
#include <signal.h>

#define VERSION_MINOR 8
#define VERSION_MAJOR 1
#define VERSION_NAME "OpenBM-Gateway"

//! Handle CTRL+C and other kills
static void handle_SIGINT(int sig_num)
{
    Application::g_app->stop();
}

void handler_SEGV(int sig)
{
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, 2);
  exit(1);
}

/*
 * 
 */
int main(int argc, char** argv)
{
    using namespace boost::program_options;

    options_description argDesc("Allowed options");

    std::string port;
    std::string logfile;
    int loglevel = 0;
    std::string eventfile;
    unsigned numClients;
    unsigned short tcpport;
    std::string address;
    unsigned aliveTimeout;
    std::string firmwareFile;
    std::string firmwarePassword;
    unsigned short webport;
    std::string webroot;
    std::string webuser;
    std::string webpassword;
    unsigned int webqueuesize;

    argDesc.add_options()
        ("help,h", "print this help message")
        ("device,d", value<std::string>(&port)->default_value("/dev/ttyS0"), "serial port to find connected IBus interface (interface must support CTS signals)")
        ("loglevel,l", value<int>(&loglevel)->default_value(0), "Log level to be used: (0=info, 1=debug, 2=warn, 3=error, 4=raw ibus messages only).")
        ("logfile,f", value<std::string>(&logfile), "Specify log file name if you would like to write log to file. Otherwise log will be printed to the console.")
        ("eventfile,e", value<std::string>(&eventfile), "Event file containing system commands to be executed on received messages.")
        ("clients,c", value<unsigned>(&numClients)->default_value(openbm::ServerMaxNumClients), "Specify max amount of clients which can connect to the gateway server.")
        ("port,p", value<unsigned short>(&tcpport)->default_value(openbm::Port), "TCP/IP port used to bind the gateway to. Clients connects to consecutive ports.")
        ("address,i", value<std::string>(&address)->default_value(openbm::Address), "TCP/IP address where to bind the gateway to")
        ("alivetimeout,t", value<unsigned>(&aliveTimeout)->default_value(openbm::ClientAliveTimeout/1000), "Time in sec to close client connection if not received any ping message from client. The default value is set according to IBus-Gateway protocol.")

        ("webport,o", value<unsigned short>(&webport)->default_value(openbm::WebPort), "Port used for built-in webserver.")
        ("webroot,r", value<std::string>(&webroot)->default_value("html/"), "HTML root directory of the OpenBM gateway server")
        ("webuser", value<std::string>(&webuser)->default_value("bmw"), "Default username to use for the webserver")
        ("webpassword", value<std::string>(&webpassword)->default_value("e39"), "Default password to use for the webserver")
        ("webqueue", value<unsigned int>(&webqueuesize)->default_value(100), "Number of IBus messages to hold in the queue for web interface. This makes sure that no message got lost.")

        ("firmware,u", value<std::string>(&firmwareFile), "Firmware file to upload to OpenBM")
        ("password,w", value<std::string>(&firmwarePassword), "Firmware password which was delivered with the firmware")
        ("version,v", "show version")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, argDesc), vm);
    notify(vm);

    if (vm.count("version") || vm.count("help"))
    {
        std::cout << VERSION_NAME << " v" << VERSION_MAJOR << "." << VERSION_MINOR << " (build on " << __DATE__ << " at " << __TIME__ << ")" << std::endl;
        if (vm.count("help"))
            std::cout << argDesc << std::endl;

        return EXIT_SUCCESS;
    }

    // set log level and file
    if (loglevel < 0) loglevel = 0;
    if (loglevel > 4) loglevel = 4;
    Logger::instance()->setLevel((Logger::Level)loglevel);
    if (vm.count("logfile"))
        Logger::instance()->setFile(logfile);

    Application::g_app = new Application(port, address, tcpport, numClients, aliveTimeout * 1000);
    Application::g_app->setWebport(webport);
    Application::g_app->setHTMLRoot(webroot);
    Application::g_app->setWebCredentials(webuser, webpassword);
    Application::g_app->setWebQueueSize(webqueuesize);

    // if firmware file and password specified, then setup this
    if (firmwareFile.length() && firmwarePassword.length() == 4)
    {
        // load firmware file
        try
        {
            // read firmware data
            FILE* file = fopen(firmwareFile.c_str(), "rb");
            if (file == NULL)throw std::string("not found");

            //std::ifstream file(firmwareFile.c_str(), std::ifstream::in | std::ifstream::binary);
            //if (!file.good()) throw std::string("not found");

            std::vector<unsigned char> firmwareData;
            //while (file.good())
            //    firmwareData.push_back(file.get());
            while (!feof(file))
            {
                unsigned char byte = 0;
                if (fread(&byte, 1, 1, file) == 1)
                    firmwareData.push_back(byte);
            }
            //firmwareData.resize(firmwareData.size()-1); // HACK, do not know why so???

            // read password
            unsigned char pwd[4] = {firmwarePassword[0], firmwarePassword[1], firmwarePassword[2], firmwarePassword[3]};

            // need to upload
            Application::g_app->uploadFirmware(firmwareData, pwd);
        }catch(...)
        {
            LOG_ERROR(std::string("Error when reading firmware file ") + firmwareFile);
        }
    }

    // install handler for catching user interrupts
    signal(SIGINT, handle_SIGINT);
    signal(SIGSEGV, handler_SEGV);

    // read event file
    if (eventfile.length())
    {
        try
        {
            std::ifstream file(eventfile.c_str(), std::ifstream::in);
            if (!file.good()) throw std::string("not found");
            std::string line;
            int iline = 0;
            while (file.good() && std::getline(file,line))
            {
                iline++;
                std::string msg;
                std::string exe;

                typedef boost::tokenizer<boost::char_separator<char> >  tokenizer;

                // read event definition
                tokenizer tok(line, boost::char_separator<char>("="));
                tokenizer::const_iterator it = tok.begin();
                if (it != tok.end()) {msg = *it; it++; }
                if (it != tok.end()) exe = *it;

                boost::trim(msg);
                boost::trim(exe);

                // parse message data
                std::vector<unsigned char> raw;
                if (msg[0] == '\"' && msg[msg.length()-1] == '\"')
                {
                    msg[0] = ' ';
                    msg[msg.length()-1] = ' ';

                    boost::tokenizer<> ttk(msg);
                    for (boost::tokenizer<>::const_iterator it = ttk.begin(); it != ttk.end(); it++)
                    {
                        char* p = NULL;
                        unsigned long val = strtoul(it->c_str(), &p, 16);
                        if (*p != '\0')
                        {
                            LOG_ERROR(std::string("Event file: Parse error in line ") + boost::lexical_cast<std::string>(iline) + std::string(" ") + *it +std::string(" is not a number"));
                            break;
                        }else
                            raw.push_back((unsigned char)(val));
                    }
                }else
                {
                    LOG_WARN(std::string("Event file: Cannot parse line: ") + boost::lexical_cast<std::string>(iline));
                }

                if (raw.size() > 3)
                {
                    Application::g_app->getIBus()->addSystemEvent(IBus::Message(&raw[0], raw.size()), exe);
                    LOG_INFO(std::string("Add system event: ") + msg + std::string(" -> ") + exe);
                }
            }
            file.close();
        }catch(...)
        {
            LOG_ERROR(std::string("Error when parsing event file ") + eventfile);
        }
    }

    // execute application
    Application::g_app->execute();

    delete Application::g_app;
        

    return EXIT_SUCCESS;
}

