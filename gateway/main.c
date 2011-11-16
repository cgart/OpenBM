/***************************************************************************
 *   Copyright (c) 2010   Art Tevs                                         *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 3 of        *
 *   the License, or (at your option) any later version.                   *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesse General Public License for more details.                    *
 *                                                                         *
 *   The full license is in LICENSE file included with this distribution.  *
 ***************************************************************************/


#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <sys/ioctl.h>
#include <time.h>   // time calls
#include <stdlib.h>
#include <signal.h>
#include <sys/time.h>
#include <dbus/dbus.h>
#include <pthread.h>   /* Posix 1003.1c threads */

//------------------------------------------------------------------------------
// Macros & Defines
//------------------------------------------------------------------------------
#define VERSION_MAJOR 0
#define VERSION_MINOR 2

// sleep time for the process in the main loop (in usec)
//#define SLEEP_TIME 500

// how long to wait before timeout on recivie (in ms)
#define IBUS_RX_TIMEOUT 50

// how long to wait for free bus (~ 3ms)
#define IBUS_TIMEOUT_WAIT_FREE_BUS 3
#define IBUS_TIMEOUT_TX_COLLISION 3

// how long to wait between sending of two messages
#define IBUS_TIMEOUT_TX_AFTER_SEND 2

// default value how often we would like to resubmit a message
#define TX_RETRIES 5

// how long to wait for a message in tx buffer, before message is rejected (in usec)
#define IBUS_TX_TIMEOUT 1000000

#define RX_BUFFER_SIZE 1024


//*** Device Codes ***
#define IBUS_DEV_GM      0X00    // Body Module
#define IBUS_DEV_CDC     0x18    // CD Changer
#define IBUS_DEV_FUH     0x28    // Radio controlled clock
#define IBUS_DEV_CCM     0x30    // Check control module
#define IBUS_DEV_GT      0x3B    // Graphics driver (in navigation system)
#define IBUS_DEV_DIA     0x3F    // Diagnostic
#define IBUS_DEV_FBZV    0x40    // Remote control central locking
#define IBUS_DEV_GTF     0x43    // Graphics driver for rear screen (in navigation system)
#define IBUS_DEV_EWS     0x44    // Immobiliser
#define IBUS_DEV_CID     0x46    // Central information display (flip-up LCD screen)
#define IBUS_DEV_MFL     0x50    // Multi function steering wheel
#define IBUS_DEV_MM      0x51    // Mirror memory
#define IBUS_DEV_IHK     0x5B    // Integrated heating and air conditioning
#define IBUS_DEV_PDC     0x60    // Park distance control
#define IBUS_DEV_ONL     0x67    // unknown
#define IBUS_DEV_RAD     0x68    // Radio
#define IBUS_DEV_DSP     0x6A    // Digital signal processing audio amplifier
#define IBUS_DEV_SM1     0x72    // Seat memory
#define IBUS_DEV_CDCD    0x76    // CD changer, DIN size.
#define IBUS_DEV_NAVE    0x7F    // Navigation (Europe)
#define IBUS_DEV_IKE     0x80    // Instrument cluster electronics
#define IBUS_DEV_MM1     0x9B    // Mirror memory
#define IBUS_DEV_MM2     0x9C    // Mirror memory
#define IBUS_DEV_FMID    0xA0    // Rear multi-info-display
#define IBUS_DEV_ABM     0xA4    // Air bag module
#define IBUS_DEV_KAM     0xA8    // unknown
#define IBUS_DEV_ASP     0xAC    // unknown
#define IBUS_DEV_SES     0xB0    // Speed recognition system
#define IBUS_DEV_NAVJ    0xBB    // Navigation (Japan)
#define IBUS_DEV_GLO     0xBF    // Global, broadcast address
#define IBUS_DEV_MID     0xC0    // Multi-info display
#define IBUS_DEV_TEL     0xC8    // Telephone
#define IBUS_DEV_LCM     0xD0    // Light control module
#define IBUS_DEV_SM2     0xDA    // Seat memory
#define IBUS_DEV_GTHL    0xDA    // unknown
#define IBUS_DEV_IRIS    0xE0    // Integrated radio information system
#define IBUS_DEV_ANZV    0xE7    // Front display
#define IBUS_DEV_ISP     0xE8    // unknown
#define IBUS_DEV_TV      0xED    // Television
#define IBUS_DEV_BMBT    0xF0    // On-board monitor operating part
#define IBUS_DEV_CSU     0xF5    // unknown
#define IBUS_DEV_LOC     0xFF    // Local


//*** Message Types ***
#define IBUS_MSG_LAMP_STATE         0x5B    // Lamp state
#define IBUS_MSG_VEHICLE_CTRL       0x0C    // Vehicle Control (mostly used from diagnose)
#define IBUS_MSG_UPDATE_MID_BOTTOM  0x21    // update information on text display
#define IBUS_MSG_UPDATE_MID_TOP     0x23    // update information on text display
#define IBUS_MSG_RADIO_ENCODER      0x32    // MID's radio encoder was rotated
#define IBUS_MSG_BMBT_ENCODER       0x3B    // MID's radio encoder was rotated
#define IBUS_MSG_BUTTON             0x31    // MID's button state change

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------
typedef unsigned char bool;
typedef unsigned char byte;
typedef byte uint8_t;

#define true 1
#define false 0

bool g_running = true;
int g_SerialDevice = -1;
//bool g_msgReceived = false;
DBusConnection* g_dbusConn = NULL;

pthread_cond_t g_waitForActionCond;
pthread_mutex_t g_waitForActionMutex;

pthread_mutex_t g_dbusMutex;
bool g_dbusStop = false;

byte g_RxBuffer[RX_BUFFER_SIZE];
unsigned g_RxPos = 0;
unsigned g_RxLen = 2;
unsigned g_TxReadPos = 0;
unsigned g_TxWritePos = 0;

struct IBusMessage;

struct IBusMessage
{
    // list
    struct IBusMessage* next;

    // ibus msg data
    byte src;
    byte dst;
    byte* data;
    unsigned dataLength;
    byte chk;

    // timing properties
    int retries;  // how much trials left until message is marked as not deliverable
    suseconds_t valid_until;
};

// transmit buffer
struct IBusMessage* g_TxList = NULL;

enum State
{
    IDLE,
    RECEIVE,
    WAIT,
    TRANSMIT
}g_State = IDLE;

//--------------------------------------------------------------------------
// Timer interrupt handles.
// This will put is into IDLE state back and will clean up all buffers
//--------------------------------------------------------------------------
void _sig_Timeout(int status)
{
    g_RxPos = 0;
    g_RxLen = 2;
    g_State = IDLE;

    tcflush(g_SerialDevice, TCIOFLUSH);
    pthread_cond_signal(&g_waitForActionCond);
}

//--------------------------------------------------------------------------
// Setup timer signal handlers
//--------------------------------------------------------------------------
void timer_setup()
{
    struct sigaction stimer;
    stimer.sa_handler = _sig_Timeout;
    sigemptyset(&stimer.sa_mask);   //saio.sa_mask = 0;
    stimer.sa_flags = SA_NOMASK | SA_ONESHOT;
    stimer.sa_restorer = NULL;
    sigaction(SIGALRM,&stimer,NULL);

}

//--------------------------------------------------------------------------
// Start time out timer (in ms)
//--------------------------------------------------------------------------
void timer_start(unsigned ms)
{
    timer_setup();

    struct itimerval timer;

    gettimeofday(&timer.it_value, NULL);

    timer.it_value.tv_usec = ms * 1000;
    timer.it_value.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    timer.it_interval.tv_sec = 0;
    
    setitimer(ITIMER_REAL, &timer, NULL);
}

//--------------------------------------------------------------------------
// Calculate checksum of the buffer
//--------------------------------------------------------------------------
uint8_t ibus_calcChecksum(uint8_t* pBuffer)
{
    if(pBuffer == NULL)
        return 0;

    unsigned i;
    unsigned len;
    uint8_t checksum;

    checksum = 0;
    len = pBuffer[1] + 1;

    for(i = 0; i < len; i++)
        checksum ^= pBuffer[i];

    return checksum;
}

//--------------------------------------------------------------------------
// Push new ibus message on the list and return this back
// send message over ibus, serial send
//--------------------------------------------------------------------------
struct IBusMessage* ibus_send(byte src, byte dst, byte* data, unsigned len, int trials)
{
    // get last element in the tx queue
    struct IBusMessage* current = g_TxList;
    while (current != NULL && current->next != NULL)
        current = current->next;

    // create new message
    struct IBusMessage* msg = malloc(sizeof(struct IBusMessage));
    msg->next = NULL;
    msg->src = src;
    msg->dst = dst;
    msg->retries = trials;
    byte* buffer = malloc(sizeof(byte) * len);
    memcpy(buffer, data, sizeof(byte) * len);
    msg->data = buffer;
    msg->dataLength = len;

    struct timeval tv;
    gettimeofday(&tv, NULL);
    msg->valid_until = tv.tv_usec + IBUS_TX_TIMEOUT;

    // put message into output queue, message will be sent when bus get free
    if (current == NULL)
        g_TxList = msg;
    else
        current->next = msg;

    pthread_cond_signal(&g_waitForActionCond);

    return msg;
}

//--------------------------------------------------------------------------
// React on input on serial port
//--------------------------------------------------------------------------
void _sig_SerialIO (int status)
{
    if (status != SIGIO) return;

    // read received bytes
    int num_bytes = 0 ;
    if (ioctl( g_SerialDevice, FIONREAD, &num_bytes ) == 0 )
    {
        if (num_bytes == 0) return;
        
        unsigned oldPos = g_RxPos;
        
        if (num_bytes > RX_BUFFER_SIZE - g_RxPos)
            num_bytes = RX_BUFFER_SIZE - g_RxPos;

        // read into buffer
        if ( read(g_SerialDevice, &g_RxBuffer[g_RxPos], num_bytes ) == num_bytes )
        {
            g_RxPos += num_bytes;
        }

        // flush read buffer
        tcflush(g_SerialDevice, TCIFLUSH);

        // check if no data received, then just indicate as error
        if (oldPos == g_RxPos)
        {
            timer_start(IBUS_TIMEOUT_WAIT_FREE_BUS);
            return;
        }
    }

    // if this was IBUS's len flag
    if (g_RxPos >= 2) g_RxLen = g_RxBuffer[1] + 2;

    // if we have reached the end of our frame, then check checksum
    if (g_RxPos >= g_RxLen)
    {
        g_State = IDLE;
        
        uint8_t chk = ibus_calcChecksum(g_RxBuffer);

        // if received message is ok, then indicate this
        if (chk == g_RxBuffer[g_RxLen-1])
            pthread_cond_signal(&g_waitForActionCond);
    }else
    {
        // reset rx timeout counter
        timer_start(IBUS_RX_TIMEOUT);
        g_State = RECEIVE;
    }

}

//--------------------------------------------------------------------------
bool serial_port_get_cts(int fd)
{
    if (fd < 0) return false;

    int status;
    if (ioctl(fd, TIOCMGET, &status) < 0) return false;
    return (status & TIOCM_CTS) != 0;
}

//--------------------------------------------------------------------------
// Open serial port
//--------------------------------------------------------------------------
int open_port(const char* portfile)
{
    int fd; // file description for the serial port

    fd = open(portfile, O_RDWR | O_NOCTTY | O_NDELAY);

    if(fd == -1) // if open is unsucessful
    {
        printf("open_port: Unable to open %s. \n", portfile);
        return -1;
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    //install the serial handler before making the device asynchronous
    struct sigaction saio;
    saio.sa_handler = _sig_SerialIO;
    sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
    saio.sa_flags = SA_NOMASK;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);

    if (fcntl( fd, F_SETOWN, getpid() ) < 0 || fcntl( fd, F_SETFL, FASYNC ) < 0)
    {
        printf("open_port: Unable to set signal handler%s. \n", portfile);
        return -1;
    }

    return(fd);
}

//--------------------------------------------------------------------------
bool get_port_settings(int fd, struct termios* settings)
{
    bzero( settings, sizeof( struct termios ) ) ;
    if (tcgetattr(fd, settings ) < 0) return false;
    return true;
}

//--------------------------------------------------------------------------
void set_port_settings(int fd, struct termios* settings)
{
    tcsetattr(fd, TCSANOW, settings);
}

//--------------------------------------------------------------------------
// Configure port with right settings
//--------------------------------------------------------------------------
void configure_port(int fd)
{
    struct termios port_settings;      // structure to store the port settings in
    bzero( &port_settings, sizeof( port_settings ) ) ;

    cfsetispeed(&port_settings, B9600);    // set baud rates
    cfsetospeed(&port_settings, B9600);

    port_settings.c_cflag = CRTSCTS | PARENB | CS8 | CREAD | CLOCAL;    // set even parity, stop bits, data bits
    port_settings.c_iflag = INPCK;
    port_settings.c_lflag = 0;

    port_settings.c_cc[ VMIN  ] = 0;
    port_settings.c_cc[ VTIME ] = 0;

    tcflush(fd, TCIFLUSH );
    set_port_settings(fd, &port_settings);    // apply the settings to the port
}


//--------------------------------------------------------------------------
// Handle CTRL+C and other kills
//--------------------------------------------------------------------------
void handle_SIGINT(int sig_num)
{
    g_running = false;
    pthread_cond_signal(&g_waitForActionCond);
}

//--------------------------------------------------------------------------
// Send message over dbus which indicates message on the ibus
//--------------------------------------------------------------------------
void dbus_send_ibus_msg(byte src, byte dst, byte* data, unsigned len)
{
    // create a signal and check for errors
    DBusMessage* msg = dbus_message_new_signal("/openbus/ibus/bmw", // object name of the signal
        "openbus.ibus", // interface name of the signal
        "msg"); // name of the signal
    if (NULL == msg)
    {
        fprintf(stderr, "Out Of Memory!\n");
        return;
    }

    // append arguments onto signal
    DBusMessageIter args;
    dbus_message_iter_init_append(msg, &args);

    // first byte is always our src and second byte is dst
    dbus_message_iter_append_basic(&args, DBUS_TYPE_BYTE, &src);
    dbus_message_iter_append_basic(&args, DBUS_TYPE_BYTE, &dst);

    // last bytes are data bytes of ibus msg
    {
        DBusMessageIter array_iter;
        dbus_message_iter_open_container(&args, DBUS_TYPE_ARRAY, DBUS_TYPE_BYTE_AS_STRING, &array_iter);
        dbus_message_iter_append_fixed_array(&array_iter, DBUS_TYPE_BYTE, &data, len);
        dbus_message_iter_close_container(&args, &array_iter);
    }

    // send the message and flush the connection
    dbus_connection_send(g_dbusConn, msg, NULL);
    dbus_connection_flush(g_dbusConn);

    // free the message
    dbus_message_unref(msg);


    // ------------------
    // Now we invoke a method call on the dst node sitting on dbus
    // ------------------

}

//--------------------------------------------------------------------------
// Filter incoming messages on dbus. So transmit msg to ibus 
//--------------------------------------------------------------------------
DBusHandlerResult dbus_filter_incoming(DBusConnection *connection, DBusMessage *message, void *user_data)
{
    // ok we've received an ibus formed msg signal, so send it to serial port
    if (dbus_message_is_signal (message, "openbus.ibus", "msg"))
    {
        // temporary variables
        dbus_uint64_t value;
        byte src = 0, dst = 0;
        byte* data = NULL;
        unsigned len = 0;

        // append arguments onto signal
        DBusMessageIter args;
        dbus_message_iter_init(message, &args);

        // first byte is always our src and second byte is dst
        dbus_message_iter_get_basic (&args, &value);
        int type = dbus_message_iter_get_arg_type (&args);
        if (type != DBUS_TYPE_BYTE || !dbus_message_iter_has_next(&args))
        {
            fprintf(stderr, "Received message is not well formed (error on/after src), check sender!\n");
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
        }
        src = value;
        dbus_message_iter_next(&args);

        // now read dst byte
        dbus_message_iter_get_basic (&args, &value);
        type = dbus_message_iter_get_arg_type (&args);
        if (type != DBUS_TYPE_BYTE || !dbus_message_iter_has_next(&args))
        {
            fprintf(stderr, "Received message is not well formed (error on/after dst), check sender!\n");
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
        }
        dst = value;
        dbus_message_iter_next(&args);

        // last bytes are data bytes of ibus msg
        type = dbus_message_iter_get_arg_type (&args);
        if (type != DBUS_TYPE_ARRAY)
        {
            fprintf(stderr, "Received message is not well formed (array of msg data is missing), check sender!\n");
            return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
        }
        {
            DBusMessageIter array_iter;
            dbus_message_iter_recurse(&args, &array_iter);
            type = dbus_message_iter_get_arg_type (&args);
            if (type != DBUS_TYPE_BYTE)
            {
                fprintf(stderr, "Received message is not well formed (data msg contains wrong type), check sender!\n");
                return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
            }

            len = 0;
            data = (byte*)malloc(RX_BUFFER_SIZE * sizeof(byte));
            while(dbus_message_iter_has_next(&array_iter) && len < RX_BUFFER_SIZE)
            {
                dbus_message_iter_get_basic(&array_iter, &value);
                data[len++] = (byte)value;
            }
        }

        // send message to the ibus
        ibus_send(src,dst,data,len, TX_RETRIES);
        
        // ok messasge was handled
        return DBUS_HANDLER_RESULT_HANDLED;
    }

    // inform that msg could not be handled yet
    return DBUS_HANDLER_RESULT_NOT_YET_HANDLED;
}

//--------------------------------------------------------------------------
// DBus thread
//--------------------------------------------------------------------------
void* dbus_thread(void* ptr)
{
    pthread_mutex_lock(&g_dbusMutex);

    dbus_connection_dispatch((DBusConnection*)ptr);

    pthread_mutex_unlock(&g_dbusMutex);

    if (g_dbusStop) return NULL;
}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // default settings
    char* deviceName = "/dev/ttyS0";
    char* dbusName = "openbus.ibus.bmw";
    
    // if no arguments, then stop
    if (argc < 1)
    {
        printf("%s [options] device\nEstablish a connection between IBus devices on a serial port with DBus applications. ", argv[0]);
        printf("Default device is %s\n", deviceName);
        printf("\t-b test.method.server - Name of the D-Bus connection, where to send messages, default: %s\n", dbusName);
        printf("\t-v Print version number\n");
        return EXIT_SUCCESS;
    }

    // read command arguments
    int i=1;
    while (i < argc)
    {
        if (!strcmp(argv[i],"-v"))
        {
            printf("%s - %d.%d\n", argv[0], VERSION_MAJOR, VERSION_MINOR);
            return EXIT_SUCCESS;
        }else if (!strcmp(argv[i],"-b"))
        {
            i++;
            free(dbusName);
            dbusName = (char*) malloc((strlen(argv[i]) + sizeof('\0')) * sizeof(char));
            strcpy(dbusName, argv[i]);
        }
        i++;
    }

    // get device name
    if (argc > 1)
    {
        //free (deviceName);
        deviceName = malloc(sizeof(char) * (strlen(argv[argc-1]) + 1));
        sprintf(deviceName, "%s", argv[argc-1]);
    }

    // open port and store old settings
    int device = open_port(deviceName);
    if (device < 0) exit(1);
    struct termios oldSettings;
    g_SerialDevice = device;

    get_port_settings(device, &oldSettings);
    configure_port(device);

    // install handler for catching user interrupts
    struct sigaction handler;
    handler.sa_handler = handle_SIGINT; //* <=== ERROR */
    handler.sa_flags = 0;
    sigemptyset(&handler.sa_mask);
    sigaction(SIGINT, &handler, NULL);

    // variables and buffers to hold
    struct timeval timer_old;

    // setup dbus
    DBusError err;
    dbus_error_init(&err);

    // connect to the bus
    DBusConnection* dBusConn = dbus_bus_get(DBUS_BUS_SESSION, &err);
    if (dbus_error_is_set(&err))
    {
        fprintf(stderr, "D-Bus Connection Error (%s)\n", err.message);
        dbus_error_free(&err);
    }
    if (dBusConn == NULL) exit(1);
    g_dbusConn = dBusConn;
    
    // request a name on the bus
    int ret = dbus_bus_request_name(dBusConn, dbusName, DBUS_NAME_FLAG_REPLACE_EXISTING, &err);
    if (dbus_error_is_set(&err))
    {
        fprintf(stderr, "Name Error (%s)\n", err.message);
        dbus_error_free(&err);
    }
    if (ret != DBUS_REQUEST_NAME_REPLY_PRIMARY_OWNER) exit(1);

    dbus_send_ibus_msg(IBUS_DEV_BMBT, IBUS_DEV_RAD, g_RxBuffer, 5);

    // add a rule for which messages we want to see
    dbus_bus_add_match(dBusConn, "type='signal',interface='openbus.ibus'", NULL); // see signals from the given interface
    dbus_connection_flush(dBusConn);

    // init pthread and set waitforaction mutex to locked state
    // only when it get unlocked
    //pthread_init();
    pthread_cond_init(&g_waitForActionCond, NULL);
    pthread_mutex_init(&g_waitForActionMutex, NULL);

    pthread_t dbusThread;
    pthread_mutex_init(&g_dbusMutex, NULL);
    pthread_create(&dbusThread, NULL, dbus_thread, dBusConn);
    g_dbusStop = false;
    
    // main loop
    while(g_running)
    {
        // wait as long as condition is not set
        pthread_cond_wait(&g_waitForActionCond,&g_waitForActionMutex);

        // go through the list of all messages in transmit queue and check if a message
        // is outdated, remove it then from the queue
        struct timeval tv;
        gettimeofday(&tv, NULL);
        struct IBusMessage* currentMsg = g_TxList;
        struct IBusMessage** prevMsg = &g_TxList;
        while (currentMsg != NULL)
        {
            if (currentMsg->valid_until < tv.tv_usec)
            {
                (*prevMsg)->next = currentMsg->next;
                currentMsg = currentMsg->next;
                free(currentMsg->data);
                free(currentMsg);
                continue;
            }
            
            prevMsg = &currentMsg;
            currentMsg = currentMsg->next;
        }

        // if we are in the idle state, then check if something has to be done
        if (g_State == IDLE)
        {
            // if we have data in the queue, then send as soon as
            if (g_TxList != NULL)
            {
                // check if bus is busy, then wait until it get free
                if (serial_port_get_cts(device))
                {
                    g_State = WAIT;
                    timer_start(IBUS_TIMEOUT_WAIT_FREE_BUS);
                    continue;
                }

                // ok bus is free and we have data to send, then start from the first one
                // in the queue and try to submit it
                struct IBusMessage* msg = g_TxList;

                if (msg->retries-- < 0)
                {
                    g_TxList = msg->next;
                    free(msg->data);
                    free(msg);
                    _sig_Timeout(SIGALRM);
                    continue;
                }

                // send bytewise and check if status of the control line changes
                // if changes, then stop transmitting and wait until it get free again

                // send SRC
                if (write(device, &msg->src, 1) != 1 || serial_port_get_cts(device))
                {
                    g_State = WAIT;
                    timer_start(IBUS_TIMEOUT_TX_COLLISION);
                    continue;
                }

                // send LEN
                byte len = msg->dataLength + 2;
                if (write(device, &len, 1) != 1 || serial_port_get_cts(device))
                {
                    g_State = WAIT;
                    timer_start(IBUS_TIMEOUT_TX_COLLISION);
                    continue;
                }

                // send DST
                if (write(device, &msg->dst, 1) != 1 || serial_port_get_cts(device))
                {
                    g_State = WAIT;
                    timer_start(IBUS_TIMEOUT_TX_COLLISION);
                    continue;
                }

                // send DATA
                len = msg->dataLength;
                byte* data = msg->data;
                while (len > 0)
                {
                    if (write(device, data, 1) != 1 || serial_port_get_cts(device))
                    {
                        g_State = WAIT;
                        timer_start(IBUS_TIMEOUT_TX_COLLISION);
                        continue;
                    }
                    data++;
                    len--;
                }

                // send CHK
                if (write(device, &msg->chk, 1) != 1 || serial_port_get_cts(device))
                {
                    g_State = WAIT;
                    timer_start(IBUS_TIMEOUT_TX_COLLISION);
                    continue;
                }

                // if everythign went fine, then remove this message from the list
                g_TxList = msg->next;
                free(msg->data);
                free(msg);

                g_State = WAIT;
                timer_start(IBUS_TIMEOUT_TX_AFTER_SEND);
            }
            // ok if we are not transmitting, then we have received something
            // so react on this adequately
            else if(g_RxPos >= g_RxLen)
            {
                // send ibus msg over dbus
                dbus_send_ibus_msg(g_RxBuffer[0], g_RxBuffer[2], &g_RxBuffer[3], g_RxBuffer[1]-2);

                // indicate that we now wait for free bus
                _sig_Timeout(SIGALRM);
            }
        }

        
        // short sleep, in order to process other system tasks
       // usleep(SLEEP_TIME);
    }

    g_dbusStop = true;
    pthread_join(dbusThread, NULL);
    
    // close dbus
    dbus_connection_unref(dBusConn);

    // close serial port
    set_port_settings(device, &oldSettings);
    close(device);

    // ok
    return (EXIT_SUCCESS);
}

