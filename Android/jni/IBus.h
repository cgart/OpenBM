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

#ifndef _IBUS_H
#define	_IBUS_H

#include <vector>
#include <queue>
#include <string>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/asio/deadline_timer.hpp>
#include "ExtSerial.h"

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
#define IBUS_MSG_DEV_POLL           0x01    // Poll any device
#define IBUS_MSG_DEV_READY          0x02    // answer to poll message
#define IBUS_MSG_LAMP_STATE         0x5B    // Lamp state
#define IBUS_MSG_VEHICLE_CTRL       0x0C    // Vehicle Control (mostly used from diagnose)
#define IBUS_MSG_IKE_STATE          0x13    // IKE informs about its state
#define IBUS_MSG_MID_STATE_BUTTONS  0x20    // MID's main buttons (on/off, TEL, ...)
#define IBUS_MSG_UPDATE_MID_BOTTOM  0x21    // update information on text display
#define IBUS_MSG_MID_ACK_TEXT       0x22    // mid aknowledges that text was received
#define IBUS_MSG_UPDATE_MID_TOP     0x23    // update information on text display
#define IBUS_MSG_RADIO_ENCODER      0x32    // MID's radio encoder was rotated
#define IBUS_MSG_BMBT_BUTTON        0x48    // action with BMBT button
#define IBUS_MSG_BMBT_ENCODER       0x49    // BMBT encoder was rotated
#define IBUS_MSG_BUTTON             0x31    // MID's button state change
#define IBUS_MSG_UPDATE_MID_TOP_FREQ 0x24   // update frequency field of the radio
#define IBUS_MSG_LED                0x2B    // set status-LED state
#define IBUS_MSG_LED_SPECIAL        0x2D    // set status-LED state (special function, defining blink ratio)


#define IBUS_MSG_OPENBM_TO          0xFA    // message sent to OpenBM (not BMW specified)
#define IBUS_MSG_OPENBM_FROM        0xFB    // message received from OpenBM (not BMW specified)

#define IBUS_MSG_OPENBM_GET_VERSION 0x00    // second data byte: get version of the firmware
#define IBUS_MSG_OPENBM_GET_TICKS   0x01    // second data byte: get number of ticks
#define IBUS_MSG_OPENBM_GET_PHOTO   0x02    // second data byte: get value of the photo sensor
#define IBUS_MSG_OPENBM_GET_DIMMER  0x03    // second data byte: get value of the backlight dimmer
#define IBUS_MSG_OPENBM_GET_TEMP    0x04    // second data byte: get value of the temperature sensor
#define IBUS_MSG_OPENBM_SPECIAL_REQ 0xFF    // second data byte: special request message (i.e. update firmware)
#define IBUS_MSG_OPENBM_SETTINGS    0xFE    // second data byte: write/read settings of OpenBM to/from EEPROM

/**
 * Implementation for the Ibus living on the serial port.
 *
 * Received messages passed to the given callback function.
 *
 * WARNING: wenn passing a message to the callback, the main ibus thread blocks,
 * hence it might miss new messages if the callback function take too long. There is
 * a buffer which will be filled up, but it can take up only one message. Hence when
 * ibus thread blocks you would miss messages after next one. Take care
 * that your callback function returns immediately!
 * 
 * Priorities:
 *  Messages to be sent are put into a priority queue. Every
 *  message can have a priority indicating the order of messages to be sent.
 *  Higher priority number means, message will be send earlier then this with smaller number.
 *  Messages with same priority will be send in the order they were added to the queue.
 **/
class IBus
{
public:
    
    //! full message consists of bytes
    typedef unsigned char Data;

    //! Unique ID type of a message
    typedef unsigned MessageID;

    // default message id
    static const MessageID NO_ID = 0;

    // amount of bytes holding the received data (shall be enough to hold around 8 full blown-messages, or 200 average size messages)
    static const unsigned RX_BUFFER_SIZE = 2048;

    // how long to wait before timeout on recivie (in usec)
    static const unsigned IBUS_RX_TIMEOUT = 20000;

    // how long to wait for free bus (in usec)
    static const unsigned IBUS_TIMEOUT_WAIT_FREE_BUS = 2000;

    // how long to wait when collision while sending was detected (in usec)
    static const unsigned IBUS_TIMEOUT_TX_COLLISION_MIN = 2000;
    static const unsigned IBUS_TIMEOUT_TX_COLLISION_MAX = 10000;

    // how long to wait between sending of two messages (in usec)
    static const unsigned IBUS_TIMEOUT_TX_AFTER_SEND = 3000;
    static const unsigned IBUS_TIMEOUT_EMPTY = 1;

    /**
     * Exception class which will be thrown when something gone bade with
     * ibus messages.
     **/
    class Error : public std::exception
    {
    public:
        virtual std::string asString() const = 0;
    };

    class MessageHasBadLength : public Error
    {
    public:
        virtual std::string asString() const { return "Message has wrong amount of bytes, must be at least 5"; }
    };
    class MessageHasBadChecksum : public Error
    {
    public:
        virtual std::string asString() const { return "Message has wrong checksum"; }
    };
    class MessageNotWellFormed : public Error
    {
    public:
        virtual std::string asString() const { return "Message is not well formed one, check if you initialize raw data properly"; }
    };
    class MessageNotEqual : public Error
    {
    public:
        virtual std::string asString() const { return "Message is not equal to the given one"; }
    };

    /**
     * Class representing an ibus message
     **/
    class Message
    {
    public:
        // default number of trials before message got removed from tx queue
        static const int defaultNumberOfTrials = 5;

        // default priority of a message
        static const int defaultPriority = 100;

        //! Create an empty message which is meant to be send from node s to node d
        Message(Data src, Data dst, Data data, int priority = defaultPriority);

        //! Copy constructor for a message
        Message(const Message& msg);
        Message& operator=(const Message& msg);

        //! Create message from a data buffer (i.e. full ibus message)
        Message(Data* rawBuffer, unsigned size);

        //! fill message with data
        void setData(const std::vector<Data>& data);

        //! Get full data vector of the message
        std::vector<Data> getData() const;

        //! Set how often you would like this message be resend (MAX_INT for almost infinite :) )
        inline void setTrials(unsigned trials) { m_Trials = trials; }
        inline int getTrials() const { return m_Trials; }

        //! get source node of the message
        inline Data getSource() const { if (m_raw.size() < 5) throw MessageHasBadLength(); return m_raw[0]; }

        //! get destionation node of the message
        inline Data getDestination() const { if (m_raw.size() < 5) throw MessageHasBadLength(); return m_raw[2]; }

        //! get first data byte of the message
        inline Data getFirstDataByte() const { if (m_raw.size() < 5) throw MessageHasBadLength(); return m_raw[3]; }

        //! get message checksum
        inline Data getChecksum() const { if (m_raw.size() < 5) throw MessageHasBadLength(); return m_raw[m_raw.size()-1]; }
        
        //! Get priority of a message
        inline int getPriority() const { return m_Priority; }

        //! We compare two messages by their priority
        inline bool operator<(const Message& b) const
        {
            return (m_Priority == b.m_Priority) ? m_ID > b.m_ID : m_Priority < b.m_Priority;
        }

        // compare if two messages are the same
        inline bool operator==(const Message& b) const
        {
            if (m_raw.size() != b.m_raw.size()) return false;
            
            for (unsigned i=0; i < m_raw.size(); i++)
                if (m_raw[i] != b.m_raw[i]) return false;
            return true;
        }
        inline bool operator!=(const Message& b) const { return !operator==(b); }
        
        //! Get message unique id
        inline MessageID getID() const { return m_ID; }

        //! Assign an ID to the message (use this only if you know what you are doing!)
        inline void setID(MessageID id) { m_ID = id; }

        //! Return string containing raw message (hex number separated by empty space)
        std::string rawToString();

        //! Convert message into its raw representation (byte string as it will be send throug the bus)
        inline const std::vector<Data>& getRaw() const { return m_raw; }
        
        //! Parse message from a specially formatted string (ss,dd,mm1,mm2,...)
        static Message* parseFromSDMString(const std::string&);
        
    private:
        friend class IBus;
        
        //! Compute checksum of a message (xor add of all raw bytes)
        Data computeChecksum();

        std::vector<Data> m_raw;
        unsigned m_Trials;
        int m_Priority;

        MessageID m_ID;
    };

    //! Create ibus instance
    IBus();

    //! Close instance and release memory, also the serial port will be closed if open
    virtual ~IBus();

    //! Put new message into send queue and returns new message id
    MessageID send(Message* msg);

    enum CTSType
    {
        NO,
        HARDWARE,
        GPIO
    };
    
    //! Open port to use for ibus communication (e.g. on linux /dev/ttyS0, COM0 on windows)
    bool open(const std::string& port, CTSType type = HARDWARE);

    //! Close serial port (is automatically called in destructor)
    void close();
    
    //! Start IBus thread, call this before the main loop
    void start();
    
    //! Stop IBus running, this will stop the main ibus thread
    void stop();

    //! Get state of the CTS line (can be used for hardware collision detection)
    bool getCTS();

    //! Type of callback function when new message over ibus is received
    typedef boost::function<void (const IBus::Message& msg)> OnMessageReceiveCallback;

    //! Set callback function to react on new messages (Note: the execution of IBus thread is blocked, until the callback returns)
    inline void setMessageReceiveCallback(OnMessageReceiveCallback cb) { m_msgRxCallback = cb; }

    //! Add new system event (call with system(str);) to be executed if specified message was received
    void addSystemEvent(const Message& msg, const std::string& str) { m_systemEventList.push_back(std::pair<Message,std::string>(msg,str)); }

    //! Type of callback function when ibus thread stopps
    typedef boost::function<void (void)> OnStopIBusThreadCallback;
    inline void setStopIBusThreadCallback(OnStopIBusThreadCallback cb) { m_stopIBusThreadCallback = cb; }
    
protected:

    CTSType mCTSType;
    
    struct CompareMessagePointer
    {
        bool operator()(Message* a, Message* b)
        {
            return (*a) < (*b);
        }
    };

    void run_ibus();
    void run_ioservice();
    void prepareReceive(bool startTimer);
    void onReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void timeOut(const boost::system::error_code& error);
    void timeWait(unsigned microseconds);
    
    // callback function executed on new message
    OnMessageReceiveCallback m_msgRxCallback;
    OnStopIBusThreadCallback m_stopIBusThreadCallback;
    
    // Serial io service implementation from boost (for timers and serial port)
    boost::asio::io_service m_ioService;

    // timer used for timeouts detection
    boost::asio::deadline_timer m_Timer;

    // serial port
    extserial::serial_port m_Serial;
    
    // thread executing the ibus code
    boost::shared_ptr<boost::thread> m_iBusThread;
    boost::shared_ptr<boost::thread> m_ioThread;

    // condition variable marking that there is somethign happens on the ibus
    boost::condition_variable m_actionCond;
    
    // indicate if we should stop execution
    bool m_running;
    
    // priority queue holding all the messages
    std::priority_queue<Message*, std::vector<Message*>, CompareMessagePointer > m_TxQueue;
    std::queue<Message*> m_RxQueue;
    boost::mutex m_rxQueueMutex;
    boost::mutex m_txQueueMutex;
    Message m_lastSendMessage;
    
    // last id given to a message
    MessageID m_lastID;

    // buffer for storing currently received message
    boost::array<Data, RX_BUFFER_SIZE> m_buffer;
    std::size_t m_bufferMsgLength;
    std::size_t m_bufferMsgPos;

    // current state of the ibus service
    enum State{
        IDLE,
        WAIT,
        RECEIVE,
        TRANSMIT
    };

    State m_State;

    // list of system events to be executed on message receive
    typedef std::pair<Message, std::string> EventPair;
    typedef std::vector<EventPair> EventList;
    EventList m_systemEventList;
    
};

#endif	/* _IBUS_H */

