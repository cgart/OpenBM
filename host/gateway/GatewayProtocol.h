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


#ifndef _GATEWAYPROTOCOL_H
#define	_GATEWAYPROTOCOL_H

namespace openbm
{
    // -------------------------------------------------------------------------
    // Defaults
    // -------------------------------------------------------------------------
    const unsigned short WebPort = 8080;
    const unsigned short Port = 4287;
    const std::string Address = "127.0.0.1";
    const unsigned ClientAliveTimeout = 10000;
    const unsigned ClientPingTime = 3000;
    const unsigned ClientRetryPing = 3;
    const unsigned ServerMaxNumClients = 8;

    // -------------------------------------------------------------------------
    // Messages
    // -------------------------------------------------------------------------
    class MessageHeader
    {
        public:
            MessageHeader(unsigned char s, unsigned char d, short p, unsigned char l) : src(s), dst(d), len(l), lenx(0), priority(p), reserved2(0), reserved3(0) {}
            unsigned char src;
            unsigned char dst;
            unsigned char len;
            unsigned char lenx;

            short priority;
            
            // reserved and are completely ignored now
            unsigned char reserved2;
            unsigned char reserved3;
    };
    class ControlMessage : public MessageHeader
    {
        public:
            ControlMessage(unsigned char cmd1, unsigned char cmd2) : MessageHeader(cmd1, cmd2, 0, 0) {}

            static bool isControlMessage(const MessageHeader& msg) { if (msg.len == 0) return true; return false; }
    };
    class HelloMessage : public ControlMessage
    {
        public:
            HelloMessage() : ControlMessage('h', 'i') {}

            static bool isHelloMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return false;
                if (msg.src == 'h' && msg.dst == 'i') return true;
                return false;
            }
    };
    class ConnectToMessage : public ControlMessage
    {
        public:
            ConnectToMessage(unsigned short port) : ControlMessage('c', 't') { priority = (short)port; }

            // if this is a connect to message, then return port where to connect, otherwise 0
            static unsigned short isConnectToMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return 0;
                if (msg.src == 'c' && msg.dst == 't') return (unsigned short)msg.priority;
                return 0;
            }
    };
    class PingMessage : public ControlMessage
    {
        public:
            PingMessage() : ControlMessage(0xAA, 0xAA) {}

            static bool isPingMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return false;
                if (msg.src == 0xAA && msg.dst == 0xAA) return true;
                return false;
            }
    };
    class DisconnectMessage : public ControlMessage
    {
        public:
            DisconnectMessage() : ControlMessage(0,0) {}

            static bool isDisconnectMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return false;
                if (msg.src == 0 && msg.dst == 0) return true;
                return false;
            }
    };
    
    //--------------------------------------------------------------------------
    //            Messages for Firmware update handling 
    //            the updates are performed by the Gateway
    //--------------------------------------------------------------------------
    class GetFirmwareVersionMessage : public ControlMessage
    {
        public:
            GetFirmwareVersionMessage() : ControlMessage('f', 'g') {}

            static bool isGetFirmwareVersionMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return false;
                if (msg.src == 'f' && msg.dst == 'g') return true;
                return false;
            }

            void setVersion(unsigned char major, unsigned char minor) { priority = ((unsigned short)major << 8) | minor; }
            unsigned char getMajor() const { return priority >> 8; }
            unsigned char getMinor() const { return priority & 0xFF; }
    };
    class InitFirmwareUpdateMessage : public ControlMessage
    {
        public:
            InitFirmwareUpdateMessage(const std::vector<unsigned char>& pwd) : ControlMessage('f', 'i')
            {
                reserved2 = pwd[2];
                reserved3 = pwd[3];
                priority = ((unsigned short)pwd[0] << 8) | pwd[1];
            }

            static bool isInitFirmwareUpdateMessage(const MessageHeader& msg)
            {
                if (!ControlMessage::isControlMessage(msg)) return false;
                if (msg.src == 'f' && msg.dst == 'i') return true;
                return false;
            }

            void getPassword(std::vector<unsigned char>& pwd)
            {
                pwd.resize(4,0);
                pwd[0] = (priority >> 8) & 0xFF;
                pwd[1] = priority & 0xFF;
                pwd[2] = reserved2;
                pwd[3] = reserved3;
            }
    };
    class UploadFirmwareMessage : public ControlMessage
    {
        public:
            UploadFirmwareMessage(unsigned short dataLength, unsigned short dataChecksum, unsigned char verMajor, unsigned char verMinor) : ControlMessage('f', 'u')
            {
                len = (dataLength >> 8) & 0xFF;
                lenx = dataLength & 0xFF;

                priority = dataChecksum;

                reserved2 = verMajor;
                reserved3 = verMinor;
            }

            static bool isUploadFirmwareMessage(const MessageHeader& msg)
            {
                if (msg.reserved3 == 0 && msg.reserved2 == 0) return false;
                if (msg.src == 'f' && msg.dst == 'u') return true;
                return false;
            }

            unsigned char getMajorVersion() const { return reserved2; }
            unsigned char getMinorVersion() const { return reserved3; }
            unsigned short getChecksum() const { return priority; }
            unsigned short getErrorCode() const { return priority; }
            unsigned short getDataLength() const { return ((unsigned short)len << 8) | (unsigned short)lenx; }
    };

};

#endif	/* _GATEWAYPROTOCOL_H */

