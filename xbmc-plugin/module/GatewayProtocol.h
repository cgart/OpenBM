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
    class MessageHeader
    {
        public:
            MessageHeader(unsigned char s, unsigned char d, short p, unsigned char l) : src(s), dst(d), len(l), priority(p) {}
            unsigned char src;
            unsigned char dst;
            unsigned char len;
            unsigned char reserved1;

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
};

#endif	/* _GATEWAYPROTOCOL_H */

