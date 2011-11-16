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

#ifndef _EXTSERIAL_H
#define	_EXTSERIAL_H

#include <boost/asio/serial_port.hpp>
#include <boost/asio/detail/descriptor_ops.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

#if defined(GENERATING_DOCUMENTATION)
# define BOOST_ASIO_OPTION_STORAGE implementation_defined
#elif defined(BOOST_WINDOWS) || defined(__CYGWIN__)
# define BOOST_ASIO_OPTION_STORAGE DCB
#else
# define BOOST_ASIO_OPTION_STORAGE termios
#endif

namespace extserial
{
    /**
     * CTS bit gettable option. Used to check if CTS bit is set
     **/
    class cts_bit
    {
    public:
        explicit cts_bit(bool bit = false) : value_(bit) {}
        inline bool value() const { return value_; }
        inline boost::system::error_code store(int& storage, boost::system::error_code& ec) const
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            if (value_) storage |= TIOCM_CTS;
            else storage &= ~(TIOCM_CTS);
        #endif
            ec = boost::system::error_code();
            return ec;
        }
        inline boost::system::error_code load(const int& storage, boost::system::error_code& ec)
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            value_ = ((storage & TIOCM_CTS) == TIOCM_CTS);
        #endif
            ec = boost::system::error_code();
            return ec;
        }
    private:
        bool value_;
    };

    /**
     * Disbale enable byte receive option. Can be used with boost's asio
     * to control receive line. User serialPort::set_option(...)
     **/
    class receive_mode
    {
    public:
        explicit receive_mode(bool enable = true) : value_(enable) {}
        inline bool value() const { return value_; }
        boost::system::error_code store(BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& ec) const
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            if (value_) storage.c_cflag |= CREAD;
            else storage.c_cflag &= ~CREAD;
        #endif
            ec = boost::system::error_code();
            return ec;
        }
        boost::system::error_code load(const BOOST_ASIO_OPTION_STORAGE& storage, boost::system::error_code& ec)
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            value_ = (storage.c_cflag & CREAD) == CREAD;
        #endif
            ec = boost::system::error_code();
            return ec;
        }
        inline void set(bool enable) { value_ = enable; }
    private:
        bool value_;
    };

    /**
     * Extended implementation of boost's asio serial port service.
     * The implementation supports of getting state bits like (CTS, DTR, ...)
     **/
    class serial_port_service : public boost::asio::serial_port_service
    {
    public:
        explicit serial_port_service(boost::asio::io_service& io_service) :
                boost::asio::serial_port_service(io_service)
        {

        }

        /// Get a serial port option.
        template <typename GettableSerialPortBit>
        boost::system::error_code get_bit(implementation_type& impl,
            GettableSerialPortBit& bit, boost::system::error_code& ec)
        {
            int status;
            //boost::asio::detail::descriptor_ops::state_type state;
            
            //int res = 
            boost::asio::detail::descriptor_ops::error_wrapper(::ioctl(native(impl), TIOCMGET, &status), ec);
            //ec = boost::system::error_code();
            //boost::asio::detail::descriptor_ops::ioctl(native(impl), state, TIOCMGET, &status, ec);
            if (ec) return ec;

            return bit.load(status, ec);
        }

        /// Flush serial port buffers
        boost::system::error_code flush(implementation_type& impl, boost::system::error_code& ec, bool inputBuf = true, bool outputBuf = true)
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            if (!inputBuf && !outputBuf) return ec;
            int arg = TCIOFLUSH;
            if (!inputBuf && outputBuf) arg = TCOFLUSH;
            if (inputBuf && !outputBuf) arg = TCIFLUSH;
            boost::asio::detail::descriptor_ops::error_wrapper(::tcflush(native(impl), arg), ec);
        #endif
            return ec;
        }
        /// Drain transmit buffers (block until completly send)
        boost::system::error_code drain(implementation_type& impl, boost::system::error_code& ec)
        {
        #if defined(BOOST_WINDOWS) || defined(__CYGWIN__)
            #error "Not implemented for windows system currently!"
        #else
            //boost::asio::detail::descriptor_ops::error_wrapper(::tcdrain(native(impl)), ec);
        #endif
            return ec;
        }
    };


    /**
     * Extended implementation of boost's asio serial port.
     * The implementation supports of getting state bits like (CTS, DTR, ...)
     **/
    class serial_port : public boost::asio::basic_serial_port<serial_port_service>
    {
    public:
        explicit serial_port(boost::asio::io_service& io_service) :
                boost::asio::basic_serial_port<serial_port_service>(io_service)
        {

        }

        /**
         * Get port BITs like CTS, DTR, etc...
         **/
        template <typename GettableSerialPortBit>
        void get_bit(GettableSerialPortBit& bit)
        {
            boost::system::error_code ec;
            service.get_bit(implementation, bit, ec);
            boost::asio::detail::throw_error(ec);
        }

        /**
         * Flush serial port buffers.
         **/
        void flush(bool inputBuffer = true, bool outputBuffer = true)
        {
            boost::system::error_code ec;
            service.flush(implementation, ec, inputBuffer, outputBuffer);
            boost::asio::detail::throw_error(ec);
        }

        /**
         * Drain serial port transmit buffer.
         **/
        void drain()
        {
            boost::system::error_code ec;
            service.drain(implementation, ec);
            boost::asio::detail::throw_error(ec);
        }
    };

}; // end namespace

#undef BOOST_ASIO_OPTION_STORAGE

#endif	/* _EXTSERIAL_H */
