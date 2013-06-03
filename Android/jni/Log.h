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

#ifndef _LOG_H
#define	_LOG_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

class Logger
{
public:

    enum Level
    {
        INFO=0,
        DEBUG=1,
        WARN=2,
        ERROR=3,
        RAW=4
    };

    typedef void (*Callback)(const std::string&);
    
    Logger() : m_callback(NULL), m_level(INFO){}
    
    void log(Level level, const std::string& str);

    void lock() { m_mutex.lock(); }
    void unlock() { m_mutex.unlock(); }

    static Logger* instance() { return &g_logger; }
    static void slog(Level level, const std::string& s) { g_logger.log(level, s); }

    void setFile(const std::string& file) { m_logFile = file; }
    void setLevel(Level level) { m_level = level; }
    void setCallback(Callback cb) { m_callback = cb; }
    
    Level getLevel() const { return m_level; }
    
private:
    static Logger g_logger;
    std::string m_logFile;
    Callback m_callback;
    
    boost::mutex m_mutex;
    Level m_level;
};

#define LOG_RAW(s)    Logger::slog(Logger::RAW, std::string(s))
#define LOG_ERROR(s)  Logger::slog(Logger::ERROR, std::string("error: ") + std::string(s))
#define LOG_WARN(s)   Logger::slog(Logger::WARN, std::string("warn: ") + std::string(s))
#define LOG_DEBUG(s)  Logger::slog(Logger::DEBUG, std::string("debug: ") + std::string(s))
#define LOG_INFO(s)   Logger::slog(Logger::INFO, std::string("info: ") + std::string(s))

#endif	/* _LOG_H */

