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

#include <iostream>
#include <boost/date_time.hpp>
#include <sstream>
#include <fstream>

Logger Logger::g_logger;

//------------------------------------------------------------------------------
void Logger::log(Level level, const std::string& str)
{
    using namespace std;

    boost::mutex::scoped_lock lock(m_mutex);

    if (level < m_level) return;
    
    boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
        
    string outstr;
    stringstream stream(outstr);
    stream << boost::posix_time::to_simple_string(time) << ": " << str << std::endl;

    if (m_logFile.length() > 0)
    {
        fstream file;
        file.open(m_logFile.c_str(), ios_base::out | ios_base::app);
        file << stream.str();
    }else if (m_callback)
    {
        m_callback(stream.str());
    }else
    {
        clog << stream.str();
    }
}

