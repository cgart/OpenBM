/***************************************************************************
 *                                                                         *
 *   (c) Art Tevs, MPI Informatik Saarbruecken                             *
 *       mailto: <tevs@mpi-sb.mpg.de>                                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/
#include "ibus.h"

// singleton
IBus IBus::mSingleton;

//--------------------------------------------------------------------------
IBus* IBus::initialize()
{
    // intialize pointer
    IBus* ptr = &IBus::mSingleton;

    // per default we do not use any uart interface
    ptr->mBusUart = NULL;

    // return interface pointer
    return ptr;
}

//--------------------------------------------------------------------------
void IBus::uartReceiveCallback(uint8_t c)
{

}


