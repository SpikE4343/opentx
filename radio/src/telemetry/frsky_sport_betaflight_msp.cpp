/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"

#include "betaflight_msp.h"

bool sendMsg( uint8_t* buffer, uint8_t size)
{
    int stepBytes = sizeof(uint32_t);
    int fullPacketCount = size / stepBytes;
    
    SportTelemetryPacket packet;
    packet.physicalId = getDataId(LOCAL_SENSOR_ID);
    packet.primId = REQUEST_FRAME_ID;
    packet.dataId = ;
    
    for( int i=0; i < fullPacketCount; ++i )
    {
        packet.value = (uint32_t)buffer[i*stepBytes];
        sportTelemetryPush(LOCAL_SENSOR_ID, REQUEST_FRAME_ID, dataId, value)
        sportOutputPushPacket( )
        size -= stepBytes;
    }

    if( size > 0 )
    {

    }

    
}


bool BFMspDecoder::decodePacket(SportTelemetryPacket& packet)
{
    ++packetsReceived;

    uint8_t idx      = 2;
    uint8_t head     = packet.raw[idx];
    bool err_flag = (head & 0x20) != 0;
    ++idx;

    if(err_flag)
    {
        // error flag set
        started = false;
        ++packetErrors;

        debugPrintf("msp error: %d", packetErrors);

        // return error
        // CRC checking missing

        //return payload[idx]
        return false;
    }

    bool start = (head & 0x10) != 0;
    uint8_t seq = head & 0x0F;

    // switch(state)
    // {
    //   case BF_MSP_DECODE_NONE:
    //     state = BF_MSP_DECODE_LISTENING;
    //     // fall through intended
    //   case BF_MSP_DECODE_LISTENING:
    //     break;

    //   case BF_MSP_DECODE_COLLECTING:
    //     break;

    //   case BF_MSP_DECODE_COMPLETE:
    //     break;

    //   case BF_MSP_DECODE_ERROR:
    //     break;
    // }

    if( start )
    {
        // start flag set
        messageBufferPosition = 0;

        messageSize = packet.raw[idx];
        //messageCrc  = messageSize ^ lastReq;
        ++idx;
        started = true;

        ++packetsStarted;
    }
    else if( !started )
    {
        ++orderErrors;
        return false;
    }
    else if ( ((sportMspRemoteSeq + 1) & 0x0F) != seq )
    {
        ++orderErrors;
        started = false;
        return false;
    }

    while (idx <= 6 && messageBufferPosition <= messageSize)
    {
        messageBuffer[messageBufferPosition] = packet.raw[idx];
        messageCrc = messageCrc ^ packet.raw[idx];
        ++messageBufferPosition;
        ++idx;
    }

    if( idx > 6 )
    {
        sportMspRemoteSeq = seq;
        return true;
    }

    // check CRC
    if( messageCrc != packet.raw[idx])
    {
        started = false;
        ++crcErrors;
        debugPrintf("msp errors: %d\r\n", crcErrors);
        return false;
    }

    ++messagesReceived;
    started = false;

    debugPrintf("msp m: %d, p: %d, pe: %d\r\n", 
        messagesReceived, 
        packetsReceived, 
        packetErrors);

    return true;
}

//static BFMspDecoder* s_msp = NULL;
