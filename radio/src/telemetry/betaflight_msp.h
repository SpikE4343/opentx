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
#ifndef BETAFLIGHT_MSP_H
#define BETAFLIGHT_MSP_H

#include "opentx.h"
#include "betaflight_msp_msgs.h"

#define MSP_MSG_BUFFER_SIZE 256

// Protocol version
#define SPORT_MSP_VERSION (1 << 5)

#define SPORT_MSP_STARTFLAG (1 << 4)

// Sensor ID used by the local LUA script
#define LOCAL_SENSOR_ID 0x0D

// Sensor ID used by the MSP server (BF, CF, MW, etc...)
#define REMOTE_SENSOR_ID 0x1B

#define REQUEST_FRAME_ID 0x30
#define REPLY_FRAME_ID   0x32

enum BFMspDecodeState
{
    BF_MSP_DECODE_NONE,
    BF_MSP_DECODE_LISTENING,
    BF_MSP_DECODE_COLLECTING,
    BF_MSP_DECODE_COMPLETE,
    BF_MSP_DECODE_ERROR,
};

class BFMspDecoder
{
private:
  bool started;
  BFMspDecodeState state;
  uint8_t packetsReceived;
  uint8_t packetsStarted;
  uint8_t packetErrors;
  uint8_t orderErrors;
  uint8_t crcErrors;
  uint8_t payloadSize;
 
  int8_t sportMspRemoteSeq;

  uint8_t messagesReceived;
  uint8_t messageCrc;
  uint8_t messageSize;
 
  uint8_t messageBufferPosition;
  uint8_t messageBuffer[MSP_MSG_BUFFER_SIZE];
public:
  BFMspDecoder() 
    : started(false)
    , state(BF_MSP_DECODE_NONE)
    , packetsReceived(0)
    , packetsStarted(0)
    , packetErrors(0)
    , orderErrors(0)
    , crcErrors(0)
    , payloadSize(0)
    , sportMspRemoteSeq(0)
    , messagesReceived(0)
    , messageCrc(0)
    , messageSize(0)
    , messageBufferPosition(0)
  {
    
  }

  virtual ~BFMspDecoder() { }

public:
  bool decodePacket(SportTelemetryPacket& packet);

  uint8_t* getMessageBuffer() { return messageBuffer; }
  uint8_t getMessageSize() const { return messageSize; }
};

//static BFMspDecoder* s_msp = NULL;
#endif