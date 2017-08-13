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

 // TODO: move to build configuration 


#ifndef BETAFLIGHT_MSP_H
#define BETAFLIGHT_MSP_H

//#define BETAFLIGHT_MSP_SIMULATOR

#include "opentx.h"
#include "betaflight_msp_msgs.h"

#define MSP_MSG_BUFFER_SIZE 256

// Protocol version
#define SPORT_MSP_VERSION (1)

#define SPORT_MSP_STARTFLAG (1 << 4)

// Sensor ID used by the local LUA script
#define LOCAL_SENSOR_ID 0x0D

// Sensor ID used by the MSP server (BF, CF, MW, etc...)
#define REMOTE_SENSOR_ID 0x1B

#define REQUEST_FRAME_ID 0x30
#define REPLY_FRAME_ID   0x32

struct BFConfigVTX_t
{
  uint8_t deviceType;
  uint8_t band;
  uint8_t channel;
  uint8_t powerId;
  uint8_t pitmode;
};

struct BFConfig_t
{
  uint8_t maxRateProfileNum;
  uint8_t rateProfile;

  uint8_t pids[10][3];

  uint8_t rcRate;
  uint8_t rcExpo;
  uint8_t rates[3]; // Roll, Pitch, Yaw

  uint8_t dynThrPID;
  uint8_t thrMid;
  uint8_t thrExpo;
  uint16_t tpa_breakpoint;
  uint8_t rcYawExpo;
  uint8_t rcYawRate;

  BFConfigVTX_t videoTx;
};

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
  bool complete;
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
    , complete(false)
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
  void reset() 
  {
    started = false;
    complete = false;
  }

  bool hasStarted() const { return started;  }
  bool isComplete() const { return complete; }

  bool decodePacket(SportTelemetryPacket& packet);

  uint8_t* getMessageBuffer() { return messageBuffer; }
  uint8_t getMessageSize() const { return messageSize; }
};

class BFMspEncoder
{
private: 
  uint8_t messageSequence;
  uint8_t messageCrc;
  uint8_t* messageBuffer;
  uint8_t messageBufferPosition;
  uint8_t messageSize;
  uint8_t mspMessageType;

  BFMspRequestFrameHeader_t mspHeader;
public:
  BFMspEncoder()
    : messageSequence(0)
    , messageCrc(0)
    , messageBuffer(NULL)
    , messageBufferPosition(0)
    , messageSize(0)
    , mspMessageType(0)
  {

  }

  virtual ~BFMspEncoder() { }

public:

  // returns true when message buffer is empty or unset
  bool isComplete() const 
  {
    return mspMessageType == 0;
  }

  // Translate message to smart port telemetry packets
  bool encodeMessage(uint8_t cmd, uint8_t* buffer, uint8_t size);

  // set next packet based on buffer contents returns false when 
  // no more packets are needed
  bool fillNextPacket(SportTelemetryPacket& packet);

private:
  void reset();
};

//static BFMspDecoder* s_msp = NULL;


#ifdef BETAFLIGHT_MSP_SIMULATOR
void simHandleSmartPortMspFrame(uint8_t* sp_frame);
void initBFSimulatorConfig();
#endif

#endif