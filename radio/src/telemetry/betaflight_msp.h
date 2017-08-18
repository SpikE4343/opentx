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

// simple buffer-based serializer/deserializer without implicit size check
// little-endian encoding implemneted now

typedef struct sbuf_s {
  uint8_t *ptr;          // data pointer must be first (sbuff_t* is equivalent to uint8_t **)
  uint8_t *end;
} sbuf_t;

void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val);
void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);

uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(sbuf_t *dst, void *data, int len);

int sbufBytesRemaining(sbuf_t *buf);
int sbufSizeBytes(sbuf_t *buf, int totalSize);

uint8_t* sbufPtr(sbuf_t *buf);
const uint8_t* sbufConstPtr(const sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);

void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);




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
  uint8_t* messageBuffer;

public:
  BFMspDecoder(uint8_t* buffer) 
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
    , messageBuffer(buffer)
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


class BetaflightController
{
public:
  enum MessageStateType
  {
    MS_IDLE,
    MS_SENDING,
    MS_RECEIVING,
    MS_RECEIVED
  };

  enum ConnectionStateType
  {
    CS_NONE,
    CS_DISCONNECTED,
    CS_CONNECTING,
    CS_CONNECTED,
    CS_DISCONNECTING
  };

  BetaflightController()
    : messageState(MS_IDLE)
    , connectionState(CS_NONE)
    , messageType(MSP_NONE)
    , decoder(&messageBuffer[0])
    , messageSize(0)
  {
  }

  BFConfig_t config;
 
public:

  void update();
  bool createSaveMessage(uint8_t msgType, sbuf_t* buf);
  bool sendMessage(uint8_t type, uint8_t* msg, uint8_t size);

  uint8_t getMessageType() const { return messageType; }
  uint8_t* getMessageBuffer() { return messageBuffer; }
  uint8_t getMessageSize() const { return messageSize; }

  bool isConnected() const { return connectionState == CS_CONNECTED; }

  ConnectionStateType getConnectionState() const { return connectionState; }
  MessageStateType getMessageState() const { return messageState; }

private:

  bool sendMessage();
  bool recvMessage();

  void updateConnection();
  void updateMessage();

  bool processReply();

  uint8_t messageType = MSP_NONE;
  uint8_t messageBuffer[MSP_MSG_BUFFER_SIZE];
  uint8_t messageSize;

  ConnectionStateType connectionState;
  MessageStateType messageState;
 
  BFMspDecoder decoder;
  BFMspEncoder encoder;
};


#ifdef BETAFLIGHT_MSP_SIMULATOR
void simHandleSmartPortMspFrame(uint8_t* sp_frame);
void initBFSimulatorConfig();
#endif

#endif