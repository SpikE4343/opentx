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
#include "stdio.h"

#include "betaflight_msp.h"

bool sendMsg(uint8_t* buffer, uint8_t size)
{
  int stepBytes = sizeof(uint32_t);
  int fullPacketCount = size / stepBytes;

  /* SportTelemetryPacket packet;
   packet.physicalId = getDataId(LOCAL_SENSOR_ID);
   packet.primId = REQUEST_FRAME_ID;
   packet.dataId = ;

   for( int i=0; i < fullPacketCount; ++i )
   {
       packet.value = (uint32_t)buffer[i*stepBytes];
       sportTelemetryPush(LOCAL_SENSOR_ID, REQUEST_FRAME_ID, dataId, value)
       sportOutputPushPacket( )
       size -= stepBytes;
   }*/

  if (size > 0)
  {

  }

  return false;
}

void BFMspEncoder::reset()
{
  memset(&mspHeader, 0, sizeof(mspHeader));
  mspMessageType = 0;
  messageSequence = 0;
  messageCrc = 0;
  messageBuffer = NULL;
  messageBufferPosition = 0;
  messageSize = 0;
}

bool BFMspEncoder::encodeMessage(uint8_t cmd, uint8_t* buffer, uint8_t size)
{
  // already sending a message
  if (mspMessageType != 0)
  {
    return false;
  }

  reset();

  mspMessageType = cmd;
  messageBuffer = buffer;
  messageSize = size;

  return true;
}

#define BIT(x, index) (((x) >> index) & 0x01)
uint8_t getLuaDataId(uint8_t physicalId)
{
  uint8_t result = physicalId;
  result += (BIT(physicalId, 0) ^ BIT(physicalId, 1) ^ BIT(physicalId, 2)) << 5;
  result += (BIT(physicalId, 2) ^ BIT(physicalId, 3) ^ BIT(physicalId, 4)) << 6;
  result += (BIT(physicalId, 0) ^ BIT(physicalId, 2) ^ BIT(physicalId, 4)) << 7;
  return result;
}

bool BFMspEncoder::fillNextPacket(SportTelemetryPacket & packet)
{
  if (isComplete())
  {
    return false;
  }

  uint8_t pos = 3;
  memset(&packet, 0, sizeof(packet));
  memset(&mspHeader, 0, sizeof(mspHeader));

  mspHeader.sequence = messageSequence++;
  packet.physicalId = getLuaDataId(LOCAL_SENSOR_ID);
  packet.primId = REQUEST_FRAME_ID;
  mspHeader.version = SPORT_MSP_VERSION;

  if (mspHeader.sequence == 0)
  {
    // first packet
    mspHeader.start = true;
    // add second byte of full message size
    packet.raw[pos++] = messageSize;
    packet.raw[pos++] = mspMessageType;

    messageCrc = messageSize ^ mspMessageType;
  }

  // write header byte
  packet.raw[2] = *((uint8_t*)(&mspHeader));

  // write message
  while (pos < sizeof(packet) && messageBufferPosition < messageSize)
  {
    uint8_t value = messageBuffer[messageBufferPosition++];
    messageCrc ^= value;
    packet.raw[pos++] = value;
  }

  if (messageBufferPosition >= messageSize -1)
  {
    packet.raw[pos++] = messageCrc;
    reset();
  }

  while (pos < sizeof(packet))
  {
    packet.raw[pos++] = 0;
  }
  
  return true;
}

bool BFMspDecoder::decodePacket(SportTelemetryPacket& packet)
{
  ++packetsReceived;

  uint8_t idx = 2;
  uint8_t head = packet.raw[idx];
  bool err_flag = (head & 0x20) != 0;
  ++idx;

  if (err_flag)
  {
    // error flag set
    started = false;
    ++packetErrors;

    debugPrintf("msp error: %d, %d", err_flag, packetErrors);

    // CRC checking missing
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

  if (start)
  {
    // start flag set
    messageBufferPosition = 0;

    messageSize = packet.raw[idx];
    //messageCrc  = messageSize ^ lastReq;
    ++idx;
    started = true;

    ++packetsStarted;
  }
  else if (!started)
  {
    ++orderErrors;
    return false;
  }
  else if (((sportMspRemoteSeq + 1) & 0x0F) != seq)
  {
    ++orderErrors;
    started = false;
    return false;
  }

  while (idx < sizeof(packet) && messageBufferPosition <= messageSize)
  {
    messageBuffer[messageBufferPosition] = packet.raw[idx];
    messageCrc = messageCrc ^ packet.raw[idx];
    ++messageBufferPosition;
    ++idx;
  }

  if (idx >= sizeof(packet))
  {
    sportMspRemoteSeq = seq;
    return false;
  }

  // check CRC
  /*if (messageCrc != packet.raw[idx])
  {
    started = false;
    ++crcErrors;
    debugPrintf("msp errors: %d\r\n", crcErrors);
    return false;
  }*/

  ++messagesReceived;
  //started = false;

  debugPrintf("msp m: %d, p: %d, pe: %d\r\n",
    messagesReceived,
    packetsReceived,
    packetErrors);

  complete = true;

  return true;
}







void sbufWriteU8(sbuf_t *dst, uint8_t val)
{
  *dst->ptr++ = val;
}

void sbufWriteU16(sbuf_t *dst, uint16_t val)
{
  sbufWriteU8(dst, val >> 0);
  sbufWriteU8(dst, val >> 8);
}

void sbufWriteU32(sbuf_t *dst, uint32_t val)
{
  sbufWriteU8(dst, val >> 0);
  sbufWriteU8(dst, val >> 8);
  sbufWriteU8(dst, val >> 16);
  sbufWriteU8(dst, val >> 24);
}

void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val)
{
  sbufWriteU8(dst, val >> 8);
  sbufWriteU8(dst, (uint8_t)val);
}

void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val)
{
  sbufWriteU8(dst, val >> 24);
  sbufWriteU8(dst, val >> 16);
  sbufWriteU8(dst, val >> 8);
  sbufWriteU8(dst, (uint8_t)val);
}


void sbufWriteData(sbuf_t *dst, const void *data, int len)
{
  memcpy(dst->ptr, data, len);
  dst->ptr += len;
}

void sbufWriteString(sbuf_t *dst, const char *string)
{
  sbufWriteData(dst, string, strlen(string));
}

uint8_t sbufReadU8(sbuf_t *src)
{
  return *src->ptr++;
}

uint16_t sbufReadU16(sbuf_t *src)
{
  uint16_t ret;
  ret = sbufReadU8(src);
  ret |= sbufReadU8(src) << 8;
  return ret;
}

uint32_t sbufReadU32(sbuf_t *src)
{
  uint32_t ret;
  ret = sbufReadU8(src);
  ret |= sbufReadU8(src) << 8;
  ret |= sbufReadU8(src) << 16;
  ret |= sbufReadU8(src) << 24;
  return ret;
}

void sbufReadData(sbuf_t *src, void *data, int len)
{
  memcpy(data, src->ptr, len);
}

// reader - return bytes remaining in buffer
// writer - return available space
int sbufBytesRemaining(sbuf_t *buf)
{
  return buf->end - buf->ptr;
}

int sbufSizeBytes(sbuf_t *buf, int totalSize)
{
  return totalSize - sbufBytesRemaining(buf);
}

uint8_t* sbufPtr(sbuf_t *buf)
{
  return buf->ptr;
}

const uint8_t* sbufConstPtr(const sbuf_t *buf)
{
  return buf->ptr;
}

// advance buffer pointer
// reader - skip data
// writer - commit written data
void sbufAdvance(sbuf_t *buf, int size)
{
  buf->ptr += size;
}

// modifies streambuf so that written data are prepared for reading
void sbufSwitchToReader(sbuf_t *buf, uint8_t *base)
{
  buf->end = buf->ptr;
  buf->ptr = base;
}

#ifdef BETAFLIGHT_MSP_SIMULATOR

/*
* SmartPort Telemetry implementation by frank26080115
* see https://github.com/frank26080115/cleanflight/wiki/Using-Smart-Port
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

enum
{
  SPSTATE_UNINITIALIZED,
  SPSTATE_INITIALIZED,
  SPSTATE_WORKING
};

enum
{
  FSSP_START_STOP = 0x7E,

  FSSP_DLE = 0x7D,
  FSSP_DLE_XOR = 0x20,

  FSSP_DATA_FRAME = 0x10,
  FSSP_MSPC_FRAME = 0x30, // MSP client frame
  FSSP_MSPS_FRAME = 0x32, // MSP server frame

                          // ID of sensor. Must be something that is polled by FrSky RX
  FSSP_SENSOR_ID1 = 0x1B,
  FSSP_SENSOR_ID2 = 0x0D,
  FSSP_SENSOR_ID3 = 0x34,
  FSSP_SENSOR_ID4 = 0x67
  // there are 32 ID's polled by smartport master
  // remaining 3 bits are crc (according to comments in openTx code)
};

// these data identifiers are obtained from https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky.h
enum
{
  FSSP_DATAID_SPEED = 0x0830,
  FSSP_DATAID_VFAS = 0x0210,
  FSSP_DATAID_CURRENT = 0x0200,
  FSSP_DATAID_RPM = 0x050F,
  FSSP_DATAID_ALTITUDE = 0x0100,
  FSSP_DATAID_FUEL = 0x0600,
  FSSP_DATAID_ADC1 = 0xF102,
  FSSP_DATAID_ADC2 = 0xF103,
  FSSP_DATAID_LATLONG = 0x0800,
  FSSP_DATAID_CAP_USED = 0x0600,
  FSSP_DATAID_VARIO = 0x0110,
  FSSP_DATAID_CELLS = 0x0300,
  FSSP_DATAID_CELLS_LAST = 0x030F,
  FSSP_DATAID_HEADING = 0x0840,
  FSSP_DATAID_ACCX = 0x0700,
  FSSP_DATAID_ACCY = 0x0710,
  FSSP_DATAID_ACCZ = 0x0720,
  FSSP_DATAID_T1 = 0x0400,
  FSSP_DATAID_T2 = 0x0410,
  FSSP_DATAID_GPS_ALT = 0x0820,
  FSSP_DATAID_A3 = 0x0900,
  FSSP_DATAID_A4 = 0x0910
};

const uint16_t frSkyDataIdTable[] = {
  FSSP_DATAID_SPEED     ,
  FSSP_DATAID_VFAS      ,
  FSSP_DATAID_CURRENT   ,
  //FSSP_DATAID_RPM       ,
  FSSP_DATAID_ALTITUDE  ,
  FSSP_DATAID_FUEL      ,
  //FSSP_DATAID_ADC1      ,
  //FSSP_DATAID_ADC2      ,
  FSSP_DATAID_LATLONG   ,
  FSSP_DATAID_LATLONG   , // twice
                          //FSSP_DATAID_CAP_USED  ,
  FSSP_DATAID_VARIO     ,
  //FSSP_DATAID_CELLS     ,
  //FSSP_DATAID_CELLS_LAST,
  FSSP_DATAID_HEADING   ,
  FSSP_DATAID_ACCX      ,
  FSSP_DATAID_ACCY      ,
  FSSP_DATAID_ACCZ      ,
  FSSP_DATAID_T1        ,
  FSSP_DATAID_T2        ,
  FSSP_DATAID_GPS_ALT   ,
  FSSP_DATAID_A4        ,
  0
};

// return positive for ACK, negative on error, zero for no reply
typedef enum {
  MSP_RESULT_ACK = 1,
  MSP_RESULT_ERROR = -1,
  MSP_RESULT_NO_REPLY = 0,
  MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
} mspResult_e;

typedef enum {
  MSP_DIRECTION_REPLY = 0,
  MSP_DIRECTION_REQUEST = 1
} mspDirection_e;

typedef struct mspPacket_s {
  sbuf_t buf;
  int16_t cmd;
  int16_t result;
  uint8_t direction;
} mspPacket_t;

#define __USE_C99_MATH // for roundf()
#define SMARTPORT_BAUD 57600
#define SMARTPORT_UART_MODE MODE_RXTX
#define SMARTPORT_SERVICE_TIMEOUT_MS 1 // max allowed time to find a value to send

PACK(struct smartPortFrame_t {
  uint8_t  sensorId;
  uint8_t  frameId;
  uint16_t valueId;
  uint32_t data;
  uint8_t  crc;
});

#define SMARTPORT_FRAME_SIZE  sizeof(smartPortFrame_t)
#define SMARTPORT_TX_BUF_SIZE 256

#define SMARTPORT_PAYLOAD_OFFSET offsetof(smartPortFrame_t, valueId)
#define SMARTPORT_PAYLOAD_SIZE   (SMARTPORT_FRAME_SIZE - SMARTPORT_PAYLOAD_OFFSET - 1)

static smartPortFrame_t smartPortRxBuffer;
static uint8_t smartPortRxBytes = 0;
static bool smartPortFrameReceived = false;

#define SMARTPORT_MSP_VERSION    1
#define SMARTPORT_MSP_VER_SHIFT  5
#define SMARTPORT_MSP_VER_MASK   (0x7 << SMARTPORT_MSP_VER_SHIFT)
#define SMARTPORT_MSP_VERSION_S  (SMARTPORT_MSP_VERSION << SMARTPORT_MSP_VER_SHIFT)

#define SMARTPORT_MSP_ERROR_FLAG (1 << 5)
#define SMARTPORT_MSP_START_FLAG (1 << 4)
#define SMARTPORT_MSP_SEQ_MASK   0x0F

#define SMARTPORT_MSP_RX_BUF_SIZE 64

static uint8_t smartPortMspTxBuffer[SMARTPORT_TX_BUF_SIZE];
static mspPacket_t smartPortMspReply;
static bool smartPortMspReplyPending = false;

#define SMARTPORT_MSP_RES_ERROR (-10)

#define ARRAYLEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAYEND(x) (&(x)[ARRAYLEN(x)])

enum {
  SMARTPORT_MSP_VER_MISMATCH = 0,
  SMARTPORT_MSP_CRC_ERROR = 1,
  SMARTPORT_MSP_ERROR = 2
};

typedef enum {
  PID_ROLL,
  PID_PITCH,
  PID_YAW,
  PID_ALT,
  PID_POS,
  PID_POSR,
  PID_NAVR,
  PID_LEVEL,
  PID_MAG,
  PID_VEL,
  PID_ITEM_COUNT
} pidIndex_e;


static bool smartPortSendMspReply();

static BFConfig_t simConfig;

void initBFSimulatorConfig()
{
  static bool initialized = false;
  if (initialized)
  {
    return;
  }

  initialized = true;

  memset(&simConfig, 0, sizeof(simConfig));
  
  // TODO: gui to edit config
  for (int d = 0; d < PID_ITEM_COUNT; ++d)
  {
    simConfig.pids[d][0] = d*3;
    simConfig.pids[d][1] = d*3+1;
    simConfig.pids[d][2] = d*3+2;
  }
}

static int mspFcProcessCommand(mspPacket_t* packet, mspPacket_t* reply)
{
  reply->cmd = packet->cmd;
  reply->result = MSP_RESULT_ACK;
  sbuf_t* src = &packet->buf;
  sbuf_t* dst = &reply->buf;

  switch (packet->cmd)
  {
  case MSP_PID:
    for (int i = 0; i < PID_ITEM_COUNT; i++) 
    {
      sbufWriteU8(dst, simConfig.pids[i][0]);
      sbufWriteU8(dst, simConfig.pids[i][1]);
      sbufWriteU8(dst, simConfig.pids[i][2]);
    }
    break;

  case MSP_SET_PID:
    for (int i = 0; i < PID_ITEM_COUNT; i++)
    {
      simConfig.pids[i][0] = sbufReadU8(src);
      simConfig.pids[i][1] = sbufReadU8(src);
      simConfig.pids[i][2] = sbufReadU8(src);
    }
    break;

  case MSP_RC_TUNING:
    sbufWriteU8(dst, simConfig.rcRate);
    sbufWriteU8(dst, simConfig.rcExpo);
    for (int i = 0 ; i < 3; i++) {
        sbufWriteU8(dst, simConfig.rates[i]); // R,P,Y see flight_dynamics_index_t
    }
    sbufWriteU8(dst, simConfig.dynThrPID);
    sbufWriteU8(dst, simConfig.thrMid8);
    sbufWriteU8(dst, simConfig.thrExpo8);
    sbufWriteU16(dst, simConfig.tpa_breakpoint);
    sbufWriteU8(dst, simConfig.rcYawExpo8);
    sbufWriteU8(dst, simConfig.rcYawRate8);
    break;

  case MSP_SET_RC_TUNING:
    if (sbufBytesRemaining(src) >= 10) 
    {
        simConfig.rcRate = sbufReadU8(src);
        simConfig.rcExpo = sbufReadU8(src);
        for (int i = 0; i < 3; i++) 
        {
            simConfig.rates[i] = sbufReadU8(src);
        }

        //value = sbufReadU8(src);
        simConfig.dynThrPID = sbufReadU8(src); //MIN(value, CONTROL_RATE_CONFIG_TPA_MAX);
        simConfig.thrMid = sbufReadU8(src);
        simConfig.thrExpo = sbufReadU8(src);
        simConfig.tpa_breakpoint = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 1) 
        {
            simConfig.rcYawExpo = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 1) 
        {
            simConfig.rcYawRate = sbufReadU8(src);
        }
        //generateThrottleCurve();
    } 
    else 
    {
        reply->result = MSP_RESULT_ERROR;
    }
    break;

  default:
    reply->result = MSP_RESULT_CMD_UNKNOWN;
    break;
  }

  debugPrintf("bf-sim: processed command: %d, result: %d", packet->cmd, reply->result);
  return reply->result;
}

static void smartPortSendPackageEx(uint8_t frameId, uint8_t* data)
{
  SportTelemetryPacket packet;
  
  // physicalId isn't used at this level since 
  // we are bypassing normal s-port transmission
  
  packet.primId = frameId;
  uint8_t* p = &packet.raw[2];
  
  for (int i = 0; i < SMARTPORT_FRAME_SIZE; ++i)
  {
    p[i] = data[i];
  }

  if (!betaflightInputTelemetryFifo->hasSpace(sizeof(packet)))
  {
    return;
  }

  for (uint8_t i = 0; i<sizeof(packet); i++)
  {
    betaflightInputTelemetryFifo->push(packet.raw[i]);
  }
}

static void initSmartPortMspReply(int16_t cmd)
{
  smartPortMspReply.buf.ptr = smartPortMspTxBuffer;
  smartPortMspReply.buf.end = ARRAYEND(smartPortMspTxBuffer);

  smartPortMspReply.cmd = cmd;
  smartPortMspReply.result = 0;
}

static void processMspPacket(mspPacket_t* packet)
{
  initSmartPortMspReply(0);

  if (mspFcProcessCommand(packet, &smartPortMspReply) == MSP_RESULT_ERROR) {
    sbufWriteU8(&smartPortMspReply.buf, SMARTPORT_MSP_ERROR);
  }

  // change streambuf direction
  sbufSwitchToReader(&smartPortMspReply.buf, smartPortMspTxBuffer);
  smartPortMspReplyPending = true;

  while (smartPortSendMspReply());
}

/**
* Request frame format:
* - Header: 1 byte
*   - Reserved: 2 bits (future use)
*   - Error-flag: 1 bit
*   - Start-flag: 1 bit
*   - CSeq: 4 bits
*
* - MSP payload:
*   - if Error-flag == 0:
*     - size: 1 byte
*     - payload
*     - CRC (request type included)
*   - if Error-flag == 1:
*     - size: 1 byte (== 1)
*     - error: 1 Byte
*       - 0: Version mismatch (type=0)
*       - 1: Sequence number error
*       - 2: MSP error
*     - CRC (request type included)
*/
static bool smartPortSendMspReply()
{
  static uint8_t checksum = 0;
  static uint8_t seq = 0;

  uint8_t packet[SMARTPORT_PAYLOAD_SIZE];
  uint8_t* p = packet;
  uint8_t* end = p + SMARTPORT_PAYLOAD_SIZE;

  sbuf_t* txBuf = &smartPortMspReply.buf;

  // detect first reply packet
  if (txBuf->ptr == smartPortMspTxBuffer) {

    // header
    uint8_t head = SMARTPORT_MSP_START_FLAG | (seq++ & SMARTPORT_MSP_SEQ_MASK);
    if (smartPortMspReply.result < 0) {
      head |= SMARTPORT_MSP_ERROR_FLAG;
    }
    *p++ = head;

    uint8_t size = sbufBytesRemaining(txBuf);
    *p++ = size;

    checksum = size ^ smartPortMspReply.cmd;
  }
  else {
    // header
    *p++ = (seq++ & SMARTPORT_MSP_SEQ_MASK);
  }

  while ((p < end) && (sbufBytesRemaining(txBuf) > 0)) {
    *p = sbufReadU8(txBuf);
    checksum ^= *p++; // MSP checksum
  }

  // to be continued...
  if (p == end) {
    smartPortSendPackageEx(FSSP_MSPS_FRAME, packet);
    return true;
  }

  // nothing left in txBuf,
  // append the MSP checksum
  *p++ = checksum;

  // pad with zeros
  while (p < end)
    *p++ = 0;

  smartPortSendPackageEx(FSSP_MSPS_FRAME, packet);
  return false;
}

void smartPortSendErrorReply(uint8_t error, int16_t cmd)
{
  initSmartPortMspReply(cmd);
  sbufWriteU8(&smartPortMspReply.buf, error);
  smartPortMspReply.result = SMARTPORT_MSP_RES_ERROR;

  sbufSwitchToReader(&smartPortMspReply.buf, smartPortMspTxBuffer);
  smartPortMspReplyPending = true;

  while( smartPortSendMspReply());
}


/**
* Request frame format:
* - Header: 1 byte
*   - Version: 3 bits
*   - Start-flag: 1 bit
*   - CSeq: 4 bits
*
* - MSP payload:
*   - Size: 1 Byte
*   - Type: 1 Byte
*   - payload...
*   - CRC
*/
void handleSmartPortMspFrame(smartPortFrame_t* sp_frame)
{
  static uint8_t mspBuffer[SMARTPORT_MSP_RX_BUF_SIZE];
  static uint8_t mspStarted = 0;
  static uint8_t lastSeq = 0;
  static uint8_t checksum = 0;
  static mspPacket_t cmd;

  // re-assemble MSP frame & forward to MSP port when complete
  uint8_t* p = ((uint8_t*)sp_frame) + SMARTPORT_PAYLOAD_OFFSET;
  uint8_t* end = p + SMARTPORT_PAYLOAD_SIZE;

  uint8_t head = *p++;
  uint8_t seq = head & SMARTPORT_MSP_SEQ_MASK;
  uint8_t version = (head & SMARTPORT_MSP_VER_MASK) >> SMARTPORT_MSP_VER_SHIFT;

  if (version != SMARTPORT_MSP_VERSION) {
    mspStarted = 0;
    smartPortSendErrorReply(SMARTPORT_MSP_VER_MISMATCH, 0);
    return;
  }

  // check start-flag
  if (head & SMARTPORT_MSP_START_FLAG) {

    //TODO: if (p_size > SMARTPORT_MSP_RX_BUF_SIZE) error!
    uint8_t p_size = *p++;
    cmd.cmd = *p++;
    cmd.result = 0;

    cmd.buf.ptr = mspBuffer;
    cmd.buf.end = mspBuffer + p_size;

    checksum = p_size ^ cmd.cmd;
    mspStarted = 1;
  }
  else if (!mspStarted) {
    // no start packet yet, throw this one away
    return;
  }
  else if (((lastSeq + 1) & SMARTPORT_MSP_SEQ_MASK) != seq) {
    // packet loss detected!
    mspStarted = 0;
    return;
  }

  // copy payload bytes
  while ((p < end) && sbufBytesRemaining(&cmd.buf)) {
    checksum ^= *p;
    sbufWriteU8(&cmd.buf, *p++);
  }

  // reached end of smart port frame
  if (p == end) {
    lastSeq = seq;
    return;
  }

  // last byte must be the checksum
  if (checksum != *p) {
    mspStarted = 0;
    smartPortSendErrorReply(SMARTPORT_MSP_CRC_ERROR, cmd.cmd);
    return;
  }

  // end of MSP packet reached
  mspStarted = 0;
  sbufSwitchToReader(&cmd.buf, mspBuffer);

  processMspPacket(&cmd);
}

// hook for skipping serialization 
void simHandleSmartPortMspFrame(uint8_t* sp_frame)
{
  handleSmartPortMspFrame((smartPortFrame_t*)sp_frame);
}

#endif
