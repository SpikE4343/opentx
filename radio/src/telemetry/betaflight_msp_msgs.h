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
#ifndef BETAFLIGHT_MSP_MESSAGES_H
#define BETAFLIGHT_MSP_MESSAGES_H

#include "opentx.h"

enum BFMspMessageType
{
  MSP_NONE = 0,
  MSP_VTX_CONFIG = 88,
  MSP_VTX_SET_CONFIG = 89,

  MSP_PID_ADVANCED = 94,
  MSP_SET_PID_ADVANCED = 95,
 
  MSP_RC_TUNING = 111,
  MSP_PID = 112,

  MSP_SET_PID = 202,
  MSP_SET_RC_TUNING = 204
};

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
PACK(struct BFMspRequestFrameHeader_t 
{
    uint8_t sequence : 4;
    uint8_t start : 1;
    uint8_t version : 3;
});


/**
* Response frame format:
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
PACK(struct BFMspResponseFrameHeader_t
{
  uint8_t unsused : 2;
  uint8_t error : 1;
  uint8_t start : 1;
  uint8_t sequence : 4;
});


PACK(struct BFMspErrorPayload_t
{
  uint8_t size;
  uint8_t error;
  uint8_t crc;
});


// MSP_STATUS
PACK(struct BFMspStatusResponseMsg_t 
{
    uint16_t serialTime;
    uint16_t i2cErrors;
    uint16_t sensors;
    uint32_t flightModes;
    uint8_t profile;
    uint16_t systemLoad;
    uint16_t gyroTime;
    uint8_t flightModeFlagsCount;
    uint8_t* flightModeFlags;
});


// MSP_RC_TUNING
PACK(struct BFMspRcResponseMsg_t 
{
    uint8_t rcRate;
    uint8_t rcExpo;
    uint8_t rates[3]; // R, P, Y
    uint8_t dynThrottlePID; // TPAs
    uint8_t throttleMid;
    uint8_t throttleExpo;
    uint16_t throttle_bp;
    uint8_t rcYawExpo;
    uint8_t rcYawRate;

});

// MSP_PID
PACK(struct BFMspPidResponseMsg_t 
{
    uint8_t roll[3]; // P, I, D
    uint8_t pitch[3]; // P, I, D
    uint8_t yaw[3]; // P, I, D
});
#endif