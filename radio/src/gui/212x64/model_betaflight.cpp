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
#include "telemetry/betaflight_msp.h"

#define MENU_WIDTH 8*FW
#define MESSAGE_TIMEOUT_SECS 10
#define BF_MENU_TITLE "Betaflight"

enum BFPageType
{
  PIDS_PAGE,
  RATES_PAGE,
  THROTTLE_PAGE,
  VTX_PAGE,

  NUM_BF_PAGES
};


enum BFPageState
{
  PAGE_NONE,
  PAGE_LOADING,
  PAGE_LOADED,
  PAGE_SAVE,
  PAGE_SAVING
};

struct BFPage;

typedef void (*drawPage_t)( BFPage& page, event_t event );

struct BFPage
{
  BFPageType type;
  BFPageState state;
  BFMspMessageType readCmd;
  BFMspMessageType writeCmd;
  const char* name;
  drawPage_t drawPage;
  int editMode;
  int selectedPos;
  int maxPos;

  void handleSelectedPositionInput(event_t& event);
  void handleFieldEditing(event_t& event);

  void updateState();
  
  bool processReply(uint8_t msgType, BFMspDecoder& decoder);
};



int bfMenuPos = 0;
int bfEditPage = -1;
int selectedPage = 0;

BFConfig_t bfConfig;

BFMspDecoder mspDecoder;
BFMspEncoder mspEncoder;

bool connected = false;
bool sendingMessage = false;
uint8_t messageReplyTimeout = 0;
uint8_t sendMspMessageType = MSP_NONE;

void drawPIDsPage( BFPage& page, event_t event );


BFPage pages[] =
{
  { PIDS_PAGE,     PAGE_NONE, MSP_PID,        MSP_SET_PID,        "PIDs    ", drawPIDsPage,  0, 0, 9 },
  { RATES_PAGE,    PAGE_NONE, MSP_RC_TUNING,  MSP_SET_RC_TUNING,  "Rates   ", NULL,          0, 0, 0 },
  { THROTTLE_PAGE, PAGE_NONE, MSP_NONE,       MSP_NONE,           "Throttle", NULL,          0, 0, 0 },
  { VTX_PAGE,      PAGE_NONE, MSP_VTX_CONFIG, MSP_VTX_SET_CONFIG, "Video Tx", NULL,          0, 0, 0 }
};

bool BFPage::processReply(uint8_t msgType, BFMspDecoder& decoder)
{
  if (!decoder.hasStarted() || !decoder.isComplete())
  {
    return false;
  }

  uint8_t* buffer = decoder.getMessageBuffer();
  uint8_t size = decoder.getMessageSize();

  switch (msgType)
  {
  case MSP_PID:
    if (size >= sizeof(bfConfig.pids))
    {
      memcpy(&bfConfig.pids, buffer, sizeof(bfConfig.pids));
    }
    return true;
  }

  decoder.reset();

  return true;
}

void BFPage::updateState()
{
  switch (state)
  {
  case PAGE_NONE:
    if (readCmd == MSP_NONE)
    {
      state = PAGE_LOADED;
    }
    else if (connected && !sendingMessage)
    {
      debugPrintf("Sending message");
      sendMspMessageType = readCmd;
      mspEncoder.encodeMessage(readCmd, NULL, 0);
      sendingMessage = true;
      state = PAGE_LOADING;
    }
    break;

  case PAGE_LOADING:
    if (connected && sendingMessage)
    {
      if (processReply(sendMspMessageType, mspDecoder))
      {
        state = PAGE_LOADED;
      }
    }
    break;

  case PAGE_LOADED:
    break;

  case PAGE_SAVE:
    if (writeCmd != MSP_NONE && connected && !sendingMessage)
    {
      debugPrintf("Sending save message");
      sendMspMessageType = writeCmd;
      mspEncoder.encodeMessage(writeCmd, NULL, 0);
      sendingMessage = true;
      state = PAGE_SAVING;
    }
    break;

  case PAGE_SAVING:
    if (sendingMessage)
      state = PAGE_LOADED;
    break;
  }
}
void BFPage::handleSelectedPositionInput(event_t& event)
{
  if( editMode != EDIT_SELECT_FIELD )
  {
    return;
  }

  switch (event)
  {
    case EVT_KEY_FIRST(KEY_DOWN):
      ++selectedPos;
      break;

    case EVT_KEY_FIRST(KEY_UP):
      --selectedPos;
      break;
  }

  selectedPos %= maxPos;
}

void BFPage::handleFieldEditing(event_t& event)
{
  switch( editMode )
  {
    case EDIT_SELECT_FIELD:
      if( event == EVT_KEY_LONG(KEY_ENTER))
      {
        editMode = EDIT_MODIFY_FIELD;
      }
      break;
    case EDIT_MODIFY_FIELD:
      switch (event)
      {
        case EVT_KEY_LONG(KEY_ENTER):
        case EVT_KEY_LONG(KEY_EXIT):
          editMode = EDIT_SELECT_FIELD;
          break;
      }
      break;
  }
}


//
//
bool recvMessage()
{
  if (betaflightInputTelemetryFifo == NULL)
  {
    return false;
  }

  if (betaflightInputTelemetryFifo->isEmpty())
  {
    return false;
  }

  uint32_t size = betaflightInputTelemetryFifo->size();
  debugPrintf("bf-buf: %u", size);
  SportTelemetryPacket packet;

  if ( size < sizeof(packet))
  {
    return false;
  }

  debugPrintf("packet recv: ");
  for (uint8_t i = 0; i<sizeof(packet); i++)
  {
    betaflightInputTelemetryFifo->pop(packet.raw[i]);
    debugPrintf("%0X", packet.raw[i]);
  }
  debugPrintf("\r\n");


  if (!mspDecoder.decodePacket(packet))
  {
    return false;
  }

  // valid msp message decoded
  debugPrintf("message decoded!\r\n");

  return true;
}

#ifdef BETAFLIGHT_MSP_SIMULATOR
// TODO merge it with S.PORT update function when finished
void sportOutputPushPacketSim(SportTelemetryPacket* packet)
{
  uint8_t buf[sizeof(SportTelemetryPacket)+1];
  uint8_t* pos = buf+1;
  uint16_t crc = 0;

  for (uint8_t i = 1; i<sizeof(SportTelemetryPacket); i++) {
    uint8_t byte = packet->raw[i];
    *(pos++) = byte;
    crc += byte; // 0-1FF
    crc += crc >> 8; // 0-100
    crc &= 0x00ff;
  }

  *(pos++) = 0xFF - crc;
  buf[0] = packet->raw[0];

  simHandleSmartPortMspFrame(&buf[0]);
}

#endif

//
//
void sendMessage()
{
  if( mspEncoder.isComplete() )
  {
    return;
  }

  SportTelemetryPacket packet;
  if( !mspEncoder.fillNextPacket(packet) )
  {
    return;
  }
  
#ifdef BETAFLIGHT_MSP_SIMULATOR
  sportOutputPushPacketSim(&packet);
#else
  sportOutputPushPacket(&packet);
#endif
}




//
//
void updateMessages()
{
  connected = TELEMETRY_RSSI() > 0;

  recvMessage();
  sendMessage();
}

//
//
void drawPIDsPage( BFPage& page, event_t event )
{
//  SIMPLE_SUBMENU( "Betaflight > PIDs", 9);
  
  coord_t x = MENU_WIDTH + FW*2;
  coord_t y = MENU_HEADER_HEIGHT + 1;

  int step= 28;

  lcdDrawText(x + FW*6 + (FW/2),          y, "P", 0 );
  lcdDrawText(x + FW*6+step + (FW / 2),   y, "I", 0 );
  lcdDrawText(x + FW*6+step*2 + (FW / 2), y, "D", 0 );

  lcdDrawText(x, y+FH*2 + 2, "ROLL",  0 );
  lcdDrawText(x, y+FH*3 + 2, "PITCH", 0 );
  lcdDrawText(x, y+FH*4 + 2, "YAW",   0 );

  uint8_t pidNumStartX = x + FW*6;
  uint8_t pidNumStartY = y + FH*2;

  page.updateState();

  page.handleSelectedPositionInput(event);

  for( int axis=0; axis < 3; ++axis)
  {
    for( int t=0; t < 3; ++t )
    {
      int i = t * 3 + axis;
      bool selectedField = page.selectedPos == i && page.editMode > EDIT_SELECT_MENU;
      LcdFlags attr = (selectedField ? (page.editMode>0 ? BLINK|INVERS : INVERS) : 0);

      if( selectedField && connected && page.state == PAGE_LOADED)
      {
        page.handleFieldEditing(event);

        switch( page.editMode )
        {
          case EDIT_MODIFY_FIELD:
            switch (event)
            {
              case EVT_KEY_FIRST(KEY_DOWN):
                --bfConfig.pids[axis][t];
                AUDIO_KEY_PRESS();
                break;

              case EVT_KEY_FIRST(KEY_UP):
                ++bfConfig.pids[axis][t];
                AUDIO_KEY_PRESS();
                break;
            }

            bfConfig.pids[axis][t] %= 255;
            break;
        }
      }

      if (page.state != PAGE_LOADED)
      {
        lcdDrawText(
          pidNumStartX + t*step,
          pidNumStartY + axis*(FH + 2),
          "--",
          0);
      }
      else
      {
        lcdDrawNumber(
          pidNumStartX + t*step,
          pidNumStartY + axis*(FH + 2),
          bfConfig.pids[axis][t],
          attr);
      }
    }
  }
}

//
//
void drawHeaderInfo(event_t event)
{
  int start = strlen(BF_MENU_TITLE) + 1;
  if (connected)
  {
    char buffer[64];
    sprintf(buffer, "(C, sp:%d, rp:%d)", mspEncoder.isComplete(), mspDecoder.getMessageSize());
    lcdDrawText(start * FW, 0, buffer);
  }
  else
  {
    lcdDrawText(start * FW, 0, "(NO TELEMETRY)", BLINK);
  }
}


void menuModelBetaflightMspSmartPort(event_t event)
{
  drawHeaderInfo(event);

  SIMPLE_MENU(
    BF_MENU_TITLE,
    menuTabModel,
    MENU_MODEL_BETAFLIGHT_MSP_SMARTPORT,
    NUM_BF_PAGES
  );

#ifdef BETAFLIGHT_MSP_SIMULATOR
  initBFSimulatorConfig();
#endif

  if( betaflightInputTelemetryFifo == NULL )
  {
    betaflightInputTelemetryFifo = new Fifo<uint8_t, BETAFLIGHT_TELEMETRY_INPUT_FIFO_SIZE>();
  }

  updateMessages();
  
  // update menu position
  if( bfEditPage == -1 )
  {
    switch (event)
    {
      case EVT_KEY_FIRST(KEY_DOWN):
        ++bfMenuPos;
        killEvents(event);
        event = 0;
        AUDIO_KEY_PRESS();
        break;

      case EVT_KEY_FIRST(KEY_UP):
        --bfMenuPos;
        killEvents(event);
        event = 0;
        AUDIO_KEY_PRESS();
        break;
    }
  }

  bfMenuPos %= NUM_BF_PAGES;

  for( int p=0; p < NUM_BF_PAGES; ++p )
  {
    BFPage& page = pages[p];
    int i = p + menuVerticalOffset;
    coord_t y = MENU_HEADER_HEIGHT + 1 + p*FH;
    bool selected = bfMenuPos == p;

    lcdDrawText(0, y, page.name, selected ? (page.editMode > 0 ? BLINK|INVERS : INVERS) : 0 );

    if (selected)
    {
      // page edit mode inputs
     
      if (bfEditPage == -1)
      {
        if (event == EVT_KEY_FIRST(KEY_ENTER) 
          && connected 
          && page.state == PAGE_LOADED)
        {
          bfEditPage = page.type;
          page.editMode = EDIT_SELECT_FIELD;
          event = 0;
          AUDIO_KEY_PRESS();
        }
      }
      else
      {
        if (page.editMode == EDIT_SELECT_FIELD
          && event == EVT_KEY_FIRST(KEY_EXIT) || !connected)
        {
          bfEditPage = -1;
          page.editMode = EDIT_SELECT_MENU;
          event = 0;
          AUDIO_KEY_PRESS();

          if (connected)
          {
            page.state = PAGE_SAVE;
          }
        }
      }

      if( page.drawPage != NULL)
      {
        page.drawPage(page, event);
      }
    }
  }
}
