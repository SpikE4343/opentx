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
  void reload();
};

int bfMenuPos = 0;
int bfEditPage = -1;
int selectedPage = 0;

static BetaflightController* bf = NULL;

void drawPIDsPage( BFPage& page, event_t event );

BFPage pages[] =
{
  { PIDS_PAGE,     PAGE_NONE, MSP_PID,        MSP_SET_PID,        "PIDs    ", drawPIDsPage,  0, 0, 9 },
  { RATES_PAGE,    PAGE_NONE, MSP_RC_TUNING,  MSP_SET_RC_TUNING,  "Rates   ", NULL,          0, 0, 0 },
  { THROTTLE_PAGE, PAGE_NONE, MSP_NONE,       MSP_NONE,           "Throttle", NULL,          0, 0, 0 },
  { VTX_PAGE,      PAGE_NONE, MSP_VTX_CONFIG, MSP_VTX_SET_CONFIG, "Video Tx", NULL,          0, 0, 0 }
};



void BFPage::updateState()
{
  switch (state)
  {
  case PAGE_NONE:
    if (readCmd == MSP_NONE)
    {
      state = PAGE_LOADED;
    }
    else if (bf->isConnected())
    {
      debugPrintf("Sending message\r\n");
      bf->sendMessage(readCmd, NULL, 0);
      state = PAGE_LOADING;
    }
    break;

  case PAGE_LOADING:
    if (bf->isConnected() && bf->getMessageState() == BetaflightController::MS_IDLE)
    {
      state = PAGE_LOADED;
    }
    break;

  case PAGE_LOADED:
    break;

  case PAGE_SAVE:
    if (writeCmd != MSP_NONE 
      && bf->isConnected() 
      && bf->getMessageState() == BetaflightController::MS_IDLE)
    {
      debugPrintf("Sending save message\r\n");

      sbuf_t saveBuffer;
      saveBuffer.ptr = bf->getMessageBuffer();
      saveBuffer.end = saveBuffer.ptr + MSP_MSG_BUFFER_SIZE;

      bf->createSaveMessage(writeCmd, &saveBuffer);
      uint32_t size = sbufSizeBytes(&saveBuffer, MSP_MSG_BUFFER_SIZE);

      sbufSwitchToReader(&saveBuffer, bf->getMessageBuffer());

      bf->sendMessage(writeCmd, saveBuffer.ptr, size);
      state = PAGE_SAVING;
    }
    break;

  case PAGE_SAVING:
    if (bf->getMessageState() != BetaflightController::MS_IDLE)
    {
      state = PAGE_LOADED;
    }
    break;
  }
}

void BFPage::reload()
{
  state = PAGE_NONE;
  editMode = EDIT_SELECT_MENU;
  selectedPos = 0;
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

  selectedPos = abs(selectedPos >= maxPos ? 0 : selectedPos % maxPos);
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

      if( selectedField && bf->isConnected() && page.state == PAGE_LOADED)
      {
        page.handleFieldEditing(event);

        switch( page.editMode )
        {
          case EDIT_MODIFY_FIELD:
            switch (event)
            {
              case EVT_KEY_FIRST(KEY_DOWN):
                --bf->config.pids[axis][t];
                AUDIO_KEY_PRESS();
                break;

              case EVT_KEY_FIRST(KEY_UP):
                ++bf->config.pids[axis][t];
                AUDIO_KEY_PRESS();
                break;
            }

            bf->config.pids[axis][t] %= 255;
            break;
        }
      }

      if (page.state != PAGE_LOADED && page.state != PAGE_SAVING)
      {
        lcdDrawText(
          pidNumStartX + t*step,
          pidNumStartY + axis*(FH + 2),
          "---",
          0);
      }
      else
      {
        lcdDrawNumber(
          pidNumStartX + t*step,
          pidNumStartY + axis*(FH + 2),
          bf->config.pids[axis][t],
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
  if (bf != NULL && bf->isConnected())
  {
    char buffer[64];
    sprintf(buffer, "(C, c:%d, m:%d, mt:%d, ms:%d)"
      , bf->getConnectionState()
      , bf->getMessageState()
      , bf->getMessageType()
      , bf->getMessageSize()
    );

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

  if (bf == NULL)
  {
    bf = new BetaflightController();
  }

  bf->update();

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

  bfMenuPos = abs(bfMenuPos >= NUM_BF_PAGES ? 0 : bfMenuPos % NUM_BF_PAGES);

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
          && bf->isConnected()
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
          && event == EVT_KEY_FIRST(KEY_EXIT) || !bf->isConnected())
        {
          bfEditPage = -1;
          page.editMode = EDIT_SELECT_MENU;
          event = 0;
          AUDIO_KEY_PRESS();

          if (bf->isConnected())
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


bool BetaflightController::sendMessage(uint8_t type, uint8_t* msg, uint8_t size)
{
  messageType = type;
  switch (messageState)
  {
  case MS_IDLE:
    if (encoder.encodeMessage(type, msg, size))
    {
      messageState = MS_SENDING;
      return true;
    }
  }

  return false;
}


void BetaflightController::updateConnection()
{
  switch (connectionState)
  {
  case CS_NONE:
    connectionState = CS_DISCONNECTED;
    break;

  case CS_DISCONNECTED:
    if (TELEMETRY_RSSI() > 0)
    {
      connectionState = CS_CONNECTED;
      messageState = MS_IDLE;
    }
    break;

  case CS_CONNECTING:
    break;

  case CS_CONNECTED:
    if (TELEMETRY_RSSI() <= 0)
    {
      connectionState = CS_DISCONNECTED;
      messageState = MS_DISABLED;
      encoder.reset();
      decoder.reset();

      pages[bfMenuPos].reload();
    }
    updateMessage();
    break;

  case CS_DISCONNECTING:
    break;
  }
}

void BetaflightController::updateMessage()
{
  switch (messageState)
  {
  case MS_IDLE:
    messageTimeout = 0;
    break;

  case MS_SENDING:
    if (!sendMessage())
    {
      messageState = MS_RECEIVING;
    }

    break;

  case MS_RECEIVING:
    if (recvMessage())
    {
      messageState = MS_RECEIVED;
    }
   /* else if (++messageTimeout >= 20)
    {
      messageTimeout = 0;
      messageState = MS_IDLE;
      messageType = MSP_NONE;
      messageSize = 0;
      encoder.reset();
      decoder.reset();
    }*/
    break;

  case MS_RECEIVED:
    if (processReply())
    {
      messageState = MS_IDLE;
      messageType = MSP_NONE;
    }
    break;
  }
}

void BetaflightController::update()
{
  updateConnection();
}

//
//
bool BetaflightController::recvMessage()
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

  if (size < sizeof(packet))
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


  if (!decoder.decodePacket(packet))
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
  uint8_t buf[sizeof(SportTelemetryPacket) + 1];
  uint8_t* pos = buf + 1;
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
bool BetaflightController::sendMessage()
{
  if (encoder.isComplete())
  {
    return false;
  }

  SportTelemetryPacket packet;
  if (!encoder.fillNextPacket(packet))
  {
    return false;
  }

#ifdef BETAFLIGHT_MSP_SIMULATOR
  sportOutputPushPacketSim(&packet);
#else
  sportOutputPushPacket(&packet);
#endif
  return true;
}

bool BetaflightController::createSaveMessage(uint8_t msgType, sbuf_t* buf)
{
  switch (msgType)
  {
  case MSP_SET_PID:
    sbufWriteData(buf, &bf->config.pids, sizeof(bf->config.pids));
    return true;

  default:
    return false;
  }

  return true;
}

bool BetaflightController::processReply()
{
  uint8_t* buffer = decoder.getMessageBuffer();
  uint8_t size = decoder.getMessageSize();

  switch (messageType)
  {
  case MSP_PID:
    if (size >= sizeof(bf->config.pids))
    {
      memcpy(&bf->config.pids, buffer, sizeof(bf->config.pids));
    }
    break;
  }

  decoder.reset();

  return true;
}