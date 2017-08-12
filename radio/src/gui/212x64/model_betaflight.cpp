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
#include "telemetry/betaflight_msp.h"

enum BFPageType
{
  PIDS_PAGE,
  RATES_PAGE,
  THROTTLE_PAGE,
  VTX_PAGE,

  NUM_BF_PAGES
};



#define MENU_WIDTH 8*FW

struct BFPage;

//assert( sizeof(pageNames) == NUM_BF_PAGES);

typedef void (*drawPage_t)( BFPage& page, event_t event );

struct BFPage
{
  BFPageType type;
  const char* name;
  drawPage_t drawPage;
  int editMode;
  int selectedPos;
  int maxPos;

  void handleSelectedPositionInput(event_t& event);
  void handleFieldEditing(event_t& event);
};


int bfMenuPos = 0;
int bfEditPage = -1;
int selectedPage = 0;
uint8_t pids[3][3];
BFMspDecoder mspDecoder;


void drawPIDsPage( BFPage& page, event_t event );


BFPage pages[] =
{
  { PIDS_PAGE,     "PIDs    ", drawPIDsPage,  0, 0, 9 },
  { RATES_PAGE,    "Rates   ", NULL,          0, 0, 0 },
  { THROTTLE_PAGE, "Throttle", NULL,          0, 0, 0 },
  { VTX_PAGE,      "Video Tx", NULL,          0, 0, 0 }
};

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

void updateMessages()
{
  if( betaflightInputTelemetryFifo == NULL )
  {
    return;
  }

  if( betaflightInputTelemetryFifo->isEmpty() )
  {
    //debugPrintf("msp fifo empty\r\n");
    return;
  }

  SportTelemetryPacket packet;

  if( betaflightInputTelemetryFifo->size() < sizeof(packet))
  {
    return;
  }

  debugPrintf("packet recv: ");
  for (uint8_t i=0; i<sizeof(packet); i++) 
  {
      betaflightInputTelemetryFifo->pop(packet.raw[i]);
      debugPrintf("%0X", packet.raw[i]);
  }
  debugPrintf("\r\n");


  if( !mspDecoder.decodePacket( packet ) )
  {
    return;
  }

  // valid msp message decoded
  debugPrintf("message decoded!\r\n");
}

void drawPIDsPage( BFPage& page, event_t event )
{
//  SIMPLE_SUBMENU( "Betaflight > PIDs", 9);
  
  coord_t x = MENU_WIDTH + FW*2;
  coord_t y = MENU_HEADER_HEIGHT + 1 + FH;

  int step= 28;

  lcdDrawText(x + FW*6,        y, "P", 0 );
  lcdDrawText(x + FW*6+step,   y, "I", 0 );
  lcdDrawText(x + FW*6+step*2, y, "D", 0 );

  lcdDrawText(x, y+FH*1 + 2, "ROLL",  0 );
  lcdDrawText(x, y+FH*2 + 2, "PITCH", 0 );
  lcdDrawText(x, y+FH*3 + 2, "YAW",   0 );

  uint8_t pidNumStartX = x + FW*6;
  uint8_t pidNumStartY = y + FH*2;

  page.handleSelectedPositionInput(event);

  for( int axis=0; axis < 3; ++axis)
  {
    for( int t=0; t < 3; ++t )
    {
      int i = t * 3 + axis;
      bool selectedField = page.selectedPos == i && page.editMode > EDIT_SELECT_MENU;
      LcdFlags attr = (selectedField ? (page.editMode>0 ? BLINK|INVERS : INVERS) : 0);

      if( selectedField )
      {
        page.handleFieldEditing(event);

        switch( page.editMode )
        {
          case EDIT_MODIFY_FIELD:
            switch (event)
            {
              case EVT_KEY_FIRST(KEY_DOWN):
                --pids[axis][t];
                break;

              case EVT_KEY_FIRST(KEY_UP):
                ++pids[axis][t];
                break;
            }

            pids[axis][t] %= 255;
            break;
        }
      }

      lcdDrawNumber(
        pidNumStartX + t*step,
        pidNumStartY + axis*(FH + 2),
        pids[axis][t],
        attr);
    }
  }
}

//assert( sizeof(pageNames) == NUM_BF_PAGES);



void menuModelBetaflightMspSmartPort(event_t event)
{
  SIMPLE_MENU(
    "Betaflight",
    menuTabModel,
    MENU_MODEL_BETAFLIGHT_MSP_SMARTPORT,
    NUM_BF_PAGES
  );

  if( betaflightInputTelemetryFifo == NULL )
  {
    betaflightInputTelemetryFifo = new Fifo<uint8_t, BETAFLIGHT_TELEMETRY_INPUT_FIFO_SIZE>();
  }

  updateMessages();
  
  if( bfEditPage == -1 )
  {
    switch (event)
    {
      case EVT_KEY_FIRST(KEY_DOWN):
        ++bfMenuPos;
        killEvents(event);
        event = 0;
        break;

      case EVT_KEY_FIRST(KEY_UP):
        --bfMenuPos;
        killEvents(event);
        event = 0;
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

    if( selected )
    {
      if( bfEditPage == -1 )
      {
        if( event == EVT_KEY_FIRST(KEY_ENTER) )
        {
          bfEditPage = page.type;
          page.editMode = EDIT_SELECT_FIELD;
          event = 0;
        }
      }
      else
      {
        if( page.editMode == EDIT_SELECT_FIELD
           && event == EVT_KEY_FIRST(KEY_EXIT))
        {
          bfEditPage = -1;
          page.editMode = EDIT_SELECT_MENU;
          event=0;
        }
      }

      if( page.drawPage != NULL)
      {
        page.drawPage(page, event);
      }
    }
  }
}
