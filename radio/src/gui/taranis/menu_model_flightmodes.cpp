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

#include "../../opentx.h"

void displayFlightModes(coord_t x, coord_t y, FlightModesType value)
{
  lcdDrawText(x, y, STR_FP);
  x = lcdNextPos + 1;
  for (uint8_t p=0; p<MAX_FLIGHT_MODES; p++) {
    lcdDrawChar(x, y, ((value & (1<<p)) ? '-' : '0'+p));
    x += 5;
  }
}

FlightModesType editFlightModes(coord_t x, coord_t y, uint8_t event, FlightModesType value, uint8_t attr)
{
  lcd_putsColumnLeft(x, y, STR_FLMODE);

  int posHorz = menuHorizontalPosition;

  for (int p=0; p<MAX_FLIGHT_MODES; p++) {
    LcdFlags flags = 0;
    if (attr) {
      flags |= INVERS;
      if (posHorz==p) flags |= BLINK;
    }
    if (value & (1<<p))
      lcdDrawChar(x, y, ' ', flags|FIXEDWIDTH);
    else
      lcdDrawChar(x, y, '0'+p, flags);
    x += FW;
  }

  if (attr) {
    if (s_editMode && event==EVT_KEY_BREAK(KEY_ENTER)) {
      s_editMode = 0;
      value ^= (1<<posHorz);
      storageDirty(EE_MODEL);
    }
  }

  return value;
}

enum FlightModesItems {
  ITEM_FLIGHT_MODES_NAME,
  ITEM_FLIGHT_MODES_SWITCH,
  ITEM_FLIGHT_MODES_TRIM_RUD,
  ITEM_FLIGHT_MODES_TRIM_ELE,
  ITEM_FLIGHT_MODES_TRIM_THR,
  ITEM_FLIGHT_MODES_TRIM_AIL,
  ITEM_FLIGHT_MODES_FADE_IN,
  ITEM_FLIGHT_MODES_FADE_OUT,
  ITEM_FLIGHT_MODES_COUNT,
  ITEM_FLIGHT_MODES_LAST = ITEM_FLIGHT_MODES_COUNT-1
};

bool isTrimModeAvailable(int mode)
{
  return (mode < 0 || (mode%2) == 0 || (mode/2) != menuVerticalPosition);
}

void menuModelFlightModesAll(uint8_t event)
{
  MENU(STR_MENUFLIGHTPHASES, menuTabModel, e_FlightModesAll, MAX_FLIGHT_MODES+1, { NAVIGATION_LINE_BY_LINE|(ITEM_FLIGHT_MODES_LAST-1), NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, NAVIGATION_LINE_BY_LINE|ITEM_FLIGHT_MODES_LAST, 0 });

  int8_t sub = menuVerticalPosition;

  horzpos_t posHorz = menuHorizontalPosition;
  if (sub==0 && posHorz > 0) { posHorz += 1; }

  if (sub<MAX_FLIGHT_MODES && posHorz>=0) {
    drawColumnHeader(STR_PHASES_HEADERS, posHorz);
  }

  for (uint8_t i=0; i<LCD_LINES-1; i++) {
    coord_t y = MENU_HEADER_HEIGHT + 1 + i*FH;
    uint8_t k = i+menuVerticalOffset;

    if (k==MAX_FLIGHT_MODES) {
      // last line available - add the "check trims" line
      lcd_putsLeft((LCD_LINES-1)*FH+1, STR_CHECKTRIMS);
      putsFlightMode(OFS_CHECKTRIMS, (LCD_LINES-1)*FH+1, mixerCurrentFlightMode+1);
      if (sub==MAX_FLIGHT_MODES) {
        if (!trimsCheckTimer) {
          if (event == EVT_KEY_FIRST(KEY_ENTER)) {
            trimsCheckTimer = 200; // 2 seconds trims cancelled
            s_editMode = 1;
            killEvents(event);
          }
          else {
            lcdInvertLastLine();
            s_editMode = 0;
          }
        }
        else {
          if (event == EVT_KEY_FIRST(KEY_EXIT)) {
            trimsCheckTimer = 0;
            s_editMode = 0;
            killEvents(event);
          }
        }
      }
      return;
    }

    FlightModeData *p = flightModeAddress(k);

    putsFlightMode(0, y, k+1, (getFlightMode()==k ? BOLD : 0) | ((sub==k && menuHorizontalPosition<0) ? INVERS : 0));

    for (uint8_t j=0; j<ITEM_FLIGHT_MODES_COUNT; j++) {
      uint8_t attr = ((sub==k && posHorz==j) ? ((s_editMode>0) ? BLINK|INVERS : INVERS) : 0);
      uint8_t active = (attr && s_editMode>0) ;
      switch (j) {
        case ITEM_FLIGHT_MODES_NAME:
          editName(4*FW-1, y, p->name, sizeof(p->name), event, attr);
          break;

        case ITEM_FLIGHT_MODES_SWITCH:
          if (k>0) {
            putsSwitches((5+LEN_FLIGHT_MODE_NAME)*FW+FW/2, y, p->swtch, attr);
            if (active) CHECK_INCDEC_MODELSWITCH(event, p->swtch, SWSRC_FIRST_IN_MIXES, SWSRC_LAST_IN_MIXES, isSwitchAvailableInMixes);
          }
          break;

        case ITEM_FLIGHT_MODES_TRIM_RUD:
        case ITEM_FLIGHT_MODES_TRIM_ELE:
        case ITEM_FLIGHT_MODES_TRIM_THR:
        case ITEM_FLIGHT_MODES_TRIM_AIL:
        {
          uint8_t t = j-ITEM_FLIGHT_MODES_TRIM_RUD;
          putsTrimMode((4+LEN_FLIGHT_MODE_NAME)*FW+j*(5*FW/2), y, k, t, attr);
          if (active) {
            trim_t & v = p->trim[t];
            v.mode = checkIncDec(event, v.mode==TRIM_MODE_NONE ? -1 : v.mode, -1, k==0 ? 0 : 2*MAX_FLIGHT_MODES-1, EE_MODEL, isTrimModeAvailable);
          }
          break;
        }

        case ITEM_FLIGHT_MODES_FADE_IN:
          lcdDrawNumber(32*FW-2, y, (10/DELAY_STEP)*p->fadeIn, attr|PREC1);
          if (active) p->fadeIn = checkIncDec(event, p->fadeIn, 0, DELAY_MAX, EE_MODEL|NO_INCDEC_MARKS);
          break;

        case ITEM_FLIGHT_MODES_FADE_OUT:
          lcdDrawNumber(35*FW, y, (10/DELAY_STEP)*p->fadeOut, attr|PREC1);
          if (active) p->fadeOut = checkIncDec(event, p->fadeOut, 0, DELAY_MAX, EE_MODEL|NO_INCDEC_MARKS);
          break;

      }
    }
  }
}