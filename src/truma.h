#pragma once
#ifndef myTRUMA_H
#define myTRUMA_H
#include <Arduino.h>

struct truma {
  uint8_t code;
  char text[20];
};

truma ENERGY_MIX_MAPPING[] = {     //ID:20  Data:3
  {0x00, "electricity"},
  {0xFA, "gas/mix"}};

truma ENERGY_MODE_MAPPING[] = {    //ID:20  Data:4
  {0x00, "gas"},
  {0x09, "mix/electricity 1"},
  {0x12, "mix/electricity 2"}};

truma ENERGY_MODE_2_MAPPING[] = {  //ID:20  Data:5&0F
  {0x01, "Gas"},
  {0x02, "Electricity"},
  {0x03, "Gas/Electricity"}};

truma VENT_MODE_MAPPING[] = {      //ID:20  Data:5 >> 4
  {0x00, "Off"},
  {0x0B, "Eco"},
  {0x0D, "High"},
  {0x01, "Vent 1"}, {0x02, "Vent 2"},
  {0x03, "Vent 3"}, {0x04, "Vent 4"},
  {0x05, "Vent 5"}, {0x06, "Vent 6"},
  {0x07, "Vent 7"}, {0x08, "Vent 8"},
  {0x09, "Vent 9"}, {0x0A, "Vent 10"}};

truma VENT_OR_OPERATING_STATUS[] = {  //ID:21(61)  Data:5
  {0x01, "off"},
  {0x22, "on + airvent"},
  {0x02, "on"},
  {0x31, "error (?)"},
  {0x32, "fatal error"},
  {0x21, "airvent (?)"}};

truma CP_PLUS_DISPLAY_STATUS_MAPPING[] = {  //ID:22(e2)  Data:1
  {0xF0, "heating on"},
  {0x20, "standby ac on"},
  {0x00, "standby ac off"},
  {0xD0, "error"},
  {0x70, "fatal error"},
  {0x50, "boiler on"},
  {0x40, "boiler off"}};

truma HEATING_STATUS_MAPPING[] = {    //ID:22(e2)  Data:2
  {0x10, "boiler eco done"},
  {0x11, "boiler eco heating"},
  {0x30, "boiler hot done"},
  {0x31, "boiler hot heating"}};

truma HEATING_STATUS_2_MAPPING[] = {  //ID:22(e2)  Data:3
  {0x04, "normal"},
  {0x05, "error"},
  {0xFF, "fatal error (?)"},
  {0xFE, "normal (?)"}};

char truma_unbekannt[] = "????";


char* trumaMap(truma map[], size_t size, uint8_t code) {
  for (size_t i = 0; i < size; i++) {
    if (map[i].code == code) {
      //DEBUG__PRINTF("trmaMap - %02x %s\n", code,map[i].text );
      return map[i].text;
    }
  }
  return truma_unbekannt;
}

#endif