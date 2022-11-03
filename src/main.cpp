#include <Arduino.h>
#include <Streaming.h>
#include <myMacros.h>
#include <myUtils.h>

#define DEBUG__EIN  //"Schalter" zum aktivieren von DEBUG-Ausgaben
#include <SoftwareSerial.h>
#include <myDebug.h>
#include <truma.h>

/* LIN PACKET:
   It consist of:
    ___________ __________ _______ ____________ _________
   |           |          |       |            |         |
   |Synch Break|Synch Byte|ID byte| Data Bytes |Checksum |
   |___________|__________|_______|____________|_________|

   Every byte have start bit and stop bit and it is send LSB first.
   Synch Break - 13 bits of dominant state ("0"), followed by 1 bit recesive state ("1")
   Synch Byte - Byte for Bound rate syncronization, always 0x55
   ID Byte - consist of parity, length and address; parity is determined by LIN standard and depends from address and message length
   Data Bytes - user defined; depend on devices on LIN bus
   Checksum - inverted 256 checksum; data bytes are sumed up and then inverted
*/
// https://github.com/mestrode/Lin-Interface-Library/blob/main/src/Lin_Interface.cpp

// LIN serial Interface
const uint16_t linSpeed = 9600;  // speed LIN of IBS-Sensor (do not change)
const int16_t linRX     = 2;     // rot    13;   // RX Pin LIN serial
const int16_t linTX     = 3;     // orange 12;   // TX Pin LIN serial

//#define test
#ifndef test
  SoftwareSerial linSerial(linTX, linRX);  // RX, TX
#else
  #define linSerial Serial
#endif

byte LinMessage[9] = {0};

float tRoom;   // Room temperature [degC]
float tWater;  // Water temperature [degC]

uint8_t ProtectedID;
uint8_t ID;
bool ChecksumValid;
int8_t bytes_received = -4;
bool decode=true;

//------------------------------------------
void setup();
void loop();
bool linLoop();
void linDecode();
void serialCommand();
uint8_t getChecksum(uint8_t ProtectedID, uint8_t dataLen);
template <typename... T>
void Serial_printf(const char* str, T... args);
//-----------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  while (!Serial && (millis() < 3000))
    ;
  Serial << "\n\n" << ProjektName << " - " << VERSION << "  (" << BUILDDATE << "  " __TIME__ << ")" << endl;

  linSerial.begin(linSpeed);
}

void loop() {
  if (linLoop()) {
    if (decode) linDecode();
  }
  #ifndef test
   serialCommand();
   #endif
}



bool linLoop() {
  while (linSerial.available()) {
    if (bytes_received >= (8 + 1))  // max 8x Data + 1x Checksum
    {
      // receive max 9 Bytes: 8 Data + 1 Chksum
      break;
    }
    switch (bytes_received) {
      case -4:  //??
      case -3:  // break = 0x00
      case -2:  // sync = 0x55
      {
        // warten auf break und sync
        uint8_t buffer = linSerial.read();
        // Sync and PID may to be verified here
        if (buffer == 0x00) {  // break
          bytes_received = -3;
          // DEBUG__PRINTLN("break");
        }
        if (buffer == 0x55) {  // sync
          bytes_received = -2;
          // DEBUG__PRINTLN("sync");
        }
        break;
      }
      case -1:  // Protected ID
        ProtectedID    = linSerial.read();
        ID             = ProtectedID & 0x3f;
        bytes_received = -1;
        // DEBUG__PRINTF("PID=%x\n",ProtectedID);
        break;

      default:  // Data 0...7, Checksum
        // Receive and save only Data Byte (send by slave)
        LinMessage[bytes_received] = linSerial.read();
    }
    bytes_received++;
    // DEBUG__PRINTF("bytes_received=%i\n",bytes_received);
  }

  if (bytes_received >= 9) {  // alle Zeichen empfangen
    bytes_received--;
    uint8_t Checksum = LinMessage[bytes_received];

    // verify Checksum
    uint8_t checksum2 = getChecksum(ProtectedID, bytes_received);
    // DEBUG__PRINTF("ProtectedID: %02x  Checksum: %02x <-> %02x\n",ProtectedID , Checksum , checksum2 );
    ChecksumValid = (0xFF == (uint8_t)(Checksum + ~getChecksum(ProtectedID, bytes_received)));

    Serial_printf("ID: %02X --> %02X - %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X - %02X", ID, ProtectedID, LinMessage[0], LinMessage[1], LinMessage[2], LinMessage[3], LinMessage[4], LinMessage[5], LinMessage[6], LinMessage[7], Checksum);
    if (!ChecksumValid) {
      Serial_printf(" - Checksum-Error(%02X)", checksum2);
    }
    Serial.println();
    bytes_received = -4;
    linSerial.end();
    linSerial.begin(linSpeed);
    return true;
  }
  return false;
}

void linDecode() {
  switch (ID) {
    case 0x20:
      tRoom  = float(LinMessage[0] + ((LinMessage[1] & 0x0F) << 8)) / 10.0 - 273;         // Room temperature [degC]
      tWater = float((LinMessage[2] << 4) + ((LinMessage[1] & 0xF0) >> 4)) / 10.0 - 273;  // Water temperature [degC]
      Serial_printf("Raumtemperatur-Soll   = %0.1f\n", tRoom);
      Serial_printf("Wassertemperatur-Soll = %0.1f\n", tWater);
      Serial_printf("energy_mix    : %02X = %s\n", LinMessage[3], trumaMap(ENERGY_MIX_MAPPING, COUNT(ENERGY_MIX_MAPPING), LinMessage[3]));
      Serial_printf("energy_mode   : %02X = %s\n", LinMessage[4], trumaMap(ENERGY_MODE_MAPPING, COUNT(ENERGY_MODE_MAPPING), LinMessage[4]));
      Serial_printf("energy_mode-2 : %02X = %s\n", LinMessage[5] & 0x0f, trumaMap(ENERGY_MODE_2_MAPPING, COUNT(ENERGY_MODE_2_MAPPING), LinMessage[5] & 0x0f));
      Serial_printf("vent_mode     : %02X = %s\n", LinMessage[5] >> 4, trumaMap(VENT_MODE_MAPPING, COUNT(VENT_MODE_MAPPING), LinMessage[5] >> 4));
      break;

    case 0x21:
      tRoom  = float(LinMessage[0] + ((LinMessage[1] & 0x0F) << 8)) / 10.0 - 273.15;         // Room temperature [degC]
      tWater = float((LinMessage[2] << 4) + ((LinMessage[1] & 0xF0) >> 4)) / 10.0 - 273.15;  // Water temperature [degC]
      Serial_printf("Raumtemperatur-Ist   = %0.1f\n", tRoom);
      Serial_printf("Wassertemperatur-Ist = %0.1f\n", tWater);
      Serial_printf("vent_or_operating-status: %02X = %s\n", LinMessage[5], trumaMap(VENT_OR_OPERATING_STATUS, COUNT(VENT_OR_OPERATING_STATUS), LinMessage[5]));
      break;

    case 0x22:
      Serial_printf("Spannung = %0.2f\n", float(LinMessage[0] / 10.0));
      Serial_printf("cp_plus_display_status: %02X = %s\n", LinMessage[1], trumaMap(CP_PLUS_DISPLAY_STATUS_MAPPING, COUNT(CP_PLUS_DISPLAY_STATUS_MAPPING), LinMessage[1]));
      Serial_printf("heating_status        : %02X = %s\n", LinMessage[2], trumaMap(HEATING_STATUS_MAPPING, COUNT(HEATING_STATUS_MAPPING), LinMessage[2]));
      Serial_printf("heating_status2       : %02X = %s\n", LinMessage[3], trumaMap(HEATING_STATUS_2_MAPPING, COUNT(HEATING_STATUS_2_MAPPING), LinMessage[3]));
      break;
    default:
      break;
  }
}

void serialCommand() {
  String msg;
  if (Serial.available()) {
    msg = Serial.readString();
    //DEBUG__PRINTF("msg:%s", msg.c_str());
    switch (msg[0]) {
      case 'd':
        DEBUG__PRINTLN("decode on");
        decode = true;
        break;

      case 'n':
        DEBUG__PRINTLN("decode off");
        decode = false;
        break;

      default:
        break;
    }
    delay(100);
  }
}

/// @brief Checksum calculation for LIN Frame
/// @details
/// EnhancedChecksum considers ProtectedID
///     LIN 2.0 only for FrameID between 0x00..0x3B
///     LIN 2.0 uses for 0x3C and above ClassicChecksum for legacy (auto detected)
/// ClassicChecksum
///     LIN 1.x in general (use 'ProtectedID' = 0x00 to ensure that)
/// see LIN Specification 2.2A (2021-12-31) for details
///     https://microchipdeveloper.com/local--files/lin:specification/LIN-Spec_2.2_Rev_A.PDF
///     2.8.3 Example of Checksum Calculation
/// @param ProtectedID initial Byte, set to 0x00, when calc Checksum for classic LIN Frame
/// @param dataLen length of Frame (only Data Bytes)
/// @returns calculated checksum
uint8_t getChecksum(uint8_t ProtectedID, uint8_t dataLen) {
  uint16_t sum = ProtectedID;
  // test FrameID bits for classicChecksum
  if ((sum & 0x3F) >= 0x3C) {
    // LIN 1.x: legacy
    // LIN 2.0: don't include PID for ChkSum calculation on configuration and reserved frames
    sum = 0x00;
  }
  // sum up all bytes (including carryover to the high byte)
  // ID allready considered
  while (dataLen-- > 0) sum += LinMessage[dataLen];
  // add high byte (carry over) to the low byte
  while (sum >> 8) sum = (sum & 0xFF) + (sum >> 8);
  // inverting result
  return (~sum);
}

// Hilfsroutine weil Arduino kein Serial.printf hat
//  https://www.e-tinkers.com/2020/01/do-you-know-arduino-sprintf-and-floating-point/
//  https://en.cppreference.com/w/cpp/language/parameter_pack
template <typename... T>
void Serial_printf(const char* str, T... args) {
  int len = snprintf(NULL, 0, str, args...);
  if (len) {
    char buff[len + 1];
    snprintf(buff, len + 1, str, args...);
    Serial.print(buff);
  }
}
