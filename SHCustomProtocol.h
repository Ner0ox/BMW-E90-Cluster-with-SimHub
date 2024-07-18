#include <mcp_can.h>
#include <mcp_can_dfs.h>
#include <SPI.h>

#ifndef __SHCUSTOMPROTOCOL_H__
#define __SHCUSTOMPROTOCOL_H__
#define lo8(x) (uint8_t)((x)&0xff)
#define hi8(x) (uint8_t)(((x) >> 8) & 0xff)

const int SPI_CS_PIN = 10;  // CHANGE THIS TO OR 9  10 ;  AND MHZ TO 8 OR 16
MCP_CAN CAN(SPI_CS_PIN);


uint32_t timestamp100ms = 0;
uint32_t timestamp200ms = 0;
int counter2 = 0;

int c_rpm = 0;
int c_speed = 0;
int c_fuel = 0;
bool s_ignition = false;
String inputString;



//dash lights

bool s_handbrake = false;
int highbeam = 0;
int handbrake = 0;
int tc = 0;
int oilWarn = 0;
int battery = 0;
int absBlink = 0;
int leftind = 0;
int rightind = 0;





int substringCount = 0;



#include <Arduino.h>

void CanSend(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
  unsigned char DataToSend[8] = { a, b, c, d, e, f, g, h };
  CAN.sendMsgBuf(address, 0, 8, DataToSend);
}

void CanSend2B(short address, byte a, byte b) {
  unsigned char DataToSend[3] = { a, b };
  CAN.sendMsgBuf(address, 0, 2, DataToSend);
}

void CanSend3B(short address, byte a, byte b, byte c) {
  unsigned char DataToSend[3] = { a, b, c };
  CAN.sendMsgBuf(address, 0, 3, DataToSend);
}

void CanSend4B(short address, byte a, byte b, byte c, byte d, byte e) {
  unsigned char DataToSend[5] = { a, b, c, d, e };
  CAN.sendMsgBuf(address, 0, 5, DataToSend);
}

void CanSend5B(short address, byte a, byte b, byte c, byte d, byte e, byte f) {
  unsigned char DataToSend[6] = { a, b, c, d, e, f };
  CAN.sendMsgBuf(address, 0, 6, DataToSend);
}

void CanSend7B(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g) {
  unsigned char DataToSend[7] = { a, b, c, d, e, f, g };
  CAN.sendMsgBuf(address, 0, 7, DataToSend);
}

void CanSend8B(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h) {
  unsigned char DataToSend[8] = { a, b, c, d, e, f, g, h };
  CAN.sendMsgBuf(address, 0, 8, DataToSend);
}

void CanSend9B(short address, byte a, byte b, byte c, byte d, byte e, byte f, byte g, byte h, byte i) {
  unsigned char DataToSend[9] = { a, b, c, d, e, f, g, h, i };
  CAN.sendMsgBuf(address, 0, 9, DataToSend);
}


////////////////////////// ////////////////////////// LIGHTS

uint16_t light_id = 0x21A;
uint8_t lights_frame[3] = { 0x00, 0x00, 0xF7 };

void lights() {
  if (s_ignition == false) {
    lights_frame[0] = 0x00;
    CAN.sendMsgBuf(light_id, 0, 3, lights_frame);
  } else {
    if (highbeam == 1) {
      lights_frame[0] = 0x26;
      CAN.sendMsgBuf(light_id, 0, 3, lights_frame);
    } else {
      lights_frame[0] = 0x35;
      CAN.sendMsgBuf(light_id, 0, 3, lights_frame);
    }
  }
}



//////////////////////////////////////////SPEED

const uint16_t CAN_ID = 0x1A6;
uint8_t speed_frame[8] = { 0x13, 0x4D, 0x46, 0x4D, 0x33, 0x4D, 0xD0, 0xFF };
uint16_t last_speed_value = 0;
void send_speed() {
  const uint8_t delta_time = 100;  // const
  uint16_t speed_value = c_speed + last_speed_value;
  uint16_t counter = (speed_frame[6] | (speed_frame[7] << 8)) & 0x0FFF;
  counter += (float)delta_time * M_PI;
  counter2 = counter;


  speed_frame[0] = speed_value;
  speed_frame[1] = (speed_value >> 8);

  speed_frame[2] = speed_frame[0];
  speed_frame[3] = speed_frame[1];

  speed_frame[4] = speed_frame[0];
  speed_frame[5] = speed_frame[1];

  // speed_frame[6] = counter;
  speed_frame[6] = counter;
  speed_frame[7] = (counter >> 8) | 0xF0;

  CAN.sendMsgBuf(CAN_ID, 0, 8, speed_frame);
  last_speed_value = speed_value;
}




/////////////////////////////////////////////////////////////////FUEL
uint8_t fuel_frame[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint16_t CAN_ID2 = 0x349;
void canSendFuel() {
  uint16_t level = min(100 + (c_fuel * 8), 8000);
  fuel_frame[0] = level;
  fuel_frame[1] = (level >> 8);

  fuel_frame[2] = fuel_frame[0];
  fuel_frame[3] = fuel_frame[1];

  CAN.sendMsgBuf(CAN_ID2, 0, 5, fuel_frame);
}



//////////////////////////////////////////RPM
void send_rpm(uint16_t rpm) {
  rpm = rpm * 4;

  CanSend(0x0AA, 0x5F, 0x59, 0xFF, 0x00,
          rpm & 0xff,
          (rpm >> 8) & 0xff,
          0x80, 0x99);
}


//////////////////////////////////////// COUNTERS

const uint16_t AIRBAG_COUNTER_CAN_ID = 0x0D7;
uint8_t airbag_counter_frame[2] = { 0xC3, 0xFF };

void canSendAirbagCounter() {
  CAN.sendMsgBuf(AIRBAG_COUNTER_CAN_ID, 0, 2, airbag_counter_frame);
  airbag_counter_frame[0]++;
}



void sendABSBrakeCounter1() {
  static uint8_t count = 0xF0;
  CanSend2B(0x0C0, count, 0xFF);

  count++;
  if (count == 0xFF) {
    count = 0xF0;
    count++;
  }
}

const uint16_t CRUISE_COUNTER_ID = 0x200;
uint8_t CRUISE_CONTROL_FRAME[8] = { random(0, 255), random(0, 255), random(0, 255), random(0, 255), random(0, 255), random(0, 255), random(0, 255), random(0, 255) };
void sendCruiseControlCounter() {
  CAN.sendMsgBuf(CRUISE_COUNTER_ID, 0, 8, CRUISE_CONTROL_FRAME);
}

////////////////////////////////////////////////////////////HANDBRAKE

const uint16_t CAN_ID3 = 0x34F;
uint8_t handbrake_frame[2] = { 0xFE, 0xFF };
void canSendHandbrake() {
  if (s_handbrake) {
    handbrake_frame[0] = 0xFE;
  } else {
    handbrake_frame[0] = 0xFD;
  }
  CAN.sendMsgBuf(CAN_ID3, 0, 2, handbrake_frame);
}



//////////////////// ABS


const uint16_t CAN_ID_ABS = 0x19E;
uint8_t abs_frame[8] = { 0x00, 0xE0, 0xB3, 0xFC, 0xF0, 0x43, 0x00, 0x65 };
void canSendAbs() {
  abs_frame[2] = ((((abs_frame[2] >> 4) + 3) << 4) & 0xF0) | 0x03;
  CAN.sendMsgBuf(CAN_ID_ABS, 0, 8, abs_frame);
}

/////////////////////////MPG

const uint16_t CAN_ID_MPG = 0x1D0;
uint8_t mpg_frame[8] = { 0x8B, 0xFF, 0x63, 0xCD, 0x5D, 0x37, 0xCD, 0xA8 };
void canSendMPG() {
  mpg_frame[4] = counter2;
  CAN.sendMsgBuf(CAN_ID_MPG, 0, 8, mpg_frame);
}

//////////////////// IGNITION

uint8_t ignition_frame_on[5] = { 0x45, 0x42, 0x69, 0x8f, 0xE2 };
uint8_t ignition_frame_off[5] = { 0x00, 0x00, 0xC0, 0x0f, 0xE2 };
const uint16_t CAN_ID_IGN = 0x130;
void canSendIgnitionFrame() {
  if (s_ignition) {
    CAN.sendMsgBuf(CAN_ID_IGN, 0, 5, ignition_frame_on);
    ignition_frame_on[4]++;
  } else {
    CAN.sendMsgBuf(CAN_ID_IGN, 0, 5, ignition_frame_off);
    ignition_frame_off[4]++;
  }
}

////////////////////////// ERRORS


void check_engine() {
  if (battery == 1) {
    CanSend(0x592, 0x40, 0x22, 0x00, 0x31, 0xFF, 0xFF, 0xFF, 0xFF);
  } else {
    CanSend(0x592, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
  }
}



//////////////////////////  INDICATORS
uint8_t indicator_right[2] = { 0xA1, 0xF1 };
uint8_t indicator_left[2] = { 0x91, 0xF1 };
uint8_t indicator_hazard[2] = { 0xB1, 0xF1 };
uint8_t indicator_off[2] = { 0x80, 0xF0 };

int blinkerstamp = 0;

void indicators() {
  if ((leftind != 0) || (rightind != 0)) {

    if (millis() - blinkerstamp > 599) {

      if ((leftind == 1) && (rightind == 0)) {
        CAN.sendMsgBuf(0x1F6, 0, 2, indicator_left);
      } else if ((leftind == 0) && (rightind == 1)) {
        CAN.sendMsgBuf(0x1F6, 0, 2, indicator_right);
      } else if ((leftind == 1) && (rightind == 1)) {
        CAN.sendMsgBuf(0x1F6, 0, 2, indicator_hazard);
      }

      blinkerstamp = millis();
    }
  } else {
    CAN.sendMsgBuf(0x1F6, 0, 2, indicator_off);
  }
}



////////////////////////// ////////////////////////// TIME
String Time;
uint8_t s_time_hour = 3;
uint8_t s_time_minute = 34;
bool is_PM = false;  // Variable to store whether it's PM or not
const uint16_t TIME_ID = 0x39E;
uint8_t time_frame[8] = { 0x0B, 0x10, 0x00, 0x0D, 0x1F, 0xDF, 0x07, 0xF2 };
int extractHours(String Time, bool &is_PM) {
  int separatorIndex = Time.indexOf(':');                                // Find the index of the first colon
  String hourPart = Time.substring(separatorIndex - 2, separatorIndex);  // Extract hours
  hourPart.trim();                                                       // Trim any leading/trailing spaces
  String ampm = Time.substring(Time.length() - 2);                       // Extract AM/PM part
  is_PM = (ampm == "PM");                                                // Check if it's PM
  return hourPart.toInt();                                               // Return hours
}
int extractMinutes(String Time) {
  int separatorIndex = Time.indexOf(':');                                 // Find the index of the first colon
  return Time.substring(separatorIndex + 1, separatorIndex + 3).toInt();  // Extract minutes
}
void TimeAndDate() {
  int hours = extractHours(Time, is_PM);
  int minutes = extractMinutes(Time);
  if (is_PM) {
    hours += 12;
  }
  time_frame[0] = hours;    // Assign hours
  time_frame[1] = minutes;  // Assign minutes

  CAN.sendMsgBuf(TIME_ID, 0, 8, time_frame);  // Send the time frame over CAN bus
}




////////////////////GEARS to do

byte Gcount = 0x0C;

void sendGearReverse() {
  CanSend4B(0x1D2, 0x78, 0x0C, 0x8B, Gcount, 0xF0);
  Gcount = Gcount + 0x10;
  if (Gcount == 0xFC) {
    Gcount = 0x0C;
  }
}


class SHCustomProtocol {
private:

public:


  void setup() {
    CAN.begin(MCP_ANY, CAN_100KBPS, MCP_8MHZ);

    CAN.setMode(MCP_ANY);

    timestamp100ms = millis();
    timestamp200ms = millis();
  }

  void read() {
    c_rpm = FlowSerialReadStringUntil(';').toInt();
    c_speed = FlowSerialReadStringUntil(';').toInt();
    c_fuel = FlowSerialReadStringUntil(';').toInt();
    s_ignition = FlowSerialReadStringUntil(';').toInt();
    // leftind = FlowSerialReadStringUntil(';').toInt();
    // rightind = FlowSerialReadStringUntil(';').toInt();
    inputString = FlowSerialReadStringUntil(';');
    Time = FlowSerialReadStringUntil('\n');
  }

  uint32_t lastTime = 0;
  uint16_t canCounter = 0;

  void canSend() {
    uint32_t courentTime = millis();

    if (courentTime - lastTime > 10) {
      lights();
      canSendIgnitionFrame();
      send_rpm(c_rpm);
      send_speed();
      sendGearReverse();
      canSendMPG();



      highbeam = 0;

      if (canCounter % 20 == 0) {  //200 ms interval
        canSendAbs();
        sendABSBrakeCounter1();
        sendCruiseControlCounter();
        canSendAirbagCounter();
        s_handbrake = false;
      }

      if (canCounter % 50 == 0) {  //500 ms interval
        canSendFuel();
        canSendHandbrake();
        // s_handbrake = false;
      }

      if (canCounter % 60 == 0) {  //600 ms interval
        indicators();

        leftind = 0;
        rightind = 0;
      }
      if (canCounter % 100 == 0) {  //1000 ms interval
        TimeAndDate();
      }

      canCounter++;
      lastTime = courentTime;
    }
  }

  void loop() {

    if (inputString.indexOf("DL_HANDBRAKE") != -1) {
      s_handbrake = true;
    }
    if (inputString.indexOf("DL_FULLBEAM") != -1) {
      highbeam = 1;
    }

    if (inputString.indexOf("DL_TC") != -1) {
      tc = 1;
    }

    if (inputString.indexOf("DL_SIGNAL_L") != -1) {
      leftind = 1;
    }

    if (inputString.indexOf("DL_SIGNAL_R") != -1) {
      rightind = 1;
    }

    if (inputString.indexOf("DL_OILWARN") != -1) {
      oilWarn = 1;
    }

    if (inputString.indexOf("DL_BATTERY") != -1) {
      battery = 1;
    }

    if (inputString.indexOf("DL_ABS") != -1) {
      absBlink = 1;
    }

    canSend();

    /*
      0x00 (OFF)
      0x04 ; 0x05 dip light (no highbeam)
      0x06 ; 0x07 highbeam
      0x20 ; 0x21 ; fog light (green)
      0x22 ; 0x23;  High beam and fog light (green)
      0x24 ; 0x25 ; fog light (green) and dip light
      0x26 ; 0x27 ; fog light (green), high beam, dip light
      0x28 ; 0x29 ; 0x30 ; 0x31  fog light (green)
      0x32 ; 0x33  high beam and fog light (green)
      0x34 ; 0x35 fog light (green) and dip light
      0x36 ; 0x37 fog light (green), high beam and dip light
      0x38 ; 0x39 fog light (green)
      0x40 ; 0x41 fog light (orange)
      0x42 ; 0x43 fog light (orange) and high beam
      0x44 ; 0x45 fog light (orange) and dip light
      0x46 ; 0x47 fog light (orange), highbeam and dip light
      0x48 ; 0x49 fog light (orange)
      0x50 fog light (orange)
    */
  }

  void idle() {}
};

#endif
