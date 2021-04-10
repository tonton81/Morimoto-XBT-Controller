#include <XBT.h>
#include "Arduino.h"
#include <atomic>
#include <util/atomic.h>
#include "Stream.h"
#include "circular_buffer.h"
#include <FlexCAN_T4.h>
#include <isotp.h>

XBT::XBT(uint32_t _node, FlexCAN_T4_Base* _busWritePtr) {
  node = _node;
  _busToWrite = _busWritePtr;
}


void XBT::write(const XBT_led_t &config) {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 7;
  msg.buf[0] = config.ports >> 8;
  msg.buf[1] = config.ports;
  msg.buf[2] = config.red;
  msg.buf[3] = config.green;
  msg.buf[4] = config.blue;
  msg.buf[5] = config._mode;
  msg.buf[6] = config._speed;
  _busToWrite->write(msg);
}


void XBT::state() {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 2;
  msg.buf[0] = (1UL << 5);
  _busToWrite->write(msg);
}


void XBT::scan(bool state) {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 2;
  msg.buf[0] = (1UL << 4);
  msg.buf[1] = state;
  _busToWrite->write(msg);
}


void XBT::eraseMAC(uint8_t controller) {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 2;
  msg.buf[0] = (1UL << 6) | (1UL << controller);
  _busToWrite->write(msg);
}


void XBT::setMAC(uint8_t controller, const char* mac) {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 7;
  msg.buf[0] = 0x40 | (1UL << 1);
  int values[6] = { 0 };
  sscanf(mac, "%x:%x:%x:%x:%x:%x", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]);
  for ( int i = 0; i < 6; i++) msg.buf[i + 1] = values[i];
  _busToWrite->write(msg);
}


void XBT::keepAlive() {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 0;
  _busToWrite->write(msg);
}


void ext_isotp_output1(const ISOTP_data &config, const uint8_t *buf) {
  if ( config.id == 0xFADE ) {
    if ( buf[0] == 0 ) { /* show programmed connections and their state */
      Serial.print("\n***** XBT Connections *****\n");
      for ( uint8_t i = 0; i < 4; i++ ) {
        Serial.print("XBT ");
        Serial.print(i);
        Serial.print(":\tAddress: ");
        for ( uint8_t a = 0; a < 6; a++ ) {
          Serial.print(buf[2 + a + (i*6)],HEX);
          if ( a < 5 ) Serial.print(":");
        }
        Serial.print("\tStatus: ");
        Serial.print(((buf[1] & (1UL << i)) ? "Connected" : "Disconnected"));
        Serial.println();
        Serial.flush();
      }
    }
    if ( buf[0] == 1 ) { /* show found devices not programmed from scan */
      Serial.print("\nFound XBT controller at address: ");
      for ( int i = 0; i < 6; i++ ) {
        Serial.print(buf[3+i],HEX);
        if ( i < 5 ) Serial.print(":");
      }
      Serial.print("\tRSSI: ");
      Serial.println(((int16_t)(buf[1] << 8)) | buf[2]);
      Serial.flush();
    }
  }
}







