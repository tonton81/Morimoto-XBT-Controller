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


void XBT::write(XBT_led_t &config) {
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 7;
  msg.buf[0] = config.ports >> 8;
  msg.buf[1] = config.ports;

  int update = config.update();
  if ( update == -1 ) return;

  for ( uint8_t i = 0; i < 5; i++ ) { /* check for any yields */
    if ( config.yields[i] != nullptr ) {
      if ( config.yields[i]->_current ) {
        if ( ((millis() - config.yields[i]->_current) > config.yields[i]->_timeout) ) {
          config.yields[i]->_current = 0;
          continue;
        }
        if ( ((millis() - config.yields[i]->_current) < config.yields[i]->_timeout) ) {
          uint16_t last_ports = (uint16_t)((msg.buf[0]) << 8) | msg.buf[1];
          uint16_t ports = last_ports & ~(config.yields[i]->ports & 0xFFF);
          msg.buf[0] = ports >> 8;
          msg.buf[1] = ports;
        }
      }
    }
  }

  msg.buf[2] = config.red;
  msg.buf[3] = config.green;
  msg.buf[4] = config.blue;
  msg.buf[5] = config._mode;
  msg.buf[6] = config._speed;

  uint16_t current_ports = (uint16_t)((msg.buf[0]) << 8) | msg.buf[1]; /* store local color history */
  for ( uint8_t i = 0; i < 12; i++ ) {
    if ( current_ports & (1U << i) ) {
      XBT_leds_array[i].red = config.red;
      XBT_leds_array[i].green = config.green;
      XBT_leds_array[i].blue = config.blue;
    }
  }

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





bool XBT_led_t::busy() {
  if ( ((millis() - _current) > _timeout) ) {
    _current = 0;
  }
  return _current;
}


void XBT_led_t::yield(XBT_led_t& _yield) {
  bool found = 0;
  for ( uint8_t i = 0; i < 5; i++ ) { /* check if already added */
    if ( yields[i] == &_yield ) {
      found = 1;
      break;
    }
  }
  if ( !found ) {
    for ( uint8_t i = 0; i < 5; i++ ) { /* add if not already added */
      if ( yields[i] == nullptr ) {
        yields[i] = &_yield;
        break;
      }
    }
  }
}


int XBT_led_t::update() {
  _current = millis();
  if ( fadeToColorEnabled ) {
    CRGB current = CRGB(red, green, blue);
    if ( current == _target ) return -1;
    _fadeTowardColor(current, _target, _amount);
    red = current.red;
    green = current.green;
    blue = current.blue;
  }
  return 0;
}


void XBT_led_t::fadeTowardColor(const CRGB& source, const CRGB& target, uint8_t amount) {
  fadeToColorEnabled = 1;
  _source = source;
  red = source.red;
  green = source.green;
  blue = source.blue;
  _target = target;
  _amount = amount;
}


void XBT_led_t::_nblendU8TowardU8(uint8_t& cur, const uint8_t target, uint8_t amount) {
  if ( cur == target) return;
  if ( cur < target ) {
    uint8_t delta = target - cur;
    delta = scale8_video(delta, amount);
    cur += delta;
  }
  else {
    uint8_t delta = cur - target;
    delta = scale8_video(delta, amount);
    cur -= delta;
  }
}

CRGB XBT_led_t::_fadeTowardColor(CRGB& cur, const CRGB& target, uint8_t amount) {
  _nblendU8TowardU8(cur.red, target.red, amount);
  _nblendU8TowardU8(cur.green, target.green, amount);
  _nblendU8TowardU8(cur.blue, target.blue, amount);
  return cur;
}