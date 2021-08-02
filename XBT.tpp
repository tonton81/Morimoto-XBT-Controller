#include <XBT.h>
#include <FlexCAN_T4.h>
#include <isotp.h>


XBT::XBT(uint32_t _node, FlexCAN_T4_Base* _busWritePtr) {
  node = _node;
  _busToWrite = _busWritePtr;
}


void XBT::write(XBT_led_t &config) {
  if ( config.reference ) {
    config.reference->red = config.red;
    config.reference->green = config.green;
    config.reference->blue = config.blue;
  }
  CAN_message_t msg;
  msg.id = node;
  msg.flags.extended = 1;
  msg.len = 7;
  msg.buf[0] = config.ports >> 8;
  msg.buf[1] = config.ports;

  int update = config.update();
  if ( update == -1 ) return; /* exit write if fastled color cycle already complete */

  for ( uint8_t i = 0; i < 5; i++ ) { /* check for any yields */
    if ( config.yields[i] != nullptr ) {
      config.yields[i]->yield_color_set = 1;
      config.yields[i]->yielded_red = config.red;
      config.yields[i]->yielded_green = config.green;
      config.yields[i]->yielded_blue = config.blue;
    }
    if ( config.yields[i] != nullptr && config.yields[i]->busy() ) {
      uint16_t last_ports = (uint16_t)((msg.buf[0]) << 8) | msg.buf[1];
      uint16_t ports = last_ports & ~(config.yields[i]->ports & 0xFFF);
      msg.buf[0] = ports >> 8;
      msg.buf[1] = ports;
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
  msg.buf[0] = 0x40 | (1UL << controller);
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
  if ( (config.id >> abs(16 - __builtin_clzl(config.id))) == 0xFADE ) {
    if ( buf[0] == 0 ) { /* show programmed connections and their state */
      if ( config.len == 26 ) Serial.print("\n***** XBT Connections *****\n");
      else if ( buf[26] == 1 ) Serial.print("\n***** XKC Connections *****\n");
      for ( uint8_t i = 0; i < 4; i++ ) {
        if ( config.len == 26 ) Serial.print("XBT ");
        else if ( buf[26] == 1 ) Serial.print("XKC ");
        Serial.print(i);
        Serial.print(":\tAddress: ");
        for ( uint8_t a = 0; a < 6; a++ ) {
          Serial.print(buf[2 + a + (i*6)],HEX);
          if ( a < 5 ) Serial.print(":");
        }
        Serial.print("\tStatus: ");
        Serial.print(((buf[1] & (1UL << i)) ? "Connected" : "Disconnected"));
        if ( config.len > 26 && buf[26] == 1 ) Serial.printf("\tZones: %d", (uint8_t)((buf[27] >> (i * 2)) & 3U));
        Serial.println();
        Serial.flush();
      }
    }
    else { /* show found devices not programmed from scan */
      if ( buf[0] == 1 ) Serial.print("\nFound XBT controller at address: ");
      else if ( buf[0] == 2 ) Serial.print("\nFound XKC controller at address: ");
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
  if ( ((millis() - _current) > _timeout) ) _current = 0;
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
        yields[i]->yield_color_set = 1;
        yields[i]->yielded_red = red;
        yields[i]->yielded_green = green;
        yields[i]->yielded_blue = blue;
        yields[i]->red = red;
        yields[i]->green = green;
        yields[i]->blue = blue;
        break;
      }
    }
  }
}


int XBT_led_t::update() {

  _current = millis();

  if ( useHSVfading ) {
    if ( _currentHSV.h == _targetHSV.h && _currentHSV.s == _targetHSV.s && _currentHSV.v == _targetHSV.v ) {
     _finished = 1;
      useHSVfading = 0;
      return -1;
    }
    _currentHSV = blend(_sourceHSV, _targetHSV, _current_amount, SHORTEST_HUES);
    _current_amount += _amount;
    if ( _current_amount > 255 ) _current_amount = 255;
    CRGB color = _currentHSV;
    red = color.red;
    green = color.green;
    blue = color.blue;
  }

  if ( useRGBfading ) {
    if ( _currentRGB == _targetRGB ) {
      _finished = 1;
      useRGBfading = 0;
      return -1;
    }
    _fadeWithRGB(_currentRGB, _targetRGB, _amount);
    red = _currentRGB.red;
    green = _currentRGB.green;
    blue = _currentRGB.blue;
  }

  return 0;
}

void XBT_led_t::setColor(const CRGB& target) {
  _currentRGB = target;
  red = target.red;
  green = target.green;
  blue = target.blue;
}


void XBT_led_t::setBrightness(uint8_t value) {
  _targetHSV = rgb2hsv_approximate(CRGB(red, green, blue));
  _targetHSV.v = value;
  setColor(_targetHSV);
}


void XBT_led_t::fadeWithHSV(const CRGB& target, uint16_t amount) {
  if ( reference ) {
    useHSVfading = 1;
    _finished = 0;
    _sourceHSV = _currentHSV = rgb2hsv_approximate(*reference);
    _targetHSV = rgb2hsv_approximate(target);
    _amount = amount;
    _current_amount = 0;
    red = reference->red;
    green = reference->green;
    blue = reference->blue;
  }
  else if ( yield_color_set ) {
    fadeWithHSV(CRGB(yielded_red, yielded_green, yielded_blue), target, amount);
  }
  else fadeWithHSV(CRGB(red, green, blue), target, amount);
}


void XBT_led_t::fadeWithHSV(const CRGB& source, const CRGB& target, uint16_t amount) {
  useHSVfading = 1;
  _finished = 0;
  _sourceHSV = _currentHSV = rgb2hsv_approximate(source);
  _targetHSV = rgb2hsv_approximate(target);
  _amount = amount;
  _current_amount = 0;
  red = source.red;
  green = source.green;
  blue = source.blue;
}


void XBT_led_t::fadeWithRGB(const CRGB& target, uint8_t amount) {
  if ( reference ) {
    _finished = 0;
    useRGBfading = 1;
    _sourceRGB = _currentRGB = *reference;
    _targetRGB = target;
    _amount = amount;
    red = reference->red;
    green = reference->green;
    blue = reference->blue;
  }
  else if ( yield_color_set ) {
    fadeWithRGB(CRGB(yielded_red, yielded_green, yielded_blue), target, amount);
  }
  else fadeWithRGB(_currentRGB, target, amount);
}
void XBT_led_t::fadeWithRGB(const CRGB& source, const CRGB& target, uint8_t amount) {
  _finished = 0;
  useRGBfading = 1;
  _sourceRGB = _currentRGB = source;
  _targetRGB = target;
  _amount = amount;
  red = source.red;
  green = source.green;
  blue = source.blue;
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

CRGB XBT_led_t::_fadeWithRGB(CRGB& cur, const CRGB& target, uint8_t amount) {
  _nblendU8TowardU8(cur.red, target.red, amount);
  _nblendU8TowardU8(cur.green, target.green, amount);
  _nblendU8TowardU8(cur.blue, target.blue, amount);
  return cur;
}
