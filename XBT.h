#if !defined(_XBT_H_)
#define _XBT_H_
#include "Arduino.h"
#include "circular_buffer.h"

CRGB XBT_leds_array[12];

typedef struct XBT_led_t {
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  void addPort(uint8_t port) { ports |= (1UL << port); }
  void solid() { _mode = 1; }
  void fade() { _mode = 2; }
  void strobe() { _mode = 3; }
  void speed(uint8_t value) { _speed = value; }
  void yield(XBT_led_t& _yield);
  void timeout(uint32_t _time_ms) { _timeout = _time_ms; }
  bool busy();
  void fadeTowardColor(const CRGB& source, const CRGB& target, uint8_t amount);

  private:
    friend class XBT; /* to access these private members */
    uint8_t _mode = 1;
    uint8_t _speed = 1;
    uint16_t ports = ( 1UL << 15 );
    XBT_led_t *yields[5] = { nullptr };
    uint32_t _timeout = 0; /* timeout for yield */
    uint32_t _current = 0; /* millis() at time of write */
    void _nblendU8TowardU8(uint8_t& cur, const uint8_t target, uint8_t amount);
    CRGB _fadeTowardColor(CRGB& cur, const CRGB& target, uint8_t amount);
    int update();

    CRGB _source;
    CRGB _target;
    uint8_t _amount;

    bool fadeToColorEnabled = 0;

} XBT_led_t;

class XBT {
  public:
    XBT(uint32_t _node, FlexCAN_T4_Base* _busWritePtr);
    void write(XBT_led_t &config);
    void state();
    void scan(bool state = 1);
    void setMAC(uint8_t controller, const char* mac);
    void eraseMAC(uint8_t controller);
    void keepAlive();

  private:
    friend struct XBT_led_t; /* to access these private members */
    uint32_t node = 0;
    FlexCAN_T4_Base* _busToWrite = nullptr;
};

#include "XBT.tpp"

#endif
