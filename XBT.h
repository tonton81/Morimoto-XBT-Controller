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
  void fade() { _mode = 2; } /* XBT hardware free-running mode */
  void strobe() { _mode = 3; } /* XBT hardware free-running mode */
  void speed(uint8_t value) { _speed = value; }
  void yield(XBT_led_t& _yield);
  void timeout(uint32_t _time_ms) { _timeout = _time_ms; }
  bool busy();
  void fadeWithRGB(const CRGB& target, uint8_t amount);
  void fadeWithRGB(const CRGB& source, const CRGB& target, uint8_t amount);
  void fadeWithHSV(const CRGB& target, uint16_t amount);
  void fadeWithHSV(const CRGB& source, const CRGB& target, uint16_t amount);
  void setColor(const CRGB& target);
  void setBrightness(uint8_t value);
  bool finished() { return _finished; }

  private:
    friend class XBT; /* to access these private members */
    uint8_t _mode = 1;
    uint8_t _speed = 1;
    uint16_t ports = ( 1UL << 15 );
    XBT_led_t *yields[5] = { nullptr };
    uint32_t _timeout = 0; /* timeout for yield */
    uint32_t _current = 0; /* millis() at time of write */
    void _nblendU8TowardU8(uint8_t& cur, const uint8_t target, uint8_t amount);
    CRGB _fadeWithRGB(CRGB& cur, const CRGB& target, uint8_t amount);
    int update();

    CRGB _sourceRGB;
    CRGB _targetRGB;
    CRGB _currentRGB;

    CHSV _sourceHSV;
    CHSV _targetHSV;
    CHSV _currentHSV;

    uint16_t _current_amount;
    uint16_t _amount;
    bool _finished = 0;

    bool fadeToColorEnabled = 0;
    bool useHSVfading = 0;
    bool useRGBfading = 0;

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
