#if !defined(_XBT_H_)
#define _XBT_H_
#include "Arduino.h"
#include "circular_buffer.h"


typedef struct XBT_led_t {
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  void addPort(uint8_t port) { ports |= (1UL << port); }
  void solid() { _mode = 1; }
  void fade() { _mode = 2; }
  void strobe() { _mode = 3; }
  void speed(uint8_t value) { _speed = value; }

  private:
    friend class XBT; /* to access these private members */
    uint8_t _mode = 1;
    uint8_t _speed = 1;
    uint16_t ports = ( 1UL << 15 );
} XBT_led_t;

class XBT {
  public:
    XBT(uint32_t _node, FlexCAN_T4_Base* _busWritePtr);
    void write(const XBT_led_t &config);
    void state();
    void scan(bool state = 1);
    void setMAC(uint8_t controller, const char* mac);
    void eraseMAC(uint8_t controller);
    void keepAlive();

  private:
    uint32_t node = 0;
    FlexCAN_T4_Base* _busToWrite = nullptr;
};

#include "XBT.tpp"

#endif
