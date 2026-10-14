#pragma once
#include <cstdint>
class HallSensor { public:
  float rpm = 0; bool stalled = true;
  float get_rpm(uint8_t) const { return rpm; }
  bool is_stalled(uint32_t) const { return stalled; } };
