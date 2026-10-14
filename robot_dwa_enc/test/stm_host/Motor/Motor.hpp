#pragma once
#include "../HallSensor/HallSensor.hpp"
class Motor { public:
  HallSensor* s; float target_rad_s = 0;
  explicit Motor(HallSensor* h) : s(h) {}
  float get_target() const { return target_rad_s; }
  float get_rad_s() const { return s->get_rpm(4) * 0.10471975f; } };
