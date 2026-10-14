#pragma once
#include <cstdint>
class MecanumIK
{
private:
    const float r = 0.17;
    const float lx = 0.225;
    const float ly = 0.225;
public:
    MecanumIK(float r = 0.17, float lx = 0.225, float ly = 0.225) : r(r), lx(lx), ly(ly) {}
    int16_t wheel_speeds[4] = {0, 0, 0, 0};
    void calculate(float vx, float vy, float omega);
    float get_r()  const { return r; }
    float get_lx() const { return lx; }
    float get_ly() const { return ly; }
};
