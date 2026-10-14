#pragma once

// WheelOdometry - body velocity of the robot from the 4 Hall sensors.
//
// Every period_ms it takes the 4 wheel speeds, gives them a sign, runs the
// forward kinematics (inverse of MecanumIK::calculate, same r/lx/ly) and
// sends one line to the Raspberry Pi:
//
//     E|vx|vy|w|mask|t\r\n
//       vx, vy - mm/s   (same frame and units as the M3 command)
//       w      - mrad/s (+ = counter-clockwise seen from above)
//       mask   - bit i = 1: wheel i (FL, FR, RL, RR) was measured by its
//                Hall sensor; 0: Hall sees no pulses (too slow), its setpoint
//                is used instead (M3 mode) or 0 (M4 / stopped)
//       t      - HAL_GetTick(), ms
//
// Command from the Pi:  ENC|<ms>  - period (10..1000), ENC|0 - off (default).
// Answer: "ENC: <ms>".
//
// The Hall sensor has no direction, so the sign is the direction the motor is
// driven in (sign of its M3 setpoint; after M4|0|0|0|0 the last M3 direction,
// i.e. the direction the wheel is coasting in).

#include <cstdint>
#include <string_view>
#include "../Motor/Motor.hpp"
#include "../HallSensor/HallSensor.hpp"
#include "../MecanumIK/MecanumIK.hpp"

class WheelOdometry
{
public:
    WheelOdometry(Motor* fl, Motor* fr, Motor* rl, Motor* rr,
                  HallSensor* hfl, HallSensor* hfr, HallSensor* hrl, HallSensor* hrr,
                  const MecanumIK* ik);

    bool handle_command(std::string_view cmd);  // true if cmd was "ENC|<ms>"
    void poll(bool m3_mode);                    // call in every pass of while(1)

    // last result (handy in the debugger)
    float   wheel[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // rad/s, signed, FL FR RL RR
    float   vx = 0.0f, vy = 0.0f, wz = 0.0f;      // m/s, m/s, rad/s
    uint8_t mask = 0;

private:
    static constexpr uint32_t STALL_MS = 250;   // no Hall pulse longer -> not measured

    Motor*           m_[4];
    HallSensor*      h_[4];
    const MecanumIK* ik_;
    uint16_t         period_ms_ = 0;
    uint32_t         last_ms_   = 0;

    void compute(bool m3_mode);
    void send();
};
