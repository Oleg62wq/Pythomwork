extern "C" {
#include "usart.h"
}
#include "WheelOdometry.hpp"
#include <charconv>
#include <cmath>
#include <cstdio>

WheelOdometry::WheelOdometry(Motor* fl, Motor* fr, Motor* rl, Motor* rr,
                             HallSensor* hfl, HallSensor* hfr, HallSensor* hrl, HallSensor* hrr,
                             const MecanumIK* ik)
    : m_{fl, fr, rl, rr}, h_{hfl, hfr, hrl, hrr}, ik_(ik) {}

bool WheelOdometry::handle_command(std::string_view cmd)
{
    if (cmd.size() < 4 || cmd.substr(0, 4) != "ENC|") return false;

    int value = -1;
    const char* first = cmd.data() + 4;
    const char* last  = cmd.data() + cmd.size();
    auto [ptr, ec] = std::from_chars(first, last, value);

    char msg[32];
    int  len;
    if (ec != std::errc{} || ptr != last || value < 0 || value > 1000) {
        len = snprintf(msg, sizeof(msg), "Invalid number (err:-2)\r\n");
    } else {
        period_ms_ = (value > 0 && value < 10) ? 10 : static_cast<uint16_t>(value);
        last_ms_   = HAL_GetTick();
        len = snprintf(msg, sizeof(msg), "ENC: %u\r\n", static_cast<unsigned>(period_ms_));
    }
    HAL_UART_Transmit(&huart1, (const uint8_t*)msg, len, 100);
    return true;
}

void WheelOdometry::poll(bool m3_mode)
{
    if (period_ms_ == 0) return;
    uint32_t now = HAL_GetTick();
    if ((now - last_ms_) < period_ms_) return;
    last_ms_ = now;
    compute(m3_mode);
    send();
}

void WheelOdometry::compute(bool m3_mode)
{
    mask = 0;
    for (int i = 0; i < 4; i++) {
        float target = m_[i]->get_target();            // rad/s, signed (last M3 setpoint)
        float sign   = (target > 0.0f) ? 1.0f : (target < 0.0f) ? -1.0f : 0.0f;
        if (!h_[i]->is_stalled(STALL_MS)) {
            wheel[i] = sign * m_[i]->get_rad_s();      // Hall: magnitude only
            mask |= static_cast<uint8_t>(1u << i);
        } else {
            wheel[i] = m3_mode ? target : 0.0f;        // too slow for the Hall sensor
        }
    }

    // Forward kinematics = inverse of MecanumIK::calculate():
    //   w0 = (vx - vy - L*wz)/r   w1 = (vx + vy + L*wz)/r
    //   w2 = (vx + vy - L*wz)/r   w3 = (vx - vy + L*wz)/r,   L = lx + ly
    const float k = ik_->get_r() * 0.25f;
    const float L = ik_->get_lx() + ik_->get_ly();
    vx = k * ( wheel[0] + wheel[1] + wheel[2] + wheel[3]);
    vy = k * (-wheel[0] + wheel[1] + wheel[2] - wheel[3]);
    wz = k * (-wheel[0] + wheel[1] - wheel[2] + wheel[3]) / L;
}

void WheelOdometry::send()
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "E|%ld|%ld|%ld|%u|%lu\r\n",
                       static_cast<long>(lroundf(vx * 1000.0f)),
                       static_cast<long>(lroundf(vy * 1000.0f)),
                       static_cast<long>(lroundf(wz * 1000.0f)),
                       static_cast<unsigned>(mask),
                       static_cast<unsigned long>(HAL_GetTick()));
    HAL_UART_Transmit(&huart1, (const uint8_t*)msg, len, 20);
}
