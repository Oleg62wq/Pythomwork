extern "C" {
#include "usart.h"
}
#include "WheelOdometry/WheelOdometry.hpp"
#include <cstdio>
#include <cmath>
UART_HandleTypeDef huart1; uint32_t g_tick = 1000;
int main() {
  HallSensor h[4]; Motor m[4] = {Motor(&h[0]), Motor(&h[1]), Motor(&h[2]), Motor(&h[3])};
  MecanumIK ik;
  WheelOdometry od(&m[0], &m[1], &m[2], &m[3], &h[0], &h[1], &h[2], &h[3], &ik);
  int fails = 0;
  // 1) commands
  fails += !od.handle_command("ENC|20"); fails += od.handle_command("M3|1|0|0");
  od.handle_command("ENC|x"); od.handle_command("ENC|5");
  printf("answers: %s", huart1.tx); (huart1.n = 0, huart1.tx[0] = 0);
  // 2) FK(IK(u)) == u, all wheels measured, with sign from the setpoint
  const float r = 0.17f, L = 0.45f;
  float tests[][3] = {{0.3f, 0, 0}, {0, 0.2f, 0}, {0, 0, 1.0f}, {0.25f, -0.1f, -0.7f}, {-0.2f, 0.05f, 1.5f}};
  for (auto& u : tests) {
    float w[4] = {(u[0]-u[1]-L*u[2])/r, (u[0]+u[1]+L*u[2])/r, (u[0]+u[1]-L*u[2])/r, (u[0]-u[1]+L*u[2])/r};
    for (int i = 0; i < 4; i++) { m[i].target_rad_s = w[i]; h[i].stalled = false; h[i].rpm = std::fabs(w[i]) / 0.10471975f; }
    g_tick += 25; od.poll(true);
    float e = std::fmax(std::fabs(od.vx-u[0]), std::fmax(std::fabs(od.vy-u[1]), std::fabs(od.wz-u[2])));
    printf("u=(%+.2f %+.2f %+.2f) -> %s", u[0], u[1], u[2], huart1.tx); (huart1.n = 0, huart1.tx[0] = 0);
    fails += e > 1e-4;
  }
  // 3) slow wheels (Hall stalled): setpoint in M3, zero in M4
  for (int i = 0; i < 4; i++) { m[i].target_rad_s = 1.0f; h[i].stalled = true; }
  g_tick += 25; od.poll(true);  printf("stalled M3: %s", huart1.tx); fails += od.mask != 0 || std::fabs(od.vx - 0.17f) > 1e-5; (huart1.n = 0, huart1.tx[0] = 0);
  g_tick += 25; od.poll(false); printf("stalled M4: %s", huart1.tx); fails += std::fabs(od.vx) > 0; (huart1.n = 0, huart1.tx[0] = 0);
  // 4) period respected, ENC|0 stops
  od.handle_command("ENC|0"); (huart1.n = 0, huart1.tx[0] = 0); g_tick += 100; od.poll(true);
  fails += (huart1.n != 0);
  printf(fails ? "FAIL %d\n" : "ALL OK\n", fails); return fails;
}
