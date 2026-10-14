# Подключение комплекта RHC_DWA_Encoder_2026-10-14 к трекеру

Новая цепочка: `follow_rhc.py` (камера + Hailo + fusion) → UDP 127.0.0.1:5555 →
`rhc_dwa` (C, DWA каждые 0.09 с, u_prev = скорость из STM) → UART `R1/C1/S1` → STM32 →
телеметрия `O1|...` обратно.

- `install_rhc.txt` — установщик на распу (распаковывает архив, кладёт мост и `follow_rhc.py`).
- `scripts/follow_rhc.py` — вместо `scripts/follow.py`. Сам запускает `rhc_dwa`, UART и DWA не трогает.

Проверено в контейнере: тесты комплекта (`make -C raspberry_pi test`) проходят; `follow_rhc.py`
с настоящим `rhc_dwa`, фальшивой STM на pty и заглушками трекера: `reason=ok`, команды C1 уходят,
скорость из O1 видна в `act=`. На роботе не проверялось.
