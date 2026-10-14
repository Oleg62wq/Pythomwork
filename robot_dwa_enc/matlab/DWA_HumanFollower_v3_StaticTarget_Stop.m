function [u_cmd, mode_id, d_est, phi_est] = DWA_HumanFollower_v3_StaticTarget_Stop(pose, d, phi, confidence, obs, u_prev)
% DWA_HumanFollower_v3_StaticTarget_Stop
% EXPERIMENT: static human target. Human motion prediction is DISABLED.
% Inputs / outputs are identical to DWA_HumanFollower_v3.
% d(0)=1.5 and phi(0)=0.07 are supplied by Simulink/test model, NOT
% hard-coded here: incoming d, phi must change while the robot drives.
% Planner Ts = 0.09 s, rollout dt = 0.05 s, horizon = 1.20 s.
% STOP experiment: added a hysteretic HOLD state with smooth braking.
% NOTE: measurement timestamps / asynchronous sensor updates are NOT
% handled in this experimental version. Legacy SEARCH turns on sensor loss.


% ВЫХОДЫ ПО УМОЛЧАНИЮ 
u_cmd   = [0.0; 0.0; 0.0];

% pose не используется, оставлен для совместимости интерфейса
% Можно не подключать его логически, но размер входа должен оставаться 3x1.

% ПАРАМЕТРЫ 
deg = pi/180;

% Геометрия Mecanum 
r = 0.075;
L = 0.425;

% Ограничения колес 
wheel_max = 25.51;      % рад/с
wheel_acc = 22.92;      % рад/с^2

% Планировщик 
Ts  = 0.09;            % период перепланирования
dt  = 0.05;            % шаг rollout
T_h = 1.20;            % горизонт
N   = floor(T_h / dt);

% Желаемая дистанция 
d0 = 0.60;

% Ограничения ускорений корпуса 
ax_max = 1.719;
ay_max = 1.719;
aw_max = 4.04;

% Пороговые углы 
phi_keep      = 20*deg;   % комфортная зона
phi_align     = 35*deg;   % угол, где начинаем ALIGN
phi_spin      = 55*deg;   % жесткий поворот / recovery
phi_assoc     = 60*deg;   % предел обзора

% Скоростные пределы follower
vx_track_base = 0.55;
vx_align_base = 0.18;

vy_min_track  = 0.12;
vy_min_align  = 0.08;

w_track_base  = 1.60;
w_align_base  = 1.80;
w_spin_max    = 1.80;
w_search      = 0.90;

% Время поиска после потери цели
T_search = 1.20;

% СТАБИЛИЗАЦИЯ ПОСЛЕ ДОСТИЖЕНИЯ ЦЕЛИ
% Вход в остановку: расстояние +/- 2.5 см и угол +/- 0.035 рад.
% Выход: отклонение дистанции > 6 см или угол > 0.08 рад.
% Гистерезис исключает частое переключение из-за небольших колебаний.
stop_d_enter   = 0.025;
stop_d_exit    = 0.060;
stop_phi_enter = 0.035;
stop_phi_exit  = 0.080;

% Движение человека считается нулевым. Оцениватель скорости выключен.

% Нормировки функционала 
d_ref   = 0.06;
y_ref   = 0.05;
phi_ref = 10*deg;
phi_d   = 35*deg;

% Препятствия 
d_safe = 0.25;
d_coll = 0.05;
w_obs  = 35.0;

% Регуляризация и feedforward 
lam_v      = 1.0;
lam_w      = 0.4;
lam_turn   = 8.0;
lam_y_base = 2.0;

w_vff = 1.20;
w_wff = 0.25;

%  ПАМЯТЬ: только последняя измеренная позиция и режим поиска
persistent initialized x_est y_est lost_t scan_sign stop_active

if isempty(initialized)
    initialized = false;
    x_est       = d0;
    y_est       = 0.0;
    lost_t      = 0.0;
    scan_sign   = 1.0;
    stop_active = false;
end

% ВХОДЫ
vx_prev = u_prev(1);
vy_prev = u_prev(2);
w_prev  = u_prev(3);

% Только проверка валидности; проверку свежести добавим позднее.
meas_valid = (confidence > 0.05) && (~isnan(d)) && (~isnan(phi));

% TRACKER ЦЕЛИ БЕЗ ПРЕДСКАЗАНИЯ ЕЕ ДВИЖЕНИЯ
% При наличии измерения берем его напрямую, НЕ оцениваем скорость цели,
% НЕ используем x_pred/y_pred и НЕ экстраполируем между измерениями.
% Позиция нужна в текущей системе координат корпуса робота.
if meas_valid
    x_est = d * cos(phi);
    y_est = d * sin(phi);
    initialized = true;
    lost_t = 0.0;
    if abs(phi) > 0.02
        scan_sign = signNZ(phi);
    end
else
    if ~initialized
        u_cmd   = [0.0; 0.0; 0.0];
        mode_id = 0;
        d_est   = 0.0;
        phi_est = 0.0;
        return;
    end

    % Последнюю позицию лишь сохраняем для SEARCH (без прогноза).
    lost_t = lost_t + Ts;
end

d_est   = sqrt(x_est*x_est + y_est*y_est);
phi_est = atan2(y_est, x_est);

% HOLD: рядом с неподвижной целью нужно остановиться, а не продолжать
% двигаться вдоль окружности d = d0 при phi = 0.
% Важно: при потере измерений старое состояние HOLD не удерживаем;
% дальше применяется исходная логика SEARCH.
if ~meas_valid
    stop_active = false;
elseif stop_active
    if abs(d_est - d0) > stop_d_exit || abs(phi_est) > stop_phi_exit
        stop_active = false;
    end
else
    if abs(d_est - d0) <= stop_d_enter && abs(phi_est) <= stop_phi_enter
        stop_active = true;
    end
end

if stop_active
    % Внешний интерфейс mode_id НЕ меняется: 1 == TRACK/HOLD.
    mode_id = 1;
    % Не обнуляем скорости скачком: соблюдаем пределы ускорения корпуса
    % и максимального изменения угловых скоростей каждого колеса.
    u_cmd = brakeToZero(u_prev, r, L, Ts, ...
                        ax_max, ay_max, aw_max, wheel_acc);
    return;
end

% Признаки движения человека и скорости цели ОТКЛЮЧЕНЫ.
% Никакого vy_t_est, vx_t_est, phi_rate, smooth_curve или sharp_turn.
vy_need      = 0.08;  % исходный минимум при нулевой скорости цели
smooth_curve = false;
sharp_turn   = false;

% SUPERVISOR 
%  SEARCH 
if ~meas_valid
    mode_id = 0;

    if lost_t <= T_search && abs(phi_est) > 5*deg
        w_ref = 1.6 * phi_est;
    else
        w_ref = scan_sign * w_search;
    end

    w_cmd = clampToWindow(w_ref, w_prev, -w_spin_max, w_spin_max, aw_max, Ts);
    u_cmd = [0.0; 0.0; w_cmd];
    return;
end

% SPIN: только большой угол (детектор резкого движения отключен) 
if (abs(phi_est) > phi_spin) || sharp_turn
    mode_id = 3;

    w_ref = 2.0 * phi_est;
    w_cmd = clampToWindow(w_ref, w_prev, -w_spin_max, w_spin_max, aw_max, Ts);
    u_cmd = [0.0; 0.0; w_cmd];
    return;
end

%  ALIGN или TRACK (curve disabled) 
if (abs(phi_est) > phi_align) && (~smooth_curve)
    mode_id = 2;  % ALIGN

    vx_fwd_cap  = vx_align_base;
    vx_back_cap = 0.05;
    vy_cap      = max(vy_min_align, min(0.35, 0.85*vy_need));
    w_cap       = w_align_base;

    q_phi = 80.0;
    q_d   = 10.0;
    q_y   = 18.0;
    q_vis = 90.0;

    Nv_vx = 3;
    Nv_vy = 5;
    Nv_w  = 13;
else
    mode_id = 1;  % TRACK / CURVE_TRACK (curve disabled in this experiment)

    vx_fwd_cap  = min(0.65, max(vx_track_base, 0.20));
    vx_back_cap = 0.20;
    vy_cap      = max(vy_min_track, min(0.80, 1.10*vy_need));
    w_cap       = w_track_base;

    if smooth_curve
        q_phi = 22.0;
        q_d   = 24.0;
        q_y   = 12.0;
        q_vis = 30.0;

        Nv_vx = 7;
        Nv_vy = 7;
        Nv_w  = 13;
    else
        q_phi = 30.0;
        q_d   = 18.0;
        q_y   = 10.0;
        q_vis = 50.0;

        Nv_vx = 5;
        Nv_vy = 5;
        Nv_w  = 11;
    end
end

% Эксперимент: запрет бокового движения
vy_cap = 0.03;
%  FEEDFORWARD-ОПОРЫ 
vx_ref   = clampScalar(1.4*(x_est - d0), -vx_back_cap, vx_fwd_cap);
vy_ref   = clampScalar(2.0*y_est,        -vy_cap,      vy_cap);
w_ref_ff = clampScalar(2.0*phi_est, -w_cap,       w_cap);

lam_y_eff = lam_y_base;

%  DYNAMIC WINDOW 
[vx_min, vx_max] = makeWindow(vx_prev, -vx_back_cap, vx_fwd_cap, ax_max, Ts);
[vy_min, vy_max] = makeWindow(vy_prev, -vy_cap,      vy_cap,     ay_max, Ts);
[w_min,  w_max ] = makeWindow(w_prev,  -w_cap,       w_cap,      aw_max, Ts);

vx_grid = linspace(vx_min, vx_max, Nv_vx);
vy_grid = linspace(vy_min, vy_max, Nv_vy);
w_grid  = linspace(w_min,  w_max,  Nv_w);

omega_prev = BodyToWheel(vx_prev, vy_prev, w_prev, r, L);

%  ПЕРЕБОР КАНДИДАТОВ 
bestJ  = 1e12;
found  = false;

for i = 1:length(vx_grid)
    for j = 1:length(vy_grid)
        for k = 1:length(w_grid)

            vx = vx_grid(i);
            vy = vy_grid(j);
            w  = w_grid(k);

            %  Проверка по колесам 
            omega_cmd = BodyToWheel(vx, vy, w, r, L);

            if any(abs(omega_cmd) > wheel_max)
                continue;
            end

            domega = (omega_cmd - omega_prev) / Ts;
            if any(abs(domega) > wheel_acc)
                continue;
            end

            % ==================================================
            % ЛОКАЛЬНЫЙ ROLLOUT В СК РОБОТА
            % В начале planning step robot frame фиксируется как:
            % xr = 0, yr = 0, th = 0
            % obstacle points obs тоже заданы в этой же frame.
            % ==================================================
            xr = 0.0;
            yr = 0.0;
            th = 0.0;

            % Цель ведем в относительной СК робота
            xt = x_est;
            yt = y_est;

            J = 0.0;
            invalid = false;

            for n = 1:N
                %  rollout робота в локальной planning frame 
                xdot = vx*cos(th) - vy*sin(th);
                ydot = vx*sin(th) + vy*cos(th);

                xr = xr + xdot*dt;
                yr = yr + ydot*dt;
                th = th + w*dt;

                %  неподвижный человек: изменение координат только из-за робота 
                xtdot = -vx + w*yt;
                ytdot = -vy - w*xt;

                xt = xt + xtdot*dt;
                yt = yt + ytdot*dt;

                dk   = sqrt(xt*xt + yt*yt);
                phik = atan2(yt, xt);

                % Если цель ушла назад относительно робота - плохо
                if xt <= 0.0
                    J = J + 300.0 * ((-xt)/0.05)^2 * dt;
                end

                % Основные трекинговые штрафы
                J = J + q_phi * robustCost(phik / phi_ref) * dt;
                J = J + q_y   * robustCost(yt   / y_ref  ) * dt;

                % Дистанция ослабляется, но не выключается
                wd = q_d / (1.0 + (abs(phik)/phi_d)^2);
                J = J + wd * robustCost((dk - d0) / d_ref) * dt;

                % Мягкий барьер на выход из центральной зоны камеры
                if abs(phik) > phi_keep
                    J = J + q_vis * ((abs(phik) - phi_keep) / (15*deg))^2 * dt;
                end

                % За предел обзора не выпускаем
                if abs(phik) > phi_assoc
                    J = J + 1e5;
                    invalid = true;
                    break;
                end

                %  obstacle check В ЛОКАЛЬНОЙ СК РОБОТА 
                %  obstacle check по точкам лидара в локальной СК робота 
                R_robot_eq = 0.30;   % эквивалентный радиус базы
                
                for q = 1:size(obs,1)
                    if ~isnan(obs(q,1))
                        xp = obs(q,1);
                        yp = obs(q,2);
                
                        d_point = sqrt((xr - xp)^2 + (yr - yp)^2);
                        d_clear = d_point - R_robot_eq;
                
                        % жесткий запрет
                        if d_clear < d_coll
                            invalid = true;
                            break;
                        end
                
                        % мягкий штраф
                        if d_clear < d_safe
                            pen = (d_safe - d_clear) / d_safe;
                            J = J + w_obs * pen*pen * dt;
                        end
                    end
                end

                if invalid
                    break;
                end
            end

            if invalid
                continue;
            end

            %  Feedforward и регуляризация 
            J = J + w_vff * ( (vx - vx_ref)^2 + 1.5*(vy - vy_ref)^2 );
            J = J + w_wff * (w - w_ref_ff)^2;

            J = J + lam_v * ((vx - vx_prev)^2 + 0.5*(vy - vy_prev)^2);
            J = J + lam_w * (w - w_prev)^2;
            J = J + lam_y_eff * vy*vy;

            % Не поворачивать в неправильную сторону
            if phi_est * w < 0.0
                J = J + lam_turn * (phi_est * w)^2;
            end

            if J < bestJ
                bestJ = J;
                u_cmd = [vx; vy; w];
                found = true;
            end
        end
    end
end

%  FALLBACK 
if ~found
    if abs(phi_est) > phi_align
        w_ref = 1.7 * phi_est;
        w_cmd = clampToWindow(w_ref, w_prev, -w_cap, w_cap, aw_max, Ts);
        u_cmd = [0.0; 0.0; w_cmd];
    else
        vx_cmd = clampToWindow(vx_ref,   vx_prev, -vx_back_cap, vx_fwd_cap, ax_max, Ts);
        vy_cmd = clampToWindow(vy_ref,   vy_prev, -vy_cap,      vy_cap,     ay_max, Ts);
        w_cmd  = clampToWindow(w_ref_ff, w_prev,  -w_cap,       w_cap,      aw_max, Ts);
        u_cmd  = [vx_cmd; vy_cmd; w_cmd];
    end
end

end

% ==========================================================
function u = brakeToZero(u_prev, r, L, Ts, ax, ay, aw, wheel_acc)
% Плавно масштабируем всю команду к нулю (сохраняем направление
% торможения), не превышая ограничений по ускорениям корпуса и колес.
% Если alpha = 1, можем сразу передать нули без нарушения пределов.
wheel_prev = BodyToWheel(u_prev(1), u_prev(2), u_prev(3), r, L);
eps0 = 1e-12;
alpha = 1.0;
alpha = min(alpha, ax*Ts / max(abs(u_prev(1)), eps0));
alpha = min(alpha, ay*Ts / max(abs(u_prev(2)), eps0));
alpha = min(alpha, aw*Ts / max(abs(u_prev(3)), eps0));
alpha = min(alpha, wheel_acc*Ts / max(max(abs(wheel_prev)), eps0));
alpha = max(0.0, min(1.0, alpha));
u = (1.0 - alpha) * u_prev;
if max(abs(u)) < 1e-10
    u = [0.0; 0.0; 0.0];
end
end

% ==========================================================
function omega = BodyToWheel(vx, vy, w, r, L)
omega1 = (1/r) * (vx - vy - L*w);
omega2 = (1/r) * (vx + vy + L*w);
omega3 = (1/r) * (vx + vy - L*w);
omega4 = (1/r) * (vx - vy + L*w);
omega = [omega1; omega2; omega3; omega4];
end

% ==========================================================
function val = robustCost(z)
az = abs(z);
val = (z*z) / (1.0 + az);
end

% ==========================================================
function [lo, hi] = makeWindow(prev, cap_lo, cap_hi, amax, Ts)
lo = max(cap_lo, prev - amax*Ts);
hi = min(cap_hi, prev + amax*Ts);

if lo > hi
    if prev > cap_hi
        lo = prev - amax*Ts;
        hi = lo;
    elseif prev < cap_lo
        hi = prev + amax*Ts;
        lo = hi;
    else
        lo = prev;
        hi = prev;
    end
end
end

% ==========================================================
function u = clampToWindow(ref, prev, cap_lo, cap_hi, amax, Ts)
[lo, hi] = makeWindow(prev, cap_lo, cap_hi, amax, Ts);
u = min(max(ref, lo), hi);
end

% ==========================================================
function y = clampScalar(x, lo, hi)
y = min(max(x, lo), hi);
end

% ==========================================================
function s = signNZ(x)
if x > 0
    s = 1;
elseif x < 0
    s = -1;
else
    s = 0;
end
end

% ==========================================================
function a = wrapToPiLocal(a)
while a > pi
    a = a - 2*pi;
end
while a < -pi
    a = a + 2*pi;
end
end