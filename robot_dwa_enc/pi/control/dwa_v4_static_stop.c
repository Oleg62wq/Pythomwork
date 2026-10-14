/*
 * dwa_v4_static_stop.c - C port of DWA_HumanFollower_v3_StaticTarget_Stop.m
 *
 *   - line-by-line port of the MATLAB function (same constants, same order
 *     of operations, same tie-breaking), checked against Octave;
 *   - candidates are evaluated in parallel with OpenMP (like dwa_v2/v3);
 *   - optional lidar prefilter (build with -DDWA4_PREFILTER=1): points that
 *     the robot cannot reach within the horizon are dropped once before the
 *     search. They cannot add a penalty or a collision, so the command is
 *     the same; only n_obs_used changes.
 *
 * Build on the Pi:
 *   gcc -O3 -mcpu=native -std=c11 -fopenmp -fPIC -shared \
 *       -o libdwa4.so dwa_v4_static_stop.c -lm
 * Do not use -ffast-math: it breaks isnan() and exact equality with MATLAB.
 *
 * u_prev must be the CURRENT body velocity (encoders). The dynamic window,
 * the wheel acceleration check, the smoothness terms and the HOLD braking
 * are all built around it.
 */
#include "dwa4.h"
#include <math.h>
#include <stdlib.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef DWA4_PREFILTER
#define DWA4_PREFILTER 0
#endif

/* ===================== PARAMETERS (as in the .m file) ===================== */
#define DEG (M_PI / 180.0)

/* Mecanum geometry */
static const double r_wheel = 0.075;
static const double L_base  = 0.425;

/* Wheel limits */
static const double wheel_max = 25.51;   /* rad/s   */
static const double wheel_acc = 22.92;   /* rad/s^2 */

/* Planner */
static const double Ts  = 0.09;          /* replanning period */
static const double dt  = 0.05;          /* rollout step      */
static const double T_h = 1.20;          /* horizon           */

/* Desired distance */
static const double d0 = DWA4_D0;

/* Body acceleration limits */
static const double ax_max = 1.719;
static const double ay_max = 1.719;
static const double aw_max = 4.04;

/* Angle thresholds */
#define phi_keep   (20.0 * DEG)   /* comfort zone          */
#define phi_align  (35.0 * DEG)   /* ALIGN starts here      */
#define phi_spin   (55.0 * DEG)   /* hard turn / recovery   */
#define phi_assoc  (60.0 * DEG)   /* field of view limit    */

/* Follower speed limits */
static const double vx_track_base = 0.55;
static const double vx_align_base = 0.18;
static const double vy_min_track  = 0.12;
static const double vy_min_align  = 0.08;
static const double w_track_base  = 1.60;
static const double w_align_base  = 1.80;
static const double w_spin_max    = 1.80;
static const double w_search      = 0.90;

/* Search time after the target is lost */
static const double T_search = 1.20;

/* HOLD: enter at |d-d0| <= 2.5 cm and |phi| <= 0.035 rad,
 *       leave at |d-d0| >  6 cm   or  |phi| >  0.08 rad */
static const double stop_d_enter   = 0.025;
static const double stop_d_exit    = 0.060;
static const double stop_phi_enter = 0.035;
static const double stop_phi_exit  = 0.080;

/* Cost normalisation */
static const double d_ref = 0.06;
static const double y_ref = 0.05;
#define phi_ref (10.0 * DEG)
#define phi_d   (35.0 * DEG)

/* Obstacles */
static const double d_safe     = 0.25;
static const double d_coll     = 0.05;
static const double w_obs      = 35.0;
static const double R_robot_eq = 0.30;   /* equivalent base radius */

/* Regularisation and feedforward */
static const double lam_v      = 1.0;
static const double lam_w      = 0.4;
static const double lam_turn   = 8.0;
static const double lam_y_base = 2.0;
static const double w_vff      = 1.20;
static const double w_wff      = 0.25;

#define NV_MAX 16   /* max grid points per axis (13 in the .m file) */

/* ===================== HELPERS ===================== */
static void body_to_wheel(double vx, double vy, double w, double om[4])
{
    om[0] = (1.0 / r_wheel) * (vx - vy - L_base * w);
    om[1] = (1.0 / r_wheel) * (vx + vy + L_base * w);
    om[2] = (1.0 / r_wheel) * (vx + vy - L_base * w);
    om[3] = (1.0 / r_wheel) * (vx - vy + L_base * w);
}

static inline double robust_cost(double z)
{
    double az = fabs(z);
    return (z * z) / (1.0 + az);
}

static void make_window(double prev, double cap_lo, double cap_hi,
                        double amax, double ts, double *lo_out, double *hi_out)
{
    double lo = fmax(cap_lo, prev - amax * ts);
    double hi = fmin(cap_hi, prev + amax * ts);
    if (lo > hi) {
        if (prev > cap_hi)      { lo = prev - amax * ts; hi = lo; }
        else if (prev < cap_lo) { hi = prev + amax * ts; lo = hi; }
        else                    { lo = prev; hi = prev; }
    }
    *lo_out = lo; *hi_out = hi;
}

static double clamp_to_window(double ref, double prev, double cap_lo,
                              double cap_hi, double amax, double ts)
{
    double lo, hi;
    make_window(prev, cap_lo, cap_hi, amax, ts, &lo, &hi);
    return fmin(fmax(ref, lo), hi);
}

static inline double clamp_scalar(double x, double lo, double hi)
{
    return fmin(fmax(x, lo), hi);
}

static inline double sign_nz(double x)
{
    return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
}

/* MATLAB linspace: a + i*(b-a)/(n-1), the last element is exactly b */
static void linspace(double a, double b, int n, double *y)
{
    if (n == 1) { y[0] = b; return; }
    for (int i = 0; i < n; i++) y[i] = a + i * (b - a) / (n - 1);
    y[n - 1] = b;
}

/* Scale the whole command towards zero (keeps the braking direction)
 * without exceeding body and wheel acceleration limits. */
static void brake_to_zero(const double u_prev[3], double u[3])
{
    double wp[4];
    body_to_wheel(u_prev[0], u_prev[1], u_prev[2], wp);
    const double eps0 = 1e-12;
    double wmax = fmax(fmax(fabs(wp[0]), fabs(wp[1])), fmax(fabs(wp[2]), fabs(wp[3])));
    double alpha = 1.0;
    alpha = fmin(alpha, ax_max * Ts / fmax(fabs(u_prev[0]), eps0));
    alpha = fmin(alpha, ay_max * Ts / fmax(fabs(u_prev[1]), eps0));
    alpha = fmin(alpha, aw_max * Ts / fmax(fabs(u_prev[2]), eps0));
    alpha = fmin(alpha, wheel_acc * Ts / fmax(wmax, eps0));
    alpha = fmax(0.0, fmin(1.0, alpha));
    for (int m = 0; m < 3; m++) u[m] = (1.0 - alpha) * u_prev[m];
    if (fmax(fmax(fabs(u[0]), fabs(u[1])), fabs(u[2])) < 1e-10)
        u[0] = u[1] = u[2] = 0.0;
}

/* ===================== ONE CANDIDATE ===================== */
typedef struct {
    const double *vx_grid, *vy_grid, *w_grid;
    int nvx, nvy, nw;
    double omega_prev[4];
    double x_est, y_est;
    double q_phi, q_d, q_y, q_vis;
    double vx_ref, vy_ref, w_ref_ff, lam_y_eff;
    double vx_prev, vy_prev, w_prev, phi_est;
    int N;
    const dwa4_point_t *obs; int n_obs;
} cand_ctx_t;

/*
 * Cost of candidate number c = body of the triple loop in the .m file.
 * Returns 1 if the candidate is admissible (J written to *J_out), else 0.
 * c = (i*nvy + j)*nw + k has the same order as the MATLAB loops i, j, k,
 * so "smaller number" = "met earlier".
 */
static int evaluate_candidate(int c, const cand_ctx_t *cx, double *J_out)
{
    const int k = c % cx->nw;
    const int j = (c / cx->nw) % cx->nvy;
    const int i = c / (cx->nw * cx->nvy);

    const double vx = cx->vx_grid[i];
    const double vy = cx->vy_grid[j];
    const double w  = cx->w_grid[k];

    /* wheel checks */
    double om[4];
    body_to_wheel(vx, vy, w, om);
    for (int m = 0; m < 4; m++)
        if (fabs(om[m]) > wheel_max) return 0;
    for (int m = 0; m < 4; m++)
        if (fabs((om[m] - cx->omega_prev[m]) / Ts) > wheel_acc) return 0;

    /* local rollout, robot frame fixed at the start of the step */
    double xr = 0.0, yr = 0.0, th = 0.0;
    double xt = cx->x_est, yt = cx->y_est;
    double J = 0.0;
    /* a lidar point farther than this cannot change J (tiny margin keeps the
     * skip exact: sqrt is not computed for these points) */
    const double thr  = R_robot_eq + d_safe + 1e-9;
    const double thr2 = thr * thr;

    for (int n = 0; n < cx->N; n++) {
        /* robot rollout */
        double xdot = vx * cos(th) - vy * sin(th);
        double ydot = vx * sin(th) + vy * cos(th);
        xr += xdot * dt;
        yr += ydot * dt;
        th += w * dt;

        /* static person: moves in the robot frame only because of the robot */
        double xtdot = -vx + w * yt;
        double ytdot = -vy - w * xt;
        xt += xtdot * dt;
        yt += ytdot * dt;

        double dk   = sqrt(xt * xt + yt * yt);
        double phik = atan2(yt, xt);

        /* target went behind the robot */
        if (xt <= 0.0) {
            double z = (-xt) / 0.05;
            J += 300.0 * (z * z) * dt;
        }

        /* main tracking costs */
        J += cx->q_phi * robust_cost(phik / phi_ref) * dt;
        J += cx->q_y   * robust_cost(yt   / y_ref)   * dt;

        /* distance cost is weakened but not switched off */
        double rp = fabs(phik) / phi_d;
        double wd = cx->q_d / (1.0 + rp * rp);
        J += wd * robust_cost((dk - d0) / d_ref) * dt;

        /* soft barrier: leaving the central zone of the camera */
        if (fabs(phik) > phi_keep) {
            double z = (fabs(phik) - phi_keep) / (15.0 * DEG);
            J += cx->q_vis * (z * z) * dt;
        }

        /* never leave the field of view */
        if (fabs(phik) > phi_assoc) return 0;

        /* obstacles: lidar points, robot frame */
        for (int q = 0; q < cx->n_obs; q++) {
            if (isnan(cx->obs[q].x)) continue;
            double ddx = xr - cx->obs[q].x, ddy = yr - cx->obs[q].y;
            double d2  = ddx * ddx + ddy * ddy;
            if (d2 >= thr2) continue;              /* far: no sqrt needed */
            double d_clear = sqrt(d2) - R_robot_eq;

            if (d_clear < d_coll) return 0;        /* hard ban     */
            if (d_clear < d_safe) {                /* soft penalty */
                double pen = (d_safe - d_clear) / d_safe;
                J += w_obs * pen * pen * dt;
            }
        }
    }

    /* feedforward and regularisation */
    double ex = vx - cx->vx_ref, ey = vy - cx->vy_ref, ew = w - cx->w_ref_ff;
    J += w_vff * ((ex * ex) + 1.5 * (ey * ey));
    J += w_wff * (ew * ew);

    double dvx = vx - cx->vx_prev, dvy = vy - cx->vy_prev, dw = w - cx->w_prev;
    J += lam_v * ((dvx * dvx) + 0.5 * (dvy * dvy));
    J += lam_w * (dw * dw);
    J += cx->lam_y_eff * vy * vy;

    /* do not turn the wrong way */
    if (cx->phi_est * w < 0.0) {
        double z = cx->phi_est * w;
        J += lam_turn * (z * z);
    }

    *J_out = J;
    return 1;
}

/* ===================== PUBLIC ===================== */
void dwa4_init(dwa4_state_t *s)
{
    s->initialized = 0;
    s->x_est       = d0;
    s->y_est       = 0.0;
    s->lost_t      = 0.0;
    s->scan_sign   = 1.0;
    s->stop_active = 0;
}

void dwa4_step(dwa4_state_t *s, const double pose[3],
               double d, double phi, double confidence,
               const dwa4_point_t *obs, int n_obs,
               const double u_prev[3], dwa4_out_t *out)
{
    (void)pose;   /* not used, kept for interface compatibility */
    const int N = (int)floor(T_h / dt);   /* = 23 (1.2/0.05 in double is 23.999...) */

    out->u_cmd[0] = out->u_cmd[1] = out->u_cmd[2] = 0.0;
    out->mode_id = 0; out->d_est = 0.0; out->phi_est = 0.0;
    out->best_idx = -1; out->n_candidates = 0; out->n_obs_used = 0;
    out->stop_active = 0;

    const double vx_prev = u_prev[0], vy_prev = u_prev[1], w_prev = u_prev[2];

    /* validity only; freshness is checked by the caller */
    const int meas_valid = (confidence > 0.05) && !isnan(d) && !isnan(phi);

    /* ---------- TARGET: no motion prediction ---------- */
    if (meas_valid) {
        s->x_est = d * cos(phi);
        s->y_est = d * sin(phi);
        s->initialized = 1;
        s->lost_t = 0.0;
        if (fabs(phi) > 0.02) s->scan_sign = sign_nz(phi);
    } else {
        if (!s->initialized) return;          /* all zeros, mode 0 */
        s->lost_t += Ts;                      /* keep last position for SEARCH */
    }

    const double d_est   = sqrt(s->x_est * s->x_est + s->y_est * s->y_est);
    const double phi_est = atan2(s->y_est, s->x_est);
    out->d_est = d_est; out->phi_est = phi_est;

    /* ---------- HOLD with hysteresis ---------- */
    if (!meas_valid) {
        s->stop_active = 0;
    } else if (s->stop_active) {
        if (fabs(d_est - d0) > stop_d_exit || fabs(phi_est) > stop_phi_exit)
            s->stop_active = 0;
    } else {
        if (fabs(d_est - d0) <= stop_d_enter && fabs(phi_est) <= stop_phi_enter)
            s->stop_active = 1;
    }

    if (s->stop_active) {
        out->mode_id = 1;                     /* 1 == TRACK/HOLD */
        out->stop_active = 1;
        brake_to_zero(u_prev, out->u_cmd);
        return;
    }

    /* human motion features are disabled in this experiment */
    const double vy_need = 0.08;
    const int smooth_curve = 0;
    const int sharp_turn   = 0;

    /* ---------- SUPERVISOR ---------- */
    if (!meas_valid) {                                    /* SEARCH */
        out->mode_id = 0;
        double w_ref = (s->lost_t <= T_search && fabs(phi_est) > 5.0 * DEG)
                       ? 1.6 * phi_est : s->scan_sign * w_search;
        out->u_cmd[2] = clamp_to_window(w_ref, w_prev, -w_spin_max, w_spin_max, aw_max, Ts);
        return;
    }

    if (fabs(phi_est) > phi_spin || sharp_turn) {         /* SPIN */
        out->mode_id = 3;
        out->u_cmd[2] = clamp_to_window(2.0 * phi_est, w_prev, -w_spin_max, w_spin_max, aw_max, Ts);
        return;
    }

    double vx_fwd_cap, vx_back_cap, vy_cap, w_cap, q_phi, q_d, q_y, q_vis;
    int Nv_vx, Nv_vy, Nv_w;

    if (fabs(phi_est) > phi_align && !smooth_curve) {     /* ALIGN */
        out->mode_id = 2;
        vx_fwd_cap = vx_align_base; vx_back_cap = 0.05;
        vy_cap = fmax(vy_min_align, fmin(0.35, 0.85 * vy_need));
        w_cap = w_align_base;
        q_phi = 80.0; q_d = 10.0; q_y = 18.0; q_vis = 90.0;
        Nv_vx = 3; Nv_vy = 5; Nv_w = 13;
    } else {                                              /* TRACK */
        out->mode_id = 1;
        vx_fwd_cap = fmin(0.65, fmax(vx_track_base, 0.20));
        vx_back_cap = 0.20;
        vy_cap = fmax(vy_min_track, fmin(0.80, 1.10 * vy_need));
        w_cap = w_track_base;
        if (smooth_curve) {
            q_phi = 22.0; q_d = 24.0; q_y = 12.0; q_vis = 30.0;
            Nv_vx = 7; Nv_vy = 7; Nv_w = 13;
        } else {
            q_phi = 30.0; q_d = 18.0; q_y = 10.0; q_vis = 50.0;
            Nv_vx = 5; Nv_vy = 5; Nv_w = 11;
        }
    }

    /* experiment: side motion banned */
    vy_cap = 0.03;

    /* ---------- FEEDFORWARD ---------- */
    const double vx_ref    = clamp_scalar(1.4 * (s->x_est - d0), -vx_back_cap, vx_fwd_cap);
    const double vy_ref    = clamp_scalar(2.0 * s->y_est,        -vy_cap,      vy_cap);
    const double w_ref_ff  = clamp_scalar(2.0 * phi_est,         -w_cap,       w_cap);
    const double lam_y_eff = lam_y_base;

    /* ---------- DYNAMIC WINDOW around the measured velocity ---------- */
    double vx_min, vx_max, vy_min, vy_max, w_min, w_max;
    make_window(vx_prev, -vx_back_cap, vx_fwd_cap, ax_max, Ts, &vx_min, &vx_max);
    make_window(vy_prev, -vy_cap,      vy_cap,     ay_max, Ts, &vy_min, &vy_max);
    make_window(w_prev,  -w_cap,       w_cap,      aw_max, Ts, &w_min,  &w_max);

    double vx_grid[NV_MAX], vy_grid[NV_MAX], w_grid[NV_MAX];
    linspace(vx_min, vx_max, Nv_vx, vx_grid);
    linspace(vy_min, vy_max, Nv_vy, vy_grid);
    linspace(w_min,  w_max,  Nv_w,  w_grid);

    /* ---------- LIDAR POINTS ---------- */
    const dwa4_point_t *obs_used = obs;
    int n_used = (obs != NULL) ? n_obs : 0;
#if DWA4_PREFILTER
    /* Within the horizon N*dt the robot moves at most |v|max*N*dt (turning
     * changes the direction of the velocity, not its magnitude). A point
     * matters only if it is closer than R_robot_eq + d_safe, so points
     * farther than reach can never give a penalty or a collision. */
    dwa4_point_t *obs_buf = NULL;
    if (n_used > 0) {
        double vxa = fmax(fabs(vx_min), fabs(vx_max));
        double vya = fmax(fabs(vy_min), fabs(vy_max));
        double reach = sqrt(vxa * vxa + vya * vya) * (N * dt)
                       + R_robot_eq + d_safe + 1e-6;
        double reach2 = reach * reach;
        obs_buf = (dwa4_point_t *)malloc(sizeof(dwa4_point_t) * (size_t)n_used);
        if (obs_buf) {
            int m = 0;
            for (int q = 0; q < n_used; q++) {
                if (isnan(obs[q].x)) continue;
                double r2 = obs[q].x * obs[q].x + obs[q].y * obs[q].y;
                if (r2 <= reach2) obs_buf[m++] = obs[q];
            }
            obs_used = obs_buf; n_used = m;
        }
        /* malloc failed: use the full scan, the result is the same */
    }
#endif
    out->n_obs_used = n_used;

    cand_ctx_t cx = {
        .vx_grid = vx_grid, .vy_grid = vy_grid, .w_grid = w_grid,
        .nvx = Nv_vx, .nvy = Nv_vy, .nw = Nv_w,
        .x_est = s->x_est, .y_est = s->y_est,
        .q_phi = q_phi, .q_d = q_d, .q_y = q_y, .q_vis = q_vis,
        .vx_ref = vx_ref, .vy_ref = vy_ref, .w_ref_ff = w_ref_ff,
        .lam_y_eff = lam_y_eff,
        .vx_prev = vx_prev, .vy_prev = vy_prev, .w_prev = w_prev,
        .phi_est = phi_est, .N = N,
        .obs = obs_used, .n_obs = n_used,
    };
    body_to_wheel(vx_prev, vy_prev, w_prev, cx.omega_prev);

    const int n_cand = Nv_vx * Nv_vy * Nv_w;
    out->n_candidates = n_cand;

    /* ---------- CANDIDATE SEARCH (parallel) ---------- */
    double bestJ = 1e12;
    int bestIdx = -1;

    /*
     * Each thread finds the best candidate of its share (myJ, myIdx), then
     * the threads compare their leaders one by one. On equal J the smaller
     * number wins, exactly like the sequential MATLAB loop with "J < bestJ",
     * so the result does not depend on the number of threads.
     * schedule(dynamic): some candidates are rejected at once, others run the
     * whole horizon, so they are handed out in small portions.
     */
    #pragma omp parallel
    {
        double myJ = 1e12;
        int myIdx = -1;

        #pragma omp for schedule(dynamic, 4) nowait
        for (int c = 0; c < n_cand; c++) {
            double J;
            if (evaluate_candidate(c, &cx, &J) && J < myJ) {
                myJ = J; myIdx = c;
            }
        }

        #pragma omp critical(dwa4_best)
        {
            if (myIdx >= 0 && (myJ < bestJ ||
                (myJ == bestJ && (bestIdx < 0 || myIdx < bestIdx)))) {
                bestJ = myJ; bestIdx = myIdx;
            }
        }
    }
#if DWA4_PREFILTER
    free(obs_buf);
#endif

    out->best_idx = bestIdx;

    if (bestIdx >= 0) {
        int k = bestIdx % Nv_w;
        int j = (bestIdx / Nv_w) % Nv_vy;
        int i = bestIdx / (Nv_w * Nv_vy);
        out->u_cmd[0] = vx_grid[i];
        out->u_cmd[1] = vy_grid[j];
        out->u_cmd[2] = w_grid[k];
        return;
    }

    /* ---------- FALLBACK ---------- */
    if (fabs(phi_est) > phi_align) {
        out->u_cmd[0] = 0.0; out->u_cmd[1] = 0.0;
        out->u_cmd[2] = clamp_to_window(1.7 * phi_est, w_prev, -w_cap, w_cap, aw_max, Ts);
    } else {
        out->u_cmd[0] = clamp_to_window(vx_ref,   vx_prev, -vx_back_cap, vx_fwd_cap, ax_max, Ts);
        out->u_cmd[1] = clamp_to_window(vy_ref,   vy_prev, -vy_cap,      vy_cap,     ay_max, Ts);
        out->u_cmd[2] = clamp_to_window(w_ref_ff, w_prev,  -w_cap,       w_cap,      aw_max, Ts);
    }
}
