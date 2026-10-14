/*
 * dwa4.h - interface of the C port of DWA_HumanFollower_v3_StaticTarget_Stop.m
 *
 *   dwa_v4_static_stop.c   dwa4_step   - port + OpenMP + optional lidar prefilter
 *
 * Differences from the old dwa.h (v1..v3):
 *   - static target: no human velocity estimator, no prediction;
 *   - HOLD state with smooth braking (stop_active);
 *   - u_prev is an INPUT: pass the real body velocity measured by the wheel
 *     encoders (STM telemetry "E|..."), or the previous command if there is none.
 *
 * MATLAB persistent variables live in the explicit struct dwa4_state_t.
 * Call dwa4_init() once before the first step (and after a reset).
 */
#ifndef DWA4_H
#define DWA4_H

#define DWA4_D0 0.60   /* m, desired distance (d0 in the .m file) */

typedef struct {
    int    initialized;
    double x_est, y_est;      /* last measured target position, robot frame */
    double lost_t;            /* time since the last valid measurement, s   */
    double scan_sign;         /* SEARCH turn direction                       */
    int    stop_active;       /* HOLD (hysteretic stop near the target)      */
} dwa4_state_t;

typedef struct { double x, y; } dwa4_point_t;   /* lidar point, robot frame, m */

typedef struct {
    double u_cmd[3];          /* vx, vy, w                                      */
    int    mode_id;           /* 0 SEARCH, 1 TRACK/HOLD, 2 ALIGN, 3 SPIN        */
    double d_est, phi_est;
    int    best_idx;          /* best candidate number, -1 = fallback/no search */
    int    n_candidates;
    int    n_obs_used;        /* lidar points actually used by the search       */
    int    stop_active;       /* 1 = HOLD (braking to zero), mode_id stays 1    */
} dwa4_out_t;

#ifdef __cplusplus
extern "C" {
#endif

void dwa4_init(dwa4_state_t *s);

/*
 * One planning step (Ts = 0.09 s).
 *   pose[3]     - not used (kept for interface compatibility), may be NULL
 *   d, phi      - distance / bearing to the person, robot frame (phi > 0 = left);
 *                 NaN = no measurement
 *   confidence  - <= 0.05 means "no measurement"
 *   obs, n_obs  - lidar points in the robot frame (NaN x = no return)
 *   u_prev[3]   - CURRENT body velocity vx, vy, w (encoders) or previous command
 */
void dwa4_step(dwa4_state_t *s, const double pose[3],
               double d, double phi, double confidence,
               const dwa4_point_t *obs, int n_obs,
               const double u_prev[3], dwa4_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DWA4_H */
