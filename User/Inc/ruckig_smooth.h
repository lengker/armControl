#ifndef RUCKIG_SMOOTH_H
#define RUCKIG_SMOOTH_H

#ifdef __cplusplus
extern "C" {
#endif

// 3DoF joint-space smooth motion based on Ruckig.
// Units:
// - position: degrees (deg)
// - velocity: degrees per second (deg/s)
// - acceleration: degrees per second^2 (deg/s^2)
// Typical usage:
//   RuckigSmooth_Init(0.02f);
//   RuckigSmooth_Start(q0, qT);
//   while (RuckigSmooth_Step(q_cmd) == 0) { send(q_cmd); osDelay(20); }

void RuckigSmooth_Init(float dt_s);

// Returns 0 on success, negative on error (e.g. target outside joint limits).
int RuckigSmooth_Start(const float q_current_deg[3], const float q_target_deg[3]);

// Returns:
//   0: Working (q_out filled)
//   1: Finished (q_out filled with final state)
//  <0: Error
int RuckigSmooth_Step(float q_out_deg[3]);

#ifdef __cplusplus
}
#endif

#endif // RUCKIG_SMOOTH_H

