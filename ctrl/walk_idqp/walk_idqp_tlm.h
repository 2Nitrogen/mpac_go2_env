#ifndef WALK_IDQP_TLM_H
#define WALK_IDQP_TLM_H

typedef struct {
  int swing_leg[4];
  double swing_feet_err[12];
  double feet_pos[12];
  double feet_vel[12];

} WalkIdqpTelemetry;

typedef struct {
  TelemetryAttribute swing_leg[4];
  TelemetryAttribute swing_feet_err[12];
  TelemetryAttribute feet_pos[12];
  TelemetryAttribute feet_vel[12];

} TelemetryAttributes;

#endif //WALK_IDQP_TLM_H

