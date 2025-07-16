#ifndef WALK_IDQP_ARGS_H
#define WALK_IDQP_ARGS_H

namespace walk_idqp {

enum {
  NUM_DISC_ARGS = 0
};

enum {
  ARG_H = 0,
  ARG_VX,
  ARG_VY,
  ARG_VRZ,
  ARG_MU,
  ARG_T_STEP,
  ARG_H_STEP,
  ARG_T_DWELL,
  
  ARG_kp_hip,
  ARG_kp_shoulder,
  ARG_kp_knee,

  ARG_kd_hip,
  ARG_kd_shoulder,
  ARG_kd_knee,

  NUM_CONT_ARGS
};

const int disc_min[NUM_DISC_ARGS] = {};
const int disc_max[NUM_DISC_ARGS] = {};
// const double cont_min[NUM_CONT_ARGS] = {0.1,-0.3,-0.15,-0.5};
// const double cont_max[NUM_CONT_ARGS] = {0.3,0.5,0.15,0.5};
const double cont_min[NUM_CONT_ARGS] = { 0.1, -2.0, -1.0, -2.0 }; //h, vx, vy, vrz
const double cont_max[NUM_CONT_ARGS] = { 0.3,  2.0,  1.0,  2.0 };
// const double cont_min[NUM_CONT_ARGS] = {INFINITY, INFINITY, INFINITY, INFINITY, INFINITY};
// const double cont_max[NUM_CONT_ARGS] = {INFINITY, INFINITY, INFINITY, INFINITY, INFINITY};

}

#endif //WALK_IDQP_ARGS_H
