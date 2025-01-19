/*
 * File: sm_vehicle_2axle_heave_roll_types.h
 *
 * Code generated for Simulink model 'sm_vehicle_2axle_heave_roll'.
 *
 * Model version                  : 10.10
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Tue Dec 19 00:29:29 2023
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_sm_vehicle_2axle_heave_roll_types_h_
#define RTW_HEADER_sm_vehicle_2axle_heave_roll_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_U_t_
#define DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_U_t_

typedef struct {
  real_T sWhl;
  real_T trqFL;
  real_T trqFR;
  real_T trqRL;
  real_T trqRR;
} sm_vehicle_2axle_heave_roll_U_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Body_
#define DEFINED_TYPEDEF_FOR_Body_

typedef struct {
  real_T x;
  real_T y;
  real_T z;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T gx;
  real_T gy;
  real_T gz;
  real_T nRoll;
  real_T nPitch;
  real_T aRoll;
  real_T aPitch;
} Body;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireFL_
#define DEFINED_TYPEDEF_FOR_TireFL_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
} TireFL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireFR_
#define DEFINED_TYPEDEF_FOR_TireFR_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
} TireFR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireRL_
#define DEFINED_TYPEDEF_FOR_TireRL_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
} TireRL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireRR_
#define DEFINED_TYPEDEF_FOR_TireRR_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
} TireRR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_World_
#define DEFINED_TYPEDEF_FOR_World_

typedef struct {
  real_T x;
  real_T y;
  real_T z;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T gx;
  real_T gy;
  real_T gz;
  real_T nRoll;
  real_T nPitch;
  real_T nYaw;
  real_T aRoll;
  real_T aPitch;
  real_T aYaw;
  real_T Q[4];
} World;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_Y_t_
#define DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_Y_t_

typedef struct {
  Body Body;
  TireFL TireFL;
  TireFR TireFR;
  TireRL TireRL;
  TireRR TireRR;
  World World;
} sm_vehicle_2axle_heave_roll_Y_t;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_sm_vehicle_2axle_heav_T RT_MODEL_sm_vehicle_2axle_hea_T;

#endif                     /* RTW_HEADER_sm_vehicle_2axle_heave_roll_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
