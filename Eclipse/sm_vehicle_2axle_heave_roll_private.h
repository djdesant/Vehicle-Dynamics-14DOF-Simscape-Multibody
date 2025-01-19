/*
 * File: sm_vehicle_2axle_heave_roll_private.h
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

#ifndef RTW_HEADER_sm_vehicle_2axle_heave_roll_private_h_
#define RTW_HEADER_sm_vehicle_2axle_heave_roll_private_h_
#include "rtwtypes.h"
#include "sm_vehicle_2axle_heave_roll_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_atan2d_snf(real_T u0, real_T u1);

/* private model entry point functions */
extern void sm_vehicle_2axle_heave_roll_derivatives
  (RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M);

#endif                   /* RTW_HEADER_sm_vehicle_2axle_heave_roll_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
