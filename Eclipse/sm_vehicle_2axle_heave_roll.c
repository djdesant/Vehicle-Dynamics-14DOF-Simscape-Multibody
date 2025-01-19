/*
 * File: sm_vehicle_2axle_heave_roll.c
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

#include "sm_vehicle_2axle_heave_roll.h"
#include <math.h>
#include <string.h>
#include "sm_vehicle_2axle_heave_roll_private.h"
#include "rtwtypes.h"
#include <stddef.h>
#include "sm_vehicle_2axle_heave_roll_types.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

const sm_vehicle_2axle_heave_roll_Y_t sm_vehicle_2axle_heave_roll_r_0 = { { 0.0,/* x */
    0.0,                               /* y */
    0.0,                               /* z */
    0.0,                               /* vx */
    0.0,                               /* vy */
    0.0,                               /* vz */
    0.0,                               /* gx */
    0.0,                               /* gy */
    0.0,                               /* gz */
    0.0,                               /* nRoll */
    0.0,                               /* nPitch */
    0.0,                               /* aRoll */
    0.0                                /* aPitch */
  },                                   /* Body */

  { 0.0,                               /* w */
    0.0,                               /* Fx */
    0.0,                               /* Fy */
    0.0,                               /* Fz */
    0.0,                               /* Tx */
    0.0,                               /* Ty */
    0.0                                /* Tz */
  },                                   /* TireFL */

  { 0.0,                               /* w */
    0.0,                               /* Fx */
    0.0,                               /* Fy */
    0.0,                               /* Fz */
    0.0,                               /* Tx */
    0.0,                               /* Ty */
    0.0                                /* Tz */
  },                                   /* TireFR */

  { 0.0,                               /* w */
    0.0,                               /* Fx */
    0.0,                               /* Fy */
    0.0,                               /* Fz */
    0.0,                               /* Tx */
    0.0,                               /* Ty */
    0.0                                /* Tz */
  },                                   /* TireRL */

  { 0.0,                               /* w */
    0.0,                               /* Fx */
    0.0,                               /* Fy */
    0.0,                               /* Fz */
    0.0,                               /* Tx */
    0.0,                               /* Ty */
    0.0                                /* Tz */
  },                                   /* TireRR */

  { 0.0,                               /* x */
    0.0,                               /* y */
    0.0,                               /* z */
    0.0,                               /* vx */
    0.0,                               /* vy */
    0.0,                               /* vz */
    0.0,                               /* gx */
    0.0,                               /* gy */
    0.0,                               /* gz */
    0.0,                               /* nRoll */
    0.0,                               /* nPitch */
    0.0,                               /* nYaw */
    0.0,                               /* aRoll */
    0.0,                               /* aPitch */
    0.0,                               /* aYaw */

    { 0.0, 0.0, 0.0, 0.0 }             /* Q */
  }                                    /* World */
};

const sm_vehicle_2axle_heave_roll_U_t sm_vehicle_2axle_heave_roll_rtZ = { 0.0,/* sWhl */
  0.0,                                 /* trqFL */
  0.0,                                 /* trqFR */
  0.0,                                 /* trqRL */
  0.0                                  /* trqRR */
};

/* Projection for root system: '<Root>' */
void sm_vehicle_2axle_heave_roll_projection(RT_MODEL_sm_vehicle_2axle_hea_T *
  const sm_vehicle_2axle_heave_roll_M)
{
  B_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_B =
    sm_vehicle_2axle_heave_roll_M->blockIO;
  DW_sm_vehicle_2axle_heave_rol_T *sm_vehicle_2axle_heave_roll_DW =
    sm_vehicle_2axle_heave_roll_M->dwork;
  X_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_X =
    sm_vehicle_2axle_heave_roll_M->contStates;
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  char *msg;
  real_T tmp_0[120];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[31];
  boolean_T tmp;

  /* Projection for SimscapeExecutionBlock: '<S207>/STATE_1' */
  simulationData = (NeslSimulationData *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData;
  time = sm_vehicle_2axle_heave_roll_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 29;
  simulationData->mData->mContStates.mX =
    &sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_o[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX =
    &sm_vehicle_2axle_heave_roll_DW->STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX =
    &sm_vehicle_2axle_heave_roll_DW->STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep
    (sm_vehicle_2axle_heave_roll_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&sm_vehicle_2axle_heave_roll_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&sm_vehicle_2axle_heave_roll_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
  tmp_0[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
  tmp_0[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
  tmp_0[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
  tmp_0[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
  tmp_0[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
  tmp_0[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
  tmp_0[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
  tmp_0[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
  tmp_0[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
  tmp_0[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
  tmp_0[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
  tmp_0[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
  tmp_0[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
  tmp_0[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
  tmp_0[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
  tmp_1[5] = 20;
  tmp_0[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
  tmp_0[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
  tmp_0[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
  tmp_0[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
  tmp_1[6] = 24;
  tmp_0[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
  tmp_0[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
  tmp_0[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
  tmp_0[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
  tmp_1[7] = 28;
  tmp_0[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
  tmp_0[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
  tmp_0[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
  tmp_0[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
  tmp_1[8] = 32;
  tmp_0[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
  tmp_0[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
  tmp_0[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
  tmp_0[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
  tmp_1[9] = 36;
  tmp_0[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
  tmp_0[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
  tmp_0[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
  tmp_0[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
  tmp_1[10] = 40;
  tmp_0[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
  tmp_0[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
  tmp_0[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
  tmp_0[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
  tmp_1[11] = 44;
  tmp_0[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
  tmp_0[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
  tmp_0[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
  tmp_0[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
  tmp_1[12] = 48;
  tmp_0[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
  tmp_0[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
  tmp_0[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
  tmp_0[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
  tmp_1[13] = 52;
  tmp_0[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
  tmp_0[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
  tmp_0[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
  tmp_0[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
  tmp_1[14] = 56;
  tmp_0[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
  tmp_0[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
  tmp_0[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
  tmp_0[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
  tmp_1[15] = 60;
  tmp_0[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
  tmp_0[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
  tmp_0[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
  tmp_0[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
  tmp_1[16] = 64;
  tmp_0[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
  tmp_0[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
  tmp_0[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
  tmp_0[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
  tmp_1[17] = 68;
  tmp_0[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
  tmp_0[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
  tmp_0[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
  tmp_0[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
  tmp_1[18] = 72;
  tmp_0[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
  tmp_0[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
  tmp_0[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
  tmp_0[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
  tmp_1[19] = 76;
  tmp_0[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
  tmp_0[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
  tmp_0[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
  tmp_0[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
  tmp_1[20] = 80;
  tmp_0[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
  tmp_0[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
  tmp_0[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
  tmp_0[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
  tmp_1[21] = 84;
  tmp_0[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
  tmp_0[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
  tmp_0[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
  tmp_0[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
  tmp_1[22] = 88;
  tmp_0[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
  tmp_0[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
  tmp_0[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
  tmp_0[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
  tmp_1[23] = 92;
  tmp_0[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
  tmp_0[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
  tmp_0[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
  tmp_0[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
  tmp_1[24] = 96;
  tmp_0[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
  tmp_0[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
  tmp_0[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
  tmp_0[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
  tmp_1[25] = 100;
  tmp_0[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
  tmp_0[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
  tmp_0[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
  tmp_0[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
  tmp_1[26] = 104;
  tmp_0[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
  tmp_0[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
  tmp_0[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
  tmp_0[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
  tmp_1[27] = 108;
  tmp_0[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
  tmp_0[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
  tmp_0[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
  tmp_0[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
  tmp_1[28] = 112;
  tmp_0[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
  tmp_0[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
  tmp_0[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
  tmp_0[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
  tmp_1[29] = 116;
  tmp_0[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
  tmp_0[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
  tmp_0[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
  tmp_0[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
  tmp_1[30] = 120;
  simulationData->mData->mInputValues.mN = 120;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 31;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  diagnosticManager = (NeuDiagnosticManager *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator, NESL_SIM_PROJECTION,
    simulationData, diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_2axle_heave_roll_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg);
    }
  }

  /* End of Projection for SimscapeExecutionBlock: '<S207>/STATE_1' */
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si ,
  RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M,
  ExtY_sm_vehicle_2axle_heave_r_T *sm_vehicle_2axle_heave_roll_Y)
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 35;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  sm_vehicle_2axle_heave_roll_derivatives(sm_vehicle_2axle_heave_roll_M);
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  sm_vehicle_2axle_heave_roll_step(sm_vehicle_2axle_heave_roll_M,
    sm_vehicle_2axle_heave_roll_Y);
  sm_vehicle_2axle_heave_roll_projection(sm_vehicle_2axle_heave_roll_M);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  int32_T tmp;
  int32_T tmp_0;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void sm_vehicle_2axle_heave_roll_step(RT_MODEL_sm_vehicle_2axle_hea_T *const
  sm_vehicle_2axle_heave_roll_M, ExtY_sm_vehicle_2axle_heave_r_T
  *sm_vehicle_2axle_heave_roll_Y)
{
  B_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_B =
    sm_vehicle_2axle_heave_roll_M->blockIO;
  DW_sm_vehicle_2axle_heave_rol_T *sm_vehicle_2axle_heave_roll_DW =
    sm_vehicle_2axle_heave_roll_M->dwork;
  X_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_X =
    sm_vehicle_2axle_heave_roll_M->contStates;
  if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                          ((sm_vehicle_2axle_heave_roll_M->Timing.clockTick0+1)*
      sm_vehicle_2axle_heave_roll_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
    sm_vehicle_2axle_heave_roll_M->Timing.t[0] = rtsiGetT
      (&sm_vehicle_2axle_heave_roll_M->solverInfo);
  }

  {
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    real_T tmp_5[149];
    real_T tmp_7[149];
    real_T tmp_1[120];
    real_T rtb_OUTPUT_1_0[65];
    real_T rtb_OUTPUT_1_1[6];
    real_T rtb_Atan1;
    real_T rtb_TransferFcn;
    real_T time;
    real_T time_0;
    real_T time_1;
    real_T time_2;
    real_T time_3;
    real_T time_4;
    real_T *rtb_OUTPUT_1_0_0;
    real_T *rtb_OUTPUT_1_0_1;
    real_T *rtb_OUTPUT_1_0_2;
    real_T *rtb_OUTPUT_1_0_3;
    int32_T tmp_3;
    int_T tmp_6[32];
    int_T tmp_8[32];
    int_T tmp_2[31];
    boolean_T tmp;
    boolean_T tmp_0;
    boolean_T tmp_4;

    /* Step: '<S4>/Step Steer input' incorporates:
     *  SimscapeExecutionBlock: '<S207>/STATE_1'
     */
    rtb_TransferFcn = sm_vehicle_2axle_heave_roll_M->Timing.t[0];
    if (rtb_TransferFcn < 3.0) {
      rtb_Atan1 = 0.0;
    } else {
      rtb_Atan1 = 2.7925268031909272;
    }

    /* Product: '<S133>/Divide' incorporates:
     *  Constant: '<S133>/Constant'
     *  Step: '<S4>/Step Steer input'
     */
    rtb_Atan1 /= 16.0;

    /* Fcn: '<S133>/Ackerman left' incorporates:
     *  Fcn: '<S133>/Ackerman right'
     */
    rtb_Atan1 = cos(rtb_Atan1 + 2.2204460492503131e-16) / sin(rtb_Atan1 +
      2.2204460492503131e-16);

    /* Fcn: '<S133>/Ackerman left' */
    sm_vehicle_2axle_heave_roll_B->Ackermanleft = atan(1.0 / (rtb_Atan1 -
      0.26666666666666666));

    /* SimscapeInputBlock: '<S207>/INPUT_6_1_1' */
    if (sm_vehicle_2axle_heave_roll_DW->INPUT_6_1_1_FirstOutput == 0.0) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_6_1_1_FirstOutput = 1.0;
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[0] =
        sm_vehicle_2axle_heave_roll_B->Ackermanleft;
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[1] = 0.0;
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0] =
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[0];
    sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1] =
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2] =
      ((sm_vehicle_2axle_heave_roll_B->Ackermanleft -
        sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[0]) *
       181.81818181818184 - 2.0 *
       sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[1]) *
      181.81818181818184;
    sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S207>/INPUT_6_1_1' */

    /* Fcn: '<S133>/Ackerman right' */
    sm_vehicle_2axle_heave_roll_B->Ackermanright = atan(1.0 / (rtb_Atan1 +
      0.26666666666666666));

    /* SimscapeInputBlock: '<S207>/INPUT_5_1_1' */
    if (sm_vehicle_2axle_heave_roll_DW->INPUT_5_1_1_FirstOutput == 0.0) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_5_1_1_FirstOutput = 1.0;
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[0] =
        sm_vehicle_2axle_heave_roll_B->Ackermanright;
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[1] = 0.0;
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0] =
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[0];
    sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1] =
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2] =
      ((sm_vehicle_2axle_heave_roll_B->Ackermanright -
        sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[0]) *
       181.81818181818184 - 2.0 *
       sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[1]) *
      181.81818181818184;
    sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S207>/INPUT_5_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_7_1_1' incorporates:
     *  Constant: '<S119>/Constant1'
     *  Constant: '<S140>/Constant'
     *  Constant: '<S140>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_7_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_7_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_7_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_7_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_7_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_7_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_8_1_1' incorporates:
     *  Constant: '<S119>/Constant5'
     *  Constant: '<S140>/Constant'
     *  Constant: '<S140>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_8_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_8_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_8_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_8_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_8_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_8_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_9_1_1' incorporates:
     *  Constant: '<S119>/Constant3'
     *  Constant: '<S140>/Constant'
     *  Constant: '<S140>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_9_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_9_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_9_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_9_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_9_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_9_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_12_1_1' incorporates:
     *  Constant: '<S119>/Constant'
     *  Constant: '<S140>/Constant2'
     *  Constant: '<S140>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_12_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_12_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_12_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_12_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_12_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_12_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_10_1_1' incorporates:
     *  Constant: '<S119>/Constant2'
     *  Constant: '<S140>/Constant2'
     *  Constant: '<S140>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_10_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_10_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_10_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_10_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_10_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_10_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_11_1_1' incorporates:
     *  Constant: '<S119>/Constant4'
     *  Constant: '<S140>/Constant2'
     *  Constant: '<S140>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_11_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_11_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_11_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_11_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_11_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_11_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_13_1_1' incorporates:
     *  Constant: '<S121>/Constant1'
     *  Constant: '<S157>/Constant'
     *  Constant: '<S157>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_13_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_13_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_13_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_13_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_13_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_13_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_14_1_1' incorporates:
     *  Constant: '<S121>/Constant5'
     *  Constant: '<S157>/Constant'
     *  Constant: '<S157>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_14_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_14_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_14_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_14_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_14_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_14_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_15_1_1' incorporates:
     *  Constant: '<S121>/Constant3'
     *  Constant: '<S157>/Constant'
     *  Constant: '<S157>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_15_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_15_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_15_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_15_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_15_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_15_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_18_1_1' incorporates:
     *  Constant: '<S121>/Constant'
     *  Constant: '<S157>/Constant2'
     *  Constant: '<S157>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_18_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_18_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_18_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_18_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_18_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_18_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_16_1_1' incorporates:
     *  Constant: '<S121>/Constant2'
     *  Constant: '<S157>/Constant2'
     *  Constant: '<S157>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_16_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_16_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_16_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_16_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_16_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_16_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_17_1_1' incorporates:
     *  Constant: '<S121>/Constant4'
     *  Constant: '<S157>/Constant2'
     *  Constant: '<S157>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_17_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_17_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_17_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_17_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_17_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_17_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_19_1_1' incorporates:
     *  Constant: '<S123>/Constant1'
     *  Constant: '<S174>/Constant'
     *  Constant: '<S174>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_19_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_19_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_19_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_19_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_19_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_19_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_20_1_1' incorporates:
     *  Constant: '<S123>/Constant5'
     *  Constant: '<S174>/Constant'
     *  Constant: '<S174>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_20_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_20_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_20_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_20_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_20_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_20_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_21_1_1' incorporates:
     *  Constant: '<S123>/Constant3'
     *  Constant: '<S174>/Constant'
     *  Constant: '<S174>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_21_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_21_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_21_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_21_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_21_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_21_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_24_1_1' incorporates:
     *  Constant: '<S123>/Constant'
     *  Constant: '<S174>/Constant2'
     *  Constant: '<S174>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_24_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_24_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_24_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_24_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_24_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_24_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_22_1_1' incorporates:
     *  Constant: '<S123>/Constant2'
     *  Constant: '<S174>/Constant2'
     *  Constant: '<S174>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_22_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_22_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_22_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_22_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_22_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_22_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_23_1_1' incorporates:
     *  Constant: '<S123>/Constant4'
     *  Constant: '<S174>/Constant2'
     *  Constant: '<S174>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_23_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_23_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_23_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_23_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_23_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_23_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_25_1_1' incorporates:
     *  Constant: '<S125>/Constant1'
     *  Constant: '<S191>/Constant'
     *  Constant: '<S191>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_25_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_25_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_25_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_25_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_25_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_25_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_26_1_1' incorporates:
     *  Constant: '<S125>/Constant5'
     *  Constant: '<S191>/Constant'
     *  Constant: '<S191>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_26_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_26_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_26_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_26_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_26_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_26_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_27_1_1' incorporates:
     *  Constant: '<S125>/Constant3'
     *  Constant: '<S191>/Constant'
     *  Constant: '<S191>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_27_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_27_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_27_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_27_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_27_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_27_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_30_1_1' incorporates:
     *  Constant: '<S125>/Constant'
     *  Constant: '<S191>/Constant2'
     *  Constant: '<S191>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_30_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_30_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_30_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_30_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_30_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_30_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_28_1_1' incorporates:
     *  Constant: '<S125>/Constant2'
     *  Constant: '<S191>/Constant2'
     *  Constant: '<S191>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_28_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_28_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_28_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_28_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_28_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_28_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_29_1_1' incorporates:
     *  Constant: '<S125>/Constant4'
     *  Constant: '<S191>/Constant2'
     *  Constant: '<S191>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_29_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_DW->INPUT_29_1_1_Discrete[1] ==
          sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_29_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_29_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_29_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_29_1_1' */

    /* SimscapeExecutionBlock: '<S207>/STATE_1' incorporates:
     *  SimscapeExecutionBlock: '<S207>/OUTPUT_1_0'
     *  SimscapeExecutionBlock: '<S207>/OUTPUT_1_1'
     */
    simulationData = (NeslSimulationData *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData;
    rtb_Atan1 = sm_vehicle_2axle_heave_roll_M->Timing.t[0];
    time = rtb_Atan1;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 29;
    simulationData->mData->mContStates.mX =
      &sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_o[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_2axle_heave_roll_DW->STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX =
      &sm_vehicle_2axle_heave_roll_DW->STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    tmp = rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M);
    simulationData->mData->mIsMajorTimeStep = tmp;
    tmp_0 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_0;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp_0 = rtsiIsSolverComputingJacobian
      (&sm_vehicle_2axle_heave_roll_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp_0;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    tmp_0 = rtsiIsModeUpdateTimeStep(&sm_vehicle_2axle_heave_roll_M->solverInfo);
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_2[0] = 0;
    tmp_1[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
    tmp_1[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
    tmp_1[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
    tmp_1[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
    tmp_2[1] = 4;
    tmp_1[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
    tmp_1[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
    tmp_1[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
    tmp_1[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
    tmp_2[2] = 8;
    tmp_1[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
    tmp_1[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
    tmp_1[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
    tmp_1[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
    tmp_2[3] = 12;
    tmp_1[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
    tmp_1[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
    tmp_1[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
    tmp_1[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
    tmp_2[4] = 16;
    tmp_1[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
    tmp_1[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
    tmp_1[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
    tmp_1[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
    tmp_2[5] = 20;
    tmp_1[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
    tmp_1[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
    tmp_1[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
    tmp_1[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
    tmp_2[6] = 24;
    tmp_1[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
    tmp_1[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
    tmp_1[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
    tmp_1[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
    tmp_2[7] = 28;
    tmp_1[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
    tmp_1[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
    tmp_1[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
    tmp_1[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
    tmp_2[8] = 32;
    tmp_1[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
    tmp_1[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
    tmp_1[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
    tmp_1[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
    tmp_2[9] = 36;
    tmp_1[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
    tmp_1[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
    tmp_1[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
    tmp_1[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
    tmp_2[10] = 40;
    tmp_1[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
    tmp_1[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
    tmp_1[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
    tmp_1[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
    tmp_2[11] = 44;
    tmp_1[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
    tmp_1[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
    tmp_1[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
    tmp_1[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
    tmp_2[12] = 48;
    tmp_1[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
    tmp_1[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
    tmp_1[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
    tmp_1[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
    tmp_2[13] = 52;
    tmp_1[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
    tmp_1[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
    tmp_1[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
    tmp_1[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
    tmp_2[14] = 56;
    tmp_1[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
    tmp_1[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
    tmp_1[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
    tmp_1[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
    tmp_2[15] = 60;
    tmp_1[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
    tmp_1[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
    tmp_1[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
    tmp_1[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
    tmp_2[16] = 64;
    tmp_1[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
    tmp_1[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
    tmp_1[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
    tmp_1[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
    tmp_2[17] = 68;
    tmp_1[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
    tmp_1[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
    tmp_1[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
    tmp_1[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
    tmp_2[18] = 72;
    tmp_1[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
    tmp_1[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
    tmp_1[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
    tmp_1[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
    tmp_2[19] = 76;
    tmp_1[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
    tmp_1[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
    tmp_1[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
    tmp_1[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
    tmp_2[20] = 80;
    tmp_1[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
    tmp_1[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
    tmp_1[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
    tmp_1[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
    tmp_2[21] = 84;
    tmp_1[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
    tmp_1[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
    tmp_1[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
    tmp_1[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
    tmp_2[22] = 88;
    tmp_1[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
    tmp_1[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
    tmp_1[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
    tmp_1[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
    tmp_2[23] = 92;
    tmp_1[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
    tmp_1[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
    tmp_1[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
    tmp_1[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
    tmp_2[24] = 96;
    tmp_1[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
    tmp_1[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
    tmp_1[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
    tmp_1[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
    tmp_2[25] = 100;
    tmp_1[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
    tmp_1[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
    tmp_1[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
    tmp_1[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
    tmp_2[26] = 104;
    tmp_1[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
    tmp_1[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
    tmp_1[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
    tmp_1[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
    tmp_2[27] = 108;
    tmp_1[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
    tmp_1[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
    tmp_1[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
    tmp_1[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
    tmp_2[28] = 112;
    tmp_1[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
    tmp_1[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
    tmp_1[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
    tmp_1[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
    tmp_2[29] = 116;
    tmp_1[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
    tmp_1[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
    tmp_1[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
    tmp_1[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
    tmp_2[30] = 120;
    simulationData->mData->mInputValues.mN = 120;
    simulationData->mData->mInputValues.mX = &tmp_1[0];
    simulationData->mData->mInputOffsets.mN = 31;
    simulationData->mData->mInputOffsets.mX = &tmp_2[0];
    simulationData->mData->mOutputs.mN = 29;
    simulationData->mData->mOutputs.mX = &sm_vehicle_2axle_heave_roll_B->
      STATE_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    time_0 = rtb_TransferFcn;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_0;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = ne_simulator_method((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator, NESL_SIM_OUTPUTS,
      simulationData, diagnosticManager);
    if (tmp_3 != 0) {
      tmp_4 = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp_4) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg);
      }
    }

    /* SimscapeExecutionBlock: '<S207>/OUTPUT_1_0' */
    simulationData = (NeslSimulationData *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_SimData;
    time_1 = rtb_Atan1;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_1;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX =
      &sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Modes;
    tmp_4 = false;
    simulationData->mData->mFoundZcEvents = tmp_4;
    simulationData->mData->mIsMajorTimeStep = tmp;
    tmp_4 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_4;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_6[0] = 0;
    tmp_5[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
    tmp_5[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
    tmp_5[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
    tmp_5[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
    tmp_6[1] = 4;
    tmp_5[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
    tmp_5[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
    tmp_5[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
    tmp_5[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
    tmp_6[2] = 8;
    tmp_5[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
    tmp_5[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
    tmp_5[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
    tmp_5[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
    tmp_6[3] = 12;
    tmp_5[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
    tmp_5[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
    tmp_5[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
    tmp_5[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
    tmp_6[4] = 16;
    tmp_5[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
    tmp_5[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
    tmp_5[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
    tmp_5[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
    tmp_6[5] = 20;
    tmp_5[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
    tmp_5[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
    tmp_5[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
    tmp_5[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
    tmp_6[6] = 24;
    tmp_5[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
    tmp_5[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
    tmp_5[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
    tmp_5[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
    tmp_6[7] = 28;
    tmp_5[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
    tmp_5[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
    tmp_5[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
    tmp_5[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
    tmp_6[8] = 32;
    tmp_5[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
    tmp_5[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
    tmp_5[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
    tmp_5[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
    tmp_6[9] = 36;
    tmp_5[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
    tmp_5[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
    tmp_5[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
    tmp_5[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
    tmp_6[10] = 40;
    tmp_5[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
    tmp_5[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
    tmp_5[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
    tmp_5[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
    tmp_6[11] = 44;
    tmp_5[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
    tmp_5[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
    tmp_5[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
    tmp_5[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
    tmp_6[12] = 48;
    tmp_5[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
    tmp_5[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
    tmp_5[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
    tmp_5[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
    tmp_6[13] = 52;
    tmp_5[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
    tmp_5[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
    tmp_5[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
    tmp_5[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
    tmp_6[14] = 56;
    tmp_5[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
    tmp_5[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
    tmp_5[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
    tmp_5[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
    tmp_6[15] = 60;
    tmp_5[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
    tmp_5[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
    tmp_5[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
    tmp_5[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
    tmp_6[16] = 64;
    tmp_5[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
    tmp_5[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
    tmp_5[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
    tmp_5[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
    tmp_6[17] = 68;
    tmp_5[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
    tmp_5[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
    tmp_5[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
    tmp_5[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
    tmp_6[18] = 72;
    tmp_5[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
    tmp_5[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
    tmp_5[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
    tmp_5[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
    tmp_6[19] = 76;
    tmp_5[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
    tmp_5[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
    tmp_5[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
    tmp_5[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
    tmp_6[20] = 80;
    tmp_5[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
    tmp_5[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
    tmp_5[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
    tmp_5[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
    tmp_6[21] = 84;
    tmp_5[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
    tmp_5[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
    tmp_5[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
    tmp_5[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
    tmp_6[22] = 88;
    tmp_5[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
    tmp_5[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
    tmp_5[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
    tmp_5[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
    tmp_6[23] = 92;
    tmp_5[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
    tmp_5[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
    tmp_5[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
    tmp_5[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
    tmp_6[24] = 96;
    tmp_5[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
    tmp_5[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
    tmp_5[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
    tmp_5[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
    tmp_6[25] = 100;
    tmp_5[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
    tmp_5[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
    tmp_5[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
    tmp_5[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
    tmp_6[26] = 104;
    tmp_5[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
    tmp_5[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
    tmp_5[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
    tmp_5[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
    tmp_6[27] = 108;
    tmp_5[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
    tmp_5[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
    tmp_5[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
    tmp_5[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
    tmp_6[28] = 112;
    tmp_5[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
    tmp_5[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
    tmp_5[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
    tmp_5[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
    tmp_6[29] = 116;
    tmp_5[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
    tmp_5[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
    tmp_5[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
    tmp_5[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
    tmp_6[30] = 120;
    memcpy(&tmp_5[120], &sm_vehicle_2axle_heave_roll_B->STATE_1[0], 29U * sizeof
           (real_T));
    tmp_6[31] = 149;
    simulationData->mData->mInputValues.mN = 149;
    simulationData->mData->mInputValues.mX = &tmp_5[0];
    simulationData->mData->mInputOffsets.mN = 32;
    simulationData->mData->mInputOffsets.mX = &tmp_6[0];
    simulationData->mData->mOutputs.mN = 65;
    simulationData->mData->mOutputs.mX = &rtb_OUTPUT_1_0[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    time_2 = rtb_TransferFcn;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_2;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = ne_simulator_method((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS,
      simulationData, diagnosticManager);
    if (tmp_3 != 0) {
      tmp_4 = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp_4) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg_0);
      }
    }

    /* SimscapeInputBlock: '<S207>/INPUT_4_1_1' incorporates:
     *  Constant: '<S4>/Constant'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_4_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0] ==
          sm_vehicle_2axle_heave_roll_DW->INPUT_4_1_1_Discrete[1]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_4_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_4_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_4_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_4_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_1_1_1' incorporates:
     *  Constant: '<S4>/Constant1'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_1_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0] ==
          sm_vehicle_2axle_heave_roll_DW->INPUT_1_1_1_Discrete[1]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_1_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_1_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_1_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_1_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_3_1_1' incorporates:
     *  Constant: '<S4>/Constant2'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_3_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0] ==
          sm_vehicle_2axle_heave_roll_DW->INPUT_3_1_1_Discrete[1]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_3_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_3_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_3_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_3_1_1' */

    /* SimscapeInputBlock: '<S207>/INPUT_2_1_1' incorporates:
     *  Constant: '<S4>/Constant3'
     */
    sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1] = 0.0;
    sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
      sm_vehicle_2axle_heave_roll_DW->INPUT_2_1_1_Discrete[0] =
        !(sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0] ==
          sm_vehicle_2axle_heave_roll_DW->INPUT_2_1_1_Discrete[1]);
      sm_vehicle_2axle_heave_roll_DW->INPUT_2_1_1_Discrete[1] =
        sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
    }

    sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_2_1_1_Discrete[1];
    sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3] =
      sm_vehicle_2axle_heave_roll_DW->INPUT_2_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S207>/INPUT_2_1_1' */

    /* SimscapeExecutionBlock: '<S207>/OUTPUT_1_1' */
    simulationData = (NeslSimulationData *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_SimData;
    time_3 = rtb_Atan1;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_3;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX =
      &sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Modes;
    tmp_4 = false;
    simulationData->mData->mFoundZcEvents = tmp_4;
    simulationData->mData->mIsMajorTimeStep = tmp;
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_8[0] = 0;
    tmp_7[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
    tmp_7[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
    tmp_7[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
    tmp_7[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
    tmp_8[1] = 4;
    tmp_7[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
    tmp_7[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
    tmp_7[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
    tmp_7[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
    tmp_8[2] = 8;
    tmp_7[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
    tmp_7[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
    tmp_7[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
    tmp_7[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
    tmp_8[3] = 12;
    tmp_7[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
    tmp_7[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
    tmp_7[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
    tmp_7[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
    tmp_8[4] = 16;
    tmp_7[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
    tmp_7[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
    tmp_7[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
    tmp_7[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
    tmp_8[5] = 20;
    tmp_7[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
    tmp_7[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
    tmp_7[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
    tmp_7[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
    tmp_8[6] = 24;
    tmp_7[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
    tmp_7[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
    tmp_7[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
    tmp_7[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
    tmp_8[7] = 28;
    tmp_7[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
    tmp_7[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
    tmp_7[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
    tmp_7[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
    tmp_8[8] = 32;
    tmp_7[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
    tmp_7[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
    tmp_7[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
    tmp_7[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
    tmp_8[9] = 36;
    tmp_7[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
    tmp_7[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
    tmp_7[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
    tmp_7[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
    tmp_8[10] = 40;
    tmp_7[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
    tmp_7[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
    tmp_7[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
    tmp_7[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
    tmp_8[11] = 44;
    tmp_7[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
    tmp_7[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
    tmp_7[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
    tmp_7[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
    tmp_8[12] = 48;
    tmp_7[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
    tmp_7[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
    tmp_7[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
    tmp_7[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
    tmp_8[13] = 52;
    tmp_7[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
    tmp_7[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
    tmp_7[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
    tmp_7[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
    tmp_8[14] = 56;
    tmp_7[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
    tmp_7[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
    tmp_7[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
    tmp_7[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
    tmp_8[15] = 60;
    tmp_7[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
    tmp_7[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
    tmp_7[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
    tmp_7[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
    tmp_8[16] = 64;
    tmp_7[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
    tmp_7[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
    tmp_7[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
    tmp_7[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
    tmp_8[17] = 68;
    tmp_7[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
    tmp_7[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
    tmp_7[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
    tmp_7[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
    tmp_8[18] = 72;
    tmp_7[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
    tmp_7[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
    tmp_7[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
    tmp_7[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
    tmp_8[19] = 76;
    tmp_7[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
    tmp_7[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
    tmp_7[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
    tmp_7[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
    tmp_8[20] = 80;
    tmp_7[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
    tmp_7[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
    tmp_7[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
    tmp_7[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
    tmp_8[21] = 84;
    tmp_7[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
    tmp_7[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
    tmp_7[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
    tmp_7[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
    tmp_8[22] = 88;
    tmp_7[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
    tmp_7[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
    tmp_7[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
    tmp_7[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
    tmp_8[23] = 92;
    tmp_7[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
    tmp_7[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
    tmp_7[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
    tmp_7[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
    tmp_8[24] = 96;
    tmp_7[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
    tmp_7[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
    tmp_7[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
    tmp_7[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
    tmp_8[25] = 100;
    tmp_7[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
    tmp_7[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
    tmp_7[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
    tmp_7[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
    tmp_8[26] = 104;
    tmp_7[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
    tmp_7[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
    tmp_7[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
    tmp_7[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
    tmp_8[27] = 108;
    tmp_7[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
    tmp_7[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
    tmp_7[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
    tmp_7[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
    tmp_8[28] = 112;
    tmp_7[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
    tmp_7[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
    tmp_7[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
    tmp_7[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
    tmp_8[29] = 116;
    tmp_7[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
    tmp_7[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
    tmp_7[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
    tmp_7[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
    tmp_8[30] = 120;
    memcpy(&tmp_7[120], &sm_vehicle_2axle_heave_roll_B->STATE_1[0], 29U * sizeof
           (real_T));
    tmp_8[31] = 149;
    simulationData->mData->mInputValues.mN = 149;
    simulationData->mData->mInputValues.mX = &tmp_7[0];
    simulationData->mData->mInputOffsets.mN = 32;
    simulationData->mData->mInputOffsets.mX = &tmp_8[0];
    simulationData->mData->mOutputs.mN = 6;
    simulationData->mData->mOutputs.mX = &rtb_OUTPUT_1_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    time_4 = rtb_TransferFcn;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_4;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = ne_simulator_method((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Simulator, NESL_SIM_OUTPUTS,
      simulationData, diagnosticManager);
    if (tmp_3 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg_1);
      }
    }

    /* Selector: '<S40>/Select y axis' */
    rtb_OUTPUT_1_0_0 = &rtb_OUTPUT_1_0[41];

    /* Selector: '<S40>/Select x axis' */
    rtb_OUTPUT_1_0_1 = &rtb_OUTPUT_1_0[41];

    /* Selector: '<S67>/R(3, 2:3)' */
    rtb_OUTPUT_1_0_2 = &rtb_OUTPUT_1_0[56];

    /* Selector: '<S67>/R([2 1], 1)' */
    rtb_OUTPUT_1_0_3 = &rtb_OUTPUT_1_0[56];

    /* Selector: '<S67>/R(3, 1)' */
    rtb_TransferFcn = rtb_OUTPUT_1_0[58];

    /* Outport: '<Root>/y' incorporates:
     *  BusCreator generated from: '<Root>/y'
     */
    sm_vehicle_2axle_heave_roll_Y->y.World.x = rtb_OUTPUT_1_0[0];
    sm_vehicle_2axle_heave_roll_Y->y.World.y = rtb_OUTPUT_1_0[2];
    sm_vehicle_2axle_heave_roll_Y->y.World.z = rtb_OUTPUT_1_0[4];
    sm_vehicle_2axle_heave_roll_Y->y.World.vx = rtb_OUTPUT_1_0[1];
    sm_vehicle_2axle_heave_roll_Y->y.World.vy = rtb_OUTPUT_1_0[3];
    sm_vehicle_2axle_heave_roll_Y->y.World.vz = rtb_OUTPUT_1_0[5];
    sm_vehicle_2axle_heave_roll_Y->y.World.gx = rtb_OUTPUT_1_1[0];
    sm_vehicle_2axle_heave_roll_Y->y.World.gy = rtb_OUTPUT_1_1[1];
    sm_vehicle_2axle_heave_roll_Y->y.World.gz = rtb_OUTPUT_1_1[2];
    sm_vehicle_2axle_heave_roll_Y->y.World.nRoll = rtb_OUTPUT_1_0[10];
    sm_vehicle_2axle_heave_roll_Y->y.World.nPitch = rtb_OUTPUT_1_0[11];
    sm_vehicle_2axle_heave_roll_Y->y.World.nYaw = rtb_OUTPUT_1_0[12];
    sm_vehicle_2axle_heave_roll_Y->y.World.Q[0] = rtb_OUTPUT_1_0[6];
    sm_vehicle_2axle_heave_roll_Y->y.World.Q[1] = rtb_OUTPUT_1_0[7];
    sm_vehicle_2axle_heave_roll_Y->y.World.Q[2] = rtb_OUTPUT_1_0[8];
    sm_vehicle_2axle_heave_roll_Y->y.World.Q[3] = rtb_OUTPUT_1_0[9];

    /* Fcn: '<S52>/magnitude' incorporates:
     *  Selector: '<S40>/Select y axis'
     */
    rtb_Atan1 = rtb_OUTPUT_1_0_0[3] * rtb_OUTPUT_1_0_0[3] + rtb_OUTPUT_1_0_0[4] *
      rtb_OUTPUT_1_0_0[4];
    if (rtb_Atan1 < 0.0) {
      rtb_Atan1 = -sqrt(-rtb_Atan1);
    } else {
      rtb_Atan1 = sqrt(rtb_Atan1);
    }

    /* Trigonometry: '<S52>/Trigonometric Function1' incorporates:
     *  Fcn: '<S52>/magnitude'
     *  Selector: '<S40>/Select y axis'
     */
    sm_vehicle_2axle_heave_roll_B->TrigonometricFunction1 = rt_atan2d_snf
      (rtb_OUTPUT_1_0_0[5], rtb_Atan1);

    /* Fcn: '<S51>/magnitude' incorporates:
     *  Selector: '<S40>/Select x axis'
     */
    rtb_Atan1 = (rtb_OUTPUT_1_0_1[0] * rtb_OUTPUT_1_0_1[0] + rtb_OUTPUT_1_0_1[1]
                 * rtb_OUTPUT_1_0_1[1]) + rtb_OUTPUT_1_0_1[2] *
      rtb_OUTPUT_1_0_1[2];
    if (rtb_Atan1 < 0.0) {
      rtb_Atan1 = -sqrt(-rtb_Atan1);
    } else {
      rtb_Atan1 = sqrt(rtb_Atan1);
    }

    /* Product: '<S51>/Divide1' incorporates:
     *  Fcn: '<S51>/magnitude'
     *  Product: '<S51>/Divide'
     *  Selector: '<S40>/Select x axis'
     *  Trigonometry: '<S51>/Cos'
     */
    rtb_Atan1 = rtb_OUTPUT_1_0_1[2] / cos
      (sm_vehicle_2axle_heave_roll_B->TrigonometricFunction1) / rtb_Atan1;

    /* Trigonometry: '<S51>/Asin' */
    if (rtb_Atan1 > 1.0) {
      rtb_Atan1 = 1.0;
    } else if (rtb_Atan1 < -1.0) {
      rtb_Atan1 = -1.0;
    }

    /* Gain: '<S40>/Flip sign for -x axis ' incorporates:
     *  Trigonometry: '<S51>/Asin'
     */
    sm_vehicle_2axle_heave_roll_B->Flipsignforxaxis = -asin(rtb_Atan1);

    /* Outport: '<Root>/y' incorporates:
     *  BusCreator generated from: '<Root>/y'
     *  Selector: '<S67>/R(3, 2:3)'
     *  TransferFcn: '<S40>/Transfer Fcn'
     *  TransferFcn: '<S40>/Transfer Fcn1'
     *  Trigonometry: '<S67>/Atan2'
     */
    sm_vehicle_2axle_heave_roll_Y->y.Body.x = rtb_OUTPUT_1_0[50];
    sm_vehicle_2axle_heave_roll_Y->y.Body.y = rtb_OUTPUT_1_0[51];
    sm_vehicle_2axle_heave_roll_Y->y.Body.z = rtb_OUTPUT_1_0[52];
    sm_vehicle_2axle_heave_roll_Y->y.Body.vx = rtb_OUTPUT_1_0[53];
    sm_vehicle_2axle_heave_roll_Y->y.Body.vy = rtb_OUTPUT_1_0[54];
    sm_vehicle_2axle_heave_roll_Y->y.Body.vz = rtb_OUTPUT_1_0[55];
    sm_vehicle_2axle_heave_roll_Y->y.Body.gx = rtb_OUTPUT_1_1[3];
    sm_vehicle_2axle_heave_roll_Y->y.Body.gy = rtb_OUTPUT_1_1[4];
    sm_vehicle_2axle_heave_roll_Y->y.Body.gz = rtb_OUTPUT_1_1[5];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.w = rtb_OUTPUT_1_0[13];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Fx = rtb_OUTPUT_1_0[17];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Fy = rtb_OUTPUT_1_0[18];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Fz = rtb_OUTPUT_1_0[19];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Tx = rtb_OUTPUT_1_0[20];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Ty = rtb_OUTPUT_1_0[21];
    sm_vehicle_2axle_heave_roll_Y->y.TireFL.Tz = rtb_OUTPUT_1_0[22];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.w = rtb_OUTPUT_1_0[14];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Fx = rtb_OUTPUT_1_0[23];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Fy = rtb_OUTPUT_1_0[24];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Fz = rtb_OUTPUT_1_0[25];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Tx = rtb_OUTPUT_1_0[26];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Ty = rtb_OUTPUT_1_0[27];
    sm_vehicle_2axle_heave_roll_Y->y.TireFR.Tz = rtb_OUTPUT_1_0[28];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.w = rtb_OUTPUT_1_0[15];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Fx = rtb_OUTPUT_1_0[29];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Fy = rtb_OUTPUT_1_0[30];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Fz = rtb_OUTPUT_1_0[31];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Tx = rtb_OUTPUT_1_0[32];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Ty = rtb_OUTPUT_1_0[33];
    sm_vehicle_2axle_heave_roll_Y->y.TireRL.Tz = rtb_OUTPUT_1_0[34];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.w = rtb_OUTPUT_1_0[16];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Fx = rtb_OUTPUT_1_0[35];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Fy = rtb_OUTPUT_1_0[36];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Fz = rtb_OUTPUT_1_0[37];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Tx = rtb_OUTPUT_1_0[38];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Ty = rtb_OUTPUT_1_0[39];
    sm_vehicle_2axle_heave_roll_Y->y.TireRR.Tz = rtb_OUTPUT_1_0[40];
    sm_vehicle_2axle_heave_roll_Y->y.Body.nRoll = -10000.0 *
      sm_vehicle_2axle_heave_roll_X->TransferFcn_CSTATE + 100.0 *
      sm_vehicle_2axle_heave_roll_B->TrigonometricFunction1;
    sm_vehicle_2axle_heave_roll_Y->y.Body.nPitch = -10000.0 *
      sm_vehicle_2axle_heave_roll_X->TransferFcn1_CSTATE + 100.0 *
      sm_vehicle_2axle_heave_roll_B->Flipsignforxaxis;
    sm_vehicle_2axle_heave_roll_Y->y.Body.aRoll =
      sm_vehicle_2axle_heave_roll_B->TrigonometricFunction1;
    sm_vehicle_2axle_heave_roll_Y->y.Body.aPitch =
      sm_vehicle_2axle_heave_roll_B->Flipsignforxaxis;
    sm_vehicle_2axle_heave_roll_Y->y.World.aRoll = rt_atan2d_snf
      (rtb_OUTPUT_1_0_2[5], rtb_OUTPUT_1_0_2[8]);

    /* Trigonometry: '<S67>/Asin' incorporates:
     *  Gain: '<S67>/Multiply2'
     */
    if (-rtb_TransferFcn > 1.0) {
      rtb_Atan1 = 1.0;
    } else if (-rtb_TransferFcn < -1.0) {
      rtb_Atan1 = -1.0;
    } else {
      rtb_Atan1 = -rtb_TransferFcn;
    }

    /* Outport: '<Root>/y' incorporates:
     *  Selector: '<S67>/R([2 1], 1)'
     *  Trigonometry: '<S67>/Asin'
     *  Trigonometry: '<S67>/Atan1'
     */
    sm_vehicle_2axle_heave_roll_Y->y.World.aPitch = asin(rtb_Atan1);
    sm_vehicle_2axle_heave_roll_Y->y.World.aYaw = rt_atan2d_snf
      (rtb_OUTPUT_1_0_3[1], rtb_OUTPUT_1_0_3[0]);
  }

  if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    char *msg;
    real_T tmp_0[120];
    real_T time;
    int32_T tmp_2;
    int_T tmp_1[31];
    boolean_T tmp;

    /* Update for SimscapeExecutionBlock: '<S207>/STATE_1' */
    simulationData = (NeslSimulationData *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData;
    time = sm_vehicle_2axle_heave_roll_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 29;
    simulationData->mData->mContStates.mX =
      &sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_o[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_2axle_heave_roll_DW->STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX =
      &sm_vehicle_2axle_heave_roll_DW->STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep
      (sm_vehicle_2axle_heave_roll_M);
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian
      (&sm_vehicle_2axle_heave_roll_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
      (&sm_vehicle_2axle_heave_roll_M->solverInfo);
    tmp_1[0] = 0;
    tmp_0[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
    tmp_0[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
    tmp_0[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
    tmp_0[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
    tmp_1[1] = 4;
    tmp_0[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
    tmp_0[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
    tmp_0[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
    tmp_0[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
    tmp_1[2] = 8;
    tmp_0[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
    tmp_0[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
    tmp_0[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
    tmp_0[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
    tmp_1[3] = 12;
    tmp_0[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
    tmp_0[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
    tmp_0[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
    tmp_0[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
    tmp_1[4] = 16;
    tmp_0[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
    tmp_0[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
    tmp_0[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
    tmp_0[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
    tmp_1[5] = 20;
    tmp_0[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
    tmp_0[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
    tmp_0[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
    tmp_0[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
    tmp_1[6] = 24;
    tmp_0[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
    tmp_0[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
    tmp_0[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
    tmp_0[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
    tmp_1[7] = 28;
    tmp_0[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
    tmp_0[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
    tmp_0[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
    tmp_0[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
    tmp_1[8] = 32;
    tmp_0[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
    tmp_0[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
    tmp_0[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
    tmp_0[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
    tmp_1[9] = 36;
    tmp_0[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
    tmp_0[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
    tmp_0[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
    tmp_0[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
    tmp_1[10] = 40;
    tmp_0[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
    tmp_0[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
    tmp_0[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
    tmp_0[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
    tmp_1[11] = 44;
    tmp_0[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
    tmp_0[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
    tmp_0[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
    tmp_0[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
    tmp_1[12] = 48;
    tmp_0[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
    tmp_0[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
    tmp_0[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
    tmp_0[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
    tmp_1[13] = 52;
    tmp_0[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
    tmp_0[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
    tmp_0[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
    tmp_0[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
    tmp_1[14] = 56;
    tmp_0[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
    tmp_0[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
    tmp_0[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
    tmp_0[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
    tmp_1[15] = 60;
    tmp_0[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
    tmp_0[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
    tmp_0[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
    tmp_0[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
    tmp_1[16] = 64;
    tmp_0[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
    tmp_0[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
    tmp_0[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
    tmp_0[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
    tmp_1[17] = 68;
    tmp_0[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
    tmp_0[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
    tmp_0[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
    tmp_0[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
    tmp_1[18] = 72;
    tmp_0[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
    tmp_0[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
    tmp_0[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
    tmp_0[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
    tmp_1[19] = 76;
    tmp_0[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
    tmp_0[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
    tmp_0[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
    tmp_0[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
    tmp_1[20] = 80;
    tmp_0[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
    tmp_0[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
    tmp_0[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
    tmp_0[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
    tmp_1[21] = 84;
    tmp_0[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
    tmp_0[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
    tmp_0[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
    tmp_0[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
    tmp_1[22] = 88;
    tmp_0[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
    tmp_0[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
    tmp_0[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
    tmp_0[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
    tmp_1[23] = 92;
    tmp_0[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
    tmp_0[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
    tmp_0[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
    tmp_0[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
    tmp_1[24] = 96;
    tmp_0[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
    tmp_0[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
    tmp_0[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
    tmp_0[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
    tmp_1[25] = 100;
    tmp_0[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
    tmp_0[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
    tmp_0[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
    tmp_0[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
    tmp_1[26] = 104;
    tmp_0[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
    tmp_0[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
    tmp_0[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
    tmp_0[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
    tmp_1[27] = 108;
    tmp_0[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
    tmp_0[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
    tmp_0[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
    tmp_0[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
    tmp_1[28] = 112;
    tmp_0[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
    tmp_0[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
    tmp_0[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
    tmp_0[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
    tmp_1[29] = 116;
    tmp_0[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
    tmp_0[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
    tmp_0[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
    tmp_0[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
    tmp_1[30] = 120;
    simulationData->mData->mInputValues.mN = 120;
    simulationData->mData->mInputValues.mX = &tmp_0[0];
    simulationData->mData->mInputOffsets.mN = 31;
    simulationData->mData->mInputOffsets.mX = &tmp_1[0];
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_2 = ne_simulator_method((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator, NESL_SIM_UPDATE,
      simulationData, diagnosticManager);
    if (tmp_2 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg);
      }
    }

    /* End of Update for SimscapeExecutionBlock: '<S207>/STATE_1' */
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(sm_vehicle_2axle_heave_roll_M)) {
    rt_ertODEUpdateContinuousStates(&sm_vehicle_2axle_heave_roll_M->solverInfo,
      sm_vehicle_2axle_heave_roll_M, sm_vehicle_2axle_heave_roll_Y);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++sm_vehicle_2axle_heave_roll_M->Timing.clockTick0;
    sm_vehicle_2axle_heave_roll_M->Timing.t[0] = rtsiGetSolverStopTime
      (&sm_vehicle_2axle_heave_roll_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      sm_vehicle_2axle_heave_roll_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void sm_vehicle_2axle_heave_roll_derivatives(RT_MODEL_sm_vehicle_2axle_hea_T *
  const sm_vehicle_2axle_heave_roll_M)
{
  B_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_B =
    sm_vehicle_2axle_heave_roll_M->blockIO;
  DW_sm_vehicle_2axle_heave_rol_T *sm_vehicle_2axle_heave_roll_DW =
    sm_vehicle_2axle_heave_roll_M->dwork;
  X_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_X =
    sm_vehicle_2axle_heave_roll_M->contStates;
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  XDot_sm_vehicle_2axle_heave_r_T *_rtXdot;
  char *msg;
  real_T tmp_0[120];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[31];
  boolean_T tmp;
  _rtXdot = ((XDot_sm_vehicle_2axle_heave_r_T *)
             sm_vehicle_2axle_heave_roll_M->derivs);

  /* Derivatives for SimscapeInputBlock: '<S207>/INPUT_6_1_1' */
  _rtXdot->sm_vehicle_2axle_heave_rollVehi[0] =
    sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[1];
  _rtXdot->sm_vehicle_2axle_heave_rollVehi[1] =
    ((sm_vehicle_2axle_heave_roll_B->Ackermanleft -
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[0]) *
     181.81818181818184 - 2.0 *
     sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVehi[1]) *
    181.81818181818184;

  /* Derivatives for SimscapeInputBlock: '<S207>/INPUT_5_1_1' */
  _rtXdot->sm_vehicle_2axle_heave_rollVe_d[0] =
    sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[1];
  _rtXdot->sm_vehicle_2axle_heave_rollVe_d[1] =
    ((sm_vehicle_2axle_heave_roll_B->Ackermanright -
      sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[0]) *
     181.81818181818184 - 2.0 *
     sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_d[1]) *
    181.81818181818184;

  /* Derivatives for SimscapeExecutionBlock: '<S207>/STATE_1' */
  simulationData = (NeslSimulationData *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData;
  time = sm_vehicle_2axle_heave_roll_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 29;
  simulationData->mData->mContStates.mX =
    &sm_vehicle_2axle_heave_roll_X->sm_vehicle_2axle_heave_rollVe_o[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX =
    &sm_vehicle_2axle_heave_roll_DW->STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX =
    &sm_vehicle_2axle_heave_roll_DW->STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep
    (sm_vehicle_2axle_heave_roll_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&sm_vehicle_2axle_heave_roll_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&sm_vehicle_2axle_heave_roll_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[0];
  tmp_0[1] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[1];
  tmp_0[2] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[2];
  tmp_0[3] = sm_vehicle_2axle_heave_roll_B->INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[0];
  tmp_0[5] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[1];
  tmp_0[6] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[2];
  tmp_0[7] = sm_vehicle_2axle_heave_roll_B->INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[0];
  tmp_0[9] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[1];
  tmp_0[10] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[2];
  tmp_0[11] = sm_vehicle_2axle_heave_roll_B->INPUT_3_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[0];
  tmp_0[13] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[1];
  tmp_0[14] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[2];
  tmp_0[15] = sm_vehicle_2axle_heave_roll_B->INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[0];
  tmp_0[17] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[1];
  tmp_0[18] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[2];
  tmp_0[19] = sm_vehicle_2axle_heave_roll_B->INPUT_6_1_1[3];
  tmp_1[5] = 20;
  tmp_0[20] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[0];
  tmp_0[21] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[1];
  tmp_0[22] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[2];
  tmp_0[23] = sm_vehicle_2axle_heave_roll_B->INPUT_5_1_1[3];
  tmp_1[6] = 24;
  tmp_0[24] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[0];
  tmp_0[25] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[1];
  tmp_0[26] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[2];
  tmp_0[27] = sm_vehicle_2axle_heave_roll_B->INPUT_7_1_1[3];
  tmp_1[7] = 28;
  tmp_0[28] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[0];
  tmp_0[29] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[1];
  tmp_0[30] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[2];
  tmp_0[31] = sm_vehicle_2axle_heave_roll_B->INPUT_8_1_1[3];
  tmp_1[8] = 32;
  tmp_0[32] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[0];
  tmp_0[33] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[1];
  tmp_0[34] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[2];
  tmp_0[35] = sm_vehicle_2axle_heave_roll_B->INPUT_9_1_1[3];
  tmp_1[9] = 36;
  tmp_0[36] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[0];
  tmp_0[37] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[1];
  tmp_0[38] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[2];
  tmp_0[39] = sm_vehicle_2axle_heave_roll_B->INPUT_12_1_1[3];
  tmp_1[10] = 40;
  tmp_0[40] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[0];
  tmp_0[41] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[1];
  tmp_0[42] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[2];
  tmp_0[43] = sm_vehicle_2axle_heave_roll_B->INPUT_10_1_1[3];
  tmp_1[11] = 44;
  tmp_0[44] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[0];
  tmp_0[45] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[1];
  tmp_0[46] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[2];
  tmp_0[47] = sm_vehicle_2axle_heave_roll_B->INPUT_11_1_1[3];
  tmp_1[12] = 48;
  tmp_0[48] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[0];
  tmp_0[49] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[1];
  tmp_0[50] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[2];
  tmp_0[51] = sm_vehicle_2axle_heave_roll_B->INPUT_13_1_1[3];
  tmp_1[13] = 52;
  tmp_0[52] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[0];
  tmp_0[53] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[1];
  tmp_0[54] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[2];
  tmp_0[55] = sm_vehicle_2axle_heave_roll_B->INPUT_14_1_1[3];
  tmp_1[14] = 56;
  tmp_0[56] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[0];
  tmp_0[57] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[1];
  tmp_0[58] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[2];
  tmp_0[59] = sm_vehicle_2axle_heave_roll_B->INPUT_15_1_1[3];
  tmp_1[15] = 60;
  tmp_0[60] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[0];
  tmp_0[61] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[1];
  tmp_0[62] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[2];
  tmp_0[63] = sm_vehicle_2axle_heave_roll_B->INPUT_18_1_1[3];
  tmp_1[16] = 64;
  tmp_0[64] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[0];
  tmp_0[65] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[1];
  tmp_0[66] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[2];
  tmp_0[67] = sm_vehicle_2axle_heave_roll_B->INPUT_16_1_1[3];
  tmp_1[17] = 68;
  tmp_0[68] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[0];
  tmp_0[69] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[1];
  tmp_0[70] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[2];
  tmp_0[71] = sm_vehicle_2axle_heave_roll_B->INPUT_17_1_1[3];
  tmp_1[18] = 72;
  tmp_0[72] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[0];
  tmp_0[73] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[1];
  tmp_0[74] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[2];
  tmp_0[75] = sm_vehicle_2axle_heave_roll_B->INPUT_19_1_1[3];
  tmp_1[19] = 76;
  tmp_0[76] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[0];
  tmp_0[77] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[1];
  tmp_0[78] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[2];
  tmp_0[79] = sm_vehicle_2axle_heave_roll_B->INPUT_20_1_1[3];
  tmp_1[20] = 80;
  tmp_0[80] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[0];
  tmp_0[81] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[1];
  tmp_0[82] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[2];
  tmp_0[83] = sm_vehicle_2axle_heave_roll_B->INPUT_21_1_1[3];
  tmp_1[21] = 84;
  tmp_0[84] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[0];
  tmp_0[85] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[1];
  tmp_0[86] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[2];
  tmp_0[87] = sm_vehicle_2axle_heave_roll_B->INPUT_24_1_1[3];
  tmp_1[22] = 88;
  tmp_0[88] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[0];
  tmp_0[89] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[1];
  tmp_0[90] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[2];
  tmp_0[91] = sm_vehicle_2axle_heave_roll_B->INPUT_22_1_1[3];
  tmp_1[23] = 92;
  tmp_0[92] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[0];
  tmp_0[93] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[1];
  tmp_0[94] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[2];
  tmp_0[95] = sm_vehicle_2axle_heave_roll_B->INPUT_23_1_1[3];
  tmp_1[24] = 96;
  tmp_0[96] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[0];
  tmp_0[97] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[1];
  tmp_0[98] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[2];
  tmp_0[99] = sm_vehicle_2axle_heave_roll_B->INPUT_25_1_1[3];
  tmp_1[25] = 100;
  tmp_0[100] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[0];
  tmp_0[101] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[1];
  tmp_0[102] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[2];
  tmp_0[103] = sm_vehicle_2axle_heave_roll_B->INPUT_26_1_1[3];
  tmp_1[26] = 104;
  tmp_0[104] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[0];
  tmp_0[105] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[1];
  tmp_0[106] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[2];
  tmp_0[107] = sm_vehicle_2axle_heave_roll_B->INPUT_27_1_1[3];
  tmp_1[27] = 108;
  tmp_0[108] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[0];
  tmp_0[109] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[1];
  tmp_0[110] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[2];
  tmp_0[111] = sm_vehicle_2axle_heave_roll_B->INPUT_30_1_1[3];
  tmp_1[28] = 112;
  tmp_0[112] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[0];
  tmp_0[113] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[1];
  tmp_0[114] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[2];
  tmp_0[115] = sm_vehicle_2axle_heave_roll_B->INPUT_28_1_1[3];
  tmp_1[29] = 116;
  tmp_0[116] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[0];
  tmp_0[117] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[1];
  tmp_0[118] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[2];
  tmp_0[119] = sm_vehicle_2axle_heave_roll_B->INPUT_29_1_1[3];
  tmp_1[30] = 120;
  simulationData->mData->mInputValues.mN = 120;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 31;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  simulationData->mData->mDx.mN = 29;
  simulationData->mData->mDx.mX = &_rtXdot->sm_vehicle_2axle_heave_rollVe_o[0];
  diagnosticManager = (NeuDiagnosticManager *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator, NESL_SIM_DERIVATIVES,
    simulationData, diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_2axle_heave_roll_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg);
    }
  }

  /* End of Derivatives for SimscapeExecutionBlock: '<S207>/STATE_1' */

  /* Derivatives for TransferFcn: '<S40>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += -100.0 *
    sm_vehicle_2axle_heave_roll_X->TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE +=
    sm_vehicle_2axle_heave_roll_B->TrigonometricFunction1;

  /* Derivatives for TransferFcn: '<S40>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += -100.0 *
    sm_vehicle_2axle_heave_roll_X->TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE +=
    sm_vehicle_2axle_heave_roll_B->Flipsignforxaxis;
}

/* Model initialize function */
void sm_vehicle_2axle_heave_roll_initialize(RT_MODEL_sm_vehicle_2axle_hea_T *
  const sm_vehicle_2axle_heave_roll_M, ExtU_sm_vehicle_2axle_heave_r_T
  *sm_vehicle_2axle_heave_roll_U, ExtY_sm_vehicle_2axle_heave_r_T
  *sm_vehicle_2axle_heave_roll_Y)
{

  DW_sm_vehicle_2axle_heave_rol_T *sm_vehicle_2axle_heave_roll_DW =
    sm_vehicle_2axle_heave_roll_M->dwork;
  X_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_X =
    sm_vehicle_2axle_heave_roll_M->contStates;
  B_sm_vehicle_2axle_heave_roll_T *sm_vehicle_2axle_heave_roll_B =
    sm_vehicle_2axle_heave_roll_M->blockIO;
  XDis_sm_vehicle_2axle_heave_r_T *sm_vehicle_2axle_heave_rol_XDis =
    ((XDis_sm_vehicle_2axle_heave_r_T *)
     sm_vehicle_2axle_heave_roll_M->contStateDisabled);

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                          &sm_vehicle_2axle_heave_roll_M->Timing.simTimeStep);
    rtsiSetTPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo, &rtmGetTPtr
                (sm_vehicle_2axle_heave_roll_M));
    rtsiSetStepSizePtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                       &sm_vehicle_2axle_heave_roll_M->Timing.stepSize0);
    rtsiSetdXPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                 &sm_vehicle_2axle_heave_roll_M->derivs);
    rtsiSetContStatesPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo, (real_T **)
                         &sm_vehicle_2axle_heave_roll_M->contStates);
    rtsiSetNumContStatesPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
      &sm_vehicle_2axle_heave_roll_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
      &sm_vehicle_2axle_heave_roll_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr
      (&sm_vehicle_2axle_heave_roll_M->solverInfo,
       &sm_vehicle_2axle_heave_roll_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
      &sm_vehicle_2axle_heave_roll_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
      (boolean_T**) &sm_vehicle_2axle_heave_roll_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                          (&rtmGetErrorStatus(sm_vehicle_2axle_heave_roll_M)));
    rtsiSetRTModelPtr(&sm_vehicle_2axle_heave_roll_M->solverInfo,
                      sm_vehicle_2axle_heave_roll_M);
  }

  rtsiSetSimTimeStep(&sm_vehicle_2axle_heave_roll_M->solverInfo, MAJOR_TIME_STEP);
  sm_vehicle_2axle_heave_roll_M->intgData.f[0] =
    sm_vehicle_2axle_heave_roll_M->odeF[0];
  sm_vehicle_2axle_heave_roll_M->contStates = ((X_sm_vehicle_2axle_heave_roll_T *)
    sm_vehicle_2axle_heave_roll_X);
  sm_vehicle_2axle_heave_roll_M->contStateDisabled =
    ((XDis_sm_vehicle_2axle_heave_r_T *) sm_vehicle_2axle_heave_rol_XDis);
  sm_vehicle_2axle_heave_roll_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&sm_vehicle_2axle_heave_roll_M->solverInfo, (void *)
                    &sm_vehicle_2axle_heave_roll_M->intgData);
  rtsiSetIsMinorTimeStepWithModeChange
    (&sm_vehicle_2axle_heave_roll_M->solverInfo, false);
  rtsiSetSolverName(&sm_vehicle_2axle_heave_roll_M->solverInfo,"ode1");
  rtmSetTPtr(sm_vehicle_2axle_heave_roll_M,
             &sm_vehicle_2axle_heave_roll_M->Timing.tArray[0]);
  sm_vehicle_2axle_heave_roll_M->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) sm_vehicle_2axle_heave_roll_B), 0,
                sizeof(B_sm_vehicle_2axle_heave_roll_T));

  /* states (continuous) */
  {
    (void) memset((void *)sm_vehicle_2axle_heave_roll_X, 0,
                  sizeof(X_sm_vehicle_2axle_heave_roll_T));
  }

  /* disabled states */
  {
    (void) memset((void *)sm_vehicle_2axle_heave_rol_XDis, 0,
                  sizeof(XDis_sm_vehicle_2axle_heave_r_T));
  }

  /* states (dwork) */
  (void) memset((void *)sm_vehicle_2axle_heave_roll_DW, 0,
                sizeof(DW_sm_vehicle_2axle_heave_rol_T));

  /* external inputs */
  sm_vehicle_2axle_heave_roll_U->u = sm_vehicle_2axle_heave_roll_rtZ;

  /* external outputs */
  sm_vehicle_2axle_heave_roll_Y->y = sm_vehicle_2axle_heave_roll_r_0;

  {
    NeModelParameters modelParameters;
    NeModelParameters modelParameters_0;
    NeModelParameters modelParameters_1;
    NeslSimulationData *tmp_1;
    NeslSimulator *tmp;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    real_T tmp_2;
    int32_T tmp_3;
    boolean_T tmp_0;

    /* Start for SimscapeExecutionBlock: '<S207>/STATE_1' */
    tmp = nesl_lease_simulator(
      "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 0, 0);
    sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator);
    if (tmp_0) {
      sm_vehicle_2axle_heave_roll_836bb176_1_gateway();
      tmp = nesl_lease_simulator(
        "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 0, 0);
      sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator = (void *)tmp;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr = (void *)diagnosticManager;
    modelParameters.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters.mSolverAbsTol = 0.001;
    modelParameters.mSolverRelTol = 0.001;
    modelParameters.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters.mStartTime = 0.0;
    modelParameters.mLoadInitialState = false;
    modelParameters.mUseSimState = false;
    modelParameters.mLinTrimCompile = false;
    modelParameters.mLoggingMode = SSC_LOGGING_ON;
    modelParameters.mRTWModifiedTimeStamp = 6.24834411E+8;
    modelParameters.mUseModelRefSolver = false;
    modelParameters.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters.mFixedStepSize = tmp_2;
    tmp_0 = false;
    modelParameters.mVariableStepSolver = tmp_0;
    tmp_0 = false;
    modelParameters.mIsUsingODEN = tmp_0;
    modelParameters.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->STATE_1_Simulator, &modelParameters,
      diagnosticManager);
    if (tmp_3 != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp_0) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S207>/STATE_1' */

    /* Start for SimscapeExecutionBlock: '<S207>/OUTPUT_1_0' */
    tmp = nesl_lease_simulator(
      "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 1, 0);
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Simulator);
    if (tmp_0) {
      sm_vehicle_2axle_heave_roll_836bb176_1_gateway();
      tmp = nesl_lease_simulator(
        "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 1, 0);
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Simulator = (void *)tmp;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_DiagMgr = (void *)
      diagnosticManager;
    modelParameters_0.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters_0.mSolverAbsTol = 0.001;
    modelParameters_0.mSolverRelTol = 0.001;
    modelParameters_0.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_0.mStartTime = 0.0;
    modelParameters_0.mLoadInitialState = false;
    modelParameters_0.mUseSimState = false;
    modelParameters_0.mLinTrimCompile = false;
    modelParameters_0.mLoggingMode = SSC_LOGGING_ON;
    modelParameters_0.mRTWModifiedTimeStamp = 6.24834411E+8;
    modelParameters_0.mUseModelRefSolver = false;
    modelParameters_0.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters_0.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters_0.mFixedStepSize = tmp_2;
    tmp_0 = false;
    modelParameters_0.mVariableStepSolver = tmp_0;
    tmp_0 = false;
    modelParameters_0.mIsUsingODEN = tmp_0;
    modelParameters_0.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_Simulator, &modelParameters_0,
      diagnosticManager);
    if (tmp_3 != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp_0) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg_0);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S207>/OUTPUT_1_0' */

    /* Start for SimscapeExecutionBlock: '<S207>/OUTPUT_1_1' */
    tmp = nesl_lease_simulator(
      "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 1, 1);
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Simulator = (void *)tmp;
    tmp_0 = pointer_is_null(sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Simulator);
    if (tmp_0) {
      sm_vehicle_2axle_heave_roll_836bb176_1_gateway();
      tmp = nesl_lease_simulator(
        "sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1", 1, 1);
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Simulator = (void *)tmp;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_DiagMgr = (void *)
      diagnosticManager;
    modelParameters_1.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters_1.mSolverAbsTol = 0.001;
    modelParameters_1.mSolverRelTol = 0.001;
    modelParameters_1.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_1.mStartTime = 0.0;
    modelParameters_1.mLoadInitialState = false;
    modelParameters_1.mUseSimState = false;
    modelParameters_1.mLinTrimCompile = false;
    modelParameters_1.mLoggingMode = SSC_LOGGING_ON;
    modelParameters_1.mRTWModifiedTimeStamp = 6.24834411E+8;
    modelParameters_1.mUseModelRefSolver = false;
    modelParameters_1.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters_1.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters_1.mFixedStepSize = tmp_2;
    tmp_0 = false;
    modelParameters_1.mVariableStepSolver = tmp_0;
    tmp_0 = false;
    modelParameters_1.mIsUsingODEN = tmp_0;
    modelParameters_1.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_Simulator, &modelParameters_1,
      diagnosticManager);
    if (tmp_3 != 0) {
      tmp_0 = error_buffer_is_empty(rtmGetErrorStatus
        (sm_vehicle_2axle_heave_roll_M));
      if (tmp_0) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(sm_vehicle_2axle_heave_roll_M, msg_1);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S207>/OUTPUT_1_1' */

    /* InitializeConditions for TransferFcn: '<S40>/Transfer Fcn' */
    sm_vehicle_2axle_heave_roll_X->TransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S40>/Transfer Fcn1' */
    sm_vehicle_2axle_heave_roll_X->TransferFcn1_CSTATE = 0.0;
  }
//  if (hf) {
//    	  fclose(hf);
//      }
}

/* Model terminate function */
void sm_vehicle_2axle_heave_roll_terminate(RT_MODEL_sm_vehicle_2axle_hea_T *
  const sm_vehicle_2axle_heave_roll_M)
{
  DW_sm_vehicle_2axle_heave_rol_T *sm_vehicle_2axle_heave_roll_DW =
    sm_vehicle_2axle_heave_roll_M->dwork;

  /* Terminate for SimscapeExecutionBlock: '<S207>/STATE_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_2axle_heave_roll_DW->STATE_1_SimData);
  nesl_erase_simulator("sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S207>/OUTPUT_1_0' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_0_SimData);
  nesl_erase_simulator("sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S207>/OUTPUT_1_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_2axle_heave_roll_DW->OUTPUT_1_1_SimData);
  nesl_erase_simulator("sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();

}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
