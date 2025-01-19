/*
 * File: sm_vehicle_vtk.c
 *
 * Code generated for Simulink model 'sm_vehicle_vtk'.
 *
 * Model version                  : 11.0
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Aug 19 21:56:35 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "sm_vehicle_vtk.h"
#include <math.h>
#include <string.h>
#include "sm_vehicle_vtk_private.h"
#include "rtwtypes.h"
#include "sm_vehicle_vtk_types.h"
#include <stddef.h>
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Exported block parameters */
struct_S57YbmiMjO6n4oTP6qUIC InitVehicle = {
  {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
  },

  {
    0.0,
    0.0,
    0.0,
    0.0
  }
} ;                                    /* Variable: InitVehicle
                                        * Referenced by:
                                        *   '<S2>/Subsystem_around_RTP_041AAD1B_VelocityTargetValue'
                                        *   '<S2>/Subsystem_around_RTP_087BC81F_VelocityTargetValue'
                                        *   '<S2>/Subsystem_around_RTP_5BD4A88C_VelocityTargetValue'
                                        *   '<S2>/Subsystem_around_RTP_9CB22C5A_VelocityTargetValue'
                                        *   '<S6>/Subsystem_around_RTP_2B721290_PxPositionTargetValue'
                                        *   '<S6>/Subsystem_around_RTP_2B721290_PyPositionTargetValue'
                                        *   '<S6>/Subsystem_around_RTP_2B721290_PzPositionTargetValue'
                                        */

const sm_vehicle_vtk_U_t sm_vehicle_vtk_rtZsm_vehicle_vt = { 0.0,/* sWhl */
  0.0,                                 /* trqFL */
  0.0,                                 /* trqFR */
  0.0,                                 /* trqRL */
  0.0,                                 /* trqRR */

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 },                             /* T_FL */

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 },                             /* T_FR */

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 },                             /* T_RL */

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 }                              /* T_RR */
};

/* Projection for root system: '<Root>' */
void sm_vehicle_vtk_projection(RT_MODEL_sm_vehicle_vtk_T *const sm_vehicle_vtk_M)
{
  B_sm_vehicle_vtk_T *sm_vehicle_vtk_B = sm_vehicle_vtk_M->blockIO;
  DW_sm_vehicle_vtk_T *sm_vehicle_vtk_DW = sm_vehicle_vtk_M->dwork;
  X_sm_vehicle_vtk_T *sm_vehicle_vtk_X = sm_vehicle_vtk_M->contStates;
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  char *msg;
  real_T tmp_0[120];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[31];
  boolean_T tmp;

  /* Projection for SimscapeExecutionBlock: '<S279>/STATE_1' */
  simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->STATE_1_SimData;
  time = sm_vehicle_vtk_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 29;
  simulationData->mData->mContStates.mX =
    &sm_vehicle_vtk_X->sm_vehicle_vtkVehicleCamera_Fra[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &sm_vehicle_vtk_DW->STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mHadEvents = false;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(sm_vehicle_vtk_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&sm_vehicle_vtk_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&sm_vehicle_vtk_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
  tmp_0[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
  tmp_0[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
  tmp_0[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
  tmp_0[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
  tmp_0[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
  tmp_0[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
  tmp_0[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
  tmp_0[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
  tmp_0[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
  tmp_0[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
  tmp_0[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
  tmp_0[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
  tmp_0[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
  tmp_0[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
  tmp_0[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
  tmp_1[5] = 20;
  tmp_0[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
  tmp_0[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
  tmp_0[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
  tmp_0[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
  tmp_1[6] = 24;
  tmp_0[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
  tmp_0[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
  tmp_0[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
  tmp_0[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
  tmp_1[7] = 28;
  tmp_0[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
  tmp_0[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
  tmp_0[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
  tmp_0[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
  tmp_1[8] = 32;
  tmp_0[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
  tmp_0[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
  tmp_0[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
  tmp_0[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
  tmp_1[9] = 36;
  tmp_0[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
  tmp_0[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
  tmp_0[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
  tmp_0[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
  tmp_1[10] = 40;
  tmp_0[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
  tmp_0[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
  tmp_0[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
  tmp_0[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
  tmp_1[11] = 44;
  tmp_0[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
  tmp_0[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
  tmp_0[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
  tmp_0[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
  tmp_1[12] = 48;
  tmp_0[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
  tmp_0[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
  tmp_0[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
  tmp_0[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
  tmp_1[13] = 52;
  tmp_0[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
  tmp_0[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
  tmp_0[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
  tmp_0[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
  tmp_1[14] = 56;
  tmp_0[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
  tmp_0[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
  tmp_0[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
  tmp_0[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
  tmp_1[15] = 60;
  tmp_0[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
  tmp_0[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
  tmp_0[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
  tmp_0[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
  tmp_1[16] = 64;
  tmp_0[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
  tmp_0[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
  tmp_0[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
  tmp_0[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
  tmp_1[17] = 68;
  tmp_0[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
  tmp_0[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
  tmp_0[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
  tmp_0[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
  tmp_1[18] = 72;
  tmp_0[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
  tmp_0[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
  tmp_0[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
  tmp_0[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
  tmp_1[19] = 76;
  tmp_0[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
  tmp_0[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
  tmp_0[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
  tmp_0[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
  tmp_1[20] = 80;
  tmp_0[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
  tmp_0[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
  tmp_0[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
  tmp_0[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
  tmp_1[21] = 84;
  tmp_0[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
  tmp_0[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
  tmp_0[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
  tmp_0[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
  tmp_1[22] = 88;
  tmp_0[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
  tmp_0[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
  tmp_0[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
  tmp_0[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
  tmp_1[23] = 92;
  tmp_0[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
  tmp_0[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
  tmp_0[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
  tmp_0[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
  tmp_1[24] = 96;
  tmp_0[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
  tmp_0[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
  tmp_0[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
  tmp_0[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
  tmp_1[25] = 100;
  tmp_0[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
  tmp_0[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
  tmp_0[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
  tmp_0[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
  tmp_1[26] = 104;
  tmp_0[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
  tmp_0[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
  tmp_0[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
  tmp_0[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
  tmp_1[27] = 108;
  tmp_0[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
  tmp_0[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
  tmp_0[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
  tmp_0[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
  tmp_1[28] = 112;
  tmp_0[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
  tmp_0[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
  tmp_0[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
  tmp_0[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
  tmp_1[29] = 116;
  tmp_0[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
  tmp_0[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
  tmp_0[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
  tmp_0[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
  tmp_1[30] = 120;
  simulationData->mData->mInputValues.mN = 120;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 31;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  diagnosticManager = (NeuDiagnosticManager *)sm_vehicle_vtk_DW->STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)
    sm_vehicle_vtk_DW->STATE_1_Simulator, NESL_SIM_PROJECTION, simulationData,
    diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(sm_vehicle_vtk_M, msg);
    }
  }

  /* End of Projection for SimscapeExecutionBlock: '<S279>/STATE_1' */
}

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si ,
  RT_MODEL_sm_vehicle_vtk_T *const sm_vehicle_vtk_M, ExtU_sm_vehicle_vtk_T
  *sm_vehicle_vtk_U, ExtY_sm_vehicle_vtk_T *sm_vehicle_vtk_Y)
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 59;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  sm_vehicle_vtk_derivatives(sm_vehicle_vtk_M);
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  sm_vehicle_vtk_step(sm_vehicle_vtk_M, sm_vehicle_vtk_U, sm_vehicle_vtk_Y);
  sm_vehicle_vtk_projection(sm_vehicle_vtk_M);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
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
void sm_vehicle_vtk_step(RT_MODEL_sm_vehicle_vtk_T *const sm_vehicle_vtk_M,
  ExtU_sm_vehicle_vtk_T *sm_vehicle_vtk_U, ExtY_sm_vehicle_vtk_T
  *sm_vehicle_vtk_Y)
{
  B_sm_vehicle_vtk_T *sm_vehicle_vtk_B = sm_vehicle_vtk_M->blockIO;
  DW_sm_vehicle_vtk_T *sm_vehicle_vtk_DW = sm_vehicle_vtk_M->dwork;
  X_sm_vehicle_vtk_T *sm_vehicle_vtk_X = sm_vehicle_vtk_M->contStates;
  if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&sm_vehicle_vtk_M->solverInfo,
                          ((sm_vehicle_vtk_M->Timing.clockTick0+1)*
      sm_vehicle_vtk_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(sm_vehicle_vtk_M)) {
    sm_vehicle_vtk_M->Timing.t[0] = rtsiGetT(&sm_vehicle_vtk_M->solverInfo);
  }

  {
    NeParameterBundle expl_temp;
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diag;
    NeuDiagnosticTree *diagTree;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    char *msg_2;
    real_T tmp_4[149];
    real_T tmp_6[149];
    real_T tmp_1[120];
    real_T rtb_T[16];
    real_T rtb_Tc[16];
    real_T rtb_Tc_n[16];
    real_T rtb_Tc_o[16];
    real_T rtb_Tc_p[16];
    real_T rtb_Tt[16];
    real_T rtb_Tt_b[16];
    real_T rtb_Tt_j[16];
    real_T rtb_Tt_l[16];
    real_T rtb_Tw[16];
    real_T rtb_Tw_b[16];
    real_T rtb_Tw_g[16];
    real_T rtb_Tw_n[16];
    real_T rtb_VectorConcatenate[12];
    real_T rtb_VectorConcatenate_a[12];
    real_T rtb_VectorConcatenate_b[12];
    real_T rtb_VectorConcatenate_dq[12];
    real_T rtb_VectorConcatenate_e[12];
    real_T rtb_VectorConcatenate_g1[12];
    real_T rtb_VectorConcatenate_g2[12];
    real_T rtb_VectorConcatenate_iq[12];
    real_T rtb_VectorConcatenate_j[12];
    real_T rtb_VectorConcatenate_jb[12];
    real_T rtb_VectorConcatenate_jc[12];
    real_T rtb_VectorConcatenate_o[12];
    real_T tmp[10];
    real_T rtb_Selector[9];
    real_T rtb_Divide;
    real_T rtb_Fx;
    real_T rtb_Fy;
    real_T rtb_Fz;
    real_T rtb_Q_idx_0;
    real_T rtb_Q_idx_1;
    real_T rtb_Q_idx_2;
    real_T rtb_Q_idx_3;
    real_T rtb_Tx;
    real_T rtb_Ty;
    real_T rtb_Tz;
    real_T rtb_World_Q_idx_0;
    real_T rtb_World_Q_idx_1;
    real_T rtb_World_Q_idx_2;
    real_T rtb_World_Q_idx_3;
    real_T rtb_World_aPitch;
    real_T rtb_World_aRoll;
    real_T rtb_World_aYaw;
    real_T rtb_World_gx;
    real_T rtb_World_gy;
    real_T rtb_World_gz;
    real_T rtb_World_nPitch;
    real_T rtb_World_nRoll;
    real_T rtb_World_nYaw;
    real_T rtb_World_vx;
    real_T rtb_World_vy;
    real_T rtb_World_vz;
    real_T rtb_World_x;
    real_T rtb_World_y;
    real_T rtb_World_z;
    real_T rtb_aPitch;
    real_T rtb_aRoll;
    real_T rtb_aYaw;
    real_T rtb_gy;
    real_T rtb_gz;
    real_T rtb_nPitch;
    real_T rtb_nRoll;
    real_T rtb_w;
    real_T rtb_x;
    real_T time;
    real_T time_0;
    real_T time_1;
    real_T time_2;
    real_T time_3;
    real_T time_4;
    real_T time_tmp;
    int32_T i;
    int32_T rtb_T_tmp;
    int_T tmp_5[32];
    int_T tmp_7[32];
    int_T tmp_2[31];
    boolean_T ok;
    boolean_T tmp_0;
    boolean_T tmp_3;

    /* Product: '<S137>/Divide' incorporates:
     *  Constant: '<S137>/Constant'
     */
    rtb_Divide = sm_vehicle_vtk_U->u.sWhl / 16.0;

    /* Fcn: '<S137>/Ackerman left' incorporates:
     *  Fcn: '<S137>/Ackerman right'
     */
    rtb_Divide = cos(rtb_Divide + 2.2204460492503131e-16) / sin(rtb_Divide +
      2.2204460492503131e-16);

    /* Fcn: '<S137>/Ackerman left' */
    sm_vehicle_vtk_B->Ackermanleft = atan(1.0 / (rtb_Divide -
      0.26666666666666666));

    /* SimscapeInputBlock: '<S279>/INPUT_6_1_1' */
    if (sm_vehicle_vtk_DW->INPUT_6_1_1_FirstOutput == 0.0) {
      sm_vehicle_vtk_DW->INPUT_6_1_1_FirstOutput = 1.0;
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[0] =
        sm_vehicle_vtk_B->Ackermanleft;
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[1] = 0.0;
    }

    sm_vehicle_vtk_B->INPUT_6_1_1[0] =
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[0];
    sm_vehicle_vtk_B->INPUT_6_1_1[1] =
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[1];
    sm_vehicle_vtk_B->INPUT_6_1_1[2] = ((sm_vehicle_vtk_B->Ackermanleft -
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[0]) * 181.81818181818184
      - 2.0 * sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[1]) *
      181.81818181818184;
    sm_vehicle_vtk_B->INPUT_6_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S279>/INPUT_6_1_1' */

    /* Fcn: '<S137>/Ackerman right' */
    sm_vehicle_vtk_B->Ackermanright = atan(1.0 / (rtb_Divide +
      0.26666666666666666));

    /* SimscapeInputBlock: '<S279>/INPUT_5_1_1' */
    if (sm_vehicle_vtk_DW->INPUT_5_1_1_FirstOutput == 0.0) {
      sm_vehicle_vtk_DW->INPUT_5_1_1_FirstOutput = 1.0;
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[0] =
        sm_vehicle_vtk_B->Ackermanright;
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[1] = 0.0;
    }

    sm_vehicle_vtk_B->INPUT_5_1_1[0] =
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[0];
    sm_vehicle_vtk_B->INPUT_5_1_1[1] =
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[1];
    sm_vehicle_vtk_B->INPUT_5_1_1[2] = ((sm_vehicle_vtk_B->Ackermanright -
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[0]) * 181.81818181818184
      - 2.0 * sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[1]) *
      181.81818181818184;
    sm_vehicle_vtk_B->INPUT_5_1_1[3] = 0.0;

    /* End of SimscapeInputBlock: '<S279>/INPUT_5_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_7_1_1' incorporates:
     *  Constant: '<S171>/Constant'
     *  Constant: '<S171>/Constant1'
     *  TransferFcn: '<S124>/Transfer Fcn'
     */
    sm_vehicle_vtk_B->INPUT_7_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn_CSTATE;
    sm_vehicle_vtk_B->INPUT_7_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_7_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_7_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_7_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_7_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_7_1_1_Discrete[1] = sm_vehicle_vtk_B->
        INPUT_7_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_7_1_1[2] = sm_vehicle_vtk_DW->INPUT_7_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_7_1_1[3] = sm_vehicle_vtk_DW->INPUT_7_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_7_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_8_1_1' incorporates:
     *  Constant: '<S171>/Constant'
     *  Constant: '<S171>/Constant1'
     *  TransferFcn: '<S124>/Transfer Fcn1'
     */
    sm_vehicle_vtk_B->INPUT_8_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn1_CSTATE;
    sm_vehicle_vtk_B->INPUT_8_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_8_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_8_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_8_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_8_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_8_1_1_Discrete[1] = sm_vehicle_vtk_B->
        INPUT_8_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_8_1_1[2] = sm_vehicle_vtk_DW->INPUT_8_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_8_1_1[3] = sm_vehicle_vtk_DW->INPUT_8_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_8_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_9_1_1' incorporates:
     *  Constant: '<S171>/Constant'
     *  Constant: '<S171>/Constant1'
     *  TransferFcn: '<S124>/Transfer Fcn2'
     */
    sm_vehicle_vtk_B->INPUT_9_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn2_CSTATE;
    sm_vehicle_vtk_B->INPUT_9_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_9_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_9_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_9_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_9_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_9_1_1_Discrete[1] = sm_vehicle_vtk_B->
        INPUT_9_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_9_1_1[2] = sm_vehicle_vtk_DW->INPUT_9_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_9_1_1[3] = sm_vehicle_vtk_DW->INPUT_9_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_9_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_12_1_1' incorporates:
     *  Constant: '<S171>/Constant2'
     *  Constant: '<S171>/Constant3'
     *  TransferFcn: '<S124>/Transfer Fcn3'
     */
    sm_vehicle_vtk_B->INPUT_12_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn3_CSTATE;
    sm_vehicle_vtk_B->INPUT_12_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_12_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_12_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_12_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_12_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_12_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_12_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_12_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_12_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_12_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_12_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_12_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_10_1_1' incorporates:
     *  Constant: '<S171>/Constant2'
     *  Constant: '<S171>/Constant3'
     *  TransferFcn: '<S124>/Transfer Fcn5'
     */
    sm_vehicle_vtk_B->INPUT_10_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn5_CSTATE;
    sm_vehicle_vtk_B->INPUT_10_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_10_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_10_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_10_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_10_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_10_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_10_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_10_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_10_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_10_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_10_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_10_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_11_1_1' incorporates:
     *  Constant: '<S171>/Constant2'
     *  Constant: '<S171>/Constant3'
     *  TransferFcn: '<S124>/Transfer Fcn4'
     */
    sm_vehicle_vtk_B->INPUT_11_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn4_CSTATE;
    sm_vehicle_vtk_B->INPUT_11_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_11_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_11_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_11_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_11_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_11_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_11_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_11_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_11_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_11_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_11_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_11_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_13_1_1' incorporates:
     *  Constant: '<S199>/Constant'
     *  Constant: '<S199>/Constant1'
     *  TransferFcn: '<S126>/Transfer Fcn'
     */
    sm_vehicle_vtk_B->INPUT_13_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn_CSTATE_p;
    sm_vehicle_vtk_B->INPUT_13_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_13_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_13_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_13_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_13_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_13_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_13_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_13_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_13_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_13_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_13_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_13_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_14_1_1' incorporates:
     *  Constant: '<S199>/Constant'
     *  Constant: '<S199>/Constant1'
     *  TransferFcn: '<S126>/Transfer Fcn1'
     */
    sm_vehicle_vtk_B->INPUT_14_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn1_CSTATE_o;
    sm_vehicle_vtk_B->INPUT_14_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_14_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_14_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_14_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_14_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_14_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_14_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_14_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_14_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_14_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_14_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_14_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_15_1_1' incorporates:
     *  Constant: '<S199>/Constant'
     *  Constant: '<S199>/Constant1'
     *  TransferFcn: '<S126>/Transfer Fcn2'
     */
    sm_vehicle_vtk_B->INPUT_15_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn2_CSTATE_j;
    sm_vehicle_vtk_B->INPUT_15_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_15_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_15_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_15_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_15_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_15_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_15_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_15_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_15_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_15_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_15_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_15_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_18_1_1' incorporates:
     *  Constant: '<S199>/Constant2'
     *  Constant: '<S199>/Constant3'
     *  TransferFcn: '<S126>/Transfer Fcn3'
     */
    sm_vehicle_vtk_B->INPUT_18_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn3_CSTATE_o;
    sm_vehicle_vtk_B->INPUT_18_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_18_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_18_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_18_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_18_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_18_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_18_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_18_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_18_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_18_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_18_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_18_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_16_1_1' incorporates:
     *  Constant: '<S199>/Constant2'
     *  Constant: '<S199>/Constant3'
     *  TransferFcn: '<S126>/Transfer Fcn5'
     */
    sm_vehicle_vtk_B->INPUT_16_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn5_CSTATE_j;
    sm_vehicle_vtk_B->INPUT_16_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_16_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_16_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_16_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_16_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_16_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_16_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_16_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_16_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_16_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_16_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_16_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_17_1_1' incorporates:
     *  Constant: '<S199>/Constant2'
     *  Constant: '<S199>/Constant3'
     *  TransferFcn: '<S126>/Transfer Fcn4'
     */
    sm_vehicle_vtk_B->INPUT_17_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn4_CSTATE_n;
    sm_vehicle_vtk_B->INPUT_17_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_17_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_17_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_17_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_17_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_17_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_17_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_17_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_17_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_17_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_17_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_17_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_19_1_1' incorporates:
     *  Constant: '<S227>/Constant'
     *  Constant: '<S227>/Constant1'
     *  TransferFcn: '<S128>/Transfer Fcn'
     */
    sm_vehicle_vtk_B->INPUT_19_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn_CSTATE_e;
    sm_vehicle_vtk_B->INPUT_19_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_19_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_19_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_19_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_19_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_19_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_19_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_19_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_19_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_19_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_19_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_19_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_20_1_1' incorporates:
     *  Constant: '<S227>/Constant'
     *  Constant: '<S227>/Constant1'
     *  TransferFcn: '<S128>/Transfer Fcn1'
     */
    sm_vehicle_vtk_B->INPUT_20_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn1_CSTATE_d;
    sm_vehicle_vtk_B->INPUT_20_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_20_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_20_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_20_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_20_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_20_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_20_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_20_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_20_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_20_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_20_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_20_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_21_1_1' incorporates:
     *  Constant: '<S227>/Constant'
     *  Constant: '<S227>/Constant1'
     *  TransferFcn: '<S128>/Transfer Fcn2'
     */
    sm_vehicle_vtk_B->INPUT_21_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn2_CSTATE_c;
    sm_vehicle_vtk_B->INPUT_21_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_21_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_21_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_21_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_21_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_21_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_21_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_21_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_21_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_21_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_21_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_21_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_24_1_1' incorporates:
     *  Constant: '<S227>/Constant2'
     *  Constant: '<S227>/Constant3'
     *  TransferFcn: '<S128>/Transfer Fcn3'
     */
    sm_vehicle_vtk_B->INPUT_24_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn3_CSTATE_j;
    sm_vehicle_vtk_B->INPUT_24_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_24_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_24_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_24_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_24_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_24_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_24_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_24_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_24_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_24_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_24_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_24_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_22_1_1' incorporates:
     *  Constant: '<S227>/Constant2'
     *  Constant: '<S227>/Constant3'
     *  TransferFcn: '<S128>/Transfer Fcn5'
     */
    sm_vehicle_vtk_B->INPUT_22_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn5_CSTATE_b;
    sm_vehicle_vtk_B->INPUT_22_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_22_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_22_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_22_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_22_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_22_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_22_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_22_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_22_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_22_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_22_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_22_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_23_1_1' incorporates:
     *  Constant: '<S227>/Constant2'
     *  Constant: '<S227>/Constant3'
     *  TransferFcn: '<S128>/Transfer Fcn4'
     */
    sm_vehicle_vtk_B->INPUT_23_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn4_CSTATE_h;
    sm_vehicle_vtk_B->INPUT_23_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_23_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_23_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_23_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_23_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_23_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_23_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_23_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_23_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_23_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_23_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_23_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_25_1_1' incorporates:
     *  Constant: '<S255>/Constant'
     *  Constant: '<S255>/Constant1'
     *  TransferFcn: '<S130>/Transfer Fcn'
     */
    sm_vehicle_vtk_B->INPUT_25_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn_CSTATE_n;
    sm_vehicle_vtk_B->INPUT_25_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_25_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_25_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_25_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_25_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_25_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_25_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_25_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_25_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_25_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_25_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_25_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_26_1_1' incorporates:
     *  Constant: '<S255>/Constant'
     *  Constant: '<S255>/Constant1'
     *  TransferFcn: '<S130>/Transfer Fcn1'
     */
    sm_vehicle_vtk_B->INPUT_26_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn1_CSTATE_k;
    sm_vehicle_vtk_B->INPUT_26_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_26_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_26_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_26_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_26_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_26_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_26_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_26_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_26_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_26_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_26_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_26_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_27_1_1' incorporates:
     *  Constant: '<S255>/Constant'
     *  Constant: '<S255>/Constant1'
     *  TransferFcn: '<S130>/Transfer Fcn2'
     */
    sm_vehicle_vtk_B->INPUT_27_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn2_CSTATE_cy;
    sm_vehicle_vtk_B->INPUT_27_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_27_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_27_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_27_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_27_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_27_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_27_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_27_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_27_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_27_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_27_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_27_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_30_1_1' incorporates:
     *  Constant: '<S255>/Constant2'
     *  Constant: '<S255>/Constant3'
     *  TransferFcn: '<S130>/Transfer Fcn3'
     */
    sm_vehicle_vtk_B->INPUT_30_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn3_CSTATE_f;
    sm_vehicle_vtk_B->INPUT_30_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_30_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_30_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_30_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_30_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_30_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_30_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_30_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_30_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_30_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_30_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_30_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_28_1_1' incorporates:
     *  Constant: '<S255>/Constant2'
     *  Constant: '<S255>/Constant3'
     *  TransferFcn: '<S130>/Transfer Fcn5'
     */
    sm_vehicle_vtk_B->INPUT_28_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn5_CSTATE_f;
    sm_vehicle_vtk_B->INPUT_28_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_28_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_28_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_28_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_28_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_28_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_28_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_28_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_28_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_28_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_28_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_28_1_1' */

    /* SimscapeInputBlock: '<S279>/INPUT_29_1_1' incorporates:
     *  Constant: '<S255>/Constant2'
     *  Constant: '<S255>/Constant3'
     *  TransferFcn: '<S130>/Transfer Fcn4'
     */
    sm_vehicle_vtk_B->INPUT_29_1_1[0] = 100.0 *
      sm_vehicle_vtk_X->TransferFcn4_CSTATE_g;
    sm_vehicle_vtk_B->INPUT_29_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_29_1_1[2] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      sm_vehicle_vtk_DW->INPUT_29_1_1_Discrete[0] =
        !(sm_vehicle_vtk_DW->INPUT_29_1_1_Discrete[1] ==
          sm_vehicle_vtk_B->INPUT_29_1_1[2]);
      sm_vehicle_vtk_DW->INPUT_29_1_1_Discrete[1] =
        sm_vehicle_vtk_B->INPUT_29_1_1[2];
    }

    sm_vehicle_vtk_B->INPUT_29_1_1[2] = sm_vehicle_vtk_DW->
      INPUT_29_1_1_Discrete[1];
    sm_vehicle_vtk_B->INPUT_29_1_1[3] = sm_vehicle_vtk_DW->
      INPUT_29_1_1_Discrete[0];

    /* End of SimscapeInputBlock: '<S279>/INPUT_29_1_1' */
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* SimscapeRtp: '<S278>/RTP_1' incorporates:
       *  Constant: '<S2>/Subsystem_around_RTP_041AAD1B_VelocityTargetValue'
       *  Constant: '<S2>/Subsystem_around_RTP_087BC81F_VelocityTargetValue'
       *  Constant: '<S2>/Subsystem_around_RTP_5BD4A88C_VelocityTargetValue'
       *  Constant: '<S2>/Subsystem_around_RTP_9CB22C5A_VelocityTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PxPositionTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PxVelocityTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PyPositionTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PyVelocityTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PzPositionTargetValue'
       *  Constant: '<S6>/Subsystem_around_RTP_2B721290_PzVelocityTargetValue'
       */
      if (sm_vehicle_vtk_DW->RTP_1_SetParametersNeeded) {
        tmp[0] = InitVehicle.Wheel.wFL;
        tmp[1] = InitVehicle.Wheel.wFR;
        tmp[2] = InitVehicle.Vehicle.px;
        tmp[3] = 0.0;
        tmp[4] = InitVehicle.Vehicle.py;
        tmp[5] = 0.0;
        tmp[6] = InitVehicle.Vehicle.pz;
        tmp[7] = 0.0;
        tmp[8] = InitVehicle.Wheel.wRR;
        tmp[9] = InitVehicle.Wheel.wRL;
        diag = rtw_create_diagnostics();
        diagTree = neu_diagnostic_manager_get_initial_tree(diag);
        expl_temp.mRealParameters.mN = 10;
        expl_temp.mRealParameters.mX = &tmp[0];
        expl_temp.mLogicalParameters.mN = 0;
        expl_temp.mLogicalParameters.mX = NULL;
        expl_temp.mIntegerParameters.mN = 0;
        expl_temp.mIntegerParameters.mX = NULL;
        expl_temp.mIndexParameters.mN = 0;
        expl_temp.mIndexParameters.mX = NULL;
        ok = nesl_rtp_manager_set_rtps((NeslRtpManager *)
          sm_vehicle_vtk_DW->RTP_1_RtpManager, sm_vehicle_vtk_M->Timing.t[0],
          expl_temp, diag);
        if (!ok) {
          ok = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
          if (ok) {
            msg = rtw_diagnostics_msg(diagTree);
            rtmSetErrorStatus(sm_vehicle_vtk_M, msg);
          }
        }
      }

      sm_vehicle_vtk_DW->RTP_1_SetParametersNeeded = false;

      /* End of SimscapeRtp: '<S278>/RTP_1' */
    }

    /* SimscapeExecutionBlock: '<S279>/STATE_1' incorporates:
     *  SimscapeExecutionBlock: '<S279>/OUTPUT_1_0'
     *  SimscapeExecutionBlock: '<S279>/OUTPUT_1_1'
     */
    simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->STATE_1_SimData;
    rtb_Divide = sm_vehicle_vtk_M->Timing.t[0];
    time = rtb_Divide;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 29;
    simulationData->mData->mContStates.mX =
      &sm_vehicle_vtk_X->sm_vehicle_vtkVehicleCamera_Fra[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &sm_vehicle_vtk_DW->STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->STATE_1_Modes;
    ok = false;
    simulationData->mData->mFoundZcEvents = ok;
    simulationData->mData->mHadEvents = false;
    ok = rtmIsMajorTimeStep(sm_vehicle_vtk_M);
    simulationData->mData->mIsMajorTimeStep = ok;
    tmp_0 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_0;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp_0 = rtsiIsSolverComputingJacobian(&sm_vehicle_vtk_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp_0;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    tmp_0 = rtsiIsModeUpdateTimeStep(&sm_vehicle_vtk_M->solverInfo);
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_2[0] = 0;
    tmp_1[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
    tmp_1[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
    tmp_1[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
    tmp_1[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
    tmp_2[1] = 4;
    tmp_1[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
    tmp_1[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
    tmp_1[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
    tmp_1[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
    tmp_2[2] = 8;
    tmp_1[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
    tmp_1[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
    tmp_1[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
    tmp_1[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
    tmp_2[3] = 12;
    tmp_1[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
    tmp_1[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
    tmp_1[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
    tmp_1[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
    tmp_2[4] = 16;
    tmp_1[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
    tmp_1[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
    tmp_1[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
    tmp_1[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
    tmp_2[5] = 20;
    tmp_1[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
    tmp_1[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
    tmp_1[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
    tmp_1[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
    tmp_2[6] = 24;
    tmp_1[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
    tmp_1[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
    tmp_1[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
    tmp_1[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
    tmp_2[7] = 28;
    tmp_1[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
    tmp_1[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
    tmp_1[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
    tmp_1[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
    tmp_2[8] = 32;
    tmp_1[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
    tmp_1[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
    tmp_1[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
    tmp_1[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
    tmp_2[9] = 36;
    tmp_1[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
    tmp_1[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
    tmp_1[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
    tmp_1[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
    tmp_2[10] = 40;
    tmp_1[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
    tmp_1[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
    tmp_1[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
    tmp_1[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
    tmp_2[11] = 44;
    tmp_1[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
    tmp_1[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
    tmp_1[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
    tmp_1[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
    tmp_2[12] = 48;
    tmp_1[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
    tmp_1[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
    tmp_1[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
    tmp_1[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
    tmp_2[13] = 52;
    tmp_1[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
    tmp_1[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
    tmp_1[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
    tmp_1[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
    tmp_2[14] = 56;
    tmp_1[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
    tmp_1[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
    tmp_1[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
    tmp_1[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
    tmp_2[15] = 60;
    tmp_1[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
    tmp_1[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
    tmp_1[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
    tmp_1[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
    tmp_2[16] = 64;
    tmp_1[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
    tmp_1[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
    tmp_1[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
    tmp_1[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
    tmp_2[17] = 68;
    tmp_1[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
    tmp_1[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
    tmp_1[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
    tmp_1[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
    tmp_2[18] = 72;
    tmp_1[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
    tmp_1[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
    tmp_1[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
    tmp_1[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
    tmp_2[19] = 76;
    tmp_1[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
    tmp_1[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
    tmp_1[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
    tmp_1[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
    tmp_2[20] = 80;
    tmp_1[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
    tmp_1[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
    tmp_1[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
    tmp_1[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
    tmp_2[21] = 84;
    tmp_1[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
    tmp_1[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
    tmp_1[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
    tmp_1[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
    tmp_2[22] = 88;
    tmp_1[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
    tmp_1[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
    tmp_1[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
    tmp_1[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
    tmp_2[23] = 92;
    tmp_1[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
    tmp_1[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
    tmp_1[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
    tmp_1[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
    tmp_2[24] = 96;
    tmp_1[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
    tmp_1[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
    tmp_1[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
    tmp_1[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
    tmp_2[25] = 100;
    tmp_1[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
    tmp_1[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
    tmp_1[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
    tmp_1[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
    tmp_2[26] = 104;
    tmp_1[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
    tmp_1[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
    tmp_1[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
    tmp_1[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
    tmp_2[27] = 108;
    tmp_1[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
    tmp_1[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
    tmp_1[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
    tmp_1[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
    tmp_2[28] = 112;
    tmp_1[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
    tmp_1[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
    tmp_1[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
    tmp_1[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
    tmp_2[29] = 116;
    tmp_1[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
    tmp_1[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
    tmp_1[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
    tmp_1[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
    tmp_2[30] = 120;
    simulationData->mData->mInputValues.mN = 120;
    simulationData->mData->mInputValues.mX = &tmp_1[0];
    simulationData->mData->mInputOffsets.mN = 31;
    simulationData->mData->mInputOffsets.mX = &tmp_2[0];
    simulationData->mData->mOutputs.mN = 29;
    simulationData->mData->mOutputs.mX = &sm_vehicle_vtk_B->STATE_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    simulationData->mData->mDstateHasChanged = false;
    time_tmp = sm_vehicle_vtk_M->Timing.t[0];
    time_0 = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_0;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mHadEvents = false;
    diag = (NeuDiagnosticManager *)sm_vehicle_vtk_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diag);
    i = ne_simulator_method((NeslSimulator *)
      sm_vehicle_vtk_DW->STATE_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diag);
    if (i != 0) {
      tmp_3 = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp_3) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg_0);
      }
    }

    /* End of SimscapeExecutionBlock: '<S279>/STATE_1' */

    /* SimscapeExecutionBlock: '<S279>/OUTPUT_1_0' */
    simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->OUTPUT_1_0_SimData;
    time_1 = rtb_Divide;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_1;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_vtk_DW->OUTPUT_1_0_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->OUTPUT_1_0_Modes;
    tmp_3 = false;
    simulationData->mData->mFoundZcEvents = tmp_3;
    simulationData->mData->mHadEvents = false;
    simulationData->mData->mIsMajorTimeStep = ok;
    tmp_3 = false;
    simulationData->mData->mIsSolverAssertCheck = tmp_3;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_5[0] = 0;
    tmp_4[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
    tmp_4[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
    tmp_4[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
    tmp_4[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
    tmp_5[1] = 4;
    tmp_4[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
    tmp_4[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
    tmp_4[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
    tmp_4[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
    tmp_5[2] = 8;
    tmp_4[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
    tmp_4[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
    tmp_4[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
    tmp_4[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
    tmp_5[3] = 12;
    tmp_4[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
    tmp_4[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
    tmp_4[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
    tmp_4[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
    tmp_5[4] = 16;
    tmp_4[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
    tmp_4[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
    tmp_4[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
    tmp_4[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
    tmp_5[5] = 20;
    tmp_4[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
    tmp_4[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
    tmp_4[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
    tmp_4[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
    tmp_5[6] = 24;
    tmp_4[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
    tmp_4[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
    tmp_4[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
    tmp_4[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
    tmp_5[7] = 28;
    tmp_4[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
    tmp_4[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
    tmp_4[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
    tmp_4[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
    tmp_5[8] = 32;
    tmp_4[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
    tmp_4[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
    tmp_4[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
    tmp_4[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
    tmp_5[9] = 36;
    tmp_4[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
    tmp_4[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
    tmp_4[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
    tmp_4[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
    tmp_5[10] = 40;
    tmp_4[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
    tmp_4[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
    tmp_4[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
    tmp_4[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
    tmp_5[11] = 44;
    tmp_4[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
    tmp_4[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
    tmp_4[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
    tmp_4[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
    tmp_5[12] = 48;
    tmp_4[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
    tmp_4[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
    tmp_4[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
    tmp_4[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
    tmp_5[13] = 52;
    tmp_4[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
    tmp_4[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
    tmp_4[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
    tmp_4[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
    tmp_5[14] = 56;
    tmp_4[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
    tmp_4[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
    tmp_4[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
    tmp_4[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
    tmp_5[15] = 60;
    tmp_4[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
    tmp_4[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
    tmp_4[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
    tmp_4[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
    tmp_5[16] = 64;
    tmp_4[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
    tmp_4[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
    tmp_4[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
    tmp_4[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
    tmp_5[17] = 68;
    tmp_4[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
    tmp_4[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
    tmp_4[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
    tmp_4[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
    tmp_5[18] = 72;
    tmp_4[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
    tmp_4[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
    tmp_4[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
    tmp_4[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
    tmp_5[19] = 76;
    tmp_4[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
    tmp_4[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
    tmp_4[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
    tmp_4[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
    tmp_5[20] = 80;
    tmp_4[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
    tmp_4[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
    tmp_4[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
    tmp_4[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
    tmp_5[21] = 84;
    tmp_4[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
    tmp_4[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
    tmp_4[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
    tmp_4[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
    tmp_5[22] = 88;
    tmp_4[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
    tmp_4[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
    tmp_4[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
    tmp_4[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
    tmp_5[23] = 92;
    tmp_4[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
    tmp_4[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
    tmp_4[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
    tmp_4[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
    tmp_5[24] = 96;
    tmp_4[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
    tmp_4[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
    tmp_4[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
    tmp_4[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
    tmp_5[25] = 100;
    tmp_4[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
    tmp_4[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
    tmp_4[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
    tmp_4[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
    tmp_5[26] = 104;
    tmp_4[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
    tmp_4[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
    tmp_4[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
    tmp_4[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
    tmp_5[27] = 108;
    tmp_4[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
    tmp_4[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
    tmp_4[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
    tmp_4[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
    tmp_5[28] = 112;
    tmp_4[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
    tmp_4[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
    tmp_4[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
    tmp_4[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
    tmp_5[29] = 116;
    tmp_4[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
    tmp_4[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
    tmp_4[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
    tmp_4[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
    tmp_5[30] = 120;
    memcpy(&tmp_4[120], &sm_vehicle_vtk_B->STATE_1[0], 29U * sizeof(real_T));
    tmp_5[31] = 149;
    simulationData->mData->mInputValues.mN = 149;
    simulationData->mData->mInputValues.mX = &tmp_4[0];
    simulationData->mData->mInputOffsets.mN = 32;
    simulationData->mData->mInputOffsets.mX = &tmp_5[0];
    simulationData->mData->mOutputs.mN = 221;
    simulationData->mData->mOutputs.mX = &sm_vehicle_vtk_B->OUTPUT_1_0[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    simulationData->mData->mDstateHasChanged = false;
    time_2 = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_2;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mHadEvents = false;
    diag = (NeuDiagnosticManager *)sm_vehicle_vtk_DW->OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diag);
    i = ne_simulator_method((NeslSimulator *)
      sm_vehicle_vtk_DW->OUTPUT_1_0_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diag);
    if (i != 0) {
      tmp_3 = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp_3) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg_1);
      }
    }

    /* Reshape: '<S58>/RESHAPE' */
    rtb_VectorConcatenate[9] = sm_vehicle_vtk_B->OUTPUT_1_0[113];
    rtb_VectorConcatenate[10] = sm_vehicle_vtk_B->OUTPUT_1_0[114];
    rtb_VectorConcatenate[11] = sm_vehicle_vtk_B->OUTPUT_1_0[115];

    /* Reshape: '<S193>/RESHAPE' */
    rtb_VectorConcatenate_e[9] = sm_vehicle_vtk_B->OUTPUT_1_0[182];
    rtb_VectorConcatenate_e[10] = sm_vehicle_vtk_B->OUTPUT_1_0[183];
    rtb_VectorConcatenate_e[11] = sm_vehicle_vtk_B->OUTPUT_1_0[184];

    /* SignalConversion generated from: '<S166>/Vector Concatenate' */
    rtb_VectorConcatenate_g1[9] = sm_vehicle_vtk_B->OUTPUT_1_0[32];
    rtb_VectorConcatenate_g1[10] = sm_vehicle_vtk_B->OUTPUT_1_0[33];
    rtb_VectorConcatenate_g1[11] = sm_vehicle_vtk_B->OUTPUT_1_0[34];

    /* Reshape: '<S148>/RESHAPE' */
    rtb_VectorConcatenate_a[9] = sm_vehicle_vtk_B->OUTPUT_1_0[134];
    rtb_VectorConcatenate_a[10] = sm_vehicle_vtk_B->OUTPUT_1_0[135];
    rtb_VectorConcatenate_a[11] = sm_vehicle_vtk_B->OUTPUT_1_0[136];

    /* Reshape: '<S221>/RESHAPE' */
    rtb_VectorConcatenate_g2[9] = sm_vehicle_vtk_B->OUTPUT_1_0[194];
    rtb_VectorConcatenate_g2[10] = sm_vehicle_vtk_B->OUTPUT_1_0[195];
    rtb_VectorConcatenate_g2[11] = sm_vehicle_vtk_B->OUTPUT_1_0[196];

    /* SignalConversion generated from: '<S194>/Vector Concatenate' */
    rtb_VectorConcatenate_j[9] = sm_vehicle_vtk_B->OUTPUT_1_0[50];
    rtb_VectorConcatenate_j[10] = sm_vehicle_vtk_B->OUTPUT_1_0[51];
    rtb_VectorConcatenate_j[11] = sm_vehicle_vtk_B->OUTPUT_1_0[52];

    /* Reshape: '<S153>/RESHAPE' */
    rtb_VectorConcatenate_dq[9] = sm_vehicle_vtk_B->OUTPUT_1_0[146];
    rtb_VectorConcatenate_dq[10] = sm_vehicle_vtk_B->OUTPUT_1_0[147];
    rtb_VectorConcatenate_dq[11] = sm_vehicle_vtk_B->OUTPUT_1_0[148];

    /* Reshape: '<S249>/RESHAPE' */
    rtb_VectorConcatenate_iq[9] = sm_vehicle_vtk_B->OUTPUT_1_0[206];
    rtb_VectorConcatenate_iq[10] = sm_vehicle_vtk_B->OUTPUT_1_0[207];
    rtb_VectorConcatenate_iq[11] = sm_vehicle_vtk_B->OUTPUT_1_0[208];

    /* SignalConversion generated from: '<S222>/Vector Concatenate' */
    rtb_VectorConcatenate_b[9] = sm_vehicle_vtk_B->OUTPUT_1_0[68];
    rtb_VectorConcatenate_b[10] = sm_vehicle_vtk_B->OUTPUT_1_0[69];
    rtb_VectorConcatenate_b[11] = sm_vehicle_vtk_B->OUTPUT_1_0[70];

    /* Reshape: '<S160>/RESHAPE' */
    rtb_VectorConcatenate_jb[9] = sm_vehicle_vtk_B->OUTPUT_1_0[158];
    rtb_VectorConcatenate_jb[10] = sm_vehicle_vtk_B->OUTPUT_1_0[159];
    rtb_VectorConcatenate_jb[11] = sm_vehicle_vtk_B->OUTPUT_1_0[160];

    /* Reshape: '<S277>/RESHAPE' */
    rtb_VectorConcatenate_jc[9] = sm_vehicle_vtk_B->OUTPUT_1_0[218];
    rtb_VectorConcatenate_jc[10] = sm_vehicle_vtk_B->OUTPUT_1_0[219];
    rtb_VectorConcatenate_jc[11] = sm_vehicle_vtk_B->OUTPUT_1_0[220];

    /* SignalConversion generated from: '<S250>/Vector Concatenate' */
    rtb_VectorConcatenate_o[9] = sm_vehicle_vtk_B->OUTPUT_1_0[86];
    rtb_VectorConcatenate_o[10] = sm_vehicle_vtk_B->OUTPUT_1_0[87];
    rtb_VectorConcatenate_o[11] = sm_vehicle_vtk_B->OUTPUT_1_0[88];

    /* Reshape: '<S57>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate[0], &sm_vehicle_vtk_B->OUTPUT_1_0[104], 9U *
           sizeof(real_T));

    /* Reshape: '<S192>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_e[0], &sm_vehicle_vtk_B->OUTPUT_1_0[173], 9U *
           sizeof(real_T));

    /* Reshape: '<S175>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_g1[0], &sm_vehicle_vtk_B->OUTPUT_1_0[23], 9U *
           sizeof(real_T));

    /* Reshape: '<S147>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_a[0], &sm_vehicle_vtk_B->OUTPUT_1_0[125], 9U *
           sizeof(real_T));

    /* Reshape: '<S220>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_g2[0], &sm_vehicle_vtk_B->OUTPUT_1_0[185], 9U *
           sizeof(real_T));

    /* Reshape: '<S203>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_j[0], &sm_vehicle_vtk_B->OUTPUT_1_0[41], 9U *
           sizeof(real_T));

    /* Reshape: '<S152>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_dq[0], &sm_vehicle_vtk_B->OUTPUT_1_0[137], 9U *
           sizeof(real_T));

    /* Reshape: '<S248>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_iq[0], &sm_vehicle_vtk_B->OUTPUT_1_0[197], 9U *
           sizeof(real_T));

    /* Reshape: '<S231>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_b[0], &sm_vehicle_vtk_B->OUTPUT_1_0[59], 9U *
           sizeof(real_T));

    /* Reshape: '<S159>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_jb[0], &sm_vehicle_vtk_B->OUTPUT_1_0[149], 9U *
           sizeof(real_T));

    /* Reshape: '<S276>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_jc[0], &sm_vehicle_vtk_B->OUTPUT_1_0[209], 9U *
           sizeof(real_T));

    /* Reshape: '<S259>/RESHAPE' */
    memcpy(&rtb_VectorConcatenate_o[0], &sm_vehicle_vtk_B->OUTPUT_1_0[77], 9U *
           sizeof(real_T));

    /* SimscapeInputBlock: '<S279>/INPUT_4_1_1' */
    sm_vehicle_vtk_B->INPUT_4_1_1[0] = sm_vehicle_vtk_U->u.trqFL;
    sm_vehicle_vtk_B->INPUT_4_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_4_1_1[2] = 0.0;
    sm_vehicle_vtk_B->INPUT_4_1_1[3] = 0.0;

    /* SimscapeInputBlock: '<S279>/INPUT_1_1_1' */
    sm_vehicle_vtk_B->INPUT_1_1_1[0] = sm_vehicle_vtk_U->u.trqFR;
    sm_vehicle_vtk_B->INPUT_1_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_1_1_1[2] = 0.0;
    sm_vehicle_vtk_B->INPUT_1_1_1[3] = 0.0;

    /* SimscapeInputBlock: '<S279>/INPUT_3_1_1' */
    sm_vehicle_vtk_B->INPUT_3_1_1[0] = sm_vehicle_vtk_U->u.trqRL;
    sm_vehicle_vtk_B->INPUT_3_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_3_1_1[2] = 0.0;
    sm_vehicle_vtk_B->INPUT_3_1_1[3] = 0.0;

    /* SimscapeInputBlock: '<S279>/INPUT_2_1_1' */
    sm_vehicle_vtk_B->INPUT_2_1_1[0] = sm_vehicle_vtk_U->u.trqRR;
    sm_vehicle_vtk_B->INPUT_2_1_1[1] = 0.0;
    sm_vehicle_vtk_B->INPUT_2_1_1[2] = 0.0;
    sm_vehicle_vtk_B->INPUT_2_1_1[3] = 0.0;
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S54>/Vector Concatenate'
       *  Concatenate: '<S54>/Vector Concatenate1'
       *  Constant: '<S54>/Constant'
       *  Math: '<S54>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_T[rtb_T_tmp] = rtb_VectorConcatenate[i];
        rtb_T[rtb_T_tmp + 1] = rtb_VectorConcatenate[i + 3];
        rtb_T[rtb_T_tmp + 2] = rtb_VectorConcatenate[i + 6];
        rtb_T[rtb_T_tmp + 3] = rtb_VectorConcatenate[i + 9];
      }

      rtb_T[12] = 0.0;
      rtb_T[13] = 0.0;
      rtb_T[14] = 0.0;
      rtb_T[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S189>/Vector Concatenate'
       *  Concatenate: '<S189>/Vector Concatenate1'
       *  Constant: '<S189>/Constant'
       *  Math: '<S189>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tw[rtb_T_tmp] = rtb_VectorConcatenate_e[i];
        rtb_Tw[rtb_T_tmp + 1] = rtb_VectorConcatenate_e[i + 3];
        rtb_Tw[rtb_T_tmp + 2] = rtb_VectorConcatenate_e[i + 6];
        rtb_Tw[rtb_T_tmp + 3] = rtb_VectorConcatenate_e[i + 9];
      }

      rtb_Tw[12] = 0.0;
      rtb_Tw[13] = 0.0;
      rtb_Tw[14] = 0.0;
      rtb_Tw[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S166>/Vector Concatenate'
       *  Concatenate: '<S166>/Vector Concatenate1'
       *  Constant: '<S166>/Constant'
       *  Math: '<S166>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tt[rtb_T_tmp] = rtb_VectorConcatenate_g1[i];
        rtb_Tt[rtb_T_tmp + 1] = rtb_VectorConcatenate_g1[i + 3];
        rtb_Tt[rtb_T_tmp + 2] = rtb_VectorConcatenate_g1[i + 6];
        rtb_Tt[rtb_T_tmp + 3] = rtb_VectorConcatenate_g1[i + 9];
      }

      rtb_Tt[12] = 0.0;
      rtb_Tt[13] = 0.0;
      rtb_Tt[14] = 0.0;
      rtb_Tt[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S144>/Vector Concatenate'
       *  Concatenate: '<S144>/Vector Concatenate1'
       *  Constant: '<S144>/Constant'
       *  Math: '<S144>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tc[rtb_T_tmp] = rtb_VectorConcatenate_a[i];
        rtb_Tc[rtb_T_tmp + 1] = rtb_VectorConcatenate_a[i + 3];
        rtb_Tc[rtb_T_tmp + 2] = rtb_VectorConcatenate_a[i + 6];
        rtb_Tc[rtb_T_tmp + 3] = rtb_VectorConcatenate_a[i + 9];
      }

      rtb_Tc[12] = 0.0;
      rtb_Tc[13] = 0.0;
      rtb_Tc[14] = 0.0;
      rtb_Tc[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S217>/Vector Concatenate'
       *  Concatenate: '<S217>/Vector Concatenate1'
       *  Constant: '<S217>/Constant'
       *  Math: '<S217>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tw_n[rtb_T_tmp] = rtb_VectorConcatenate_g2[i];
        rtb_Tw_n[rtb_T_tmp + 1] = rtb_VectorConcatenate_g2[i + 3];
        rtb_Tw_n[rtb_T_tmp + 2] = rtb_VectorConcatenate_g2[i + 6];
        rtb_Tw_n[rtb_T_tmp + 3] = rtb_VectorConcatenate_g2[i + 9];
      }

      rtb_Tw_n[12] = 0.0;
      rtb_Tw_n[13] = 0.0;
      rtb_Tw_n[14] = 0.0;
      rtb_Tw_n[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S194>/Vector Concatenate'
       *  Concatenate: '<S194>/Vector Concatenate1'
       *  Constant: '<S194>/Constant'
       *  Math: '<S194>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tt_l[rtb_T_tmp] = rtb_VectorConcatenate_j[i];
        rtb_Tt_l[rtb_T_tmp + 1] = rtb_VectorConcatenate_j[i + 3];
        rtb_Tt_l[rtb_T_tmp + 2] = rtb_VectorConcatenate_j[i + 6];
        rtb_Tt_l[rtb_T_tmp + 3] = rtb_VectorConcatenate_j[i + 9];
      }

      rtb_Tt_l[12] = 0.0;
      rtb_Tt_l[13] = 0.0;
      rtb_Tt_l[14] = 0.0;
      rtb_Tt_l[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S149>/Vector Concatenate'
       *  Concatenate: '<S149>/Vector Concatenate1'
       *  Constant: '<S149>/Constant'
       *  Math: '<S149>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tc_o[rtb_T_tmp] = rtb_VectorConcatenate_dq[i];
        rtb_Tc_o[rtb_T_tmp + 1] = rtb_VectorConcatenate_dq[i + 3];
        rtb_Tc_o[rtb_T_tmp + 2] = rtb_VectorConcatenate_dq[i + 6];
        rtb_Tc_o[rtb_T_tmp + 3] = rtb_VectorConcatenate_dq[i + 9];
      }

      rtb_Tc_o[12] = 0.0;
      rtb_Tc_o[13] = 0.0;
      rtb_Tc_o[14] = 0.0;
      rtb_Tc_o[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S245>/Vector Concatenate'
       *  Concatenate: '<S245>/Vector Concatenate1'
       *  Constant: '<S245>/Constant'
       *  Math: '<S245>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tw_g[rtb_T_tmp] = rtb_VectorConcatenate_iq[i];
        rtb_Tw_g[rtb_T_tmp + 1] = rtb_VectorConcatenate_iq[i + 3];
        rtb_Tw_g[rtb_T_tmp + 2] = rtb_VectorConcatenate_iq[i + 6];
        rtb_Tw_g[rtb_T_tmp + 3] = rtb_VectorConcatenate_iq[i + 9];
      }

      rtb_Tw_g[12] = 0.0;
      rtb_Tw_g[13] = 0.0;
      rtb_Tw_g[14] = 0.0;
      rtb_Tw_g[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S222>/Vector Concatenate'
       *  Concatenate: '<S222>/Vector Concatenate1'
       *  Constant: '<S222>/Constant'
       *  Math: '<S222>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tt_j[rtb_T_tmp] = rtb_VectorConcatenate_b[i];
        rtb_Tt_j[rtb_T_tmp + 1] = rtb_VectorConcatenate_b[i + 3];
        rtb_Tt_j[rtb_T_tmp + 2] = rtb_VectorConcatenate_b[i + 6];
        rtb_Tt_j[rtb_T_tmp + 3] = rtb_VectorConcatenate_b[i + 9];
      }

      rtb_Tt_j[12] = 0.0;
      rtb_Tt_j[13] = 0.0;
      rtb_Tt_j[14] = 0.0;
      rtb_Tt_j[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S156>/Vector Concatenate'
       *  Concatenate: '<S156>/Vector Concatenate1'
       *  Constant: '<S156>/Constant'
       *  Math: '<S156>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tc_n[rtb_T_tmp] = rtb_VectorConcatenate_jb[i];
        rtb_Tc_n[rtb_T_tmp + 1] = rtb_VectorConcatenate_jb[i + 3];
        rtb_Tc_n[rtb_T_tmp + 2] = rtb_VectorConcatenate_jb[i + 6];
        rtb_Tc_n[rtb_T_tmp + 3] = rtb_VectorConcatenate_jb[i + 9];
      }

      rtb_Tc_n[12] = 0.0;
      rtb_Tc_n[13] = 0.0;
      rtb_Tc_n[14] = 0.0;
      rtb_Tc_n[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S273>/Vector Concatenate'
       *  Concatenate: '<S273>/Vector Concatenate1'
       *  Constant: '<S273>/Constant'
       *  Math: '<S273>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tw_b[rtb_T_tmp] = rtb_VectorConcatenate_jc[i];
        rtb_Tw_b[rtb_T_tmp + 1] = rtb_VectorConcatenate_jc[i + 3];
        rtb_Tw_b[rtb_T_tmp + 2] = rtb_VectorConcatenate_jc[i + 6];
        rtb_Tw_b[rtb_T_tmp + 3] = rtb_VectorConcatenate_jc[i + 9];
      }

      rtb_Tw_b[12] = 0.0;
      rtb_Tw_b[13] = 0.0;
      rtb_Tw_b[14] = 0.0;
      rtb_Tw_b[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S250>/Vector Concatenate'
       *  Concatenate: '<S250>/Vector Concatenate1'
       *  Constant: '<S250>/Constant'
       *  Math: '<S250>/Transpose'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tt_b[rtb_T_tmp] = rtb_VectorConcatenate_o[i];
        rtb_Tt_b[rtb_T_tmp + 1] = rtb_VectorConcatenate_o[i + 3];
        rtb_Tt_b[rtb_T_tmp + 2] = rtb_VectorConcatenate_o[i + 6];
        rtb_Tt_b[rtb_T_tmp + 3] = rtb_VectorConcatenate_o[i + 9];
      }

      rtb_Tt_b[12] = 0.0;
      rtb_Tt_b[13] = 0.0;
      rtb_Tt_b[14] = 0.0;
      rtb_Tt_b[15] = 1.0;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Concatenate: '<S161>/Vector Concatenate1'
       *  Constant: '<S161>/Constant'
       *  Math: '<S161>/Transpose'
       *  Reshape: '<S164>/RESHAPE'
       *  Reshape: '<S165>/RESHAPE'
       */
      for (i = 0; i < 3; i++) {
        rtb_T_tmp = i << 2;
        rtb_Tc_p[rtb_T_tmp] = (&sm_vehicle_vtk_B->OUTPUT_1_0[161])[i];
        rtb_Tc_p[rtb_T_tmp + 1] = (&sm_vehicle_vtk_B->OUTPUT_1_0[161])[i + 3];
        rtb_Tc_p[rtb_T_tmp + 2] = (&sm_vehicle_vtk_B->OUTPUT_1_0[161])[i + 6];
        rtb_Tc_p[rtb_T_tmp + 3] = (&sm_vehicle_vtk_B->OUTPUT_1_0[161])[i + 9];
      }

      rtb_Tc_p[12] = 0.0;
      rtb_Tc_p[13] = 0.0;
      rtb_Tc_p[14] = 0.0;
      rtb_Tc_p[15] = 1.0;
    }

    /* SimscapeExecutionBlock: '<S279>/OUTPUT_1_1' */
    simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->OUTPUT_1_1_SimData;
    time_3 = rtb_Divide;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_3;
    simulationData->mData->mContStates.mN = 0;
    simulationData->mData->mContStates.mX = NULL;
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX =
      &sm_vehicle_vtk_DW->OUTPUT_1_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->OUTPUT_1_1_Modes;
    tmp_3 = false;
    simulationData->mData->mFoundZcEvents = tmp_3;
    simulationData->mData->mHadEvents = false;
    simulationData->mData->mIsMajorTimeStep = ok;
    ok = false;
    simulationData->mData->mIsSolverAssertCheck = ok;
    simulationData->mData->mIsSolverCheckingCIC = false;
    simulationData->mData->mIsComputingJacobian = false;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = tmp_0;
    tmp_7[0] = 0;
    tmp_6[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
    tmp_6[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
    tmp_6[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
    tmp_6[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
    tmp_7[1] = 4;
    tmp_6[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
    tmp_6[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
    tmp_6[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
    tmp_6[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
    tmp_7[2] = 8;
    tmp_6[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
    tmp_6[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
    tmp_6[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
    tmp_6[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
    tmp_7[3] = 12;
    tmp_6[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
    tmp_6[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
    tmp_6[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
    tmp_6[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
    tmp_7[4] = 16;
    tmp_6[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
    tmp_6[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
    tmp_6[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
    tmp_6[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
    tmp_7[5] = 20;
    tmp_6[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
    tmp_6[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
    tmp_6[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
    tmp_6[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
    tmp_7[6] = 24;
    tmp_6[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
    tmp_6[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
    tmp_6[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
    tmp_6[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
    tmp_7[7] = 28;
    tmp_6[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
    tmp_6[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
    tmp_6[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
    tmp_6[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
    tmp_7[8] = 32;
    tmp_6[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
    tmp_6[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
    tmp_6[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
    tmp_6[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
    tmp_7[9] = 36;
    tmp_6[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
    tmp_6[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
    tmp_6[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
    tmp_6[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
    tmp_7[10] = 40;
    tmp_6[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
    tmp_6[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
    tmp_6[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
    tmp_6[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
    tmp_7[11] = 44;
    tmp_6[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
    tmp_6[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
    tmp_6[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
    tmp_6[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
    tmp_7[12] = 48;
    tmp_6[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
    tmp_6[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
    tmp_6[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
    tmp_6[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
    tmp_7[13] = 52;
    tmp_6[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
    tmp_6[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
    tmp_6[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
    tmp_6[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
    tmp_7[14] = 56;
    tmp_6[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
    tmp_6[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
    tmp_6[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
    tmp_6[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
    tmp_7[15] = 60;
    tmp_6[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
    tmp_6[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
    tmp_6[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
    tmp_6[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
    tmp_7[16] = 64;
    tmp_6[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
    tmp_6[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
    tmp_6[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
    tmp_6[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
    tmp_7[17] = 68;
    tmp_6[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
    tmp_6[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
    tmp_6[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
    tmp_6[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
    tmp_7[18] = 72;
    tmp_6[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
    tmp_6[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
    tmp_6[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
    tmp_6[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
    tmp_7[19] = 76;
    tmp_6[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
    tmp_6[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
    tmp_6[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
    tmp_6[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
    tmp_7[20] = 80;
    tmp_6[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
    tmp_6[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
    tmp_6[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
    tmp_6[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
    tmp_7[21] = 84;
    tmp_6[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
    tmp_6[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
    tmp_6[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
    tmp_6[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
    tmp_7[22] = 88;
    tmp_6[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
    tmp_6[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
    tmp_6[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
    tmp_6[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
    tmp_7[23] = 92;
    tmp_6[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
    tmp_6[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
    tmp_6[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
    tmp_6[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
    tmp_7[24] = 96;
    tmp_6[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
    tmp_6[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
    tmp_6[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
    tmp_6[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
    tmp_7[25] = 100;
    tmp_6[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
    tmp_6[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
    tmp_6[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
    tmp_6[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
    tmp_7[26] = 104;
    tmp_6[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
    tmp_6[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
    tmp_6[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
    tmp_6[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
    tmp_7[27] = 108;
    tmp_6[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
    tmp_6[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
    tmp_6[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
    tmp_6[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
    tmp_7[28] = 112;
    tmp_6[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
    tmp_6[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
    tmp_6[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
    tmp_6[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
    tmp_7[29] = 116;
    tmp_6[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
    tmp_6[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
    tmp_6[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
    tmp_6[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
    tmp_7[30] = 120;
    memcpy(&tmp_6[120], &sm_vehicle_vtk_B->STATE_1[0], 29U * sizeof(real_T));
    tmp_7[31] = 149;
    simulationData->mData->mInputValues.mN = 149;
    simulationData->mData->mInputValues.mX = &tmp_6[0];
    simulationData->mData->mInputOffsets.mN = 32;
    simulationData->mData->mInputOffsets.mX = &tmp_7[0];
    simulationData->mData->mOutputs.mN = 6;
    simulationData->mData->mOutputs.mX = &sm_vehicle_vtk_B->OUTPUT_1_1[0];
    simulationData->mData->mTolerances.mN = 0;
    simulationData->mData->mTolerances.mX = NULL;
    simulationData->mData->mCstateHasChanged = false;
    simulationData->mData->mDstateHasChanged = false;
    time_4 = time_tmp;
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time_4;
    simulationData->mData->mSampleHits.mN = 0;
    simulationData->mData->mSampleHits.mX = NULL;
    simulationData->mData->mIsFundamentalSampleHit = false;
    simulationData->mData->mHadEvents = false;
    diag = (NeuDiagnosticManager *)sm_vehicle_vtk_DW->OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diag);
    i = ne_simulator_method((NeslSimulator *)
      sm_vehicle_vtk_DW->OUTPUT_1_1_Simulator, NESL_SIM_OUTPUTS, simulationData,
      diag);
    if (i != 0) {
      ok = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (ok) {
        msg_2 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg_2);
      }
    }

    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Q_idx_0 = sm_vehicle_vtk_B->OUTPUT_1_0[6];
      rtb_Q_idx_1 = sm_vehicle_vtk_B->OUTPUT_1_0[7];
      rtb_Q_idx_2 = sm_vehicle_vtk_B->OUTPUT_1_0[8];
      rtb_Q_idx_3 = sm_vehicle_vtk_B->OUTPUT_1_0[9];
    }

    /* Selector: '<S41>/Select y axis' */
    memcpy(&rtb_Selector[0], &sm_vehicle_vtk_B->OUTPUT_1_0[89], 9U * sizeof
           (real_T));
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Selector: '<S73>/R([2 1], 1)'
       *  Trigonometry: '<S73>/Atan1'
       */
      rtb_aYaw = rt_atan2d_snf((&sm_vehicle_vtk_B->OUTPUT_1_0[116])[1],
        (&sm_vehicle_vtk_B->OUTPUT_1_0[116])[0]);

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_x = sm_vehicle_vtk_B->OUTPUT_1_0[0];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_aPitch = sm_vehicle_vtk_B->OUTPUT_1_0[2];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_aRoll = sm_vehicle_vtk_B->OUTPUT_1_0[4];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_nPitch = sm_vehicle_vtk_B->OUTPUT_1_0[1];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_nRoll = sm_vehicle_vtk_B->OUTPUT_1_0[3];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_gz = sm_vehicle_vtk_B->OUTPUT_1_0[5];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_gy = sm_vehicle_vtk_B->OUTPUT_1_1[0];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Tz = sm_vehicle_vtk_B->OUTPUT_1_1[1];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Ty = sm_vehicle_vtk_B->OUTPUT_1_1[2];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Tx = sm_vehicle_vtk_B->OUTPUT_1_0[10];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Fz = sm_vehicle_vtk_B->OUTPUT_1_0[11];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Fy = sm_vehicle_vtk_B->OUTPUT_1_0[12];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  Selector: '<S73>/R(3, 2:3)'
       *  Trigonometry: '<S73>/Atan2'
       */
      rtb_Fx = rt_atan2d_snf((&sm_vehicle_vtk_B->OUTPUT_1_0[116])[5],
        (&sm_vehicle_vtk_B->OUTPUT_1_0[116])[8]);
    }

    /* Selector: '<S73>/R(3, 1)' */
    rtb_Divide = sm_vehicle_vtk_B->OUTPUT_1_0[118];
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* BusCreator generated from: '<Root>/y' */
      rtb_World_x = rtb_x;
      rtb_World_y = rtb_aPitch;
      rtb_World_z = rtb_aRoll;
      rtb_World_vx = rtb_nPitch;
      rtb_World_vy = rtb_nRoll;
      rtb_World_vz = rtb_gz;
      rtb_World_gx = rtb_gy;
      rtb_World_gy = rtb_Tz;
      rtb_World_gz = rtb_Ty;
      rtb_World_nRoll = rtb_Tx;
      rtb_World_nPitch = rtb_Fz;
      rtb_World_nYaw = rtb_Fy;
      rtb_World_aRoll = rtb_Fx;

      /* Trigonometry: '<S73>/Asin' incorporates:
       *  Gain: '<S73>/Multiply2'
       */
      if (-rtb_Divide > 1.0) {
        rtb_x = 1.0;
      } else if (-rtb_Divide < -1.0) {
        rtb_x = -1.0;
      } else {
        rtb_x = -rtb_Divide;
      }

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  Trigonometry: '<S73>/Asin'
       */
      rtb_World_aPitch = asin(rtb_x);
      rtb_World_aYaw = rtb_aYaw;
      rtb_World_Q_idx_0 = rtb_Q_idx_0;
      rtb_World_Q_idx_1 = rtb_Q_idx_1;
      rtb_World_Q_idx_2 = rtb_Q_idx_2;
      rtb_World_Q_idx_3 = rtb_Q_idx_3;

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_w = sm_vehicle_vtk_B->OUTPUT_1_0[98];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Fx = sm_vehicle_vtk_B->OUTPUT_1_0[99];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Fy = sm_vehicle_vtk_B->OUTPUT_1_0[100];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Fz = sm_vehicle_vtk_B->OUTPUT_1_0[101];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Tx = sm_vehicle_vtk_B->OUTPUT_1_0[102];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Ty = sm_vehicle_vtk_B->OUTPUT_1_0[103];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_Tz = sm_vehicle_vtk_B->OUTPUT_1_1[3];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_gy = sm_vehicle_vtk_B->OUTPUT_1_1[4];

      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' */
      rtb_gz = sm_vehicle_vtk_B->OUTPUT_1_1[5];
    }

    /* Fcn: '<S53>/magnitude' incorporates:
     *  Selector: '<S41>/Select y axis'
     */
    rtb_x = rtb_Selector[3] * rtb_Selector[3] + rtb_Selector[4] * rtb_Selector[4];
    if (rtb_x < 0.0) {
      rtb_x = -sqrt(-rtb_x);
    } else {
      rtb_x = sqrt(rtb_x);
    }

    /* Trigonometry: '<S53>/Trigonometric Function1' incorporates:
     *  Fcn: '<S53>/magnitude'
     *  Selector: '<S41>/Select y axis'
     */
    sm_vehicle_vtk_B->TrigonometricFunction1 = rt_atan2d_snf(rtb_Selector[5],
      rtb_x);
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* ZeroOrderHold generated from: '<Root>/Zero-Order Hold' incorporates:
       *  TransferFcn: '<S41>/Transfer Fcn'
       */
      rtb_nRoll = -10000.0 * sm_vehicle_vtk_X->TransferFcn_CSTATE_b + 100.0 *
        sm_vehicle_vtk_B->TrigonometricFunction1;
    }

    /* Fcn: '<S52>/magnitude' incorporates:
     *  Selector: '<S41>/Select x axis'
     */
    rtb_x = ((&sm_vehicle_vtk_B->OUTPUT_1_0[89])[0] *
             (&sm_vehicle_vtk_B->OUTPUT_1_0[89])[0] +
             (&sm_vehicle_vtk_B->OUTPUT_1_0[89])[1] *
             (&sm_vehicle_vtk_B->OUTPUT_1_0[89])[1]) +
      (&sm_vehicle_vtk_B->OUTPUT_1_0[89])[2] * (&sm_vehicle_vtk_B->OUTPUT_1_0[89])
      [2];
    if (rtb_x < 0.0) {
      rtb_x = -sqrt(-rtb_x);
    } else {
      rtb_x = sqrt(rtb_x);
    }

    /* Product: '<S52>/Divide1' incorporates:
     *  Fcn: '<S52>/magnitude'
     *  Product: '<S52>/Divide'
     *  Selector: '<S41>/Select x axis'
     *  Trigonometry: '<S52>/Cos'
     */
    rtb_Q_idx_0 = (&sm_vehicle_vtk_B->OUTPUT_1_0[89])[2] / cos
      (sm_vehicle_vtk_B->TrigonometricFunction1) / rtb_x;

    /* Trigonometry: '<S52>/Asin' */
    if (rtb_Q_idx_0 > 1.0) {
      rtb_Q_idx_0 = 1.0;
    } else if (rtb_Q_idx_0 < -1.0) {
      rtb_Q_idx_0 = -1.0;
    }

    /* Gain: '<S41>/Flip sign for -x axis ' incorporates:
     *  Trigonometry: '<S52>/Asin'
     */
    sm_vehicle_vtk_B->Flipsignforxaxis = -asin(rtb_Q_idx_0);
    if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
      /* BusCreator generated from: '<Root>/y' incorporates:
       *  TransferFcn: '<S41>/Transfer Fcn1'
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      sm_vehicle_vtk_Y->y.Body.x = rtb_w;
      sm_vehicle_vtk_Y->y.Body.y = rtb_Fx;
      sm_vehicle_vtk_Y->y.Body.z = rtb_Fy;
      sm_vehicle_vtk_Y->y.Body.vx = rtb_Fz;
      sm_vehicle_vtk_Y->y.Body.vy = rtb_Tx;
      sm_vehicle_vtk_Y->y.Body.vz = rtb_Ty;
      sm_vehicle_vtk_Y->y.Body.gx = rtb_Tz;
      sm_vehicle_vtk_Y->y.Body.gy = rtb_gy;
      sm_vehicle_vtk_Y->y.Body.gz = rtb_gz;
      sm_vehicle_vtk_Y->y.Body.nRoll = rtb_nRoll;
      sm_vehicle_vtk_Y->y.Body.nPitch = -10000.0 *
        sm_vehicle_vtk_X->TransferFcn1_CSTATE_a + 100.0 *
        sm_vehicle_vtk_B->Flipsignforxaxis;
      sm_vehicle_vtk_Y->y.Body.aRoll = sm_vehicle_vtk_B->TrigonometricFunction1;
      sm_vehicle_vtk_Y->y.Body.aPitch = sm_vehicle_vtk_B->Flipsignforxaxis;

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      sm_vehicle_vtk_Y->y.TireFL.w = sm_vehicle_vtk_B->OUTPUT_1_0[13];
      sm_vehicle_vtk_Y->y.TireFL.Fx = sm_vehicle_vtk_B->OUTPUT_1_0[17];
      sm_vehicle_vtk_Y->y.TireFL.Fy = sm_vehicle_vtk_B->OUTPUT_1_0[18];
      sm_vehicle_vtk_Y->y.TireFL.Fz = sm_vehicle_vtk_B->OUTPUT_1_0[19];
      sm_vehicle_vtk_Y->y.TireFL.Tx = sm_vehicle_vtk_B->OUTPUT_1_0[20];
      sm_vehicle_vtk_Y->y.TireFL.Ty = sm_vehicle_vtk_B->OUTPUT_1_0[21];
      sm_vehicle_vtk_Y->y.TireFL.Tz = sm_vehicle_vtk_B->OUTPUT_1_0[22];

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      sm_vehicle_vtk_Y->y.TireFR.w = sm_vehicle_vtk_B->OUTPUT_1_0[14];
      sm_vehicle_vtk_Y->y.TireFR.Fx = sm_vehicle_vtk_B->OUTPUT_1_0[35];
      sm_vehicle_vtk_Y->y.TireFR.Fy = sm_vehicle_vtk_B->OUTPUT_1_0[36];
      sm_vehicle_vtk_Y->y.TireFR.Fz = sm_vehicle_vtk_B->OUTPUT_1_0[37];
      sm_vehicle_vtk_Y->y.TireFR.Tx = sm_vehicle_vtk_B->OUTPUT_1_0[38];
      sm_vehicle_vtk_Y->y.TireFR.Ty = sm_vehicle_vtk_B->OUTPUT_1_0[39];
      sm_vehicle_vtk_Y->y.TireFR.Tz = sm_vehicle_vtk_B->OUTPUT_1_0[40];

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      sm_vehicle_vtk_Y->y.TireRL.w = sm_vehicle_vtk_B->OUTPUT_1_0[15];
      sm_vehicle_vtk_Y->y.TireRL.Fx = sm_vehicle_vtk_B->OUTPUT_1_0[53];
      sm_vehicle_vtk_Y->y.TireRL.Fy = sm_vehicle_vtk_B->OUTPUT_1_0[54];
      sm_vehicle_vtk_Y->y.TireRL.Fz = sm_vehicle_vtk_B->OUTPUT_1_0[55];
      sm_vehicle_vtk_Y->y.TireRL.Tx = sm_vehicle_vtk_B->OUTPUT_1_0[56];
      sm_vehicle_vtk_Y->y.TireRL.Ty = sm_vehicle_vtk_B->OUTPUT_1_0[57];
      sm_vehicle_vtk_Y->y.TireRL.Tz = sm_vehicle_vtk_B->OUTPUT_1_0[58];

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      sm_vehicle_vtk_Y->y.TireRR.w = sm_vehicle_vtk_B->OUTPUT_1_0[16];
      sm_vehicle_vtk_Y->y.TireRR.Fx = sm_vehicle_vtk_B->OUTPUT_1_0[71];
      sm_vehicle_vtk_Y->y.TireRR.Fy = sm_vehicle_vtk_B->OUTPUT_1_0[72];
      sm_vehicle_vtk_Y->y.TireRR.Fz = sm_vehicle_vtk_B->OUTPUT_1_0[73];
      sm_vehicle_vtk_Y->y.TireRR.Tx = sm_vehicle_vtk_B->OUTPUT_1_0[74];
      sm_vehicle_vtk_Y->y.TireRR.Ty = sm_vehicle_vtk_B->OUTPUT_1_0[75];
      sm_vehicle_vtk_Y->y.TireRR.Tz = sm_vehicle_vtk_B->OUTPUT_1_0[76];

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      memcpy(&sm_vehicle_vtk_Y->y.Body.T[0], &rtb_T[0], sizeof(real_T) << 4U);

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      memcpy(&sm_vehicle_vtk_Y->y.TireFL.Tw[0], &rtb_Tw[0], sizeof(real_T) << 4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireFL.Tt[0], &rtb_Tt[0], sizeof(real_T) << 4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireFL.Tc[0], &rtb_Tc[0], sizeof(real_T) << 4U);

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      memcpy(&sm_vehicle_vtk_Y->y.TireFR.Tw[0], &rtb_Tw_n[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireFR.Tt[0], &rtb_Tt_l[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireFR.Tc[0], &rtb_Tc_o[0], sizeof(real_T) <<
             4U);

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      memcpy(&sm_vehicle_vtk_Y->y.TireRL.Tw[0], &rtb_Tw_g[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireRL.Tt[0], &rtb_Tt_j[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireRL.Tc[0], &rtb_Tc_n[0], sizeof(real_T) <<
             4U);

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  ZeroOrderHold generated from: '<Root>/Zero-Order Hold'
       */
      memcpy(&sm_vehicle_vtk_Y->y.TireRR.Tw[0], &rtb_Tw_b[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireRR.Tt[0], &rtb_Tt_b[0], sizeof(real_T) <<
             4U);
      memcpy(&sm_vehicle_vtk_Y->y.TireRR.Tc[0], &rtb_Tc_p[0], sizeof(real_T) <<
             4U);

      /* BusCreator generated from: '<Root>/y' incorporates:
       *  Outport: '<Root>/y'
       */
      sm_vehicle_vtk_Y->y.World.x = rtb_World_x;
      sm_vehicle_vtk_Y->y.World.y = rtb_World_y;
      sm_vehicle_vtk_Y->y.World.z = rtb_World_z;
      sm_vehicle_vtk_Y->y.World.vx = rtb_World_vx;
      sm_vehicle_vtk_Y->y.World.vy = rtb_World_vy;
      sm_vehicle_vtk_Y->y.World.vz = rtb_World_vz;
      sm_vehicle_vtk_Y->y.World.gx = rtb_World_gx;
      sm_vehicle_vtk_Y->y.World.gy = rtb_World_gy;
      sm_vehicle_vtk_Y->y.World.gz = rtb_World_gz;
      sm_vehicle_vtk_Y->y.World.nRoll = rtb_World_nRoll;
      sm_vehicle_vtk_Y->y.World.nPitch = rtb_World_nPitch;
      sm_vehicle_vtk_Y->y.World.nYaw = rtb_World_nYaw;
      sm_vehicle_vtk_Y->y.World.aRoll = rtb_World_aRoll;
      sm_vehicle_vtk_Y->y.World.aPitch = rtb_World_aPitch;
      sm_vehicle_vtk_Y->y.World.aYaw = rtb_World_aYaw;
      sm_vehicle_vtk_Y->y.World.Q[0] = rtb_World_Q_idx_0;
      sm_vehicle_vtk_Y->y.World.Q[1] = rtb_World_Q_idx_1;
      sm_vehicle_vtk_Y->y.World.Q[2] = rtb_World_Q_idx_2;
      sm_vehicle_vtk_Y->y.World.Q[3] = rtb_World_Q_idx_3;
    }

    /* Math: '<S124>/Transpose' incorporates:
     *  Inport: '<Root>/u'
     *  Math: '<S130>/Transpose'
     */
    for (i = 0; i < 4; i++) {
      rtb_T_tmp = i << 2;
      rtb_T[rtb_T_tmp] = sm_vehicle_vtk_U->u.T_FL[i];
      rtb_T[rtb_T_tmp + 1] = sm_vehicle_vtk_U->u.T_FL[i + 4];
      rtb_T[rtb_T_tmp + 2] = sm_vehicle_vtk_U->u.T_FL[i + 8];
      rtb_T[rtb_T_tmp + 3] = sm_vehicle_vtk_U->u.T_FL[i + 12];
    }

    /* End of Math: '<S124>/Transpose' */
    for (i = 0; i < 3; i++) {
      /* Selector: '<S124>/Selector' incorporates:
       *  Math: '<S130>/Transpose'
       *  Selector: '<S130>/Selector'
       */
      rtb_T_tmp = i << 2;
      rtb_Selector[3 * i] = rtb_T[rtb_T_tmp];
      rtb_Selector[3 * i + 1] = rtb_T[rtb_T_tmp + 1];
      rtb_Selector[3 * i + 2] = rtb_T[rtb_T_tmp + 2];

      /* Selector: '<S124>/Selector1' incorporates:
       *  Math: '<S130>/Transpose'
       */
      sm_vehicle_vtk_B->Selector1[i] = rtb_T[i + 12];
    }

    /* Trigonometry: '<S125>/Atan2' incorporates:
     *  Selector: '<S125>/R(3, 2:3)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan2_e = rt_atan2d_snf(rtb_Selector[5], rtb_Selector[8]);

    /* Trigonometry: '<S125>/Atan1' incorporates:
     *  Selector: '<S125>/R([2 1], 1)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan1_b = rt_atan2d_snf(rtb_Selector[1], rtb_Selector[0]);

    /* Trigonometry: '<S125>/Asin' incorporates:
     *  Gain: '<S125>/Multiply2'
     *  Selector: '<S125>/R(3, 1)'
     *  Selector: '<S130>/Selector'
     */
    if (-rtb_Selector[2] > 1.0) {
      rtb_x = 1.0;
    } else if (-rtb_Selector[2] < -1.0) {
      rtb_x = -1.0;
    } else {
      rtb_x = -rtb_Selector[2];
    }

    /* Trigonometry: '<S125>/Asin' */
    sm_vehicle_vtk_B->Asin_d = asin(rtb_x);

    /* Math: '<S126>/Transpose' incorporates:
     *  Inport: '<Root>/u'
     *  Math: '<S130>/Transpose'
     */
    for (i = 0; i < 4; i++) {
      rtb_T_tmp = i << 2;
      rtb_T[rtb_T_tmp] = sm_vehicle_vtk_U->u.T_FR[i];
      rtb_T[rtb_T_tmp + 1] = sm_vehicle_vtk_U->u.T_FR[i + 4];
      rtb_T[rtb_T_tmp + 2] = sm_vehicle_vtk_U->u.T_FR[i + 8];
      rtb_T[rtb_T_tmp + 3] = sm_vehicle_vtk_U->u.T_FR[i + 12];
    }

    /* End of Math: '<S126>/Transpose' */
    for (i = 0; i < 3; i++) {
      /* Selector: '<S126>/Selector' incorporates:
       *  Math: '<S130>/Transpose'
       *  Selector: '<S130>/Selector'
       */
      rtb_T_tmp = i << 2;
      rtb_Selector[3 * i] = rtb_T[rtb_T_tmp];
      rtb_Selector[3 * i + 1] = rtb_T[rtb_T_tmp + 1];
      rtb_Selector[3 * i + 2] = rtb_T[rtb_T_tmp + 2];

      /* Selector: '<S126>/Selector1' incorporates:
       *  Math: '<S130>/Transpose'
       */
      sm_vehicle_vtk_B->Selector1_i[i] = rtb_T[i + 12];
    }

    /* Trigonometry: '<S127>/Atan2' incorporates:
     *  Selector: '<S127>/R(3, 2:3)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan2_n = rt_atan2d_snf(rtb_Selector[5], rtb_Selector[8]);

    /* Trigonometry: '<S127>/Atan1' incorporates:
     *  Selector: '<S127>/R([2 1], 1)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan1_i = rt_atan2d_snf(rtb_Selector[1], rtb_Selector[0]);

    /* Trigonometry: '<S127>/Asin' incorporates:
     *  Gain: '<S127>/Multiply2'
     *  Selector: '<S127>/R(3, 1)'
     *  Selector: '<S130>/Selector'
     */
    if (-rtb_Selector[2] > 1.0) {
      rtb_x = 1.0;
    } else if (-rtb_Selector[2] < -1.0) {
      rtb_x = -1.0;
    } else {
      rtb_x = -rtb_Selector[2];
    }

    /* Trigonometry: '<S127>/Asin' */
    sm_vehicle_vtk_B->Asin_j = asin(rtb_x);

    /* Math: '<S128>/Transpose' incorporates:
     *  Inport: '<Root>/u'
     *  Math: '<S130>/Transpose'
     */
    for (i = 0; i < 4; i++) {
      rtb_T_tmp = i << 2;
      rtb_T[rtb_T_tmp] = sm_vehicle_vtk_U->u.T_RL[i];
      rtb_T[rtb_T_tmp + 1] = sm_vehicle_vtk_U->u.T_RL[i + 4];
      rtb_T[rtb_T_tmp + 2] = sm_vehicle_vtk_U->u.T_RL[i + 8];
      rtb_T[rtb_T_tmp + 3] = sm_vehicle_vtk_U->u.T_RL[i + 12];
    }

    /* End of Math: '<S128>/Transpose' */
    for (i = 0; i < 3; i++) {
      /* Selector: '<S128>/Selector' incorporates:
       *  Math: '<S130>/Transpose'
       *  Selector: '<S130>/Selector'
       */
      rtb_T_tmp = i << 2;
      rtb_Selector[3 * i] = rtb_T[rtb_T_tmp];
      rtb_Selector[3 * i + 1] = rtb_T[rtb_T_tmp + 1];
      rtb_Selector[3 * i + 2] = rtb_T[rtb_T_tmp + 2];

      /* Selector: '<S128>/Selector1' incorporates:
       *  Math: '<S130>/Transpose'
       */
      sm_vehicle_vtk_B->Selector1_k[i] = rtb_T[i + 12];
    }

    /* Trigonometry: '<S129>/Atan2' incorporates:
     *  Selector: '<S129>/R(3, 2:3)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan2_k = rt_atan2d_snf(rtb_Selector[5], rtb_Selector[8]);

    /* Trigonometry: '<S129>/Atan1' incorporates:
     *  Selector: '<S129>/R([2 1], 1)'
     *  Selector: '<S130>/Selector'
     */
    sm_vehicle_vtk_B->Atan1_g = rt_atan2d_snf(rtb_Selector[1], rtb_Selector[0]);

    /* Trigonometry: '<S129>/Asin' incorporates:
     *  Gain: '<S129>/Multiply2'
     *  Selector: '<S129>/R(3, 1)'
     *  Selector: '<S130>/Selector'
     */
    if (-rtb_Selector[2] > 1.0) {
      rtb_x = 1.0;
    } else if (-rtb_Selector[2] < -1.0) {
      rtb_x = -1.0;
    } else {
      rtb_x = -rtb_Selector[2];
    }

    /* Trigonometry: '<S129>/Asin' */
    sm_vehicle_vtk_B->Asin_o = asin(rtb_x);

    /* Math: '<S130>/Transpose' incorporates:
     *  Inport: '<Root>/u'
     */
    for (i = 0; i < 4; i++) {
      rtb_T_tmp = i << 2;
      rtb_T[rtb_T_tmp] = sm_vehicle_vtk_U->u.T_RR[i];
      rtb_T[rtb_T_tmp + 1] = sm_vehicle_vtk_U->u.T_RR[i + 4];
      rtb_T[rtb_T_tmp + 2] = sm_vehicle_vtk_U->u.T_RR[i + 8];
      rtb_T[rtb_T_tmp + 3] = sm_vehicle_vtk_U->u.T_RR[i + 12];
    }

    /* End of Math: '<S130>/Transpose' */
    for (i = 0; i < 3; i++) {
      /* Selector: '<S130>/Selector' incorporates:
       *  Math: '<S130>/Transpose'
       */
      rtb_T_tmp = i << 2;
      rtb_Selector[3 * i] = rtb_T[rtb_T_tmp];
      rtb_Selector[3 * i + 1] = rtb_T[rtb_T_tmp + 1];
      rtb_Selector[3 * i + 2] = rtb_T[rtb_T_tmp + 2];

      /* Selector: '<S130>/Selector1' incorporates:
       *  Math: '<S130>/Transpose'
       */
      sm_vehicle_vtk_B->Selector1_l[i] = rtb_T[i + 12];
    }

    /* Trigonometry: '<S131>/Atan2' incorporates:
     *  Selector: '<S130>/Selector'
     *  Selector: '<S131>/R(3, 2:3)'
     */
    sm_vehicle_vtk_B->Atan2_h = rt_atan2d_snf(rtb_Selector[5], rtb_Selector[8]);

    /* Trigonometry: '<S131>/Atan1' incorporates:
     *  Selector: '<S130>/Selector'
     *  Selector: '<S131>/R([2 1], 1)'
     */
    sm_vehicle_vtk_B->Atan1_n = rt_atan2d_snf(rtb_Selector[1], rtb_Selector[0]);

    /* Trigonometry: '<S131>/Asin' incorporates:
     *  Gain: '<S131>/Multiply2'
     *  Selector: '<S130>/Selector'
     *  Selector: '<S131>/R(3, 1)'
     */
    if (-rtb_Selector[2] > 1.0) {
      rtb_x = 1.0;
    } else if (-rtb_Selector[2] < -1.0) {
      rtb_x = -1.0;
    } else {
      rtb_x = -rtb_Selector[2];
    }

    /* Trigonometry: '<S131>/Asin' */
    sm_vehicle_vtk_B->Asin_f = asin(rtb_x);
  }

  if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
    NeslSimulationData *simulationData;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    char *msg;
    real_T tmp_0[120];
    real_T time;
    int32_T tmp_2;
    int_T tmp_1[31];
    boolean_T tmp;

    /* Update for SimscapeExecutionBlock: '<S279>/STATE_1' */
    simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->STATE_1_SimData;
    time = sm_vehicle_vtk_M->Timing.t[0];
    simulationData->mData->mTime.mN = 1;
    simulationData->mData->mTime.mX = &time;
    simulationData->mData->mContStates.mN = 29;
    simulationData->mData->mContStates.mX =
      &sm_vehicle_vtk_X->sm_vehicle_vtkVehicleCamera_Fra[0];
    simulationData->mData->mDiscStates.mN = 0;
    simulationData->mData->mDiscStates.mX = &sm_vehicle_vtk_DW->STATE_1_Discrete;
    simulationData->mData->mModeVector.mN = 0;
    simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->STATE_1_Modes;
    tmp = false;
    simulationData->mData->mFoundZcEvents = tmp;
    simulationData->mData->mHadEvents = false;
    simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep
      (sm_vehicle_vtk_M);
    tmp = false;
    simulationData->mData->mIsSolverAssertCheck = tmp;
    simulationData->mData->mIsSolverCheckingCIC = false;
    tmp = rtsiIsSolverComputingJacobian(&sm_vehicle_vtk_M->solverInfo);
    simulationData->mData->mIsComputingJacobian = tmp;
    simulationData->mData->mIsEvaluatingF0 = false;
    simulationData->mData->mIsSolverRequestingReset = false;
    simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
      (&sm_vehicle_vtk_M->solverInfo);
    tmp_1[0] = 0;
    tmp_0[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
    tmp_0[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
    tmp_0[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
    tmp_0[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
    tmp_1[1] = 4;
    tmp_0[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
    tmp_0[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
    tmp_0[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
    tmp_0[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
    tmp_1[2] = 8;
    tmp_0[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
    tmp_0[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
    tmp_0[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
    tmp_0[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
    tmp_1[3] = 12;
    tmp_0[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
    tmp_0[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
    tmp_0[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
    tmp_0[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
    tmp_1[4] = 16;
    tmp_0[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
    tmp_0[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
    tmp_0[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
    tmp_0[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
    tmp_1[5] = 20;
    tmp_0[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
    tmp_0[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
    tmp_0[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
    tmp_0[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
    tmp_1[6] = 24;
    tmp_0[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
    tmp_0[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
    tmp_0[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
    tmp_0[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
    tmp_1[7] = 28;
    tmp_0[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
    tmp_0[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
    tmp_0[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
    tmp_0[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
    tmp_1[8] = 32;
    tmp_0[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
    tmp_0[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
    tmp_0[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
    tmp_0[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
    tmp_1[9] = 36;
    tmp_0[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
    tmp_0[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
    tmp_0[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
    tmp_0[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
    tmp_1[10] = 40;
    tmp_0[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
    tmp_0[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
    tmp_0[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
    tmp_0[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
    tmp_1[11] = 44;
    tmp_0[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
    tmp_0[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
    tmp_0[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
    tmp_0[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
    tmp_1[12] = 48;
    tmp_0[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
    tmp_0[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
    tmp_0[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
    tmp_0[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
    tmp_1[13] = 52;
    tmp_0[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
    tmp_0[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
    tmp_0[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
    tmp_0[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
    tmp_1[14] = 56;
    tmp_0[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
    tmp_0[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
    tmp_0[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
    tmp_0[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
    tmp_1[15] = 60;
    tmp_0[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
    tmp_0[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
    tmp_0[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
    tmp_0[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
    tmp_1[16] = 64;
    tmp_0[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
    tmp_0[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
    tmp_0[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
    tmp_0[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
    tmp_1[17] = 68;
    tmp_0[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
    tmp_0[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
    tmp_0[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
    tmp_0[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
    tmp_1[18] = 72;
    tmp_0[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
    tmp_0[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
    tmp_0[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
    tmp_0[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
    tmp_1[19] = 76;
    tmp_0[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
    tmp_0[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
    tmp_0[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
    tmp_0[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
    tmp_1[20] = 80;
    tmp_0[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
    tmp_0[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
    tmp_0[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
    tmp_0[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
    tmp_1[21] = 84;
    tmp_0[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
    tmp_0[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
    tmp_0[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
    tmp_0[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
    tmp_1[22] = 88;
    tmp_0[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
    tmp_0[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
    tmp_0[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
    tmp_0[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
    tmp_1[23] = 92;
    tmp_0[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
    tmp_0[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
    tmp_0[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
    tmp_0[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
    tmp_1[24] = 96;
    tmp_0[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
    tmp_0[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
    tmp_0[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
    tmp_0[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
    tmp_1[25] = 100;
    tmp_0[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
    tmp_0[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
    tmp_0[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
    tmp_0[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
    tmp_1[26] = 104;
    tmp_0[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
    tmp_0[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
    tmp_0[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
    tmp_0[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
    tmp_1[27] = 108;
    tmp_0[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
    tmp_0[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
    tmp_0[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
    tmp_0[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
    tmp_1[28] = 112;
    tmp_0[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
    tmp_0[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
    tmp_0[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
    tmp_0[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
    tmp_1[29] = 116;
    tmp_0[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
    tmp_0[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
    tmp_0[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
    tmp_0[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
    tmp_1[30] = 120;
    simulationData->mData->mInputValues.mN = 120;
    simulationData->mData->mInputValues.mX = &tmp_0[0];
    simulationData->mData->mInputOffsets.mN = 31;
    simulationData->mData->mInputOffsets.mX = &tmp_1[0];
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_vtk_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_2 = ne_simulator_method((NeslSimulator *)
      sm_vehicle_vtk_DW->STATE_1_Simulator, NESL_SIM_UPDATE, simulationData,
      diagnosticManager);
    if (tmp_2 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg);
      }
    }

    /* End of Update for SimscapeExecutionBlock: '<S279>/STATE_1' */
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(sm_vehicle_vtk_M)) {
    rt_ertODEUpdateContinuousStates(&sm_vehicle_vtk_M->solverInfo,
      sm_vehicle_vtk_M, sm_vehicle_vtk_U, sm_vehicle_vtk_Y);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++sm_vehicle_vtk_M->Timing.clockTick0;
    sm_vehicle_vtk_M->Timing.t[0] = rtsiGetSolverStopTime
      (&sm_vehicle_vtk_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      sm_vehicle_vtk_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void sm_vehicle_vtk_derivatives(RT_MODEL_sm_vehicle_vtk_T *const
  sm_vehicle_vtk_M)
{
  B_sm_vehicle_vtk_T *sm_vehicle_vtk_B = sm_vehicle_vtk_M->blockIO;
  DW_sm_vehicle_vtk_T *sm_vehicle_vtk_DW = sm_vehicle_vtk_M->dwork;
  X_sm_vehicle_vtk_T *sm_vehicle_vtk_X = sm_vehicle_vtk_M->contStates;
  NeslSimulationData *simulationData;
  NeuDiagnosticManager *diagnosticManager;
  NeuDiagnosticTree *diagnosticTree;
  XDot_sm_vehicle_vtk_T *_rtXdot;
  char *msg;
  real_T tmp_0[120];
  real_T time;
  int32_T tmp_2;
  int_T tmp_1[31];
  boolean_T tmp;
  _rtXdot = ((XDot_sm_vehicle_vtk_T *) sm_vehicle_vtk_M->derivs);

  /* Derivatives for SimscapeInputBlock: '<S279>/INPUT_6_1_1' */
  _rtXdot->sm_vehicle_vtkVehicleSuspension[0] =
    sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[1];
  _rtXdot->sm_vehicle_vtkVehicleSuspension[1] = ((sm_vehicle_vtk_B->Ackermanleft
    - sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[0]) * 181.81818181818184
    - 2.0 * sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspension[1]) *
    181.81818181818184;

  /* Derivatives for SimscapeInputBlock: '<S279>/INPUT_5_1_1' */
  _rtXdot->sm_vehicle_vtkVehicleSuspensi_i[0] =
    sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[1];
  _rtXdot->sm_vehicle_vtkVehicleSuspensi_i[1] =
    ((sm_vehicle_vtk_B->Ackermanright -
      sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[0]) * 181.81818181818184
     - 2.0 * sm_vehicle_vtk_X->sm_vehicle_vtkVehicleSuspensi_i[1]) *
    181.81818181818184;

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE = 0.0;
  _rtXdot->TransferFcn_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += sm_vehicle_vtk_B->Selector1[0];

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += sm_vehicle_vtk_B->Selector1[1];

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE = 0.0;
  _rtXdot->TransferFcn2_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += sm_vehicle_vtk_B->Selector1[2];

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE = 0.0;
  _rtXdot->TransferFcn3_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn3_CSTATE;
  _rtXdot->TransferFcn3_CSTATE += sm_vehicle_vtk_B->Atan2_e;

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn5' */
  _rtXdot->TransferFcn5_CSTATE = 0.0;
  _rtXdot->TransferFcn5_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn5_CSTATE;
  _rtXdot->TransferFcn5_CSTATE += sm_vehicle_vtk_B->Atan1_b;

  /* Derivatives for TransferFcn: '<S124>/Transfer Fcn4' */
  _rtXdot->TransferFcn4_CSTATE = 0.0;
  _rtXdot->TransferFcn4_CSTATE += -100.0 * sm_vehicle_vtk_X->TransferFcn4_CSTATE;
  _rtXdot->TransferFcn4_CSTATE += sm_vehicle_vtk_B->Asin_d;

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_p = 0.0;
  _rtXdot->TransferFcn_CSTATE_p += -100.0 *
    sm_vehicle_vtk_X->TransferFcn_CSTATE_p;
  _rtXdot->TransferFcn_CSTATE_p += sm_vehicle_vtk_B->Selector1_i[0];

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE_o = 0.0;
  _rtXdot->TransferFcn1_CSTATE_o += -100.0 *
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_o;
  _rtXdot->TransferFcn1_CSTATE_o += sm_vehicle_vtk_B->Selector1_i[1];

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE_j = 0.0;
  _rtXdot->TransferFcn2_CSTATE_j += -100.0 *
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_j;
  _rtXdot->TransferFcn2_CSTATE_j += sm_vehicle_vtk_B->Selector1_i[2];

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE_o = 0.0;
  _rtXdot->TransferFcn3_CSTATE_o += -100.0 *
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_o;
  _rtXdot->TransferFcn3_CSTATE_o += sm_vehicle_vtk_B->Atan2_n;

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn5' */
  _rtXdot->TransferFcn5_CSTATE_j = 0.0;
  _rtXdot->TransferFcn5_CSTATE_j += -100.0 *
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_j;
  _rtXdot->TransferFcn5_CSTATE_j += sm_vehicle_vtk_B->Atan1_i;

  /* Derivatives for TransferFcn: '<S126>/Transfer Fcn4' */
  _rtXdot->TransferFcn4_CSTATE_n = 0.0;
  _rtXdot->TransferFcn4_CSTATE_n += -100.0 *
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_n;
  _rtXdot->TransferFcn4_CSTATE_n += sm_vehicle_vtk_B->Asin_j;

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_e = 0.0;
  _rtXdot->TransferFcn_CSTATE_e += -100.0 *
    sm_vehicle_vtk_X->TransferFcn_CSTATE_e;
  _rtXdot->TransferFcn_CSTATE_e += sm_vehicle_vtk_B->Selector1_k[0];

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE_d = 0.0;
  _rtXdot->TransferFcn1_CSTATE_d += -100.0 *
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_d;
  _rtXdot->TransferFcn1_CSTATE_d += sm_vehicle_vtk_B->Selector1_k[1];

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE_c = 0.0;
  _rtXdot->TransferFcn2_CSTATE_c += -100.0 *
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_c;
  _rtXdot->TransferFcn2_CSTATE_c += sm_vehicle_vtk_B->Selector1_k[2];

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE_j = 0.0;
  _rtXdot->TransferFcn3_CSTATE_j += -100.0 *
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_j;
  _rtXdot->TransferFcn3_CSTATE_j += sm_vehicle_vtk_B->Atan2_k;

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn5' */
  _rtXdot->TransferFcn5_CSTATE_b = 0.0;
  _rtXdot->TransferFcn5_CSTATE_b += -100.0 *
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_b;
  _rtXdot->TransferFcn5_CSTATE_b += sm_vehicle_vtk_B->Atan1_g;

  /* Derivatives for TransferFcn: '<S128>/Transfer Fcn4' */
  _rtXdot->TransferFcn4_CSTATE_h = 0.0;
  _rtXdot->TransferFcn4_CSTATE_h += -100.0 *
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_h;
  _rtXdot->TransferFcn4_CSTATE_h += sm_vehicle_vtk_B->Asin_o;

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_n = 0.0;
  _rtXdot->TransferFcn_CSTATE_n += -100.0 *
    sm_vehicle_vtk_X->TransferFcn_CSTATE_n;
  _rtXdot->TransferFcn_CSTATE_n += sm_vehicle_vtk_B->Selector1_l[0];

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE_k = 0.0;
  _rtXdot->TransferFcn1_CSTATE_k += -100.0 *
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_k;
  _rtXdot->TransferFcn1_CSTATE_k += sm_vehicle_vtk_B->Selector1_l[1];

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn2' */
  _rtXdot->TransferFcn2_CSTATE_cy = 0.0;
  _rtXdot->TransferFcn2_CSTATE_cy += -100.0 *
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_cy;
  _rtXdot->TransferFcn2_CSTATE_cy += sm_vehicle_vtk_B->Selector1_l[2];

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn3' */
  _rtXdot->TransferFcn3_CSTATE_f = 0.0;
  _rtXdot->TransferFcn3_CSTATE_f += -100.0 *
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_f;
  _rtXdot->TransferFcn3_CSTATE_f += sm_vehicle_vtk_B->Atan2_h;

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn5' */
  _rtXdot->TransferFcn5_CSTATE_f = 0.0;
  _rtXdot->TransferFcn5_CSTATE_f += -100.0 *
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_f;
  _rtXdot->TransferFcn5_CSTATE_f += sm_vehicle_vtk_B->Atan1_n;

  /* Derivatives for TransferFcn: '<S130>/Transfer Fcn4' */
  _rtXdot->TransferFcn4_CSTATE_g = 0.0;
  _rtXdot->TransferFcn4_CSTATE_g += -100.0 *
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_g;
  _rtXdot->TransferFcn4_CSTATE_g += sm_vehicle_vtk_B->Asin_f;

  /* Derivatives for SimscapeExecutionBlock: '<S279>/STATE_1' */
  simulationData = (NeslSimulationData *)sm_vehicle_vtk_DW->STATE_1_SimData;
  time = sm_vehicle_vtk_M->Timing.t[0];
  simulationData->mData->mTime.mN = 1;
  simulationData->mData->mTime.mX = &time;
  simulationData->mData->mContStates.mN = 29;
  simulationData->mData->mContStates.mX =
    &sm_vehicle_vtk_X->sm_vehicle_vtkVehicleCamera_Fra[0];
  simulationData->mData->mDiscStates.mN = 0;
  simulationData->mData->mDiscStates.mX = &sm_vehicle_vtk_DW->STATE_1_Discrete;
  simulationData->mData->mModeVector.mN = 0;
  simulationData->mData->mModeVector.mX = &sm_vehicle_vtk_DW->STATE_1_Modes;
  tmp = false;
  simulationData->mData->mFoundZcEvents = tmp;
  simulationData->mData->mHadEvents = false;
  simulationData->mData->mIsMajorTimeStep = rtmIsMajorTimeStep(sm_vehicle_vtk_M);
  tmp = false;
  simulationData->mData->mIsSolverAssertCheck = tmp;
  simulationData->mData->mIsSolverCheckingCIC = false;
  tmp = rtsiIsSolverComputingJacobian(&sm_vehicle_vtk_M->solverInfo);
  simulationData->mData->mIsComputingJacobian = tmp;
  simulationData->mData->mIsEvaluatingF0 = false;
  simulationData->mData->mIsSolverRequestingReset = false;
  simulationData->mData->mIsModeUpdateTimeStep = rtsiIsModeUpdateTimeStep
    (&sm_vehicle_vtk_M->solverInfo);
  tmp_1[0] = 0;
  tmp_0[0] = sm_vehicle_vtk_B->INPUT_4_1_1[0];
  tmp_0[1] = sm_vehicle_vtk_B->INPUT_4_1_1[1];
  tmp_0[2] = sm_vehicle_vtk_B->INPUT_4_1_1[2];
  tmp_0[3] = sm_vehicle_vtk_B->INPUT_4_1_1[3];
  tmp_1[1] = 4;
  tmp_0[4] = sm_vehicle_vtk_B->INPUT_1_1_1[0];
  tmp_0[5] = sm_vehicle_vtk_B->INPUT_1_1_1[1];
  tmp_0[6] = sm_vehicle_vtk_B->INPUT_1_1_1[2];
  tmp_0[7] = sm_vehicle_vtk_B->INPUT_1_1_1[3];
  tmp_1[2] = 8;
  tmp_0[8] = sm_vehicle_vtk_B->INPUT_3_1_1[0];
  tmp_0[9] = sm_vehicle_vtk_B->INPUT_3_1_1[1];
  tmp_0[10] = sm_vehicle_vtk_B->INPUT_3_1_1[2];
  tmp_0[11] = sm_vehicle_vtk_B->INPUT_3_1_1[3];
  tmp_1[3] = 12;
  tmp_0[12] = sm_vehicle_vtk_B->INPUT_2_1_1[0];
  tmp_0[13] = sm_vehicle_vtk_B->INPUT_2_1_1[1];
  tmp_0[14] = sm_vehicle_vtk_B->INPUT_2_1_1[2];
  tmp_0[15] = sm_vehicle_vtk_B->INPUT_2_1_1[3];
  tmp_1[4] = 16;
  tmp_0[16] = sm_vehicle_vtk_B->INPUT_6_1_1[0];
  tmp_0[17] = sm_vehicle_vtk_B->INPUT_6_1_1[1];
  tmp_0[18] = sm_vehicle_vtk_B->INPUT_6_1_1[2];
  tmp_0[19] = sm_vehicle_vtk_B->INPUT_6_1_1[3];
  tmp_1[5] = 20;
  tmp_0[20] = sm_vehicle_vtk_B->INPUT_5_1_1[0];
  tmp_0[21] = sm_vehicle_vtk_B->INPUT_5_1_1[1];
  tmp_0[22] = sm_vehicle_vtk_B->INPUT_5_1_1[2];
  tmp_0[23] = sm_vehicle_vtk_B->INPUT_5_1_1[3];
  tmp_1[6] = 24;
  tmp_0[24] = sm_vehicle_vtk_B->INPUT_7_1_1[0];
  tmp_0[25] = sm_vehicle_vtk_B->INPUT_7_1_1[1];
  tmp_0[26] = sm_vehicle_vtk_B->INPUT_7_1_1[2];
  tmp_0[27] = sm_vehicle_vtk_B->INPUT_7_1_1[3];
  tmp_1[7] = 28;
  tmp_0[28] = sm_vehicle_vtk_B->INPUT_8_1_1[0];
  tmp_0[29] = sm_vehicle_vtk_B->INPUT_8_1_1[1];
  tmp_0[30] = sm_vehicle_vtk_B->INPUT_8_1_1[2];
  tmp_0[31] = sm_vehicle_vtk_B->INPUT_8_1_1[3];
  tmp_1[8] = 32;
  tmp_0[32] = sm_vehicle_vtk_B->INPUT_9_1_1[0];
  tmp_0[33] = sm_vehicle_vtk_B->INPUT_9_1_1[1];
  tmp_0[34] = sm_vehicle_vtk_B->INPUT_9_1_1[2];
  tmp_0[35] = sm_vehicle_vtk_B->INPUT_9_1_1[3];
  tmp_1[9] = 36;
  tmp_0[36] = sm_vehicle_vtk_B->INPUT_12_1_1[0];
  tmp_0[37] = sm_vehicle_vtk_B->INPUT_12_1_1[1];
  tmp_0[38] = sm_vehicle_vtk_B->INPUT_12_1_1[2];
  tmp_0[39] = sm_vehicle_vtk_B->INPUT_12_1_1[3];
  tmp_1[10] = 40;
  tmp_0[40] = sm_vehicle_vtk_B->INPUT_10_1_1[0];
  tmp_0[41] = sm_vehicle_vtk_B->INPUT_10_1_1[1];
  tmp_0[42] = sm_vehicle_vtk_B->INPUT_10_1_1[2];
  tmp_0[43] = sm_vehicle_vtk_B->INPUT_10_1_1[3];
  tmp_1[11] = 44;
  tmp_0[44] = sm_vehicle_vtk_B->INPUT_11_1_1[0];
  tmp_0[45] = sm_vehicle_vtk_B->INPUT_11_1_1[1];
  tmp_0[46] = sm_vehicle_vtk_B->INPUT_11_1_1[2];
  tmp_0[47] = sm_vehicle_vtk_B->INPUT_11_1_1[3];
  tmp_1[12] = 48;
  tmp_0[48] = sm_vehicle_vtk_B->INPUT_13_1_1[0];
  tmp_0[49] = sm_vehicle_vtk_B->INPUT_13_1_1[1];
  tmp_0[50] = sm_vehicle_vtk_B->INPUT_13_1_1[2];
  tmp_0[51] = sm_vehicle_vtk_B->INPUT_13_1_1[3];
  tmp_1[13] = 52;
  tmp_0[52] = sm_vehicle_vtk_B->INPUT_14_1_1[0];
  tmp_0[53] = sm_vehicle_vtk_B->INPUT_14_1_1[1];
  tmp_0[54] = sm_vehicle_vtk_B->INPUT_14_1_1[2];
  tmp_0[55] = sm_vehicle_vtk_B->INPUT_14_1_1[3];
  tmp_1[14] = 56;
  tmp_0[56] = sm_vehicle_vtk_B->INPUT_15_1_1[0];
  tmp_0[57] = sm_vehicle_vtk_B->INPUT_15_1_1[1];
  tmp_0[58] = sm_vehicle_vtk_B->INPUT_15_1_1[2];
  tmp_0[59] = sm_vehicle_vtk_B->INPUT_15_1_1[3];
  tmp_1[15] = 60;
  tmp_0[60] = sm_vehicle_vtk_B->INPUT_18_1_1[0];
  tmp_0[61] = sm_vehicle_vtk_B->INPUT_18_1_1[1];
  tmp_0[62] = sm_vehicle_vtk_B->INPUT_18_1_1[2];
  tmp_0[63] = sm_vehicle_vtk_B->INPUT_18_1_1[3];
  tmp_1[16] = 64;
  tmp_0[64] = sm_vehicle_vtk_B->INPUT_16_1_1[0];
  tmp_0[65] = sm_vehicle_vtk_B->INPUT_16_1_1[1];
  tmp_0[66] = sm_vehicle_vtk_B->INPUT_16_1_1[2];
  tmp_0[67] = sm_vehicle_vtk_B->INPUT_16_1_1[3];
  tmp_1[17] = 68;
  tmp_0[68] = sm_vehicle_vtk_B->INPUT_17_1_1[0];
  tmp_0[69] = sm_vehicle_vtk_B->INPUT_17_1_1[1];
  tmp_0[70] = sm_vehicle_vtk_B->INPUT_17_1_1[2];
  tmp_0[71] = sm_vehicle_vtk_B->INPUT_17_1_1[3];
  tmp_1[18] = 72;
  tmp_0[72] = sm_vehicle_vtk_B->INPUT_19_1_1[0];
  tmp_0[73] = sm_vehicle_vtk_B->INPUT_19_1_1[1];
  tmp_0[74] = sm_vehicle_vtk_B->INPUT_19_1_1[2];
  tmp_0[75] = sm_vehicle_vtk_B->INPUT_19_1_1[3];
  tmp_1[19] = 76;
  tmp_0[76] = sm_vehicle_vtk_B->INPUT_20_1_1[0];
  tmp_0[77] = sm_vehicle_vtk_B->INPUT_20_1_1[1];
  tmp_0[78] = sm_vehicle_vtk_B->INPUT_20_1_1[2];
  tmp_0[79] = sm_vehicle_vtk_B->INPUT_20_1_1[3];
  tmp_1[20] = 80;
  tmp_0[80] = sm_vehicle_vtk_B->INPUT_21_1_1[0];
  tmp_0[81] = sm_vehicle_vtk_B->INPUT_21_1_1[1];
  tmp_0[82] = sm_vehicle_vtk_B->INPUT_21_1_1[2];
  tmp_0[83] = sm_vehicle_vtk_B->INPUT_21_1_1[3];
  tmp_1[21] = 84;
  tmp_0[84] = sm_vehicle_vtk_B->INPUT_24_1_1[0];
  tmp_0[85] = sm_vehicle_vtk_B->INPUT_24_1_1[1];
  tmp_0[86] = sm_vehicle_vtk_B->INPUT_24_1_1[2];
  tmp_0[87] = sm_vehicle_vtk_B->INPUT_24_1_1[3];
  tmp_1[22] = 88;
  tmp_0[88] = sm_vehicle_vtk_B->INPUT_22_1_1[0];
  tmp_0[89] = sm_vehicle_vtk_B->INPUT_22_1_1[1];
  tmp_0[90] = sm_vehicle_vtk_B->INPUT_22_1_1[2];
  tmp_0[91] = sm_vehicle_vtk_B->INPUT_22_1_1[3];
  tmp_1[23] = 92;
  tmp_0[92] = sm_vehicle_vtk_B->INPUT_23_1_1[0];
  tmp_0[93] = sm_vehicle_vtk_B->INPUT_23_1_1[1];
  tmp_0[94] = sm_vehicle_vtk_B->INPUT_23_1_1[2];
  tmp_0[95] = sm_vehicle_vtk_B->INPUT_23_1_1[3];
  tmp_1[24] = 96;
  tmp_0[96] = sm_vehicle_vtk_B->INPUT_25_1_1[0];
  tmp_0[97] = sm_vehicle_vtk_B->INPUT_25_1_1[1];
  tmp_0[98] = sm_vehicle_vtk_B->INPUT_25_1_1[2];
  tmp_0[99] = sm_vehicle_vtk_B->INPUT_25_1_1[3];
  tmp_1[25] = 100;
  tmp_0[100] = sm_vehicle_vtk_B->INPUT_26_1_1[0];
  tmp_0[101] = sm_vehicle_vtk_B->INPUT_26_1_1[1];
  tmp_0[102] = sm_vehicle_vtk_B->INPUT_26_1_1[2];
  tmp_0[103] = sm_vehicle_vtk_B->INPUT_26_1_1[3];
  tmp_1[26] = 104;
  tmp_0[104] = sm_vehicle_vtk_B->INPUT_27_1_1[0];
  tmp_0[105] = sm_vehicle_vtk_B->INPUT_27_1_1[1];
  tmp_0[106] = sm_vehicle_vtk_B->INPUT_27_1_1[2];
  tmp_0[107] = sm_vehicle_vtk_B->INPUT_27_1_1[3];
  tmp_1[27] = 108;
  tmp_0[108] = sm_vehicle_vtk_B->INPUT_30_1_1[0];
  tmp_0[109] = sm_vehicle_vtk_B->INPUT_30_1_1[1];
  tmp_0[110] = sm_vehicle_vtk_B->INPUT_30_1_1[2];
  tmp_0[111] = sm_vehicle_vtk_B->INPUT_30_1_1[3];
  tmp_1[28] = 112;
  tmp_0[112] = sm_vehicle_vtk_B->INPUT_28_1_1[0];
  tmp_0[113] = sm_vehicle_vtk_B->INPUT_28_1_1[1];
  tmp_0[114] = sm_vehicle_vtk_B->INPUT_28_1_1[2];
  tmp_0[115] = sm_vehicle_vtk_B->INPUT_28_1_1[3];
  tmp_1[29] = 116;
  tmp_0[116] = sm_vehicle_vtk_B->INPUT_29_1_1[0];
  tmp_0[117] = sm_vehicle_vtk_B->INPUT_29_1_1[1];
  tmp_0[118] = sm_vehicle_vtk_B->INPUT_29_1_1[2];
  tmp_0[119] = sm_vehicle_vtk_B->INPUT_29_1_1[3];
  tmp_1[30] = 120;
  simulationData->mData->mInputValues.mN = 120;
  simulationData->mData->mInputValues.mX = &tmp_0[0];
  simulationData->mData->mInputOffsets.mN = 31;
  simulationData->mData->mInputOffsets.mX = &tmp_1[0];
  simulationData->mData->mDx.mN = 29;
  simulationData->mData->mDx.mX = &_rtXdot->sm_vehicle_vtkVehicleCamera_Fra[0];
  diagnosticManager = (NeuDiagnosticManager *)sm_vehicle_vtk_DW->STATE_1_DiagMgr;
  diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
  tmp_2 = ne_simulator_method((NeslSimulator *)
    sm_vehicle_vtk_DW->STATE_1_Simulator, NESL_SIM_DERIVATIVES, simulationData,
    diagnosticManager);
  if (tmp_2 != 0) {
    tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
    if (tmp) {
      msg = rtw_diagnostics_msg(diagnosticTree);
      rtmSetErrorStatus(sm_vehicle_vtk_M, msg);
    }
  }

  /* End of Derivatives for SimscapeExecutionBlock: '<S279>/STATE_1' */

  /* Derivatives for TransferFcn: '<S41>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE_b = 0.0;
  _rtXdot->TransferFcn_CSTATE_b += -100.0 *
    sm_vehicle_vtk_X->TransferFcn_CSTATE_b;
  _rtXdot->TransferFcn_CSTATE_b += sm_vehicle_vtk_B->TrigonometricFunction1;

  /* Derivatives for TransferFcn: '<S41>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE_a = 0.0;
  _rtXdot->TransferFcn1_CSTATE_a += -100.0 *
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_a;
  _rtXdot->TransferFcn1_CSTATE_a += sm_vehicle_vtk_B->Flipsignforxaxis;
}

/* Model initialize function */
void sm_vehicle_vtk_initialize(RT_MODEL_sm_vehicle_vtk_T *const sm_vehicle_vtk_M,
  ExtU_sm_vehicle_vtk_T *sm_vehicle_vtk_U, ExtY_sm_vehicle_vtk_T
  *sm_vehicle_vtk_Y)
{
  DW_sm_vehicle_vtk_T *sm_vehicle_vtk_DW = sm_vehicle_vtk_M->dwork;
  X_sm_vehicle_vtk_T *sm_vehicle_vtk_X = sm_vehicle_vtk_M->contStates;
  B_sm_vehicle_vtk_T *sm_vehicle_vtk_B = sm_vehicle_vtk_M->blockIO;
  XDis_sm_vehicle_vtk_T *sm_vehicle_vtk_XDis = ((XDis_sm_vehicle_vtk_T *)
    sm_vehicle_vtk_M->contStateDisabled);

  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&sm_vehicle_vtk_M->solverInfo,
                          &sm_vehicle_vtk_M->Timing.simTimeStep);
    rtsiSetTPtr(&sm_vehicle_vtk_M->solverInfo, &rtmGetTPtr(sm_vehicle_vtk_M));
    rtsiSetStepSizePtr(&sm_vehicle_vtk_M->solverInfo,
                       &sm_vehicle_vtk_M->Timing.stepSize0);
    rtsiSetdXPtr(&sm_vehicle_vtk_M->solverInfo, &sm_vehicle_vtk_M->derivs);
    rtsiSetContStatesPtr(&sm_vehicle_vtk_M->solverInfo, (real_T **)
                         &sm_vehicle_vtk_M->contStates);
    rtsiSetNumContStatesPtr(&sm_vehicle_vtk_M->solverInfo,
      &sm_vehicle_vtk_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&sm_vehicle_vtk_M->solverInfo,
      &sm_vehicle_vtk_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&sm_vehicle_vtk_M->solverInfo,
      &sm_vehicle_vtk_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&sm_vehicle_vtk_M->solverInfo,
      &sm_vehicle_vtk_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&sm_vehicle_vtk_M->solverInfo, (boolean_T**)
      &sm_vehicle_vtk_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&sm_vehicle_vtk_M->solverInfo, (&rtmGetErrorStatus
      (sm_vehicle_vtk_M)));
    rtsiSetRTModelPtr(&sm_vehicle_vtk_M->solverInfo, sm_vehicle_vtk_M);
  }

  rtsiSetSimTimeStep(&sm_vehicle_vtk_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&sm_vehicle_vtk_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&sm_vehicle_vtk_M->solverInfo, false);
  sm_vehicle_vtk_M->intgData.f[0] = sm_vehicle_vtk_M->odeF[0];
  sm_vehicle_vtk_M->contStates = ((X_sm_vehicle_vtk_T *) sm_vehicle_vtk_X);
  sm_vehicle_vtk_M->contStateDisabled = ((XDis_sm_vehicle_vtk_T *)
    sm_vehicle_vtk_XDis);
  sm_vehicle_vtk_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&sm_vehicle_vtk_M->solverInfo, (void *)
                    &sm_vehicle_vtk_M->intgData);
  rtsiSetSolverName(&sm_vehicle_vtk_M->solverInfo,"ode1");
  rtmSetTPtr(sm_vehicle_vtk_M, &sm_vehicle_vtk_M->Timing.tArray[0]);
  sm_vehicle_vtk_M->Timing.stepSize0 = 0.001;

  /* block I/O */
  (void) memset(((void *) sm_vehicle_vtk_B), 0,
                sizeof(B_sm_vehicle_vtk_T));

  /* states (continuous) */
  {
    (void) memset((void *)sm_vehicle_vtk_X, 0,
                  sizeof(X_sm_vehicle_vtk_T));
  }

  /* disabled states */
  {
    (void) memset((void *)sm_vehicle_vtk_XDis, 0,
                  sizeof(XDis_sm_vehicle_vtk_T));
  }

  /* states (dwork) */
  (void) memset((void *)sm_vehicle_vtk_DW, 0,
                sizeof(DW_sm_vehicle_vtk_T));

  /* external inputs */
  sm_vehicle_vtk_U->u = sm_vehicle_vtk_rtZsm_vehicle_vt;

  /* external outputs */
  (void)memset(sm_vehicle_vtk_Y, 0, sizeof(ExtY_sm_vehicle_vtk_T));

  {
    NeModelParameters modelParameters;
    NeModelParameters modelParameters_0;
    NeModelParameters modelParameters_1;
    NeslRtpManager *manager;
    NeslRtpManager *manager_0;
    NeslSimulationData *tmp_1;
    NeslSimulator *tmp_0;
    NeuDiagnosticManager *diagnosticManager;
    NeuDiagnosticTree *diagnosticTree;
    NeuDiagnosticTree *diagnosticTree_0;
    NeuDiagnosticTree *diagnosticTree_1;
    char *msg;
    char *msg_0;
    char *msg_1;
    real_T tmp_2;
    int32_T tmp_3;
    boolean_T tmp;

    /* Start for SimscapeRtp: '<S278>/RTP_1' */
    manager_0 = nesl_lease_rtp_manager(
      "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 0);
    manager = manager_0;
    tmp = pointer_is_null(manager_0);
    if (tmp) {
      sm_vehicle_vtk_836bb176_1_gateway();
      manager = nesl_lease_rtp_manager(
        "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 0);
    }

    sm_vehicle_vtk_DW->RTP_1_RtpManager = (void *)manager;
    sm_vehicle_vtk_DW->RTP_1_SetParametersNeeded = true;

    /* End of Start for SimscapeRtp: '<S278>/RTP_1' */

    /* Start for SimscapeExecutionBlock: '<S279>/STATE_1' */
    tmp_0 = nesl_lease_simulator(
      "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 0, 0);
    sm_vehicle_vtk_DW->STATE_1_Simulator = (void *)tmp_0;
    tmp = pointer_is_null(sm_vehicle_vtk_DW->STATE_1_Simulator);
    if (tmp) {
      sm_vehicle_vtk_836bb176_1_gateway();
      tmp_0 = nesl_lease_simulator(
        "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 0, 0);
      sm_vehicle_vtk_DW->STATE_1_Simulator = (void *)tmp_0;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_vtk_DW->STATE_1_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_vtk_DW->STATE_1_DiagMgr = (void *)diagnosticManager;
    modelParameters.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters.mSolverAbsTol = 0.001;
    modelParameters.mSolverRelTol = 0.001;
    modelParameters.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters.mStartTime = 0.0;
    modelParameters.mLoadInitialState = false;
    modelParameters.mUseSimState = false;
    modelParameters.mLinTrimCompile = false;
    modelParameters.mLoggingMode = SSC_LOGGING_ON;
    modelParameters.mRTWModifiedTimeStamp = 6.45998134E+8;
    modelParameters.mUseModelRefSolver = false;
    modelParameters.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters.mFixedStepSize = tmp_2;
    tmp = false;
    modelParameters.mVariableStepSolver = tmp;
    tmp = false;
    modelParameters.mIsUsingODEN = tmp;
    modelParameters.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_vtk_DW->STATE_1_DiagMgr;
    diagnosticTree = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_vtk_DW->STATE_1_Simulator, &modelParameters, diagnosticManager);
    if (tmp_3 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp) {
        msg = rtw_diagnostics_msg(diagnosticTree);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S279>/STATE_1' */

    /* Start for SimscapeExecutionBlock: '<S279>/OUTPUT_1_0' */
    tmp_0 = nesl_lease_simulator(
      "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 1, 0);
    sm_vehicle_vtk_DW->OUTPUT_1_0_Simulator = (void *)tmp_0;
    tmp = pointer_is_null(sm_vehicle_vtk_DW->OUTPUT_1_0_Simulator);
    if (tmp) {
      sm_vehicle_vtk_836bb176_1_gateway();
      tmp_0 = nesl_lease_simulator(
        "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 1, 0);
      sm_vehicle_vtk_DW->OUTPUT_1_0_Simulator = (void *)tmp_0;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_vtk_DW->OUTPUT_1_0_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_vtk_DW->OUTPUT_1_0_DiagMgr = (void *)diagnosticManager;
    modelParameters_0.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters_0.mSolverAbsTol = 0.001;
    modelParameters_0.mSolverRelTol = 0.001;
    modelParameters_0.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_0.mStartTime = 0.0;
    modelParameters_0.mLoadInitialState = false;
    modelParameters_0.mUseSimState = false;
    modelParameters_0.mLinTrimCompile = false;
    modelParameters_0.mLoggingMode = SSC_LOGGING_ON;
    modelParameters_0.mRTWModifiedTimeStamp = 6.45998134E+8;
    modelParameters_0.mUseModelRefSolver = false;
    modelParameters_0.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters_0.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters_0.mFixedStepSize = tmp_2;
    tmp = false;
    modelParameters_0.mVariableStepSolver = tmp;
    tmp = false;
    modelParameters_0.mIsUsingODEN = tmp;
    modelParameters_0.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_vtk_DW->OUTPUT_1_0_DiagMgr;
    diagnosticTree_0 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_vtk_DW->OUTPUT_1_0_Simulator, &modelParameters_0,
      diagnosticManager);
    if (tmp_3 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp) {
        msg_0 = rtw_diagnostics_msg(diagnosticTree_0);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg_0);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S279>/OUTPUT_1_0' */

    /* Start for SimscapeExecutionBlock: '<S279>/OUTPUT_1_1' */
    tmp_0 = nesl_lease_simulator(
      "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 1, 1);
    sm_vehicle_vtk_DW->OUTPUT_1_1_Simulator = (void *)tmp_0;
    tmp = pointer_is_null(sm_vehicle_vtk_DW->OUTPUT_1_1_Simulator);
    if (tmp) {
      sm_vehicle_vtk_836bb176_1_gateway();
      tmp_0 = nesl_lease_simulator(
        "sm_vehicle_vtk/Vehicle/World/Solver Configuration_1", 1, 1);
      sm_vehicle_vtk_DW->OUTPUT_1_1_Simulator = (void *)tmp_0;
    }

    tmp_1 = nesl_create_simulation_data();
    sm_vehicle_vtk_DW->OUTPUT_1_1_SimData = (void *)tmp_1;
    diagnosticManager = rtw_create_diagnostics();
    sm_vehicle_vtk_DW->OUTPUT_1_1_DiagMgr = (void *)diagnosticManager;
    modelParameters_1.mSolverType = NE_SOLVER_TYPE_ODE;
    modelParameters_1.mSolverAbsTol = 0.001;
    modelParameters_1.mSolverRelTol = 0.001;
    modelParameters_1.mSolverModifyAbsTol = NE_MODIFY_ABS_TOL_NO;
    modelParameters_1.mStartTime = 0.0;
    modelParameters_1.mLoadInitialState = false;
    modelParameters_1.mUseSimState = false;
    modelParameters_1.mLinTrimCompile = false;
    modelParameters_1.mLoggingMode = SSC_LOGGING_ON;
    modelParameters_1.mRTWModifiedTimeStamp = 6.45998134E+8;
    modelParameters_1.mUseModelRefSolver = false;
    modelParameters_1.mTargetFPGAHIL = false;
    tmp_2 = 0.001;
    modelParameters_1.mSolverTolerance = tmp_2;
    tmp_2 = 0.001;
    modelParameters_1.mFixedStepSize = tmp_2;
    tmp = false;
    modelParameters_1.mVariableStepSolver = tmp;
    tmp = false;
    modelParameters_1.mIsUsingODEN = tmp;
    modelParameters_1.mZcDisabled = true;
    diagnosticManager = (NeuDiagnosticManager *)
      sm_vehicle_vtk_DW->OUTPUT_1_1_DiagMgr;
    diagnosticTree_1 = neu_diagnostic_manager_get_initial_tree(diagnosticManager);
    tmp_3 = nesl_initialize_simulator((NeslSimulator *)
      sm_vehicle_vtk_DW->OUTPUT_1_1_Simulator, &modelParameters_1,
      diagnosticManager);
    if (tmp_3 != 0) {
      tmp = error_buffer_is_empty(rtmGetErrorStatus(sm_vehicle_vtk_M));
      if (tmp) {
        msg_1 = rtw_diagnostics_msg(diagnosticTree_1);
        rtmSetErrorStatus(sm_vehicle_vtk_M, msg_1);
      }
    }

    /* End of Start for SimscapeExecutionBlock: '<S279>/OUTPUT_1_1' */

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn' */
    sm_vehicle_vtk_X->TransferFcn_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn1' */
    sm_vehicle_vtk_X->TransferFcn1_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn2' */
    sm_vehicle_vtk_X->TransferFcn2_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn3' */
    sm_vehicle_vtk_X->TransferFcn3_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn5' */
    sm_vehicle_vtk_X->TransferFcn5_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S124>/Transfer Fcn4' */
    sm_vehicle_vtk_X->TransferFcn4_CSTATE = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn' */
    sm_vehicle_vtk_X->TransferFcn_CSTATE_p = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn1' */
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_o = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn2' */
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_j = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn3' */
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_o = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn5' */
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_j = 0.0;

    /* InitializeConditions for TransferFcn: '<S126>/Transfer Fcn4' */
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_n = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn' */
    sm_vehicle_vtk_X->TransferFcn_CSTATE_e = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn1' */
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_d = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn2' */
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_c = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn3' */
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_j = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn5' */
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_b = 0.0;

    /* InitializeConditions for TransferFcn: '<S128>/Transfer Fcn4' */
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_h = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn' */
    sm_vehicle_vtk_X->TransferFcn_CSTATE_n = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn1' */
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_k = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn2' */
    sm_vehicle_vtk_X->TransferFcn2_CSTATE_cy = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn3' */
    sm_vehicle_vtk_X->TransferFcn3_CSTATE_f = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn5' */
    sm_vehicle_vtk_X->TransferFcn5_CSTATE_f = 0.0;

    /* InitializeConditions for TransferFcn: '<S130>/Transfer Fcn4' */
    sm_vehicle_vtk_X->TransferFcn4_CSTATE_g = 0.0;

    /* InitializeConditions for TransferFcn: '<S41>/Transfer Fcn' */
    sm_vehicle_vtk_X->TransferFcn_CSTATE_b = 0.0;

    /* InitializeConditions for TransferFcn: '<S41>/Transfer Fcn1' */
    sm_vehicle_vtk_X->TransferFcn1_CSTATE_a = 0.0;

    /* ConstCode for Outport: '<Root>/p' incorporates:
     *  Constant: '<Root>/Constant'
     */
    sm_vehicle_vtk_Y->p = sm_vehicle_vtk_ConstP.Constant_Value;
  }
}

/* Model terminate function */
void sm_vehicle_vtk_terminate(RT_MODEL_sm_vehicle_vtk_T *const sm_vehicle_vtk_M)
{
  DW_sm_vehicle_vtk_T *sm_vehicle_vtk_DW = sm_vehicle_vtk_M->dwork;

  /* Terminate for SimscapeExecutionBlock: '<S279>/STATE_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_vtk_DW->STATE_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_vtk_DW->STATE_1_SimData);
  nesl_erase_simulator("sm_vehicle_vtk/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S279>/OUTPUT_1_0' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_vtk_DW->OUTPUT_1_0_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_vtk_DW->OUTPUT_1_0_SimData);
  nesl_erase_simulator("sm_vehicle_vtk/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();

  /* Terminate for SimscapeExecutionBlock: '<S279>/OUTPUT_1_1' */
  neu_destroy_diagnostic_manager((NeuDiagnosticManager *)
    sm_vehicle_vtk_DW->OUTPUT_1_1_DiagMgr);
  nesl_destroy_simulation_data((NeslSimulationData *)
    sm_vehicle_vtk_DW->OUTPUT_1_1_SimData);
  nesl_erase_simulator("sm_vehicle_vtk/Vehicle/World/Solver Configuration_1");
  nesl_destroy_registry();
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
