/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'sm_vehicle_2axle_heave_roll_vtk/Vehicle/World/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_CTarget.h"

static void setTargets_0(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[0];
}

static void setTargets_1(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[1];
}

static void setTargets_2(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[2];
}

static void setTargets_3(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[3];
}

static void setTargets_4(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[4];
}

static void setTargets_5(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[5];
}

static void setTargets_17(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[6];
}

static void setTargets_19(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[7];
}

static void setTargets_21(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[8];
}

static void setTargets_23(const RuntimeDerivedValuesBundle *rtdv, double *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[9];
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_setTargets(const
  RuntimeDerivedValuesBundle *rtdv, CTarget *targets)
{
  setTargets_0(rtdv, targets[0].mValue, targets[0].mAuxiliaryTargetData);
  setTargets_1(rtdv, targets[1].mValue, targets[1].mAuxiliaryTargetData);
  setTargets_2(rtdv, targets[2].mValue, targets[2].mAuxiliaryTargetData);
  setTargets_3(rtdv, targets[3].mValue, targets[3].mAuxiliaryTargetData);
  setTargets_4(rtdv, targets[4].mValue, targets[4].mAuxiliaryTargetData);
  setTargets_5(rtdv, targets[5].mValue, targets[5].mAuxiliaryTargetData);
  setTargets_17(rtdv, targets[17].mValue, targets[17].mAuxiliaryTargetData);
  setTargets_19(rtdv, targets[19].mValue, targets[19].mAuxiliaryTargetData);
  setTargets_21(rtdv, targets[21].mValue, targets[21].mAuxiliaryTargetData);
  setTargets_23(rtdv, targets[23].mValue, targets[23].mAuxiliaryTargetData);
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_resetAsmStateVector(const void
  *mech, double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = 1.0;
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
  state[20] = xx[0];
  state[21] = xx[0];
  state[22] = xx[0];
  state[23] = xx[0];
  state[24] = xx[0];
  state[25] = xx[0];
  state[26] = xx[0];
  state[27] = xx[0];
  state[28] = xx[0];
  state[29] = xx[0];
  state[30] = xx[0];
  state[31] = xx[0];
  state[32] = xx[0];
  state[33] = xx[0];
  state[34] = xx[0];
  state[35] = xx[0];
  state[36] = xx[0];
  state[37] = xx[0];
  state[38] = xx[0];
  state[39] = xx[0];
  state[40] = xx[0];
  state[41] = xx[0];
  state[42] = xx[0];
  state[43] = xx[0];
  state[44] = xx[0];
  state[45] = xx[0];
  state[46] = xx[0];
  state[47] = xx[0];
  state[48] = xx[0];
  state[49] = xx[0];
  state[50] = xx[0];
  state[51] = xx[0];
  state[52] = xx[0];
  state[53] = xx[0];
  state[54] = xx[0];
  state[55] = xx[0];
  state[56] = xx[0];
  state[57] = xx[0];
  state[58] = xx[0];
  state[59] = xx[0];
  state[60] = xx[0];
  state[61] = xx[0];
  state[62] = xx[0];
  state[63] = xx[0];
  state[64] = xx[0];
  state[65] = xx[0];
  state[66] = xx[0];
  state[67] = xx[0];
  state[68] = xx[0];
  state[69] = xx[0];
  state[70] = xx[0];
  state[71] = xx[0];
  state[72] = xx[0];
  state[73] = xx[0];
  state[74] = xx[0];
  state[75] = xx[0];
  state[76] = xx[0];
  state[77] = xx[0];
  state[78] = xx[0];
  state[79] = xx[0];
  state[80] = xx[0];
  state[81] = xx[0];
  state[82] = xx[0];
  state[83] = xx[0];
  state[84] = xx[0];
  state[85] = xx[0];
  state[86] = xx[0];
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_initializeTrackedAngleState(
  const void *mech, const RuntimeDerivedValuesBundle *rtdv, const int
  *modeVector, const double *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computeDiscreteState(const void *
  mech, const RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_adjustPosition(const void *mech,
  const double *dofDeltas, double *state)
{
  double xx[11];
  (void) mech;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = dofDeltas[3];
  xx[5] = dofDeltas[4];
  xx[6] = dofDeltas[5];
  pm_math_Quaternion_compDeriv_ra(xx + 0, xx + 4, xx + 7);
  xx[0] = state[3] + xx[7];
  xx[1] = state[4] + xx[8];
  xx[2] = state[5] + xx[9];
  xx[3] = state[6] + xx[10];
  xx[4] = sqrt(xx[0] * xx[0] + xx[1] * xx[1] + xx[2] * xx[2] + xx[3] * xx[3]);
  xx[5] = 1.0e-64;
  if (xx[5] > xx[4])
    xx[4] = xx[5];
  state[0] = state[0] + dofDeltas[0];
  state[1] = state[1] + dofDeltas[1];
  state[2] = state[2] + dofDeltas[2];
  state[3] = xx[0] / xx[4];
  state[4] = xx[1] / xx[4];
  state[5] = xx[2] / xx[4];
  state[6] = xx[3] / xx[4];
  state[13] = state[13] + dofDeltas[6];
  state[15] = state[15] + dofDeltas[7];
  state[17] = state[17] + dofDeltas[8];
  state[19] = state[19] + dofDeltas[9];
  state[21] = state[21] + dofDeltas[10];
  state[23] = state[23] + dofDeltas[11];
  state[25] = state[25] + dofDeltas[12];
  state[27] = state[27] + dofDeltas[13];
  state[29] = state[29] + dofDeltas[14];
  state[31] = state[31] + dofDeltas[15];
  state[33] = state[33] + dofDeltas[16];
  state[34] = state[34] + dofDeltas[17];
  state[35] = state[35] + dofDeltas[18];
  state[39] = state[39] + dofDeltas[19];
  state[40] = state[40] + dofDeltas[20];
  state[41] = state[41] + dofDeltas[21];
  state[45] = state[45] + dofDeltas[22];
  state[47] = state[47] + dofDeltas[23];
  state[49] = state[49] + dofDeltas[24];
  state[51] = state[51] + dofDeltas[25];
  state[52] = state[52] + dofDeltas[26];
  state[53] = state[53] + dofDeltas[27];
  state[57] = state[57] + dofDeltas[28];
  state[59] = state[59] + dofDeltas[29];
  state[61] = state[61] + dofDeltas[30];
  state[63] = state[63] + dofDeltas[31];
  state[64] = state[64] + dofDeltas[32];
  state[65] = state[65] + dofDeltas[33];
  state[69] = state[69] + dofDeltas[34];
  state[71] = state[71] + dofDeltas[35];
  state[73] = state[73] + dofDeltas[36];
  state[75] = state[75] + dofDeltas[37];
  state[76] = state[76] + dofDeltas[38];
  state[77] = state[77] + dofDeltas[39];
  state[81] = state[81] + dofDeltas[40];
  state[83] = state[83] + dofDeltas[41];
  state[85] = state[85] + dofDeltas[42];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbAsmJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[8] = state[8] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_0_3(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[3] = xx[4];
  state[4] = xx[5];
  state[5] = xx[6];
  state[6] = xx[7];
}

static void perturbAsmJointPrimitiveState_0_3v(double mag, double *state)
{
  double xx[15];
  xx[0] = 1.0;
  xx[1] = fabs(mag);
  xx[2] = xx[0] / (xx[1] - floor(xx[1]) + 1.0e-9);
  xx[1] = sin(xx[2]);
  xx[3] = 0.0;
  xx[4] = cos(xx[2]);
  xx[5] = sin(2.0 * xx[2]);
  xx[2] = 0.5 * mag;
  xx[6] = sqrt(xx[1] * xx[1] + xx[4] * xx[4] + xx[5] * xx[5]);
  xx[7] = xx[6] == 0.0 ? 0.0 : xx[1] / xx[6];
  xx[8] = sin(xx[2]);
  xx[9] = xx[6] == 0.0 ? 0.0 : xx[4] / xx[6];
  xx[10] = xx[6] == 0.0 ? 0.0 : xx[5] / xx[6];
  xx[11] = xx[1] == xx[3] && xx[4] == xx[3] && xx[5] == xx[3] ? xx[0] : cos(xx[2]);
  xx[12] = xx[7] * xx[8];
  xx[13] = xx[9] * xx[8];
  xx[14] = xx[10] * xx[8];
  xx[3] = state[3];
  xx[4] = state[4];
  xx[5] = state[5];
  xx[6] = state[6];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[3] = xx[7];
  state[4] = xx[8];
  state[5] = xx[9];
  state[6] = xx[10];
  state[10] = state[10] + 1.2 * mag;
  state[11] = state[11] - xx[2];
  state[12] = state[12] + 0.9 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[13] = state[13] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[13] = state[13] + mag;
  state[14] = state[14] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[15] = state[15] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[15] = state[15] + mag;
  state[16] = state[16] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[17] = state[17] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[17] = state[17] + mag;
  state[18] = state[18] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[19] = state[19] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[19] = state[19] + mag;
  state[20] = state[20] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[21] = state[21] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[21] = state[21] + mag;
  state[22] = state[22] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_6_0(double mag, double *state)
{
  state[23] = state[23] + mag;
}

static void perturbAsmJointPrimitiveState_6_0v(double mag, double *state)
{
  state[23] = state[23] + mag;
  state[24] = state[24] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_7_0(double mag, double *state)
{
  state[25] = state[25] + mag;
}

static void perturbAsmJointPrimitiveState_7_0v(double mag, double *state)
{
  state[25] = state[25] + mag;
  state[26] = state[26] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_8_0(double mag, double *state)
{
  state[27] = state[27] + mag;
}

static void perturbAsmJointPrimitiveState_8_0v(double mag, double *state)
{
  state[27] = state[27] + mag;
  state[28] = state[28] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_9_0(double mag, double *state)
{
  state[29] = state[29] + mag;
}

static void perturbAsmJointPrimitiveState_9_0v(double mag, double *state)
{
  state[29] = state[29] + mag;
  state[30] = state[30] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_10_0(double mag, double *state)
{
  state[31] = state[31] + mag;
}

static void perturbAsmJointPrimitiveState_10_0v(double mag, double *state)
{
  state[31] = state[31] + mag;
  state[32] = state[32] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_11_0(double mag, double *state)
{
  state[33] = state[33] + mag;
}

static void perturbAsmJointPrimitiveState_11_0v(double mag, double *state)
{
  state[33] = state[33] + mag;
  state[36] = state[36] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_11_1(double mag, double *state)
{
  state[34] = state[34] + mag;
}

static void perturbAsmJointPrimitiveState_11_1v(double mag, double *state)
{
  state[34] = state[34] + mag;
  state[37] = state[37] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_11_2(double mag, double *state)
{
  state[35] = state[35] + mag;
}

static void perturbAsmJointPrimitiveState_11_2v(double mag, double *state)
{
  state[35] = state[35] + mag;
  state[38] = state[38] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_12_0(double mag, double *state)
{
  state[39] = state[39] + mag;
}

static void perturbAsmJointPrimitiveState_12_0v(double mag, double *state)
{
  state[39] = state[39] + mag;
  state[42] = state[42] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_12_1(double mag, double *state)
{
  state[40] = state[40] + mag;
}

static void perturbAsmJointPrimitiveState_12_1v(double mag, double *state)
{
  state[40] = state[40] + mag;
  state[43] = state[43] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_12_2(double mag, double *state)
{
  state[41] = state[41] + mag;
}

static void perturbAsmJointPrimitiveState_12_2v(double mag, double *state)
{
  state[41] = state[41] + mag;
  state[44] = state[44] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_13_0(double mag, double *state)
{
  state[45] = state[45] + mag;
}

static void perturbAsmJointPrimitiveState_13_0v(double mag, double *state)
{
  state[45] = state[45] + mag;
  state[46] = state[46] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_14_0(double mag, double *state)
{
  state[47] = state[47] + mag;
}

static void perturbAsmJointPrimitiveState_14_0v(double mag, double *state)
{
  state[47] = state[47] + mag;
  state[48] = state[48] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_15_0(double mag, double *state)
{
  state[49] = state[49] + mag;
}

static void perturbAsmJointPrimitiveState_15_0v(double mag, double *state)
{
  state[49] = state[49] + mag;
  state[50] = state[50] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_16_0(double mag, double *state)
{
  state[51] = state[51] + mag;
}

static void perturbAsmJointPrimitiveState_16_0v(double mag, double *state)
{
  state[51] = state[51] + mag;
  state[54] = state[54] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_16_1(double mag, double *state)
{
  state[52] = state[52] + mag;
}

static void perturbAsmJointPrimitiveState_16_1v(double mag, double *state)
{
  state[52] = state[52] + mag;
  state[55] = state[55] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_16_2(double mag, double *state)
{
  state[53] = state[53] + mag;
}

static void perturbAsmJointPrimitiveState_16_2v(double mag, double *state)
{
  state[53] = state[53] + mag;
  state[56] = state[56] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_17_0(double mag, double *state)
{
  state[57] = state[57] + mag;
}

static void perturbAsmJointPrimitiveState_17_0v(double mag, double *state)
{
  state[57] = state[57] + mag;
  state[58] = state[58] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_18_0(double mag, double *state)
{
  state[59] = state[59] + mag;
}

static void perturbAsmJointPrimitiveState_18_0v(double mag, double *state)
{
  state[59] = state[59] + mag;
  state[60] = state[60] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_19_0(double mag, double *state)
{
  state[61] = state[61] + mag;
}

static void perturbAsmJointPrimitiveState_19_0v(double mag, double *state)
{
  state[61] = state[61] + mag;
  state[62] = state[62] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_20_0(double mag, double *state)
{
  state[63] = state[63] + mag;
}

static void perturbAsmJointPrimitiveState_20_0v(double mag, double *state)
{
  state[63] = state[63] + mag;
  state[66] = state[66] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_20_1(double mag, double *state)
{
  state[64] = state[64] + mag;
}

static void perturbAsmJointPrimitiveState_20_1v(double mag, double *state)
{
  state[64] = state[64] + mag;
  state[67] = state[67] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_20_2(double mag, double *state)
{
  state[65] = state[65] + mag;
}

static void perturbAsmJointPrimitiveState_20_2v(double mag, double *state)
{
  state[65] = state[65] + mag;
  state[68] = state[68] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_21_0(double mag, double *state)
{
  state[69] = state[69] + mag;
}

static void perturbAsmJointPrimitiveState_21_0v(double mag, double *state)
{
  state[69] = state[69] + mag;
  state[70] = state[70] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_22_0(double mag, double *state)
{
  state[71] = state[71] + mag;
}

static void perturbAsmJointPrimitiveState_22_0v(double mag, double *state)
{
  state[71] = state[71] + mag;
  state[72] = state[72] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_23_0(double mag, double *state)
{
  state[73] = state[73] + mag;
}

static void perturbAsmJointPrimitiveState_23_0v(double mag, double *state)
{
  state[73] = state[73] + mag;
  state[74] = state[74] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_24_0(double mag, double *state)
{
  state[75] = state[75] + mag;
}

static void perturbAsmJointPrimitiveState_24_0v(double mag, double *state)
{
  state[75] = state[75] + mag;
  state[78] = state[78] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_24_1(double mag, double *state)
{
  state[76] = state[76] + mag;
}

static void perturbAsmJointPrimitiveState_24_1v(double mag, double *state)
{
  state[76] = state[76] + mag;
  state[79] = state[79] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_24_2(double mag, double *state)
{
  state[77] = state[77] + mag;
}

static void perturbAsmJointPrimitiveState_24_2v(double mag, double *state)
{
  state[77] = state[77] + mag;
  state[80] = state[80] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_25_0(double mag, double *state)
{
  state[81] = state[81] + mag;
}

static void perturbAsmJointPrimitiveState_25_0v(double mag, double *state)
{
  state[81] = state[81] + mag;
  state[82] = state[82] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_26_0(double mag, double *state)
{
  state[83] = state[83] + mag;
}

static void perturbAsmJointPrimitiveState_26_0v(double mag, double *state)
{
  state[83] = state[83] + mag;
  state[84] = state[84] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_27_0(double mag, double *state)
{
  state[85] = state[85] + mag;
}

static void perturbAsmJointPrimitiveState_27_0v(double mag, double *state)
{
  state[85] = state[85] + mag;
  state[86] = state[86] - 0.875 * mag;
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_perturbAsmJointPrimitiveState(
  const void *mech, size_t stageIdx, size_t primIdx, double mag, boolean_T
  doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbAsmJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbAsmJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbAsmJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbAsmJointPrimitiveState_0_2v(mag, state);
    break;

   case 6:
    perturbAsmJointPrimitiveState_0_3(mag, state);
    break;

   case 7:
    perturbAsmJointPrimitiveState_0_3v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbAsmJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbAsmJointPrimitiveState_6_0v(mag, state);
    break;

   case 84:
    perturbAsmJointPrimitiveState_7_0(mag, state);
    break;

   case 85:
    perturbAsmJointPrimitiveState_7_0v(mag, state);
    break;

   case 96:
    perturbAsmJointPrimitiveState_8_0(mag, state);
    break;

   case 97:
    perturbAsmJointPrimitiveState_8_0v(mag, state);
    break;

   case 108:
    perturbAsmJointPrimitiveState_9_0(mag, state);
    break;

   case 109:
    perturbAsmJointPrimitiveState_9_0v(mag, state);
    break;

   case 120:
    perturbAsmJointPrimitiveState_10_0(mag, state);
    break;

   case 121:
    perturbAsmJointPrimitiveState_10_0v(mag, state);
    break;

   case 132:
    perturbAsmJointPrimitiveState_11_0(mag, state);
    break;

   case 133:
    perturbAsmJointPrimitiveState_11_0v(mag, state);
    break;

   case 134:
    perturbAsmJointPrimitiveState_11_1(mag, state);
    break;

   case 135:
    perturbAsmJointPrimitiveState_11_1v(mag, state);
    break;

   case 136:
    perturbAsmJointPrimitiveState_11_2(mag, state);
    break;

   case 137:
    perturbAsmJointPrimitiveState_11_2v(mag, state);
    break;

   case 144:
    perturbAsmJointPrimitiveState_12_0(mag, state);
    break;

   case 145:
    perturbAsmJointPrimitiveState_12_0v(mag, state);
    break;

   case 146:
    perturbAsmJointPrimitiveState_12_1(mag, state);
    break;

   case 147:
    perturbAsmJointPrimitiveState_12_1v(mag, state);
    break;

   case 148:
    perturbAsmJointPrimitiveState_12_2(mag, state);
    break;

   case 149:
    perturbAsmJointPrimitiveState_12_2v(mag, state);
    break;

   case 156:
    perturbAsmJointPrimitiveState_13_0(mag, state);
    break;

   case 157:
    perturbAsmJointPrimitiveState_13_0v(mag, state);
    break;

   case 168:
    perturbAsmJointPrimitiveState_14_0(mag, state);
    break;

   case 169:
    perturbAsmJointPrimitiveState_14_0v(mag, state);
    break;

   case 180:
    perturbAsmJointPrimitiveState_15_0(mag, state);
    break;

   case 181:
    perturbAsmJointPrimitiveState_15_0v(mag, state);
    break;

   case 192:
    perturbAsmJointPrimitiveState_16_0(mag, state);
    break;

   case 193:
    perturbAsmJointPrimitiveState_16_0v(mag, state);
    break;

   case 194:
    perturbAsmJointPrimitiveState_16_1(mag, state);
    break;

   case 195:
    perturbAsmJointPrimitiveState_16_1v(mag, state);
    break;

   case 196:
    perturbAsmJointPrimitiveState_16_2(mag, state);
    break;

   case 197:
    perturbAsmJointPrimitiveState_16_2v(mag, state);
    break;

   case 204:
    perturbAsmJointPrimitiveState_17_0(mag, state);
    break;

   case 205:
    perturbAsmJointPrimitiveState_17_0v(mag, state);
    break;

   case 216:
    perturbAsmJointPrimitiveState_18_0(mag, state);
    break;

   case 217:
    perturbAsmJointPrimitiveState_18_0v(mag, state);
    break;

   case 228:
    perturbAsmJointPrimitiveState_19_0(mag, state);
    break;

   case 229:
    perturbAsmJointPrimitiveState_19_0v(mag, state);
    break;

   case 240:
    perturbAsmJointPrimitiveState_20_0(mag, state);
    break;

   case 241:
    perturbAsmJointPrimitiveState_20_0v(mag, state);
    break;

   case 242:
    perturbAsmJointPrimitiveState_20_1(mag, state);
    break;

   case 243:
    perturbAsmJointPrimitiveState_20_1v(mag, state);
    break;

   case 244:
    perturbAsmJointPrimitiveState_20_2(mag, state);
    break;

   case 245:
    perturbAsmJointPrimitiveState_20_2v(mag, state);
    break;

   case 252:
    perturbAsmJointPrimitiveState_21_0(mag, state);
    break;

   case 253:
    perturbAsmJointPrimitiveState_21_0v(mag, state);
    break;

   case 264:
    perturbAsmJointPrimitiveState_22_0(mag, state);
    break;

   case 265:
    perturbAsmJointPrimitiveState_22_0v(mag, state);
    break;

   case 276:
    perturbAsmJointPrimitiveState_23_0(mag, state);
    break;

   case 277:
    perturbAsmJointPrimitiveState_23_0v(mag, state);
    break;

   case 288:
    perturbAsmJointPrimitiveState_24_0(mag, state);
    break;

   case 289:
    perturbAsmJointPrimitiveState_24_0v(mag, state);
    break;

   case 290:
    perturbAsmJointPrimitiveState_24_1(mag, state);
    break;

   case 291:
    perturbAsmJointPrimitiveState_24_1v(mag, state);
    break;

   case 292:
    perturbAsmJointPrimitiveState_24_2(mag, state);
    break;

   case 293:
    perturbAsmJointPrimitiveState_24_2v(mag, state);
    break;

   case 300:
    perturbAsmJointPrimitiveState_25_0(mag, state);
    break;

   case 301:
    perturbAsmJointPrimitiveState_25_0v(mag, state);
    break;

   case 312:
    perturbAsmJointPrimitiveState_26_0(mag, state);
    break;

   case 313:
    perturbAsmJointPrimitiveState_26_0v(mag, state);
    break;

   case 324:
    perturbAsmJointPrimitiveState_27_0(mag, state);
    break;

   case 325:
    perturbAsmJointPrimitiveState_27_0v(mag, state);
    break;
  }
}

static void computePosDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[20];
  xx[0] = 9.87654321;
  xx[1] = 2.0;
  xx[2] = xx[1] * (state[4] * state[5] - state[3] * state[6]);
  xx[3] = xx[2] * xx[2];
  xx[4] = 1.0;
  xx[5] = (state[3] * state[3] + state[4] * state[4]) * xx[1] - xx[4];
  xx[6] = xx[5] * xx[5];
  xx[7] = sqrt(xx[3] + xx[6]);
  xx[8] = xx[7] == 0.0 ? 0.0 : - xx[2] / xx[7];
  xx[9] = xx[6] + xx[3];
  xx[3] = sqrt(xx[9]);
  xx[6] = xx[3] == 0.0 ? 0.0 : xx[5] / xx[3];
  xx[10] = 0.0;
  xx[11] = (state[4] * state[6] + state[3] * state[5]) * xx[1];
  xx[1] = sqrt(xx[9] + xx[11] * xx[11]);
  xx[12] = xx[1] == 0.0 ? 0.0 : xx[5] / xx[1];
  xx[14] = xx[8];
  xx[15] = xx[6];
  xx[16] = xx[10];
  xx[17] = xx[8];
  xx[18] = xx[8];
  xx[19] = xx[12];
  xx[6] = xx[13 + (partialType)];
  xx[8] = xx[7] == 0.0 ? 0.0 : xx[5] / xx[7];
  xx[7] = xx[3] == 0.0 ? 0.0 : xx[2] / xx[3];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[2] / xx[1];
  xx[13] = xx[8];
  xx[14] = xx[7];
  xx[15] = xx[10];
  xx[16] = xx[8];
  xx[17] = xx[8];
  xx[18] = xx[3];
  xx[2] = xx[12 + (partialType)];
  xx[3] = xx[1] == 0.0 ? 0.0 : xx[11] / xx[1];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[4];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[18] = xx[3];
  xx[1] = xx[12 + (partialType)];
  xx[3] = xx[11] * xx[5];
  xx[5] = sqrt(xx[9] * xx[9] + xx[3] * xx[3]);
  xx[7] = xx[5] == 0.0 ? 0.0 : xx[9] / xx[5];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[7];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[7] = xx[11 + (partialType)];
  xx[12] = xx[10];
  xx[13] = xx[10];
  xx[14] = xx[10];
  xx[15] = xx[10];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[8] = xx[11 + (partialType)];
  xx[9] = xx[5] == 0.0 ? 0.0 : xx[3] / xx[5];
  xx[12] = xx[4];
  xx[13] = xx[4];
  xx[14] = xx[10];
  xx[15] = xx[9];
  xx[16] = xx[10];
  xx[17] = xx[10];
  xx[0] = xx[11 + (partialType)];
  matrix[0] = xx[6];
  matrix[1] = xx[2];
  matrix[2] = xx[1];
  matrix[3] = xx[7];
  matrix[4] = xx[8];
  matrix[5] = xx[0];
  matrix[6] = xx[8];
  matrix[7] = xx[8];
  matrix[8] = xx[8];
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computePosDofBlendMatrix(const
  void *mech, size_t stageIdx, size_t primIdx, const double *state, int
  partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computePosDofBlendMatrix_0_3(state, partialType, matrix);
    break;
  }
}

static void computeVelDofBlendMatrix_0_3(const double *state, int partialType,
  double *matrix)
{
  double xx[15];
  (void) state;
  xx[0] = 9.87654321;
  xx[1] = 0.0;
  xx[2] = 1.0;
  xx[4] = xx[1];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[2];
  xx[10] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[2];
  xx[9] = xx[1];
  xx[11] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[12] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[13] = xx[3 + (partialType)];
  xx[4] = xx[1];
  xx[5] = xx[1];
  xx[6] = xx[1];
  xx[7] = xx[2];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[14] = xx[3 + (partialType)];
  xx[4] = xx[2];
  xx[5] = xx[2];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[1];
  xx[9] = xx[1];
  xx[0] = xx[3 + (partialType)];
  matrix[0] = xx[10];
  matrix[1] = xx[11];
  matrix[2] = xx[12];
  matrix[3] = xx[13];
  matrix[4] = xx[14];
  matrix[5] = xx[0];
  matrix[6] = xx[13];
  matrix[7] = xx[13];
  matrix[8] = xx[13];
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computeVelDofBlendMatrix(const
  void *mech, size_t stageIdx, size_t primIdx, const double *state, int
  partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    computeVelDofBlendMatrix_0_3(state, partialType, matrix);
    break;
  }
}

static void projectPartiallyTargetedPos_0_3(const double *origState, int
  partialType, double *state)
{
  boolean_T bb[2];
  double xx[17];
  xx[0] = 2.0;
  xx[1] = (state[4] * state[6] + state[3] * state[5]) * xx[0];
  xx[2] = 0.99999999999999;
  bb[0] = fabs(xx[1]) > xx[2];
  xx[3] = 1.570796326794897;
  if (xx[1] < 0.0)
    xx[4] = -1.0;
  else if (xx[1] > 0.0)
    xx[4] = +1.0;
  else
    xx[4] = 0.0;
  xx[5] = fabs(xx[1]) > 1.0 ? atan2(xx[1], 0.0) : asin(xx[1]);
  xx[1] = bb[0] ? xx[3] * xx[4] : xx[5];
  xx[5] = (origState[4] * origState[6] + origState[3] * origState[5]) * xx[0];
  bb[1] = fabs(xx[5]) > xx[2];
  if (xx[5] < 0.0)
    xx[2] = -1.0;
  else if (xx[5] > 0.0)
    xx[2] = +1.0;
  else
    xx[2] = 0.0;
  xx[6] = fabs(xx[5]) > 1.0 ? atan2(xx[5], 0.0) : asin(xx[5]);
  xx[5] = bb[1] ? xx[3] * xx[2] : xx[6];
  xx[6] = xx[1];
  xx[7] = xx[1];
  xx[8] = xx[5];
  xx[9] = xx[5];
  xx[10] = xx[1];
  xx[11] = xx[1];
  xx[12] = xx[5];
  xx[1] = xx[6 + (partialType)];
  xx[3] = cos(xx[1]);
  xx[5] = 0.5;
  xx[6] = state[5] * state[6];
  xx[7] = state[3] * state[4];
  xx[8] = state[3] * state[3];
  xx[9] = 1.0;
  xx[10] = (xx[8] + state[5] * state[5]) * xx[0] - xx[9];
  xx[11] = (xx[6] + xx[7]) * xx[0];
  xx[10] = (xx[11] == 0.0 && xx[10] == 0.0) ? 0.0 : atan2(xx[11], xx[10]);
  xx[11] = (xx[8] + state[6] * state[6]) * xx[0] - xx[9];
  xx[12] = - (xx[0] * (xx[6] - xx[7]));
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[6] = bb[0] ? xx[5] * xx[10] : xx[11];
  xx[7] = (xx[8] + state[4] * state[4]) * xx[0] - xx[9];
  xx[10] = - (xx[0] * (state[4] * state[5] - state[3] * state[6]));
  xx[7] = (xx[10] == 0.0 && xx[7] == 0.0) ? 0.0 : atan2(xx[10], xx[7]);
  xx[8] = bb[0] ? xx[4] * xx[6] : xx[7];
  xx[4] = origState[5] * origState[6];
  xx[7] = origState[3] * origState[4];
  xx[10] = origState[3] * origState[3];
  xx[11] = (xx[10] + origState[5] * origState[5]) * xx[0] - xx[9];
  xx[12] = (xx[4] + xx[7]) * xx[0];
  xx[11] = (xx[12] == 0.0 && xx[11] == 0.0) ? 0.0 : atan2(xx[12], xx[11]);
  xx[12] = (xx[10] + origState[6] * origState[6]) * xx[0] - xx[9];
  xx[13] = - (xx[0] * (xx[4] - xx[7]));
  xx[12] = (xx[13] == 0.0 && xx[12] == 0.0) ? 0.0 : atan2(xx[13], xx[12]);
  xx[4] = bb[1] ? xx[5] * xx[11] : xx[12];
  xx[5] = (xx[10] + origState[4] * origState[4]) * xx[0] - xx[9];
  xx[7] = - (xx[0] * (origState[4] * origState[5] - origState[3] * origState[6]));
  xx[5] = (xx[7] == 0.0 && xx[5] == 0.0) ? 0.0 : atan2(xx[7], xx[5]);
  xx[0] = bb[1] ? xx[2] * xx[4] : xx[5];
  xx[9] = xx[8];
  xx[10] = xx[8];
  xx[11] = xx[8];
  xx[12] = xx[8];
  xx[13] = xx[0];
  xx[14] = xx[0];
  xx[15] = xx[0];
  xx[0] = xx[9 + (partialType)];
  xx[2] = cos(xx[0]);
  xx[5] = sin(xx[0]);
  xx[0] = sin(xx[1]);
  xx[7] = xx[6];
  xx[8] = xx[4];
  xx[9] = xx[6];
  xx[10] = xx[4];
  xx[11] = xx[6];
  xx[12] = xx[4];
  xx[13] = xx[6];
  xx[1] = xx[7 + (partialType)];
  xx[4] = cos(xx[1]);
  xx[6] = sin(xx[1]);
  xx[1] = xx[2] * xx[6];
  xx[7] = xx[4] * xx[2];
  xx[8] = xx[3] * xx[2];
  xx[9] = - (xx[3] * xx[5]);
  xx[10] = xx[0];
  xx[11] = xx[4] * xx[5] + xx[1] * xx[0];
  xx[12] = xx[7] - xx[6] * xx[0] * xx[5];
  xx[13] = - (xx[3] * xx[6]);
  xx[14] = xx[6] * xx[5] - xx[7] * xx[0];
  xx[15] = xx[1] + xx[4] * xx[0] * xx[5];
  xx[16] = xx[4] * xx[3];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 8, xx + 0);
  state[3] = xx[0];
  state[4] = xx[1];
  state[5] = xx[2];
  state[6] = xx[3];
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_projectPartiallyTargetedPos(
  const void *mech, size_t stageIdx, size_t primIdx, const double *origState,
  int partialType, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
   case 3:
    projectPartiallyTargetedPos_0_3(origState, partialType, state);
    break;
  }
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_propagateMotion(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const double *state, double
  *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[257];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = state[3];
  xx[1] = state[4];
  xx[2] = state[5];
  xx[3] = state[6];
  xx[4] = 0.9999999997549381;
  xx[5] = 2.09302257320166e-5;
  xx[6] = 4.494946017906737e-6;
  xx[7] = 5.643148656246455e-6;
  xx[8] = xx[4];
  xx[9] = xx[5];
  xx[10] = - xx[6];
  xx[11] = - xx[7];
  pm_math_Quaternion_composeInverse_ra(xx + 0, xx + 8, xx + 12);
  xx[0] = 1.499988338565875;
  xx[1] = 2.645620204187505e-6;
  xx[2] = 0.6146812028982972;
  xx[16] = xx[0];
  xx[17] = xx[1];
  xx[18] = - xx[2];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 16, xx + 19);
  xx[3] = state[0] - xx[19];
  xx[16] = state[1] - xx[20];
  xx[17] = state[2] - xx[21];
  xx[18] = - xx[4];
  xx[4] = - xx[5];
  xx[5] = 8.990128258360823e-6;
  xx[19] = 1.499986857891751 - xx[5] * state[13];
  xx[20] = 4.186040072247764e-5;
  xx[21] = - (4.248787794804562e-6 + xx[20] * state[13]);
  xx[22] = 0.9999999990834422;
  xx[23] = xx[22] * state[13] - 0.4499812030492543;
  xx[24] = 0.5;
  xx[25] = xx[24] * state[15];
  xx[26] = xx[24] * sin(xx[25]);
  xx[27] = xx[24] * cos(xx[25]);
  xx[25] = xx[26] - xx[27];
  xx[28] = xx[27] + xx[26];
  xx[29] = xx[27] + xx[26];
  xx[30] = xx[27] - xx[26];
  xx[26] = 0.2903;
  xx[27] = xx[26] * xx[29];
  xx[31] = xx[26] * xx[30];
  xx[32] = 2.0;
  xx[33] = - ((xx[27] * xx[29] + xx[31] * xx[30]) * xx[32] - xx[26]);
  xx[26] = (xx[25] * xx[31] + xx[28] * xx[27]) * xx[32];
  xx[34] = - (0.1 + xx[32] * (xx[25] * xx[27] - xx[28] * xx[31]));
  xx[27] = xx[24] * state[17];
  xx[31] = cos(xx[27]);
  xx[35] = sin(xx[27]);
  xx[27] = 0.0;
  xx[36] = 5.000000000000004e-3;
  xx[37] = - xx[36];
  xx[38] = 0.8;
  xx[39] = 0.7071067811865476;
  xx[40] = xx[24] * state[19];
  xx[41] = - (xx[39] * cos(xx[40]));
  xx[42] = - (xx[39] * sin(xx[40]));
  xx[40] = xx[24] * state[21];
  xx[43] = cos(xx[40]);
  xx[44] = sin(xx[40]);
  xx[40] = - xx[38];
  xx[45] = xx[24] * state[23];
  xx[46] = - (xx[39] * cos(xx[45]));
  xx[47] = - (xx[39] * sin(xx[45]));
  xx[45] = - (1.500013141795951 + xx[5] * state[25]);
  xx[5] = 2.961066861578549e-5 - xx[20] * state[25];
  xx[20] = xx[22] * state[25] - 0.4500081720166809;
  xx[22] = xx[24] * state[27];
  xx[48] = xx[24] * sin(xx[22]);
  xx[49] = xx[24] * cos(xx[22]);
  xx[22] = xx[48] - xx[49];
  xx[50] = xx[49] + xx[48];
  xx[51] = xx[49] + xx[48];
  xx[52] = xx[49] - xx[48];
  xx[48] = 0.2403;
  xx[49] = xx[48] * xx[51];
  xx[53] = xx[48] * xx[52];
  xx[54] = - ((xx[49] * xx[51] + xx[53] * xx[52]) * xx[32] - xx[48]);
  xx[48] = (xx[22] * xx[53] + xx[50] * xx[49]) * xx[32];
  xx[55] = - (0.05000000000000002 + xx[32] * (xx[22] * xx[49] - xx[50] * xx[53]));
  xx[49] = xx[24] * state[29];
  xx[53] = - (xx[39] * cos(xx[49]));
  xx[56] = - (xx[39] * sin(xx[49]));
  xx[49] = xx[24] * state[31];
  xx[57] = - (xx[39] * cos(xx[49]));
  xx[58] = - (xx[39] * sin(xx[49]));
  xx[39] = - 1.0;
  xx[49] = xx[24] * state[45];
  xx[59] = cos(xx[49]);
  xx[60] = sin(xx[49]);
  xx[49] = xx[24] * state[47];
  xx[61] = cos(xx[49]);
  xx[62] = sin(xx[49]);
  xx[49] = xx[24] * state[49];
  xx[63] = cos(xx[49]);
  xx[64] = sin(xx[49]);
  xx[49] = xx[24] * state[57];
  xx[65] = cos(xx[49]);
  xx[66] = sin(xx[49]);
  xx[49] = xx[24] * state[59];
  xx[67] = cos(xx[49]);
  xx[68] = sin(xx[49]);
  xx[49] = xx[24] * state[61];
  xx[69] = cos(xx[49]);
  xx[70] = sin(xx[49]);
  xx[49] = xx[24] * state[69];
  xx[71] = cos(xx[49]);
  xx[72] = sin(xx[49]);
  xx[49] = xx[24] * state[71];
  xx[73] = cos(xx[49]);
  xx[74] = sin(xx[49]);
  xx[49] = xx[24] * state[73];
  xx[75] = cos(xx[49]);
  xx[76] = sin(xx[49]);
  xx[49] = xx[24] * state[81];
  xx[77] = cos(xx[49]);
  xx[78] = sin(xx[49]);
  xx[49] = xx[24] * state[83];
  xx[79] = cos(xx[49]);
  xx[80] = sin(xx[49]);
  xx[49] = xx[24] * state[85];
  xx[24] = cos(xx[49]);
  xx[81] = sin(xx[49]);
  xx[49] = xx[59] * xx[61];
  xx[82] = xx[60] * xx[62];
  xx[83] = xx[59] * xx[62];
  xx[84] = xx[61] * xx[60];
  xx[85] = xx[18];
  xx[86] = xx[4];
  xx[87] = xx[6];
  xx[88] = xx[7];
  pm_math_Quaternion_compose_ra(xx + 12, xx + 85, xx + 89);
  xx[93] = xx[19];
  xx[94] = xx[21];
  xx[95] = xx[23];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 93, xx + 96);
  xx[99] = xx[96] + xx[3];
  xx[100] = xx[97] + xx[16];
  xx[96] = xx[98] + xx[17];
  xx[101] = xx[25];
  xx[102] = xx[28];
  xx[103] = xx[29];
  xx[104] = xx[30];
  pm_math_Quaternion_compose_ra(xx + 89, xx + 101, xx + 105);
  xx[109] = xx[33];
  xx[110] = xx[26];
  xx[111] = xx[34];
  pm_math_Quaternion_xform_ra(xx + 89, xx + 109, xx + 112);
  xx[97] = xx[112] + xx[99];
  xx[98] = xx[113] + xx[100];
  xx[112] = xx[114] + xx[96];
  xx[113] = xx[31] * xx[105] - xx[106] * xx[35];
  xx[114] = xx[105] * xx[35] + xx[31] * xx[106];
  xx[115] = xx[31] * xx[107] + xx[108] * xx[35];
  xx[116] = xx[31] * xx[108] - xx[107] * xx[35];
  xx[117] = xx[38] * xx[105] * xx[107];
  xx[118] = xx[38] * xx[107];
  xx[119] = xx[36] * xx[108];
  xx[120] = xx[38] * xx[106];
  xx[121] = xx[119] + xx[120];
  xx[122] = xx[36] * xx[107];
  xx[123] = xx[118];
  xx[124] = - xx[121];
  xx[125] = xx[122];
  pm_math_Vector3_cross_ra(xx + 106, xx + 123, xx + 126);
  xx[123] = (xx[117] + xx[126]) * xx[32] - xx[36] + xx[97];
  xx[124] = (xx[127] - xx[105] * xx[121]) * xx[32] + xx[98];
  xx[121] = xx[105] * xx[122];
  xx[125] = xx[38] + xx[32] * (xx[128] + xx[121]) + xx[112];
  xx[126] = xx[41];
  xx[127] = xx[41];
  xx[128] = xx[42];
  xx[129] = xx[42];
  pm_math_Quaternion_compose_ra(xx + 113, xx + 126, xx + 130);
  xx[134] = xx[65] * xx[67];
  xx[135] = xx[66] * xx[68];
  xx[136] = xx[65] * xx[68];
  xx[137] = xx[67] * xx[66];
  xx[138] = xx[43] * xx[105] - xx[106] * xx[44];
  xx[139] = xx[105] * xx[44] + xx[43] * xx[106];
  xx[140] = xx[43] * xx[107] + xx[108] * xx[44];
  xx[141] = xx[43] * xx[108] - xx[107] * xx[44];
  xx[142] = xx[120] - xx[119];
  xx[143] = - xx[118];
  xx[144] = xx[142];
  xx[145] = xx[122];
  pm_math_Vector3_cross_ra(xx + 106, xx + 143, xx + 118);
  xx[122] = (xx[118] - xx[117]) * xx[32] - xx[36] + xx[97];
  xx[117] = (xx[105] * xx[142] + xx[119]) * xx[32] + xx[98];
  xx[118] = xx[32] * (xx[120] + xx[121]) - xx[38] + xx[112];
  xx[142] = xx[46];
  xx[143] = xx[46];
  xx[144] = xx[47];
  xx[145] = xx[47];
  pm_math_Quaternion_compose_ra(xx + 138, xx + 142, xx + 146);
  xx[119] = xx[71] * xx[73];
  xx[120] = xx[72] * xx[74];
  xx[121] = xx[71] * xx[74];
  xx[150] = xx[73] * xx[72];
  xx[151] = xx[45];
  xx[152] = xx[5];
  xx[153] = xx[20];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 151, xx + 154);
  xx[157] = xx[154] + xx[3];
  xx[158] = xx[155] + xx[16];
  xx[154] = xx[156] + xx[17];
  xx[159] = xx[22];
  xx[160] = xx[50];
  xx[161] = xx[51];
  xx[162] = xx[52];
  pm_math_Quaternion_compose_ra(xx + 89, xx + 159, xx + 163);
  xx[167] = xx[54];
  xx[168] = xx[48];
  xx[169] = xx[55];
  pm_math_Quaternion_xform_ra(xx + 89, xx + 167, xx + 170);
  xx[155] = xx[170] + xx[157];
  xx[156] = xx[171] + xx[158];
  xx[170] = xx[172] + xx[154];
  xx[171] = xx[53];
  xx[172] = xx[53];
  xx[173] = xx[56];
  xx[174] = xx[56];
  pm_math_Quaternion_compose_ra(xx + 163, xx + 171, xx + 175);
  xx[179] = xx[38] * xx[163] * xx[165];
  xx[180] = xx[38] * xx[165];
  xx[181] = xx[36] * xx[166];
  xx[182] = xx[38] * xx[164];
  xx[183] = xx[181] + xx[182];
  xx[184] = xx[36] * xx[165];
  xx[185] = xx[180];
  xx[186] = - xx[183];
  xx[187] = xx[184];
  pm_math_Vector3_cross_ra(xx + 164, xx + 185, xx + 188);
  xx[185] = xx[163] * xx[184];
  xx[186] = xx[77] * xx[79];
  xx[187] = xx[78] * xx[80];
  xx[191] = xx[77] * xx[80];
  xx[192] = xx[79] * xx[78];
  xx[193] = xx[57];
  xx[194] = xx[57];
  xx[195] = xx[58];
  xx[196] = xx[58];
  pm_math_Quaternion_compose_ra(xx + 163, xx + 193, xx + 197);
  xx[201] = xx[182] - xx[181];
  xx[202] = - xx[180];
  xx[203] = xx[201];
  xx[204] = xx[184];
  pm_math_Vector3_cross_ra(xx + 164, xx + 202, xx + 180);
  xx[202] = state[10];
  xx[203] = state[11];
  xx[204] = state[12];
  pm_math_Quaternion_xform_ra(xx + 8, xx + 202, xx + 205);
  xx[8] = state[7];
  xx[9] = state[8];
  xx[10] = state[9];
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 8, xx + 202);
  xx[8] = - xx[0];
  xx[9] = - xx[1];
  xx[10] = xx[2];
  pm_math_Vector3_cross_ra(xx + 205, xx + 8, xx + 0);
  xx[8] = xx[202] + xx[0];
  xx[9] = xx[203] + xx[1];
  xx[0] = xx[204] + xx[2];
  pm_math_Quaternion_inverseXform_ra(xx + 85, xx + 205, xx + 202);
  pm_math_Vector3_cross_ra(xx + 205, xx + 93, xx + 208);
  xx[93] = xx[208] + xx[8];
  xx[94] = xx[209] + xx[9];
  xx[95] = xx[210] + xx[0];
  pm_math_Quaternion_inverseXform_ra(xx + 85, xx + 93, xx + 208);
  xx[1] = xx[210] + state[14];
  pm_math_Quaternion_inverseXform_ra(xx + 101, xx + 202, xx + 93);
  xx[2] = xx[94] - state[16];
  pm_math_Vector3_cross_ra(xx + 202, xx + 109, xx + 210);
  xx[109] = xx[210] + xx[208];
  xx[110] = xx[211] + xx[209];
  xx[111] = xx[212] + xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 101, xx + 109, xx + 210);
  xx[10] = xx[212] + 0.2903000000000001 * state[16];
  xx[11] = xx[93] + state[18];
  xx[94] = xx[95] * xx[35];
  xx[101] = xx[35] * xx[2];
  xx[102] = xx[2] + xx[32] * (xx[31] * xx[94] - xx[101] * xx[35]);
  xx[103] = xx[95] - (xx[31] * xx[101] + xx[94] * xx[35]) * xx[32];
  xx[94] = xx[38] * xx[2];
  xx[101] = xx[94] + xx[210];
  xx[104] = xx[36] * xx[95];
  xx[109] = xx[38] * xx[93];
  xx[110] = xx[211] - (xx[104] + xx[109]);
  xx[111] = xx[10] + xx[36] * xx[2];
  xx[184] = xx[35] * xx[111];
  xx[212] = xx[110] * xx[35];
  xx[213] = xx[110] + xx[32] * (xx[31] * xx[184] - xx[212] * xx[35]);
  xx[110] = xx[111] - (xx[31] * xx[212] + xx[184] * xx[35]) * xx[32];
  xx[214] = xx[11];
  xx[215] = xx[102];
  xx[216] = xx[103];
  pm_math_Quaternion_inverseXform_ra(xx + 126, xx + 214, xx + 217);
  xx[214] = xx[101];
  xx[215] = xx[213];
  xx[216] = xx[110];
  pm_math_Quaternion_inverseXform_ra(xx + 126, xx + 214, xx + 220);
  xx[126] = xx[93] + state[22];
  xx[127] = xx[95] * xx[44];
  xx[128] = xx[44] * xx[2];
  xx[129] = xx[2] + xx[32] * (xx[43] * xx[127] - xx[128] * xx[44]);
  xx[184] = xx[95] - (xx[43] * xx[128] + xx[127] * xx[44]) * xx[32];
  xx[127] = xx[210] - xx[94];
  xx[94] = xx[109] - xx[104] + xx[211];
  xx[104] = xx[44] * xx[111];
  xx[109] = xx[94] * xx[44];
  xx[128] = xx[94] + xx[32] * (xx[43] * xx[104] - xx[109] * xx[44]);
  xx[94] = xx[111] - (xx[43] * xx[109] + xx[104] * xx[44]) * xx[32];
  xx[214] = xx[126];
  xx[215] = xx[129];
  xx[216] = xx[184];
  pm_math_Quaternion_inverseXform_ra(xx + 142, xx + 214, xx + 223);
  xx[214] = xx[127];
  xx[215] = xx[128];
  xx[216] = xx[94];
  pm_math_Quaternion_inverseXform_ra(xx + 142, xx + 214, xx + 226);
  pm_math_Vector3_cross_ra(xx + 205, xx + 151, xx + 142);
  xx[151] = xx[142] + xx[8];
  xx[152] = xx[143] + xx[9];
  xx[153] = xx[144] + xx[0];
  pm_math_Quaternion_inverseXform_ra(xx + 85, xx + 151, xx + 142);
  xx[85] = xx[144] + state[26];
  pm_math_Quaternion_inverseXform_ra(xx + 159, xx + 202, xx + 86);
  xx[104] = xx[87] - state[28];
  pm_math_Vector3_cross_ra(xx + 202, xx + 167, xx + 151);
  xx[167] = xx[151] + xx[142];
  xx[168] = xx[152] + xx[143];
  xx[169] = xx[153] + xx[85];
  pm_math_Quaternion_inverseXform_ra(xx + 159, xx + 167, xx + 151);
  xx[87] = xx[153] + 0.2403 * state[28];
  xx[159] = xx[86];
  xx[160] = xx[104];
  xx[161] = xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 171, xx + 159, xx + 167);
  xx[109] = xx[38] * xx[104];
  xx[111] = xx[36] * xx[88];
  xx[144] = xx[38] * xx[86];
  xx[145] = xx[87] + xx[36] * xx[104];
  xx[214] = xx[109] + xx[151];
  xx[215] = xx[152] - (xx[111] + xx[144]);
  xx[216] = xx[145];
  pm_math_Quaternion_inverseXform_ra(xx + 171, xx + 214, xx + 229);
  pm_math_Quaternion_inverseXform_ra(xx + 193, xx + 159, xx + 171);
  xx[159] = xx[151] - xx[109];
  xx[160] = xx[144] - xx[111] + xx[152];
  xx[161] = xx[145];
  pm_math_Quaternion_inverseXform_ra(xx + 193, xx + 159, xx + 214);
  xx[109] = xx[60] * state[43];
  xx[111] = xx[60] * state[42];
  xx[144] = state[42] + xx[32] * (xx[59] * xx[109] - xx[111] * xx[60]);
  xx[145] = state[43] - (xx[59] * xx[111] + xx[109] * xx[60]) * xx[32];
  xx[109] = xx[62] * state[46];
  xx[111] = xx[32] * xx[61] * xx[109];
  xx[153] = state[46] - xx[32] * xx[109] * xx[62];
  xx[109] = xx[62] * state[44];
  xx[159] = xx[144] * xx[62];
  xx[160] = xx[144] - (xx[61] * xx[109] + xx[159] * xx[62]) * xx[32];
  xx[161] = state[44] + xx[32] * (xx[61] * xx[159] - xx[109] * xx[62]);
  xx[109] = xx[64] * xx[153];
  xx[159] = xx[64] * state[48];
  xx[162] = xx[161] * xx[64];
  xx[174] = xx[64] * xx[145];
  xx[193] = xx[66] * state[55];
  xx[194] = xx[66] * state[54];
  xx[195] = state[54] + xx[32] * (xx[65] * xx[193] - xx[194] * xx[66]);
  xx[196] = state[55] - (xx[65] * xx[194] + xx[193] * xx[66]) * xx[32];
  xx[193] = xx[68] * state[58];
  xx[194] = xx[32] * xx[67] * xx[193];
  xx[212] = state[58] - xx[32] * xx[193] * xx[68];
  xx[193] = xx[68] * state[56];
  xx[232] = xx[195] * xx[68];
  xx[233] = xx[195] - (xx[67] * xx[193] + xx[232] * xx[68]) * xx[32];
  xx[234] = state[56] + xx[32] * (xx[67] * xx[232] - xx[193] * xx[68]);
  xx[193] = xx[70] * xx[212];
  xx[232] = xx[70] * state[60];
  xx[235] = xx[234] * xx[70];
  xx[236] = xx[70] * xx[196];
  xx[237] = xx[72] * state[67];
  xx[238] = xx[72] * state[66];
  xx[239] = state[66] + xx[32] * (xx[71] * xx[237] - xx[238] * xx[72]);
  xx[240] = state[67] - (xx[71] * xx[238] + xx[237] * xx[72]) * xx[32];
  xx[237] = xx[74] * state[70];
  xx[238] = xx[32] * xx[73] * xx[237];
  xx[241] = state[70] - xx[32] * xx[237] * xx[74];
  xx[237] = xx[74] * state[68];
  xx[242] = xx[239] * xx[74];
  xx[243] = xx[239] - (xx[73] * xx[237] + xx[242] * xx[74]) * xx[32];
  xx[244] = state[68] + xx[32] * (xx[73] * xx[242] - xx[237] * xx[74]);
  xx[237] = xx[76] * xx[241];
  xx[242] = xx[76] * state[72];
  xx[245] = xx[244] * xx[76];
  xx[246] = xx[76] * xx[240];
  xx[247] = xx[78] * state[79];
  xx[248] = xx[78] * state[78];
  xx[249] = state[78] + xx[32] * (xx[77] * xx[247] - xx[248] * xx[78]);
  xx[250] = state[79] - (xx[77] * xx[248] + xx[247] * xx[78]) * xx[32];
  xx[247] = xx[80] * state[82];
  xx[248] = xx[32] * xx[79] * xx[247];
  xx[251] = state[82] - xx[32] * xx[247] * xx[80];
  xx[247] = xx[80] * state[80];
  xx[252] = xx[249] * xx[80];
  xx[253] = xx[249] - (xx[79] * xx[247] + xx[252] * xx[80]) * xx[32];
  xx[254] = state[80] + xx[32] * (xx[79] * xx[252] - xx[247] * xx[80]);
  xx[247] = xx[81] * xx[251];
  xx[252] = xx[81] * state[84];
  xx[255] = xx[254] * xx[81];
  xx[256] = xx[81] * xx[250];
  motionData[0] = xx[12];
  motionData[1] = xx[13];
  motionData[2] = xx[14];
  motionData[3] = xx[15];
  motionData[4] = xx[3];
  motionData[5] = xx[16];
  motionData[6] = xx[17];
  motionData[7] = xx[18];
  motionData[8] = xx[4];
  motionData[9] = xx[6];
  motionData[10] = xx[7];
  motionData[11] = xx[19];
  motionData[12] = xx[21];
  motionData[13] = xx[23];
  motionData[14] = xx[25];
  motionData[15] = xx[28];
  motionData[16] = xx[29];
  motionData[17] = xx[30];
  motionData[18] = xx[33];
  motionData[19] = xx[26];
  motionData[20] = xx[34];
  motionData[21] = xx[31];
  motionData[22] = xx[35];
  motionData[23] = xx[27];
  motionData[24] = xx[27];
  motionData[25] = xx[37];
  motionData[26] = xx[27];
  motionData[27] = xx[38];
  motionData[28] = xx[41];
  motionData[29] = xx[41];
  motionData[30] = xx[42];
  motionData[31] = xx[42];
  motionData[32] = xx[27];
  motionData[33] = xx[27];
  motionData[34] = xx[27];
  motionData[35] = xx[43];
  motionData[36] = xx[44];
  motionData[37] = xx[27];
  motionData[38] = xx[27];
  motionData[39] = xx[37];
  motionData[40] = xx[27];
  motionData[41] = xx[40];
  motionData[42] = xx[46];
  motionData[43] = xx[46];
  motionData[44] = xx[47];
  motionData[45] = xx[47];
  motionData[46] = xx[27];
  motionData[47] = xx[27];
  motionData[48] = xx[27];
  motionData[49] = xx[18];
  motionData[50] = xx[4];
  motionData[51] = xx[6];
  motionData[52] = xx[7];
  motionData[53] = xx[45];
  motionData[54] = xx[5];
  motionData[55] = xx[20];
  motionData[56] = xx[22];
  motionData[57] = xx[50];
  motionData[58] = xx[51];
  motionData[59] = xx[52];
  motionData[60] = xx[54];
  motionData[61] = xx[48];
  motionData[62] = xx[55];
  motionData[63] = xx[53];
  motionData[64] = xx[53];
  motionData[65] = xx[56];
  motionData[66] = xx[56];
  motionData[67] = xx[37];
  motionData[68] = xx[27];
  motionData[69] = xx[38];
  motionData[70] = xx[57];
  motionData[71] = xx[57];
  motionData[72] = xx[58];
  motionData[73] = xx[58];
  motionData[74] = xx[37];
  motionData[75] = xx[27];
  motionData[76] = xx[40];
  motionData[77] = 0.338430610991899;
  motionData[78] = 0.1016374984694517;
  motionData[79] = 0.03679784973285759;
  motionData[80] = - 0.934762247153553;
  motionData[81] = state[33];
  motionData[82] = state[34];
  motionData[83] = state[35];
  motionData[84] = xx[39];
  motionData[85] = xx[27];
  motionData[86] = xx[27];
  motionData[87] = xx[27];
  motionData[88] = state[39];
  motionData[89] = state[40];
  motionData[90] = state[41];
  motionData[91] = - xx[59];
  motionData[92] = xx[27];
  motionData[93] = xx[27];
  motionData[94] = - xx[60];
  motionData[95] = xx[27];
  motionData[96] = xx[27];
  motionData[97] = xx[27];
  motionData[98] = xx[61];
  motionData[99] = xx[27];
  motionData[100] = xx[62];
  motionData[101] = xx[27];
  motionData[102] = xx[27];
  motionData[103] = xx[27];
  motionData[104] = xx[27];
  motionData[105] = xx[63];
  motionData[106] = xx[64];
  motionData[107] = xx[27];
  motionData[108] = xx[27];
  motionData[109] = xx[27];
  motionData[110] = xx[27];
  motionData[111] = xx[27];
  motionData[112] = xx[39];
  motionData[113] = xx[27];
  motionData[114] = xx[27];
  motionData[115] = xx[27];
  motionData[116] = state[51];
  motionData[117] = state[52];
  motionData[118] = state[53];
  motionData[119] = - xx[65];
  motionData[120] = xx[27];
  motionData[121] = xx[27];
  motionData[122] = - xx[66];
  motionData[123] = xx[27];
  motionData[124] = xx[27];
  motionData[125] = xx[27];
  motionData[126] = xx[67];
  motionData[127] = xx[27];
  motionData[128] = xx[68];
  motionData[129] = xx[27];
  motionData[130] = xx[27];
  motionData[131] = xx[27];
  motionData[132] = xx[27];
  motionData[133] = xx[69];
  motionData[134] = xx[70];
  motionData[135] = xx[27];
  motionData[136] = xx[27];
  motionData[137] = xx[27];
  motionData[138] = xx[27];
  motionData[139] = xx[27];
  motionData[140] = xx[39];
  motionData[141] = xx[27];
  motionData[142] = xx[27];
  motionData[143] = xx[27];
  motionData[144] = state[63];
  motionData[145] = state[64];
  motionData[146] = state[65];
  motionData[147] = - xx[71];
  motionData[148] = xx[27];
  motionData[149] = xx[27];
  motionData[150] = - xx[72];
  motionData[151] = xx[27];
  motionData[152] = xx[27];
  motionData[153] = xx[27];
  motionData[154] = xx[73];
  motionData[155] = xx[27];
  motionData[156] = xx[74];
  motionData[157] = xx[27];
  motionData[158] = xx[27];
  motionData[159] = xx[27];
  motionData[160] = xx[27];
  motionData[161] = xx[75];
  motionData[162] = xx[76];
  motionData[163] = xx[27];
  motionData[164] = xx[27];
  motionData[165] = xx[27];
  motionData[166] = xx[27];
  motionData[167] = xx[27];
  motionData[168] = xx[39];
  motionData[169] = xx[27];
  motionData[170] = xx[27];
  motionData[171] = xx[27];
  motionData[172] = state[75];
  motionData[173] = state[76];
  motionData[174] = state[77];
  motionData[175] = - xx[77];
  motionData[176] = xx[27];
  motionData[177] = xx[27];
  motionData[178] = - xx[78];
  motionData[179] = xx[27];
  motionData[180] = xx[27];
  motionData[181] = xx[27];
  motionData[182] = xx[79];
  motionData[183] = xx[27];
  motionData[184] = xx[80];
  motionData[185] = xx[27];
  motionData[186] = xx[27];
  motionData[187] = xx[27];
  motionData[188] = xx[27];
  motionData[189] = xx[24];
  motionData[190] = xx[81];
  motionData[191] = xx[27];
  motionData[192] = xx[27];
  motionData[193] = xx[27];
  motionData[194] = xx[27];
  motionData[195] = xx[27];
  motionData[196] = xx[59];
  motionData[197] = xx[27];
  motionData[198] = xx[27];
  motionData[199] = xx[60];
  motionData[200] = state[39];
  motionData[201] = state[40];
  motionData[202] = state[41];
  motionData[203] = xx[49];
  motionData[204] = - xx[82];
  motionData[205] = xx[83];
  motionData[206] = xx[84];
  motionData[207] = state[39];
  motionData[208] = state[40];
  motionData[209] = state[41];
  motionData[210] = xx[63] * xx[49] + xx[82] * xx[64];
  motionData[211] = xx[49] * xx[64] - xx[63] * xx[82];
  motionData[212] = xx[63] * xx[83] + xx[84] * xx[64];
  motionData[213] = xx[63] * xx[84] - xx[83] * xx[64];
  motionData[214] = state[39];
  motionData[215] = state[40];
  motionData[216] = state[41];
  motionData[217] = xx[89];
  motionData[218] = xx[90];
  motionData[219] = xx[91];
  motionData[220] = xx[92];
  motionData[221] = xx[99];
  motionData[222] = xx[100];
  motionData[223] = xx[96];
  motionData[224] = xx[105];
  motionData[225] = xx[106];
  motionData[226] = xx[107];
  motionData[227] = xx[108];
  motionData[228] = xx[97];
  motionData[229] = xx[98];
  motionData[230] = xx[112];
  motionData[231] = xx[113];
  motionData[232] = xx[114];
  motionData[233] = xx[115];
  motionData[234] = xx[116];
  motionData[235] = xx[123];
  motionData[236] = xx[124];
  motionData[237] = xx[125];
  motionData[238] = xx[130];
  motionData[239] = xx[131];
  motionData[240] = xx[132];
  motionData[241] = xx[133];
  motionData[242] = xx[123];
  motionData[243] = xx[124];
  motionData[244] = xx[125];
  motionData[245] = xx[65];
  motionData[246] = xx[27];
  motionData[247] = xx[27];
  motionData[248] = xx[66];
  motionData[249] = state[51];
  motionData[250] = state[52];
  motionData[251] = state[53];
  motionData[252] = xx[134];
  motionData[253] = - xx[135];
  motionData[254] = xx[136];
  motionData[255] = xx[137];
  motionData[256] = state[51];
  motionData[257] = state[52];
  motionData[258] = state[53];
  motionData[259] = xx[69] * xx[134] + xx[135] * xx[70];
  motionData[260] = xx[134] * xx[70] - xx[69] * xx[135];
  motionData[261] = xx[69] * xx[136] + xx[137] * xx[70];
  motionData[262] = xx[69] * xx[137] - xx[136] * xx[70];
  motionData[263] = state[51];
  motionData[264] = state[52];
  motionData[265] = state[53];
  motionData[266] = xx[138];
  motionData[267] = xx[139];
  motionData[268] = xx[140];
  motionData[269] = xx[141];
  motionData[270] = xx[122];
  motionData[271] = xx[117];
  motionData[272] = xx[118];
  motionData[273] = xx[146];
  motionData[274] = xx[147];
  motionData[275] = xx[148];
  motionData[276] = xx[149];
  motionData[277] = xx[122];
  motionData[278] = xx[117];
  motionData[279] = xx[118];
  motionData[280] = xx[71];
  motionData[281] = xx[27];
  motionData[282] = xx[27];
  motionData[283] = xx[72];
  motionData[284] = state[63];
  motionData[285] = state[64];
  motionData[286] = state[65];
  motionData[287] = xx[119];
  motionData[288] = - xx[120];
  motionData[289] = xx[121];
  motionData[290] = xx[150];
  motionData[291] = state[63];
  motionData[292] = state[64];
  motionData[293] = state[65];
  motionData[294] = xx[75] * xx[119] + xx[120] * xx[76];
  motionData[295] = xx[119] * xx[76] - xx[75] * xx[120];
  motionData[296] = xx[75] * xx[121] + xx[150] * xx[76];
  motionData[297] = xx[75] * xx[150] - xx[121] * xx[76];
  motionData[298] = state[63];
  motionData[299] = state[64];
  motionData[300] = state[65];
  motionData[301] = xx[89];
  motionData[302] = xx[90];
  motionData[303] = xx[91];
  motionData[304] = xx[92];
  motionData[305] = xx[157];
  motionData[306] = xx[158];
  motionData[307] = xx[154];
  motionData[308] = xx[163];
  motionData[309] = xx[164];
  motionData[310] = xx[165];
  motionData[311] = xx[166];
  motionData[312] = xx[155];
  motionData[313] = xx[156];
  motionData[314] = xx[170];
  motionData[315] = xx[175];
  motionData[316] = xx[176];
  motionData[317] = xx[177];
  motionData[318] = xx[178];
  motionData[319] = (xx[179] + xx[188]) * xx[32] - xx[36] + xx[155];
  motionData[320] = (xx[189] - xx[163] * xx[183]) * xx[32] + xx[156];
  motionData[321] = xx[38] + xx[32] * (xx[190] + xx[185]) + xx[170];
  motionData[322] = xx[77];
  motionData[323] = xx[27];
  motionData[324] = xx[27];
  motionData[325] = xx[78];
  motionData[326] = state[75];
  motionData[327] = state[76];
  motionData[328] = state[77];
  motionData[329] = xx[186];
  motionData[330] = - xx[187];
  motionData[331] = xx[191];
  motionData[332] = xx[192];
  motionData[333] = state[75];
  motionData[334] = state[76];
  motionData[335] = state[77];
  motionData[336] = xx[24] * xx[186] + xx[187] * xx[81];
  motionData[337] = xx[186] * xx[81] - xx[24] * xx[187];
  motionData[338] = xx[24] * xx[191] + xx[192] * xx[81];
  motionData[339] = xx[24] * xx[192] - xx[191] * xx[81];
  motionData[340] = state[75];
  motionData[341] = state[76];
  motionData[342] = state[77];
  motionData[343] = xx[197];
  motionData[344] = xx[198];
  motionData[345] = xx[199];
  motionData[346] = xx[200];
  motionData[347] = (xx[180] - xx[179]) * xx[32] - xx[36] + xx[155];
  motionData[348] = (xx[163] * xx[201] + xx[181]) * xx[32] + xx[156];
  motionData[349] = xx[32] * (xx[182] + xx[185]) - xx[38] + xx[170];
  motionData[350] = xx[205];
  motionData[351] = xx[206];
  motionData[352] = xx[207];
  motionData[353] = xx[8];
  motionData[354] = xx[9];
  motionData[355] = xx[0];
  motionData[356] = xx[202];
  motionData[357] = xx[203];
  motionData[358] = xx[204];
  motionData[359] = xx[208];
  motionData[360] = xx[209];
  motionData[361] = xx[1];
  motionData[362] = xx[93];
  motionData[363] = xx[2];
  motionData[364] = xx[95];
  motionData[365] = xx[210];
  motionData[366] = xx[211];
  motionData[367] = xx[10];
  motionData[368] = xx[11];
  motionData[369] = xx[102];
  motionData[370] = xx[103];
  motionData[371] = xx[101];
  motionData[372] = xx[213];
  motionData[373] = xx[110];
  motionData[374] = xx[217];
  motionData[375] = xx[218] + state[20];
  motionData[376] = xx[219];
  motionData[377] = xx[220];
  motionData[378] = xx[221];
  motionData[379] = xx[222];
  motionData[380] = xx[126];
  motionData[381] = xx[129];
  motionData[382] = xx[184];
  motionData[383] = xx[127];
  motionData[384] = xx[128];
  motionData[385] = xx[94];
  motionData[386] = xx[223];
  motionData[387] = xx[224] + state[24];
  motionData[388] = xx[225];
  motionData[389] = xx[226];
  motionData[390] = xx[227];
  motionData[391] = xx[228];
  motionData[392] = xx[202];
  motionData[393] = xx[203];
  motionData[394] = xx[204];
  motionData[395] = xx[142];
  motionData[396] = xx[143];
  motionData[397] = xx[85];
  motionData[398] = xx[86];
  motionData[399] = xx[104];
  motionData[400] = xx[88];
  motionData[401] = xx[151];
  motionData[402] = xx[152];
  motionData[403] = xx[87];
  motionData[404] = xx[167];
  motionData[405] = xx[168] + state[30];
  motionData[406] = xx[169];
  motionData[407] = xx[229];
  motionData[408] = xx[230];
  motionData[409] = xx[231];
  motionData[410] = xx[171];
  motionData[411] = xx[172] + state[32];
  motionData[412] = xx[173];
  motionData[413] = xx[214];
  motionData[414] = xx[215];
  motionData[415] = xx[216];
  motionData[416] = xx[27];
  motionData[417] = xx[27];
  motionData[418] = xx[27];
  motionData[419] = - (0.7502690808970445 * state[36] + 0.62522423408087 *
                       state[37] + 0.2149208304652991 * state[38]);
  motionData[420] = 0.6401843996644798 * state[36] - 0.7682212795973762 * state
    [37];
  motionData[421] = 0.9766314743198204 * state[38] - (0.1651067553921828 *
    state[36] + 0.1375889628268189 * state[37]);
  motionData[422] = xx[27];
  motionData[423] = xx[27];
  motionData[424] = xx[27];
  motionData[425] = state[42];
  motionData[426] = state[43];
  motionData[427] = state[44];
  motionData[428] = xx[27];
  motionData[429] = xx[27];
  motionData[430] = state[46];
  motionData[431] = xx[144];
  motionData[432] = xx[145];
  motionData[433] = state[44];
  motionData[434] = - xx[111];
  motionData[435] = state[48];
  motionData[436] = xx[153];
  motionData[437] = xx[160];
  motionData[438] = xx[145];
  motionData[439] = xx[161];
  motionData[440] = state[50] - xx[111];
  motionData[441] = state[48] + xx[32] * (xx[63] * xx[109] - xx[159] * xx[64]);
  motionData[442] = xx[153] - (xx[63] * xx[159] + xx[109] * xx[64]) * xx[32];
  motionData[443] = xx[160];
  motionData[444] = xx[145] + xx[32] * (xx[63] * xx[162] - xx[174] * xx[64]);
  motionData[445] = xx[161] - (xx[63] * xx[174] + xx[162] * xx[64]) * xx[32];
  motionData[446] = xx[27];
  motionData[447] = xx[27];
  motionData[448] = xx[27];
  motionData[449] = state[54];
  motionData[450] = state[55];
  motionData[451] = state[56];
  motionData[452] = xx[27];
  motionData[453] = xx[27];
  motionData[454] = state[58];
  motionData[455] = xx[195];
  motionData[456] = xx[196];
  motionData[457] = state[56];
  motionData[458] = - xx[194];
  motionData[459] = state[60];
  motionData[460] = xx[212];
  motionData[461] = xx[233];
  motionData[462] = xx[196];
  motionData[463] = xx[234];
  motionData[464] = state[62] - xx[194];
  motionData[465] = state[60] + xx[32] * (xx[69] * xx[193] - xx[232] * xx[70]);
  motionData[466] = xx[212] - (xx[69] * xx[232] + xx[193] * xx[70]) * xx[32];
  motionData[467] = xx[233];
  motionData[468] = xx[196] + xx[32] * (xx[69] * xx[235] - xx[236] * xx[70]);
  motionData[469] = xx[234] - (xx[69] * xx[236] + xx[235] * xx[70]) * xx[32];
  motionData[470] = xx[27];
  motionData[471] = xx[27];
  motionData[472] = xx[27];
  motionData[473] = state[66];
  motionData[474] = state[67];
  motionData[475] = state[68];
  motionData[476] = xx[27];
  motionData[477] = xx[27];
  motionData[478] = state[70];
  motionData[479] = xx[239];
  motionData[480] = xx[240];
  motionData[481] = state[68];
  motionData[482] = - xx[238];
  motionData[483] = state[72];
  motionData[484] = xx[241];
  motionData[485] = xx[243];
  motionData[486] = xx[240];
  motionData[487] = xx[244];
  motionData[488] = state[74] - xx[238];
  motionData[489] = state[72] + xx[32] * (xx[75] * xx[237] - xx[242] * xx[76]);
  motionData[490] = xx[241] - (xx[75] * xx[242] + xx[237] * xx[76]) * xx[32];
  motionData[491] = xx[243];
  motionData[492] = xx[240] + xx[32] * (xx[75] * xx[245] - xx[246] * xx[76]);
  motionData[493] = xx[244] - (xx[75] * xx[246] + xx[245] * xx[76]) * xx[32];
  motionData[494] = xx[27];
  motionData[495] = xx[27];
  motionData[496] = xx[27];
  motionData[497] = state[78];
  motionData[498] = state[79];
  motionData[499] = state[80];
  motionData[500] = xx[27];
  motionData[501] = xx[27];
  motionData[502] = state[82];
  motionData[503] = xx[249];
  motionData[504] = xx[250];
  motionData[505] = state[80];
  motionData[506] = - xx[248];
  motionData[507] = state[84];
  motionData[508] = xx[251];
  motionData[509] = xx[253];
  motionData[510] = xx[250];
  motionData[511] = xx[254];
  motionData[512] = state[86] - xx[248];
  motionData[513] = state[84] + xx[32] * (xx[24] * xx[247] - xx[252] * xx[81]);
  motionData[514] = xx[251] - (xx[24] * xx[252] + xx[247] * xx[81]) * xx[32];
  motionData[515] = xx[253];
  motionData[516] = xx[250] + xx[32] * (xx[24] * xx[255] - xx[256] * xx[81]);
  motionData[517] = xx[254] - (xx[24] * xx[256] + xx[255] * xx[81]) * xx[32];
}

static size_t computeAssemblyError_0(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[10];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[0];
  xx[1] = motionData[1];
  xx[2] = motionData[2];
  xx[3] = motionData[3];
  xx[4] = - 1.795436775680343e-5;
  xx[5] = - 9.726932096251812e-6;
  xx[6] = 0.08530531197639897;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[7] + motionData[4] - motionData[81];
  error[1] = xx[8] + motionData[5] - motionData[82];
  error[2] = xx[9] + motionData[6] - motionData[83];
  return 3;
}

size_t sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computeAssemblyError(const
  void *mech, const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx,
  const int *modeVector, const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyError_0(rtdv, modeVector, motionData, error);
  }

  return 0;
}

static size_t computeAssemblyJacobian_0(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[29];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 1.0;
  xx[1] = 0.0;
  xx[2] = state[3];
  xx[3] = state[4];
  xx[4] = state[5];
  xx[5] = state[6];
  xx[6] = 0.9999999997549381;
  xx[7] = 2.09302257320166e-5;
  xx[8] = - 4.494946017906737e-6;
  xx[9] = - 5.643148656246455e-6;
  pm_math_Quaternion_composeInverse_ra(xx + 2, xx + 6, xx + 10);
  xx[2] = - 9.627097223806007e-7;
  xx[3] = - 0.08530531212892234;
  xx[4] = - 9.727134736950059e-6;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 5);
  xx[2] = - 6.937566682099615e-6;
  xx[3] = - 0.6146946872131903;
  xx[4] = - 1.957521679260031e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 14);
  xx[2] = 0.08530531230339998;
  xx[3] = - 9.635166408278131e-7;
  xx[4] = 1.795425796071189e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 17);
  xx[2] = 0.6146812024313428;
  xx[3] = - 6.97276242720263e-5;
  xx[4] = 1.499988337126269;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 20);
  xx[2] = 6.156017544248484e-6;
  xx[3] = - 1.71874620445599e-5;
  xx[4] = - 6.641306619128532e-10;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 23);
  xx[2] = - 2.308518126813466e-5;
  xx[3] = - 1.499982811128197;
  xx[4] = - 6.279008914694602e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 26);
  xx[2] = 9.87654321;
  xx[3] = - xx[0];
  J[0] = xx[0];
  J[1] = xx[1];
  J[2] = xx[1];
  J[3] = xx[5] + xx[14];
  J[4] = xx[17] + xx[20];
  J[5] = xx[23] + xx[26];
  J[16] = xx[3];
  J[17] = xx[1];
  J[18] = xx[1];
  J[43] = xx[1];
  J[44] = xx[0];
  J[45] = xx[1];
  J[46] = xx[6] + xx[15];
  J[47] = xx[18] + xx[21];
  J[48] = xx[24] + xx[27];
  J[59] = xx[1];
  J[60] = xx[3];
  J[61] = xx[1];
  J[86] = xx[1];
  J[87] = xx[1];
  J[88] = xx[0];
  J[89] = xx[7] + xx[16];
  J[90] = xx[19] + xx[22];
  J[91] = xx[25] + xx[28];
  J[102] = xx[1];
  J[103] = xx[1];
  J[104] = xx[3];
  return 3;
}

size_t sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computeAssemblyJacobian(const
  void *mech, const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx,
  boolean_T forVelocitySatisfaction, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyJacobian_0(rtdv, state, modeVector, motionData, J);
  }

  return 0;
}

size_t sm_vehicle_2axle_heave_roll_vtk_836bb176_1_computeFullAssemblyJacobian(
  const void *mech, const RuntimeDerivedValuesBundle *rtdv, const double *state,
  const int *modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[29];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  xx[0] = 1.0;
  xx[1] = 0.0;
  xx[2] = state[3];
  xx[3] = state[4];
  xx[4] = state[5];
  xx[5] = state[6];
  xx[6] = 0.9999999997549381;
  xx[7] = 2.09302257320166e-5;
  xx[8] = - 4.494946017906737e-6;
  xx[9] = - 5.643148656246455e-6;
  pm_math_Quaternion_composeInverse_ra(xx + 2, xx + 6, xx + 10);
  xx[2] = - 9.627097223806007e-7;
  xx[3] = - 0.08530531212892234;
  xx[4] = - 9.727134736950059e-6;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 5);
  xx[2] = - 6.937566682099615e-6;
  xx[3] = - 0.6146946872131903;
  xx[4] = - 1.957521679260031e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 14);
  xx[2] = 0.08530531230339998;
  xx[3] = - 9.635166408278131e-7;
  xx[4] = 1.795425796071189e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 17);
  xx[2] = 0.6146812024313428;
  xx[3] = - 6.97276242720263e-5;
  xx[4] = 1.499988337126269;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 20);
  xx[2] = 6.156017544248484e-6;
  xx[3] = - 1.71874620445599e-5;
  xx[4] = - 6.641306619128532e-10;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 23);
  xx[2] = - 2.308518126813466e-5;
  xx[3] = - 1.499982811128197;
  xx[4] = - 6.279008914694602e-5;
  pm_math_Quaternion_xform_ra(xx + 10, xx + 2, xx + 26);
  xx[2] = 9.87654321;
  xx[3] = - xx[0];
  J[0] = xx[0];
  J[1] = xx[1];
  J[2] = xx[1];
  J[3] = xx[5] + xx[14];
  J[4] = xx[17] + xx[20];
  J[5] = xx[23] + xx[26];
  J[16] = xx[3];
  J[17] = xx[1];
  J[18] = xx[1];
  J[43] = xx[1];
  J[44] = xx[0];
  J[45] = xx[1];
  J[46] = xx[6] + xx[15];
  J[47] = xx[18] + xx[21];
  J[48] = xx[24] + xx[27];
  J[59] = xx[1];
  J[60] = xx[3];
  J[61] = xx[1];
  J[86] = xx[1];
  J[87] = xx[1];
  J[88] = xx[0];
  J[89] = xx[7] + xx[16];
  J[90] = xx[19] + xx[22];
  J[91] = xx[25] + xx[28];
  J[102] = xx[1];
  J[103] = xx[1];
  J[104] = xx[3];
  return 3;
}

static boolean_T isInKinematicSingularity_0(const RuntimeDerivedValuesBundle
  *rtdv, const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

boolean_T sm_vehicle_2axle_heave_roll_vtk_836bb176_1_isInKinematicSingularity(
  const void *mech, const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx,
  const int *modeVector, const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
   case 0:
    return isInKinematicSingularity_0(rtdv, modeVector, motionData);
  }

  return 0;
}

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_convertStateVector(const void
  *asmMech, const RuntimeDerivedValuesBundle *rtdv, const void *simMech, const
  double *asmState, const int *asmModeVector, const int *simModeVector, double
  *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[20];
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  xx[0] = 0.338430610991899;
  xx[1] = 0.1016374984694517;
  xx[2] = 0.03679784973285759;
  xx[3] = - 0.934762247153553;
  xx[4] = - xx[0];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[3];
  xx[8] = asmState[3];
  xx[9] = asmState[4];
  xx[10] = asmState[5];
  xx[11] = asmState[6];
  xx[12] = 0.9999999997549381;
  xx[13] = 2.09302257320166e-5;
  xx[14] = - 4.494946017906737e-6;
  xx[15] = - 5.643148656246455e-6;
  pm_math_Quaternion_composeInverse_ra(xx + 8, xx + 12, xx + 16);
  pm_math_Quaternion_inverseCompose_ra(xx + 0, xx + 16, xx + 8);
  pm_math_Quaternion_inverseCompose_ra(xx + 4, xx + 8, xx + 0);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 12, xx + 4);
  xx[0] = asmState[10];
  xx[1] = asmState[11];
  xx[2] = asmState[12];
  pm_math_Quaternion_xform_ra(xx + 12, xx + 0, xx + 8);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 8, xx + 0);
  simState[0] = asmState[33];
  simState[1] = asmState[34];
  simState[2] = asmState[35];
  simState[3] = asmState[36];
  simState[4] = asmState[37];
  simState[5] = asmState[38];
  simState[6] = xx[4];
  simState[7] = xx[5];
  simState[8] = xx[6];
  simState[9] = xx[7];
  simState[10] = xx[0];
  simState[11] = xx[1];
  simState[12] = xx[2];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  simState[20] = asmState[20];
  simState[21] = asmState[21];
  simState[22] = asmState[22];
  simState[23] = asmState[23];
  simState[24] = asmState[24];
  simState[25] = asmState[25];
  simState[26] = asmState[26];
  simState[27] = asmState[27];
  simState[28] = asmState[28];
  simState[29] = asmState[29];
  simState[30] = asmState[30];
  simState[31] = asmState[31];
  simState[32] = asmState[32];
  simState[33] = asmState[39];
  simState[34] = asmState[40];
  simState[35] = asmState[41];
  simState[36] = asmState[42];
  simState[37] = asmState[43];
  simState[38] = asmState[44];
  simState[39] = asmState[45];
  simState[40] = asmState[46];
  simState[41] = asmState[47];
  simState[42] = asmState[48];
  simState[43] = asmState[49];
  simState[44] = asmState[50];
  simState[45] = asmState[51];
  simState[46] = asmState[52];
  simState[47] = asmState[53];
  simState[48] = asmState[54];
  simState[49] = asmState[55];
  simState[50] = asmState[56];
  simState[51] = asmState[57];
  simState[52] = asmState[58];
  simState[53] = asmState[59];
  simState[54] = asmState[60];
  simState[55] = asmState[61];
  simState[56] = asmState[62];
  simState[57] = asmState[63];
  simState[58] = asmState[64];
  simState[59] = asmState[65];
  simState[60] = asmState[66];
  simState[61] = asmState[67];
  simState[62] = asmState[68];
  simState[63] = asmState[69];
  simState[64] = asmState[70];
  simState[65] = asmState[71];
  simState[66] = asmState[72];
  simState[67] = asmState[73];
  simState[68] = asmState[74];
  simState[69] = asmState[75];
  simState[70] = asmState[76];
  simState[71] = asmState[77];
  simState[72] = asmState[78];
  simState[73] = asmState[79];
  simState[74] = asmState[80];
  simState[75] = asmState[81];
  simState[76] = asmState[82];
  simState[77] = asmState[83];
  simState[78] = asmState[84];
  simState[79] = asmState[85];
  simState[80] = asmState[86];
}
