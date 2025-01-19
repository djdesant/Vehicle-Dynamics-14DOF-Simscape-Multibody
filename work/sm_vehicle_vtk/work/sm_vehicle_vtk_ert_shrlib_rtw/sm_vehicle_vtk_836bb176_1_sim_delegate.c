/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'sm_vehicle_vtk/Vehicle/World/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "pm_default_allocator.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ssc_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void sm_vehicle_vtk_836bb176_1_resetSimStateVector(const void *mech, double
  *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = 1.0;
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
}

static void perturbSimJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbSimJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_1(double mag, double *state)
{
  state[1] = state[1] + mag;
}

static void perturbSimJointPrimitiveState_0_1v(double mag, double *state)
{
  state[1] = state[1] + mag;
  state[4] = state[4] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_0_2(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbSimJointPrimitiveState_0_2v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_1_0(double mag, double *state)
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
  xx[0] = state[6];
  xx[1] = state[7];
  xx[2] = state[8];
  xx[3] = state[9];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 0, xx + 4);
  state[6] = xx[4];
  state[7] = xx[5];
  state[8] = xx[6];
  state[9] = xx[7];
}

static void perturbSimJointPrimitiveState_1_0v(double mag, double *state)
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
  xx[3] = state[6];
  xx[4] = state[7];
  xx[5] = state[8];
  xx[6] = state[9];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 3, xx + 7);
  state[6] = xx[7];
  state[7] = xx[8];
  state[8] = xx[9];
  state[9] = xx[10];
  state[10] = state[10] + 1.2 * mag;
  state[11] = state[11] - xx[2];
  state[12] = state[12] + 0.9 * mag;
}

static void perturbSimJointPrimitiveState_2_0(double mag, double *state)
{
  state[13] = state[13] + mag;
}

static void perturbSimJointPrimitiveState_2_0v(double mag, double *state)
{
  state[13] = state[13] + mag;
  state[14] = state[14] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_3_0(double mag, double *state)
{
  state[15] = state[15] + mag;
}

static void perturbSimJointPrimitiveState_3_0v(double mag, double *state)
{
  state[15] = state[15] + mag;
  state[16] = state[16] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_4_0(double mag, double *state)
{
  state[17] = state[17] + mag;
}

static void perturbSimJointPrimitiveState_4_0v(double mag, double *state)
{
  state[17] = state[17] + mag;
  state[18] = state[18] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_5_0(double mag, double *state)
{
  state[19] = state[19] + mag;
}

static void perturbSimJointPrimitiveState_5_0v(double mag, double *state)
{
  state[19] = state[19] + mag;
  state[20] = state[20] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_6_0(double mag, double *state)
{
  state[21] = state[21] + mag;
}

static void perturbSimJointPrimitiveState_6_0v(double mag, double *state)
{
  state[21] = state[21] + mag;
  state[22] = state[22] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_7_0(double mag, double *state)
{
  state[23] = state[23] + mag;
}

static void perturbSimJointPrimitiveState_7_0v(double mag, double *state)
{
  state[23] = state[23] + mag;
  state[24] = state[24] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_8_0(double mag, double *state)
{
  state[25] = state[25] + mag;
}

static void perturbSimJointPrimitiveState_8_0v(double mag, double *state)
{
  state[25] = state[25] + mag;
  state[26] = state[26] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_9_0(double mag, double *state)
{
  state[27] = state[27] + mag;
}

static void perturbSimJointPrimitiveState_9_0v(double mag, double *state)
{
  state[27] = state[27] + mag;
  state[28] = state[28] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_10_0(double mag, double *state)
{
  state[29] = state[29] + mag;
}

static void perturbSimJointPrimitiveState_10_0v(double mag, double *state)
{
  state[29] = state[29] + mag;
  state[30] = state[30] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_11_0(double mag, double *state)
{
  state[31] = state[31] + mag;
}

static void perturbSimJointPrimitiveState_11_0v(double mag, double *state)
{
  state[31] = state[31] + mag;
  state[32] = state[32] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_12_0(double mag, double *state)
{
  state[33] = state[33] + mag;
}

static void perturbSimJointPrimitiveState_12_0v(double mag, double *state)
{
  state[33] = state[33] + mag;
  state[36] = state[36] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_12_1(double mag, double *state)
{
  state[34] = state[34] + mag;
}

static void perturbSimJointPrimitiveState_12_1v(double mag, double *state)
{
  state[34] = state[34] + mag;
  state[37] = state[37] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_12_2(double mag, double *state)
{
  state[35] = state[35] + mag;
}

static void perturbSimJointPrimitiveState_12_2v(double mag, double *state)
{
  state[35] = state[35] + mag;
  state[38] = state[38] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_13_0(double mag, double *state)
{
  state[39] = state[39] + mag;
}

static void perturbSimJointPrimitiveState_13_0v(double mag, double *state)
{
  state[39] = state[39] + mag;
  state[40] = state[40] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_14_0(double mag, double *state)
{
  state[41] = state[41] + mag;
}

static void perturbSimJointPrimitiveState_14_0v(double mag, double *state)
{
  state[41] = state[41] + mag;
  state[42] = state[42] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_15_0(double mag, double *state)
{
  state[43] = state[43] + mag;
}

static void perturbSimJointPrimitiveState_15_0v(double mag, double *state)
{
  state[43] = state[43] + mag;
  state[44] = state[44] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_16_0(double mag, double *state)
{
  state[45] = state[45] + mag;
}

static void perturbSimJointPrimitiveState_16_0v(double mag, double *state)
{
  state[45] = state[45] + mag;
  state[48] = state[48] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_16_1(double mag, double *state)
{
  state[46] = state[46] + mag;
}

static void perturbSimJointPrimitiveState_16_1v(double mag, double *state)
{
  state[46] = state[46] + mag;
  state[49] = state[49] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_16_2(double mag, double *state)
{
  state[47] = state[47] + mag;
}

static void perturbSimJointPrimitiveState_16_2v(double mag, double *state)
{
  state[47] = state[47] + mag;
  state[50] = state[50] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_17_0(double mag, double *state)
{
  state[51] = state[51] + mag;
}

static void perturbSimJointPrimitiveState_17_0v(double mag, double *state)
{
  state[51] = state[51] + mag;
  state[52] = state[52] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_18_0(double mag, double *state)
{
  state[53] = state[53] + mag;
}

static void perturbSimJointPrimitiveState_18_0v(double mag, double *state)
{
  state[53] = state[53] + mag;
  state[54] = state[54] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_19_0(double mag, double *state)
{
  state[55] = state[55] + mag;
}

static void perturbSimJointPrimitiveState_19_0v(double mag, double *state)
{
  state[55] = state[55] + mag;
  state[56] = state[56] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_20_0(double mag, double *state)
{
  state[57] = state[57] + mag;
}

static void perturbSimJointPrimitiveState_20_0v(double mag, double *state)
{
  state[57] = state[57] + mag;
  state[60] = state[60] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_20_1(double mag, double *state)
{
  state[58] = state[58] + mag;
}

static void perturbSimJointPrimitiveState_20_1v(double mag, double *state)
{
  state[58] = state[58] + mag;
  state[61] = state[61] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_20_2(double mag, double *state)
{
  state[59] = state[59] + mag;
}

static void perturbSimJointPrimitiveState_20_2v(double mag, double *state)
{
  state[59] = state[59] + mag;
  state[62] = state[62] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_21_0(double mag, double *state)
{
  state[63] = state[63] + mag;
}

static void perturbSimJointPrimitiveState_21_0v(double mag, double *state)
{
  state[63] = state[63] + mag;
  state[64] = state[64] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_22_0(double mag, double *state)
{
  state[65] = state[65] + mag;
}

static void perturbSimJointPrimitiveState_22_0v(double mag, double *state)
{
  state[65] = state[65] + mag;
  state[66] = state[66] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_23_0(double mag, double *state)
{
  state[67] = state[67] + mag;
}

static void perturbSimJointPrimitiveState_23_0v(double mag, double *state)
{
  state[67] = state[67] + mag;
  state[68] = state[68] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_24_0(double mag, double *state)
{
  state[69] = state[69] + mag;
}

static void perturbSimJointPrimitiveState_24_0v(double mag, double *state)
{
  state[69] = state[69] + mag;
  state[72] = state[72] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_24_1(double mag, double *state)
{
  state[70] = state[70] + mag;
}

static void perturbSimJointPrimitiveState_24_1v(double mag, double *state)
{
  state[70] = state[70] + mag;
  state[73] = state[73] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_24_2(double mag, double *state)
{
  state[71] = state[71] + mag;
}

static void perturbSimJointPrimitiveState_24_2v(double mag, double *state)
{
  state[71] = state[71] + mag;
  state[74] = state[74] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_25_0(double mag, double *state)
{
  state[75] = state[75] + mag;
}

static void perturbSimJointPrimitiveState_25_0v(double mag, double *state)
{
  state[75] = state[75] + mag;
  state[76] = state[76] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_26_0(double mag, double *state)
{
  state[77] = state[77] + mag;
}

static void perturbSimJointPrimitiveState_26_0v(double mag, double *state)
{
  state[77] = state[77] + mag;
  state[78] = state[78] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_27_0(double mag, double *state)
{
  state[79] = state[79] + mag;
}

static void perturbSimJointPrimitiveState_27_0v(double mag, double *state)
{
  state[79] = state[79] + mag;
  state[80] = state[80] - 0.875 * mag;
}

void sm_vehicle_vtk_836bb176_1_perturbSimJointPrimitiveState(const void *mech,
  size_t stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity,
  double *state)
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
    perturbSimJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbSimJointPrimitiveState_0_0v(mag, state);
    break;

   case 2:
    perturbSimJointPrimitiveState_0_1(mag, state);
    break;

   case 3:
    perturbSimJointPrimitiveState_0_1v(mag, state);
    break;

   case 4:
    perturbSimJointPrimitiveState_0_2(mag, state);
    break;

   case 5:
    perturbSimJointPrimitiveState_0_2v(mag, state);
    break;

   case 12:
    perturbSimJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbSimJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbSimJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbSimJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbSimJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbSimJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbSimJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbSimJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbSimJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbSimJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbSimJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbSimJointPrimitiveState_6_0v(mag, state);
    break;

   case 84:
    perturbSimJointPrimitiveState_7_0(mag, state);
    break;

   case 85:
    perturbSimJointPrimitiveState_7_0v(mag, state);
    break;

   case 96:
    perturbSimJointPrimitiveState_8_0(mag, state);
    break;

   case 97:
    perturbSimJointPrimitiveState_8_0v(mag, state);
    break;

   case 108:
    perturbSimJointPrimitiveState_9_0(mag, state);
    break;

   case 109:
    perturbSimJointPrimitiveState_9_0v(mag, state);
    break;

   case 120:
    perturbSimJointPrimitiveState_10_0(mag, state);
    break;

   case 121:
    perturbSimJointPrimitiveState_10_0v(mag, state);
    break;

   case 132:
    perturbSimJointPrimitiveState_11_0(mag, state);
    break;

   case 133:
    perturbSimJointPrimitiveState_11_0v(mag, state);
    break;

   case 144:
    perturbSimJointPrimitiveState_12_0(mag, state);
    break;

   case 145:
    perturbSimJointPrimitiveState_12_0v(mag, state);
    break;

   case 146:
    perturbSimJointPrimitiveState_12_1(mag, state);
    break;

   case 147:
    perturbSimJointPrimitiveState_12_1v(mag, state);
    break;

   case 148:
    perturbSimJointPrimitiveState_12_2(mag, state);
    break;

   case 149:
    perturbSimJointPrimitiveState_12_2v(mag, state);
    break;

   case 156:
    perturbSimJointPrimitiveState_13_0(mag, state);
    break;

   case 157:
    perturbSimJointPrimitiveState_13_0v(mag, state);
    break;

   case 168:
    perturbSimJointPrimitiveState_14_0(mag, state);
    break;

   case 169:
    perturbSimJointPrimitiveState_14_0v(mag, state);
    break;

   case 180:
    perturbSimJointPrimitiveState_15_0(mag, state);
    break;

   case 181:
    perturbSimJointPrimitiveState_15_0v(mag, state);
    break;

   case 192:
    perturbSimJointPrimitiveState_16_0(mag, state);
    break;

   case 193:
    perturbSimJointPrimitiveState_16_0v(mag, state);
    break;

   case 194:
    perturbSimJointPrimitiveState_16_1(mag, state);
    break;

   case 195:
    perturbSimJointPrimitiveState_16_1v(mag, state);
    break;

   case 196:
    perturbSimJointPrimitiveState_16_2(mag, state);
    break;

   case 197:
    perturbSimJointPrimitiveState_16_2v(mag, state);
    break;

   case 204:
    perturbSimJointPrimitiveState_17_0(mag, state);
    break;

   case 205:
    perturbSimJointPrimitiveState_17_0v(mag, state);
    break;

   case 216:
    perturbSimJointPrimitiveState_18_0(mag, state);
    break;

   case 217:
    perturbSimJointPrimitiveState_18_0v(mag, state);
    break;

   case 228:
    perturbSimJointPrimitiveState_19_0(mag, state);
    break;

   case 229:
    perturbSimJointPrimitiveState_19_0v(mag, state);
    break;

   case 240:
    perturbSimJointPrimitiveState_20_0(mag, state);
    break;

   case 241:
    perturbSimJointPrimitiveState_20_0v(mag, state);
    break;

   case 242:
    perturbSimJointPrimitiveState_20_1(mag, state);
    break;

   case 243:
    perturbSimJointPrimitiveState_20_1v(mag, state);
    break;

   case 244:
    perturbSimJointPrimitiveState_20_2(mag, state);
    break;

   case 245:
    perturbSimJointPrimitiveState_20_2v(mag, state);
    break;

   case 252:
    perturbSimJointPrimitiveState_21_0(mag, state);
    break;

   case 253:
    perturbSimJointPrimitiveState_21_0v(mag, state);
    break;

   case 264:
    perturbSimJointPrimitiveState_22_0(mag, state);
    break;

   case 265:
    perturbSimJointPrimitiveState_22_0v(mag, state);
    break;

   case 276:
    perturbSimJointPrimitiveState_23_0(mag, state);
    break;

   case 277:
    perturbSimJointPrimitiveState_23_0v(mag, state);
    break;

   case 288:
    perturbSimJointPrimitiveState_24_0(mag, state);
    break;

   case 289:
    perturbSimJointPrimitiveState_24_0v(mag, state);
    break;

   case 290:
    perturbSimJointPrimitiveState_24_1(mag, state);
    break;

   case 291:
    perturbSimJointPrimitiveState_24_1v(mag, state);
    break;

   case 292:
    perturbSimJointPrimitiveState_24_2(mag, state);
    break;

   case 293:
    perturbSimJointPrimitiveState_24_2v(mag, state);
    break;

   case 300:
    perturbSimJointPrimitiveState_25_0(mag, state);
    break;

   case 301:
    perturbSimJointPrimitiveState_25_0v(mag, state);
    break;

   case 312:
    perturbSimJointPrimitiveState_26_0(mag, state);
    break;

   case 313:
    perturbSimJointPrimitiveState_26_0v(mag, state);
    break;

   case 324:
    perturbSimJointPrimitiveState_27_0(mag, state);
    break;

   case 325:
    perturbSimJointPrimitiveState_27_0v(mag, state);
    break;
  }
}

void sm_vehicle_vtk_836bb176_1_perturbFlexibleBodyState(const void *mech, size_t
  stageIdx, double mag, boolean_T doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch (stageIdx * 2 + (doPerturbVelocity ? 1 : 0))
  {
  }
}

void sm_vehicle_vtk_836bb176_1_constructStateVector(const void *mech, const
  double *solverState, const double *u, const double *uDot, double
  *discreteState, double *fullState)
{
  (void) mech;
  (void) discreteState;
  fullState[0] = solverState[0];
  fullState[1] = solverState[1];
  fullState[2] = solverState[2];
  fullState[3] = solverState[3];
  fullState[4] = solverState[4];
  fullState[5] = solverState[5];
  fullState[6] = solverState[6];
  fullState[7] = solverState[7];
  fullState[8] = solverState[8];
  fullState[9] = solverState[9];
  fullState[10] = solverState[10];
  fullState[11] = solverState[11];
  fullState[12] = solverState[12];
  fullState[13] = solverState[13];
  fullState[14] = solverState[14];
  fullState[15] = solverState[15];
  fullState[16] = solverState[16];
  fullState[17] = u[4];
  fullState[18] = uDot[4];
  fullState[19] = solverState[17];
  fullState[20] = solverState[18];
  fullState[21] = u[5];
  fullState[22] = uDot[5];
  fullState[23] = solverState[19];
  fullState[24] = solverState[20];
  fullState[25] = solverState[21];
  fullState[26] = solverState[22];
  fullState[27] = solverState[23];
  fullState[28] = solverState[24];
  fullState[29] = solverState[25];
  fullState[30] = solverState[26];
  fullState[31] = solverState[27];
  fullState[32] = solverState[28];
  fullState[33] = u[6];
  fullState[34] = u[7];
  fullState[35] = u[8];
  fullState[36] = uDot[6];
  fullState[37] = uDot[7];
  fullState[38] = uDot[8];
  fullState[39] = u[10];
  fullState[40] = uDot[10];
  fullState[41] = u[11];
  fullState[42] = uDot[11];
  fullState[43] = u[9];
  fullState[44] = uDot[9];
  fullState[45] = u[12];
  fullState[46] = u[13];
  fullState[47] = u[14];
  fullState[48] = uDot[12];
  fullState[49] = uDot[13];
  fullState[50] = uDot[14];
  fullState[51] = u[16];
  fullState[52] = uDot[16];
  fullState[53] = u[17];
  fullState[54] = uDot[17];
  fullState[55] = u[15];
  fullState[56] = uDot[15];
  fullState[57] = u[18];
  fullState[58] = u[19];
  fullState[59] = u[20];
  fullState[60] = uDot[18];
  fullState[61] = uDot[19];
  fullState[62] = uDot[20];
  fullState[63] = u[22];
  fullState[64] = uDot[22];
  fullState[65] = u[23];
  fullState[66] = uDot[23];
  fullState[67] = u[21];
  fullState[68] = uDot[21];
  fullState[69] = u[24];
  fullState[70] = u[25];
  fullState[71] = u[26];
  fullState[72] = uDot[24];
  fullState[73] = uDot[25];
  fullState[74] = uDot[26];
  fullState[75] = u[28];
  fullState[76] = uDot[28];
  fullState[77] = u[29];
  fullState[78] = uDot[29];
  fullState[79] = u[27];
  fullState[80] = uDot[27];
}

void sm_vehicle_vtk_836bb176_1_extractSolverStateVector(const void *mech, const
  double *fullState, double *solverState)
{
  (void) mech;
  solverState[0] = fullState[0];
  solverState[1] = fullState[1];
  solverState[2] = fullState[2];
  solverState[3] = fullState[3];
  solverState[4] = fullState[4];
  solverState[5] = fullState[5];
  solverState[6] = fullState[6];
  solverState[7] = fullState[7];
  solverState[8] = fullState[8];
  solverState[9] = fullState[9];
  solverState[10] = fullState[10];
  solverState[11] = fullState[11];
  solverState[12] = fullState[12];
  solverState[13] = fullState[13];
  solverState[14] = fullState[14];
  solverState[15] = fullState[15];
  solverState[16] = fullState[16];
  solverState[17] = fullState[19];
  solverState[18] = fullState[20];
  solverState[19] = fullState[23];
  solverState[20] = fullState[24];
  solverState[21] = fullState[25];
  solverState[22] = fullState[26];
  solverState[23] = fullState[27];
  solverState[24] = fullState[28];
  solverState[25] = fullState[29];
  solverState[26] = fullState[30];
  solverState[27] = fullState[31];
  solverState[28] = fullState[32];
}

void sm_vehicle_vtk_836bb176_1_extractDiscreteStateVector(const void *mech,
  const double *fullState, double *discreteState)
{
  (void) mech;
  (void) fullState;
  (void) discreteState;
}

boolean_T sm_vehicle_vtk_836bb176_1_isPositionViolation(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  return 0;
}

boolean_T sm_vehicle_vtk_836bb176_1_isVelocityViolation(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const double
  *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) state;
  (void) modeVector;
  return 0;
}

PmfMessageId sm_vehicle_vtk_836bb176_1_projectStateSim(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const int
  *modeVector, double *state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  double xx[1];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) neDiagMgr;
  xx[0] = sqrt(state[6] * state[6] + state[7] * state[7] + state[8] * state[8] +
               state[9] * state[9]);
  state[6] = state[6] / xx[0];
  state[7] = state[7] / xx[0];
  state[8] = state[8] / xx[0];
  state[9] = state[9] / xx[0];
  return NULL;
}

void sm_vehicle_vtk_836bb176_1_computeConstraintError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) error;
}

void sm_vehicle_vtk_836bb176_1_resetModeVector(const void *mech, int *modeVector)
{
  (void) mech;
  (void) modeVector;
}

boolean_T sm_vehicle_vtk_836bb176_1_hasJointUpwardModeChange(const void *mech,
  const int *prevModeVector, const int *modeVector)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  return 0;
}

PmfMessageId sm_vehicle_vtk_836bb176_1_performJointUpwardModeChange(const void
  *mech, const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags,
  const int *prevModeVector, const int *modeVector, const double *input, double *
  state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) prevModeVector;
  (void) modeVector;
  (void) input;
  (void) state;
  (void) neDiagMgr;
  return NULL;
}

void sm_vehicle_vtk_836bb176_1_onModeChangedCutJoints(const void *mech, const
  int *prevModeVector, const int *modeVector, double *state)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  (void) state;
}

void sm_vehicle_vtk_836bb176_1_setVariableModeJointsToLocked(const void *mech,
  int *modeVector)
{
  (void) mech;
  (void) modeVector;
}
