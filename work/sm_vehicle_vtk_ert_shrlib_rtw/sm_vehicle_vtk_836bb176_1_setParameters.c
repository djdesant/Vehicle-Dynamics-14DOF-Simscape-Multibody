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

void sm_vehicle_vtk_836bb176_1_computeRuntimeParameters(const real_T t0[],
  real_T out[])
{
  real_T t10[1];
  real_T t12[1];
  real_T t13[1];
  real_T t14[1];
  real_T t15[1];
  real_T t16[1];
  real_T t17[1];
  real_T t18[1];
  real_T t19[1];
  real_T t20[1];
  real_T t21[1];
  t19[0ULL] = t0[0ULL];
  t20[0ULL] = t0[1ULL];
  t13[0ULL] = t0[2ULL];
  t14[0ULL] = t0[3ULL];
  t15[0ULL] = t0[4ULL];
  t16[0ULL] = t0[5ULL];
  t17[0ULL] = t0[6ULL];
  t18[0ULL] = t0[7ULL];
  t10[0ULL] = t0[8ULL];
  t21[0ULL] = t0[9ULL];
  memcpy(&t12[0], &t13[0], 8U);
  memcpy(&t13[0], &t14[0], 8U);
  memcpy(&t14[0], &t15[0], 8U);
  memcpy(&t15[0], &t16[0], 8U);
  memcpy(&t16[0], &t17[0], 8U);
  memcpy(&t17[0], &t18[0], 8U);
  memcpy(&t18[0], &t19[0], 8U);
  memcpy(&t19[0], &t20[0], 8U);
  memcpy(&t20[0], &t21[0], 8U);
  memcpy(&t21[0], &t10[0], 8U);
  out[0ULL] = t12[0ULL];
  out[1ULL] = t13[0ULL];
  out[2ULL] = t14[0ULL];
  out[3ULL] = t15[0ULL];
  out[4ULL] = t16[0ULL];
  out[5ULL] = t17[0ULL];
  out[6ULL] = t18[0ULL];
  out[7ULL] = t19[0ULL];
  out[8ULL] = t20[0ULL];
  out[9ULL] = t21[0ULL];
}

void sm_vehicle_vtk_836bb176_1_computeAsmRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  boolean_T bb[2];
  double xx[11];
  xx[0] = rtp[0];
  bb[0] = sm_core_math_anyIsInf(1, xx + 0);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = 0.0;
  xx[1] = !bb[0] && !bb[1] ? rtp[0] : xx[0];
  xx[2] = rtp[1];
  bb[0] = sm_core_math_anyIsInf(1, xx + 2);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 2);
  xx[2] = !bb[0] && !bb[1] ? rtp[1] : xx[0];
  xx[3] = rtp[2];
  bb[0] = sm_core_math_anyIsInf(1, xx + 3);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 3);
  xx[3] = !bb[0] && !bb[1] ? rtp[2] : xx[0];
  xx[4] = rtp[3];
  bb[0] = sm_core_math_anyIsInf(1, xx + 4);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 4);
  xx[4] = !bb[0] && !bb[1] ? rtp[3] : xx[0];
  xx[5] = rtp[4];
  bb[0] = sm_core_math_anyIsInf(1, xx + 5);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 5);
  xx[5] = !bb[0] && !bb[1] ? rtp[4] : xx[0];
  xx[6] = rtp[5];
  bb[0] = sm_core_math_anyIsInf(1, xx + 6);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 6);
  xx[6] = !bb[0] && !bb[1] ? rtp[5] : xx[0];
  xx[7] = rtp[6];
  bb[0] = sm_core_math_anyIsInf(1, xx + 7);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 7);
  xx[7] = !bb[0] && !bb[1] ? rtp[6] : xx[0];
  xx[8] = rtp[7];
  bb[0] = sm_core_math_anyIsInf(1, xx + 8);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 8);
  xx[8] = !bb[0] && !bb[1] ? rtp[7] : xx[0];
  xx[9] = rtp[8];
  bb[0] = sm_core_math_anyIsInf(1, xx + 9);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 9);
  xx[9] = !bb[0] && !bb[1] ? rtp[8] : xx[0];
  xx[10] = rtp[9];
  bb[0] = sm_core_math_anyIsInf(1, xx + 10);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 10);
  xx[10] = !bb[0] && !bb[1] ? rtp[9] : xx[0];
  rtdvd[0] = xx[1];
  rtdvd[1] = xx[2];
  rtdvd[2] = xx[3];
  rtdvd[3] = xx[4];
  rtdvd[4] = xx[5];
  rtdvd[5] = xx[6];
  rtdvd[6] = xx[7];
  rtdvd[7] = xx[8];
  rtdvd[8] = xx[9];
  rtdvd[9] = xx[10];
  rtdvd[10] = xx[1];
  rtdvd[11] = xx[2];
  rtdvd[12] = xx[3];
  rtdvd[13] = xx[4];
  rtdvd[14] = xx[5];
  rtdvd[15] = xx[6];
  rtdvd[16] = xx[7];
  rtdvd[17] = xx[8];
  rtdvd[18] = xx[9];
  rtdvd[19] = xx[10];
}

void sm_vehicle_vtk_836bb176_1_computeAsmRuntimeDerivedValuesInts(const double
  *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void sm_vehicle_vtk_836bb176_1_computeAsmRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  sm_vehicle_vtk_836bb176_1_computeAsmRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  sm_vehicle_vtk_836bb176_1_computeAsmRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}

void sm_vehicle_vtk_836bb176_1_computeSimRuntimeDerivedValuesDoubles(const
  double *rtp, double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void sm_vehicle_vtk_836bb176_1_computeSimRuntimeDerivedValuesInts(const double
  *rtp, int *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void sm_vehicle_vtk_836bb176_1_computeSimRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  sm_vehicle_vtk_836bb176_1_computeSimRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  sm_vehicle_vtk_836bb176_1_computeSimRuntimeDerivedValuesInts(rtp,
    rtdv->mInts.mValues);
}
