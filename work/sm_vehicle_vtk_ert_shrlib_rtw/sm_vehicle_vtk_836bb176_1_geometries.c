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

const sm_core_compiler_Plane *sm_vehicle_vtk_836bb176_1_geometry_0(const
  RuntimeDerivedValuesBundle *rtdv)
{
  static const sm_core_compiler_Plane plane = { 0 };

  (void) rtdv;
  return &plane;
}

void sm_vehicle_vtk_836bb176_1_initializeGeometries(const struct
  RuntimeDerivedValuesBundleTag *rtdv)
{
  sm_vehicle_vtk_836bb176_1_geometry_0(rtdv);
}
