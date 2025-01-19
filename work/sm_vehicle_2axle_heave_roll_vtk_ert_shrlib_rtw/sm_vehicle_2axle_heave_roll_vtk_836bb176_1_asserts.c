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

void sm_vehicle_2axle_heave_roll_vtk_836bb176_1_validateRuntimeParameters(const
  double *rtp, int32_T *satFlags)
{
  boolean_T bb[20];
  double xx[1];
  xx[0] = rtp[0];
  bb[0] = sm_core_math_anyIsInf(1, xx + 0);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[1];
  bb[2] = sm_core_math_anyIsInf(1, xx + 0);
  bb[3] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[2];
  bb[4] = sm_core_math_anyIsInf(1, xx + 0);
  bb[5] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[3];
  bb[6] = sm_core_math_anyIsInf(1, xx + 0);
  bb[7] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[4];
  bb[8] = sm_core_math_anyIsInf(1, xx + 0);
  bb[9] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[5];
  bb[10] = sm_core_math_anyIsInf(1, xx + 0);
  bb[11] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[6];
  bb[12] = sm_core_math_anyIsInf(1, xx + 0);
  bb[13] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[7];
  bb[14] = sm_core_math_anyIsInf(1, xx + 0);
  bb[15] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[8];
  bb[16] = sm_core_math_anyIsInf(1, xx + 0);
  bb[17] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[9];
  bb[18] = sm_core_math_anyIsInf(1, xx + 0);
  bb[19] = sm_core_math_anyIsNaN(1, xx + 0);
  satFlags[0] = !bb[0] ? 1 : 0;
  satFlags[1] = !bb[1] ? 1 : 0;
  satFlags[2] = !bb[2] ? 1 : 0;
  satFlags[3] = !bb[3] ? 1 : 0;
  satFlags[4] = !bb[4] ? 1 : 0;
  satFlags[5] = !bb[5] ? 1 : 0;
  satFlags[6] = !bb[6] ? 1 : 0;
  satFlags[7] = !bb[7] ? 1 : 0;
  satFlags[8] = !bb[8] ? 1 : 0;
  satFlags[9] = !bb[9] ? 1 : 0;
  satFlags[10] = !bb[10] ? 1 : 0;
  satFlags[11] = !bb[11] ? 1 : 0;
  satFlags[12] = !bb[12] ? 1 : 0;
  satFlags[13] = !bb[13] ? 1 : 0;
  satFlags[14] = !bb[14] ? 1 : 0;
  satFlags[15] = !bb[15] ? 1 : 0;
  satFlags[16] = !bb[16] ? 1 : 0;
  satFlags[17] = !bb[17] ? 1 : 0;
  satFlags[18] = !bb[18] ? 1 : 0;
  satFlags[19] = !bb[19] ? 1 : 0;
}

const NeAssertData sm_vehicle_2axle_heave_roll_vtk_836bb176_1_assertData[20] = {
  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Px/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Px/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Px/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Px/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Py/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Py/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Py/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Py/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Pz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Pz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Pz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Body to World/Body World Joint", 0,
    0, "Vehicle.Body_to_World.Body_World_Joint", "", false,
    "The parameter Pz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute FL", 0, 0,
    "Vehicle.Revolute_FL", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute FL", 0, 0,
    "Vehicle.Revolute_FL", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute FR", 0, 0,
    "Vehicle.Revolute_FR", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute FR", 0, 0,
    "Vehicle.Revolute_FR", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute RL", 0, 0,
    "Vehicle.Revolute_RL", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute RL", 0, 0,
    "Vehicle.Revolute_RL", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute RR", 0, 0,
    "Vehicle.Revolute_RR", "", false,
    "The parameter Rz/Velocity contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "sm_vehicle_2axle_heave_roll_vtk/Vehicle/Revolute RR", 0, 0,
    "Vehicle.Revolute_RR", "", false,
    "The parameter Rz/Velocity contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" }
};
