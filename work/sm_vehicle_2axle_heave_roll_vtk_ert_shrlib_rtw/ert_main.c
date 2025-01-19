/*
 * File: ert_main.c
 *
 * Code generated for Simulink model 'sm_vehicle_2axle_heave_roll_vtk'.
 *
 * Model version                  : 10.37
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Sun Jan  7 00:24:25 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include "sm_vehicle_2axle_heave_roll_vtk.h" /* Model header file */

static RT_MODEL_sm_vehicle_2axle_hea_T sm_vehicle_2axle_heave_roll__M_;
static RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_rol_MPtr =
  &sm_vehicle_2axle_heave_roll__M_;    /* Real-time model */
static B_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_v_B;/* Observable signals */
static DW_sm_vehicle_2axle_heave_rol_T sm_vehicle_2axle_heave_roll__DW;/* Observable states */
static X_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_v_X;/* Observable continuous states */
static XDis_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_rol_XDis;/* Continuous states Disabled */
static ExtU_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_v_U;/* External inputs */
static ExtY_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_v_Y;/* External outputs */

/*
 * Associating rt_OneStep with a real-time clock or interrupt service routine
 * is what makes the generated code "real-time".  The function rt_OneStep is
 * always associated with the base rate of the model.  Subrates are managed
 * by the base rate from inside the generated code.  Enabling/disabling
 * interrupts and floating point context switches are target specific.  This
 * example code indicates where these should take place relative to executing
 * the generated code step function.  Overrun behavior should be tailored to
 * your application needs.  This example simply sets an error status in the
 * real-time model and returns from rt_OneStep.
 */
void rt_OneStep(RT_MODEL_sm_vehicle_2axle_hea_T *const
                sm_vehicle_2axle_heave_roll__M);
void rt_OneStep(RT_MODEL_sm_vehicle_2axle_hea_T *const
                sm_vehicle_2axle_heave_roll__M)
{
  static boolean_T OverrunFlag = false;

  /* Disable interrupts here */

  /* Check for overrun */
  if (OverrunFlag) {
    rtmSetErrorStatus(sm_vehicle_2axle_heave_roll__M, "Overrun");
    return;
  }

  OverrunFlag = true;

  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  /* Set model inputs here */

  /* Step the model */
  sm_vehicle_2axle_heave_roll_vtk_step(sm_vehicle_2axle_heave_roll__M,
    &sm_vehicle_2axle_heave_roll_v_U, &sm_vehicle_2axle_heave_roll_v_Y);

  /* Get model outputs here */

  /* Indicate task complete */
  OverrunFlag = false;

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

/*
 * The example main function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific. This example
 * illustrates how you do this relative to initializing the model.
 */
int_T main(int_T argc, const char *argv[])
{
  RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll__M =
    sm_vehicle_2axle_heave_rol_MPtr;

  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  /* Pack model data into RTM */
  sm_vehicle_2axle_heave_roll__M->blockIO = &sm_vehicle_2axle_heave_roll_v_B;
  sm_vehicle_2axle_heave_roll__M->dwork = &sm_vehicle_2axle_heave_roll__DW;
  sm_vehicle_2axle_heave_roll__M->contStates = &sm_vehicle_2axle_heave_roll_v_X;
  sm_vehicle_2axle_heave_roll__M->contStateDisabled =
    &sm_vehicle_2axle_heave_rol_XDis;

  /* Initialize model */
  sm_vehicle_2axle_heave_roll_vtk_initialize(sm_vehicle_2axle_heave_roll__M,
    &sm_vehicle_2axle_heave_roll_v_U, &sm_vehicle_2axle_heave_roll_v_Y);

  /* Simulating the model step behavior (in non real-time) to
   *  simulate model behavior at stop time.
   */
  while ((rtmGetErrorStatus(sm_vehicle_2axle_heave_roll__M) == (NULL)) &&
         !rtmGetStopRequested(sm_vehicle_2axle_heave_roll__M)) {
    rt_OneStep(sm_vehicle_2axle_heave_roll__M);
  }

  /* Terminate model */
  sm_vehicle_2axle_heave_roll_vtk_terminate(sm_vehicle_2axle_heave_roll__M);
  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
