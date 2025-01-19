/*
 ============================================================================
 Name        : sm_vehicle.c
 Author      : 
 Version     :
 Copyright   : Maschinenfabrik Bernard KRONE GmbH & Co. KG
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include <windows.h>
#include "sm_vehicle_2axle_heave_roll.h" /* Model header file */


static RT_MODEL_sm_vehicle_2axle_hea_T sm_vehicle_2axle_heave_roll_M_;
static RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_rol_MPtr =
  &sm_vehicle_2axle_heave_roll_M_;     /* Real-time model */
static B_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_B;/* Observable signals */
static DW_sm_vehicle_2axle_heave_rol_T sm_vehicle_2axle_heave_roll_DW;/* Observable states */
static X_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_X;/* Observable continuous states */
static XDis_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_rol_XDis;/* Continuous states Disabled */
static ExtU_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_U;/* External inputs */
static ExtY_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_Y;/* External outputs */

typedef void (*InitFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*, ExtU_sm_vehicle_2axle_heave_r_T*, ExtY_sm_vehicle_2axle_heave_r_T*);
typedef void (*StepFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*, ExtY_sm_vehicle_2axle_heave_r_T*);
typedef void (*TermFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*); // Example function pointer type

/*
 * The example main function illustrates what is required by your
 * application code to initialize, execute, and terminate the generated code.
 * Attaching rt_OneStep to a real-time clock is target specific. This example
 * illustrates how you do this relative to initializing the model.
 */
int_T main(int_T argc, const char *argv[])
{

  /* Unused arguments */
  (void)(argc);
  (void)(argv);

  HINSTANCE hDLL = LoadLibrary("sm_vehicle_2axle_heave_roll_win64.dll");
  if (hDLL != NULL) {
  	// Get the function address
  	InitFunc sm_vehicle_2axle_heave_roll_initialize = (InitFunc)GetProcAddress(hDLL, "sm_vehicle_2axle_heave_roll_initialize");
  	StepFunc sm_vehicle_2axle_heave_roll_step =  (StepFunc)GetProcAddress(hDLL, "sm_vehicle_2axle_heave_roll_step");
  	TermFunc sm_vehicle_2axle_heave_roll_terminate = (TermFunc)GetProcAddress(hDLL, "sm_vehicle_2axle_heave_roll_terminate");

  	if ( (sm_vehicle_2axle_heave_roll_initialize != NULL)
  			&& (sm_vehicle_2axle_heave_roll_step != NULL)
			&& (sm_vehicle_2axle_heave_roll_terminate != NULL))
  	{
  	  RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M = sm_vehicle_2axle_heave_rol_MPtr;

	  /* Pack model data into RTM */
	  sm_vehicle_2axle_heave_roll_M->blockIO = &sm_vehicle_2axle_heave_roll_B;
	  sm_vehicle_2axle_heave_roll_M->dwork = &sm_vehicle_2axle_heave_roll_DW;
	  sm_vehicle_2axle_heave_roll_M->contStates = &sm_vehicle_2axle_heave_roll_X;
	  sm_vehicle_2axle_heave_roll_M->contStateDisabled = &sm_vehicle_2axle_heave_rol_XDis;

	  /* Initialize model */
	  sm_vehicle_2axle_heave_roll_initialize(sm_vehicle_2axle_heave_roll_M, &sm_vehicle_2axle_heave_roll_U, &sm_vehicle_2axle_heave_roll_Y);

	  /* Step the model */
	  sm_vehicle_2axle_heave_roll_step(sm_vehicle_2axle_heave_roll_M, &sm_vehicle_2axle_heave_roll_Y);

	  /* Terminate model */
	  sm_vehicle_2axle_heave_roll_terminate(sm_vehicle_2axle_heave_roll_M);

	} else {
		printf("Failed to get function address.\n");
	}

  	FreeLibrary(hDLL);

  } else {
  	puts("Error");
  	// Handle error if DLL fails to load
  }




  return 0;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
