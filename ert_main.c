/*
 * File: ert_main.c
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

#include <stddef.h>
#include <stdio.h>            /* This example main program uses printf/fflush */
#include <windows.h>
#include <time.h>
#include "sm_vehicle_2axle_heave_roll.h" /* Model header file */

static RT_MODEL_sm_vehicle_2axle_hea_T sm_vehicle_2axle_heave_roll_M_;
static RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_rol_MPtr = &sm_vehicle_2axle_heave_roll_M_;     /* Real-time model */
static B_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_B;/* Observable signals */
static DW_sm_vehicle_2axle_heave_rol_T sm_vehicle_2axle_heave_roll_DW;/* Observable states */
static X_sm_vehicle_2axle_heave_roll_T sm_vehicle_2axle_heave_roll_X;/* Observable continuous states */
static XDis_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_rol_XDis;/* Continuous states Disabled */
static ExtU_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_U;/* External inputs */
static ExtY_sm_vehicle_2axle_heave_r_T sm_vehicle_2axle_heave_roll_Y;/* External outputs */

typedef void (*InitFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*, ExtU_sm_vehicle_2axle_heave_r_T*, ExtY_sm_vehicle_2axle_heave_r_T*);
typedef void (*StepFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*, ExtY_sm_vehicle_2axle_heave_r_T*);
typedef void (*TermFunc)(const RT_MODEL_sm_vehicle_2axle_hea_T*);

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
		sm_vehicle_2axle_heave_roll_U.u.trqFL = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqFR = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqRL = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqRR = (real_T)100;

		int i;
		clock_t t;
		for (i=0; i<100; ++i)
		{
			t = clock();
			sm_vehicle_2axle_heave_roll_step(sm_vehicle_2axle_heave_roll_M, &sm_vehicle_2axle_heave_roll_Y);
			t = clock() - t;
			double time_taken = 1000000*((double)t)/CLOCKS_PER_SEC;
			printf("Step %d: x = %0.3f; elapsed time: %0.6f us\n", i, sm_vehicle_2axle_heave_roll_Y.y.Body.x, time_taken);
		}

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
