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

static RT_MODEL_sm_vehicle_2axle_hea_T sm_vehicle_2axle_heave_roll_M;
// static RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_rol_MPtr = &sm_vehicle_2axle_heave_roll_M_;     /* Real-time model */
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

//	FILE *hf = fopen("C:\\Users\\schwammw\\Desktop\\dumpMain.txt","w");
//	  if (hf) {
//	  	  *stdout = *hf;
//	  	  int i = setvbuf( stdout, NULL, _IONBF, 0 );
//	    }
//
//	  if (hf) {
//		  printf("errorStatus: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.errorStatus - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("solverInfo: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.solverInfo - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("blockIO: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.blockIO - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("contStates: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.contStates - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("periodicContStateIndices: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.periodicContStateIndices - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("periodicContStateRanges: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.periodicContStateRanges - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("derivs: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.derivs - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("contStateDisabled: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.contStateDisabled - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("zCCacheNeedsReset: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.zCCacheNeedsReset - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("derivCacheNeedsReset: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.derivCacheNeedsReset - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("CTOutputIncnstWithState: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.CTOutputIncnstWithState - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("odeF: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.odeF - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("intgData: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.intgData - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("dwork: %d\n", (char *)&sm_vehicle_2axle_heave_roll_M.dwork - (char *)&sm_vehicle_2axle_heave_roll_M);
//		  printf("\n");
//		  printf("sm_vehicle_2axle_heave_rollVehi: %d\n", (char *)&sm_vehicle_2axle_heave_rol_XDis.sm_vehicle_2axle_heave_rollVehi- (char *)&sm_vehicle_2axle_heave_rol_XDis);
//		  printf("sm_vehicle_2axle_heave_rollVe_d: %d\n", (char *)&sm_vehicle_2axle_heave_rol_XDis.sm_vehicle_2axle_heave_rollVe_d- (char *)&sm_vehicle_2axle_heave_rol_XDis);
//		  printf("sm_vehicle_2axle_heave_rollVe_o: %d\n", (char *)&sm_vehicle_2axle_heave_rol_XDis.sm_vehicle_2axle_heave_rollVe_o- (char *)&sm_vehicle_2axle_heave_rol_XDis);
//		  printf("TransferFcn_CSTATE: %d\n", (char *)&sm_vehicle_2axle_heave_rol_XDis.TransferFcn_CSTATE- (char *)&sm_vehicle_2axle_heave_rol_XDis);
//		  printf("TransferFcn1_CSTATE: %d\n", (char *)&sm_vehicle_2axle_heave_rol_XDis.TransferFcn1_CSTATE- (char *)&sm_vehicle_2axle_heave_rol_XDis);
//
//		  fclose(hf);
//	  }

	int sz_M = sizeof(sm_vehicle_2axle_heave_roll_M);
	int sz_M_errorStatus = sizeof(sm_vehicle_2axle_heave_roll_M.errorStatus);
	int sz_M_SolverInfo = sizeof(sm_vehicle_2axle_heave_roll_M.solverInfo);
	int sz_M_blockIO = sizeof(sm_vehicle_2axle_heave_roll_M.blockIO);
	int sz_M_contStates = sizeof(sm_vehicle_2axle_heave_roll_M.contStates);
	int sz_M_periodicContStateIndices = sizeof(sm_vehicle_2axle_heave_roll_M.periodicContStateIndices);
	int sz_M_periodicContStateRanges = sizeof(sm_vehicle_2axle_heave_roll_M.periodicContStateRanges);
	int sz_M_derivs = sizeof(sm_vehicle_2axle_heave_roll_M.derivs);
	int sz_M_contStateDisabled = sizeof(sm_vehicle_2axle_heave_roll_M.contStateDisabled);
	int sz_M_contStateDisabled_ = sizeof(sm_vehicle_2axle_heave_roll_M.contStateDisabled->sm_vehicle_2axle_heave_rollVe_o);
	int sz_M_zCCacheNeedsReset = sizeof(sm_vehicle_2axle_heave_roll_M.zCCacheNeedsReset);
	int sz_M_derivCacheNeedsReset = sizeof(sm_vehicle_2axle_heave_roll_M.derivCacheNeedsReset);
	int sz_M_CTOutputIncnstWithState = sizeof(sm_vehicle_2axle_heave_roll_M.CTOutputIncnstWithState);
	int sz_M_odeF = sizeof(sm_vehicle_2axle_heave_roll_M.odeF);
	int sz_M_intgData = sizeof(sm_vehicle_2axle_heave_roll_M.intgData);
	int sz_M_dwork = sizeof(sm_vehicle_2axle_heave_roll_M.dwork);
	int sz_M_Sizes = sizeof(sm_vehicle_2axle_heave_roll_M.Sizes);
	int sz_M_Timing = sizeof(sm_vehicle_2axle_heave_roll_M.Timing);


	int sz_B = sizeof(sm_vehicle_2axle_heave_roll_B);
	int sz_DW = sizeof(sm_vehicle_2axle_heave_roll_DW);
	int sz_X = sizeof(sm_vehicle_2axle_heave_roll_X);
	int sz_XDis = sizeof(sm_vehicle_2axle_heave_rol_XDis);
	int sz_U = sizeof(sm_vehicle_2axle_heave_roll_U);
	int sz_Y = sizeof(sm_vehicle_2axle_heave_roll_Y);

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
		// RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M = sm_vehicle_2axle_heave_rol_MPtr;
		//(RT_MODEL_sm_vehicle_2axle_hea_T) *(sm_vehicle_2axle_heave_roll_M);

		/* Pack model data into RTM */
		sm_vehicle_2axle_heave_roll_M.blockIO = &sm_vehicle_2axle_heave_roll_B;
		sm_vehicle_2axle_heave_roll_M.dwork = &sm_vehicle_2axle_heave_roll_DW;
		sm_vehicle_2axle_heave_roll_M.contStates = &sm_vehicle_2axle_heave_roll_X;
		sm_vehicle_2axle_heave_roll_M.contStateDisabled = &sm_vehicle_2axle_heave_rol_XDis;

		int sz_M_ = sizeof(sm_vehicle_2axle_heave_roll_M);
		int sz_M_B_ = sizeof(sm_vehicle_2axle_heave_roll_M.blockIO);
		int sz_M_DW_ = sizeof(sm_vehicle_2axle_heave_roll_M.dwork);
		int sz_M_X_ = sizeof(sm_vehicle_2axle_heave_roll_M.contStates);
		int sz_M_XDis_ = sizeof(sm_vehicle_2axle_heave_roll_M.contStateDisabled);

		/* Initialize model */
		sm_vehicle_2axle_heave_roll_initialize(&sm_vehicle_2axle_heave_roll_M, &sm_vehicle_2axle_heave_roll_U, &sm_vehicle_2axle_heave_roll_Y);

		/* Step the model */
		sm_vehicle_2axle_heave_roll_U.u.trqFL = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqFR = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqRL = (real_T)100;
		sm_vehicle_2axle_heave_roll_U.u.trqRR = (real_T)100;

		int i;
		clock_t t0, t;
		t0 = clock();
		for (i=0; i<100; ++i)
		{
			t = clock();
			sm_vehicle_2axle_heave_roll_step(&sm_vehicle_2axle_heave_roll_M, &sm_vehicle_2axle_heave_roll_Y);
			t = clock() - t;
			double time_taken = 1000000*((double)t)/CLOCKS_PER_SEC;
			printf("Step %d: x = %0.3f; elapsed time: %0.6f us\n", i, sm_vehicle_2axle_heave_roll_Y.y.Body.x, time_taken);
		}

		double time_total = 1000*(double)(clock()-t0)/CLOCKS_PER_SEC;
		printf("Total time: %0.3f ms\n", time_total);

		/* Terminate model */
		sm_vehicle_2axle_heave_roll_terminate(&sm_vehicle_2axle_heave_roll_M);

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
