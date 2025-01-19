/*
 * File: sm_vehicle_2axle_heave_roll.h
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

#ifndef RTW_HEADER_sm_vehicle_2axle_heave_roll_h_
#define RTW_HEADER_sm_vehicle_2axle_heave_roll_h_
#ifndef sm_vehicle_2axle_heave_roll_COMMON_INCLUDES_
#define sm_vehicle_2axle_heave_roll_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "nesl_rtw.h"
#include "sm_vehicle_2axle_heave_roll_836bb176_1_gateway.h"
#endif                        /* sm_vehicle_2axle_heave_roll_COMMON_INCLUDES_ */

#include "sm_vehicle_2axle_heave_roll_types.h"
#include "rt_nonfinite.h"
#include "rtGetNaN.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Ackermanleft;                 /* '<S133>/Ackerman left' */
  real_T INPUT_6_1_1[4];               /* '<S207>/INPUT_6_1_1' */
  real_T Ackermanright;                /* '<S133>/Ackerman right' */
  real_T INPUT_5_1_1[4];               /* '<S207>/INPUT_5_1_1' */
  real_T INPUT_7_1_1[4];               /* '<S207>/INPUT_7_1_1' */
  real_T INPUT_8_1_1[4];               /* '<S207>/INPUT_8_1_1' */
  real_T INPUT_9_1_1[4];               /* '<S207>/INPUT_9_1_1' */
  real_T INPUT_12_1_1[4];              /* '<S207>/INPUT_12_1_1' */
  real_T INPUT_10_1_1[4];              /* '<S207>/INPUT_10_1_1' */
  real_T INPUT_11_1_1[4];              /* '<S207>/INPUT_11_1_1' */
  real_T INPUT_13_1_1[4];              /* '<S207>/INPUT_13_1_1' */
  real_T INPUT_14_1_1[4];              /* '<S207>/INPUT_14_1_1' */
  real_T INPUT_15_1_1[4];              /* '<S207>/INPUT_15_1_1' */
  real_T INPUT_18_1_1[4];              /* '<S207>/INPUT_18_1_1' */
  real_T INPUT_16_1_1[4];              /* '<S207>/INPUT_16_1_1' */
  real_T INPUT_17_1_1[4];              /* '<S207>/INPUT_17_1_1' */
  real_T INPUT_19_1_1[4];              /* '<S207>/INPUT_19_1_1' */
  real_T INPUT_20_1_1[4];              /* '<S207>/INPUT_20_1_1' */
  real_T INPUT_21_1_1[4];              /* '<S207>/INPUT_21_1_1' */
  real_T INPUT_24_1_1[4];              /* '<S207>/INPUT_24_1_1' */
  real_T INPUT_22_1_1[4];              /* '<S207>/INPUT_22_1_1' */
  real_T INPUT_23_1_1[4];              /* '<S207>/INPUT_23_1_1' */
  real_T INPUT_25_1_1[4];              /* '<S207>/INPUT_25_1_1' */
  real_T INPUT_26_1_1[4];              /* '<S207>/INPUT_26_1_1' */
  real_T INPUT_27_1_1[4];              /* '<S207>/INPUT_27_1_1' */
  real_T INPUT_30_1_1[4];              /* '<S207>/INPUT_30_1_1' */
  real_T INPUT_28_1_1[4];              /* '<S207>/INPUT_28_1_1' */
  real_T INPUT_29_1_1[4];              /* '<S207>/INPUT_29_1_1' */
  real_T STATE_1[29];                  /* '<S207>/STATE_1' */
  real_T INPUT_4_1_1[4];               /* '<S207>/INPUT_4_1_1' */
  real_T INPUT_1_1_1[4];               /* '<S207>/INPUT_1_1_1' */
  real_T INPUT_3_1_1[4];               /* '<S207>/INPUT_3_1_1' */
  real_T INPUT_2_1_1[4];               /* '<S207>/INPUT_2_1_1' */
  real_T TrigonometricFunction1;       /* '<S52>/Trigonometric Function1' */
  real_T Flipsignforxaxis;             /* '<S40>/Flip sign for -x axis ' */
} B_sm_vehicle_2axle_heave_roll_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T INPUT_6_1_1_Discrete;         /* '<S207>/INPUT_6_1_1' */
  real_T INPUT_6_1_1_FirstOutput;      /* '<S207>/INPUT_6_1_1' */
  real_T INPUT_5_1_1_Discrete;         /* '<S207>/INPUT_5_1_1' */
  real_T INPUT_5_1_1_FirstOutput;      /* '<S207>/INPUT_5_1_1' */
  real_T INPUT_7_1_1_Discrete[2];      /* '<S207>/INPUT_7_1_1' */
  real_T INPUT_8_1_1_Discrete[2];      /* '<S207>/INPUT_8_1_1' */
  real_T INPUT_9_1_1_Discrete[2];      /* '<S207>/INPUT_9_1_1' */
  real_T INPUT_12_1_1_Discrete[2];     /* '<S207>/INPUT_12_1_1' */
  real_T INPUT_10_1_1_Discrete[2];     /* '<S207>/INPUT_10_1_1' */
  real_T INPUT_11_1_1_Discrete[2];     /* '<S207>/INPUT_11_1_1' */
  real_T INPUT_13_1_1_Discrete[2];     /* '<S207>/INPUT_13_1_1' */
  real_T INPUT_14_1_1_Discrete[2];     /* '<S207>/INPUT_14_1_1' */
  real_T INPUT_15_1_1_Discrete[2];     /* '<S207>/INPUT_15_1_1' */
  real_T INPUT_18_1_1_Discrete[2];     /* '<S207>/INPUT_18_1_1' */
  real_T INPUT_16_1_1_Discrete[2];     /* '<S207>/INPUT_16_1_1' */
  real_T INPUT_17_1_1_Discrete[2];     /* '<S207>/INPUT_17_1_1' */
  real_T INPUT_19_1_1_Discrete[2];     /* '<S207>/INPUT_19_1_1' */
  real_T INPUT_20_1_1_Discrete[2];     /* '<S207>/INPUT_20_1_1' */
  real_T INPUT_21_1_1_Discrete[2];     /* '<S207>/INPUT_21_1_1' */
  real_T INPUT_24_1_1_Discrete[2];     /* '<S207>/INPUT_24_1_1' */
  real_T INPUT_22_1_1_Discrete[2];     /* '<S207>/INPUT_22_1_1' */
  real_T INPUT_23_1_1_Discrete[2];     /* '<S207>/INPUT_23_1_1' */
  real_T INPUT_25_1_1_Discrete[2];     /* '<S207>/INPUT_25_1_1' */
  real_T INPUT_26_1_1_Discrete[2];     /* '<S207>/INPUT_26_1_1' */
  real_T INPUT_27_1_1_Discrete[2];     /* '<S207>/INPUT_27_1_1' */
  real_T INPUT_30_1_1_Discrete[2];     /* '<S207>/INPUT_30_1_1' */
  real_T INPUT_28_1_1_Discrete[2];     /* '<S207>/INPUT_28_1_1' */
  real_T INPUT_29_1_1_Discrete[2];     /* '<S207>/INPUT_29_1_1' */
  real_T INPUT_4_1_1_Discrete[2];      /* '<S207>/INPUT_4_1_1' */
  real_T INPUT_1_1_1_Discrete[2];      /* '<S207>/INPUT_1_1_1' */
  real_T INPUT_3_1_1_Discrete[2];      /* '<S207>/INPUT_3_1_1' */
  real_T INPUT_2_1_1_Discrete[2];      /* '<S207>/INPUT_2_1_1' */
  real_T STATE_1_Discrete;             /* '<S207>/STATE_1' */
  real_T OUTPUT_1_0_Discrete;          /* '<S207>/OUTPUT_1_0' */
  real_T OUTPUT_1_1_Discrete;          /* '<S207>/OUTPUT_1_1' */
  void* STATE_1_Simulator;             /* '<S207>/STATE_1' */
  void* STATE_1_SimData;               /* '<S207>/STATE_1' */
  void* STATE_1_DiagMgr;               /* '<S207>/STATE_1' */
  void* STATE_1_ZcLogger;              /* '<S207>/STATE_1' */
  void* STATE_1_TsInfo;                /* '<S207>/STATE_1' */
  void* OUTPUT_1_0_Simulator;          /* '<S207>/OUTPUT_1_0' */
  void* OUTPUT_1_0_SimData;            /* '<S207>/OUTPUT_1_0' */
  void* OUTPUT_1_0_DiagMgr;            /* '<S207>/OUTPUT_1_0' */
  void* OUTPUT_1_0_ZcLogger;           /* '<S207>/OUTPUT_1_0' */
  void* OUTPUT_1_0_TsInfo;             /* '<S207>/OUTPUT_1_0' */
  void* SINK_1_RtwLogger;              /* '<S207>/SINK_1' */
  void* SINK_1_RtwLogBuffer;           /* '<S207>/SINK_1' */
  void* SINK_1_RtwLogFcnManager;       /* '<S207>/SINK_1' */
  void* OUTPUT_1_1_Simulator;          /* '<S207>/OUTPUT_1_1' */
  void* OUTPUT_1_1_SimData;            /* '<S207>/OUTPUT_1_1' */
  void* OUTPUT_1_1_DiagMgr;            /* '<S207>/OUTPUT_1_1' */
  void* OUTPUT_1_1_ZcLogger;           /* '<S207>/OUTPUT_1_1' */
  void* OUTPUT_1_1_TsInfo;             /* '<S207>/OUTPUT_1_1' */
  int_T STATE_1_Modes;                 /* '<S207>/STATE_1' */
  int_T OUTPUT_1_0_Modes;              /* '<S207>/OUTPUT_1_0' */
  int_T OUTPUT_1_1_Modes;              /* '<S207>/OUTPUT_1_1' */
  boolean_T STATE_1_FirstOutput;       /* '<S207>/STATE_1' */
  boolean_T OUTPUT_1_0_FirstOutput;    /* '<S207>/OUTPUT_1_0' */
  boolean_T OUTPUT_1_1_FirstOutput;    /* '<S207>/OUTPUT_1_1' */
} DW_sm_vehicle_2axle_heave_rol_T;

/* Continuous states (default storage) */
typedef struct {
  real_T sm_vehicle_2axle_heave_rollVehi[2];/* '<S207>/INPUT_6_1_1' */
  real_T sm_vehicle_2axle_heave_rollVe_d[2];/* '<S207>/INPUT_5_1_1' */
  real_T sm_vehicle_2axle_heave_rollVe_o[29];/* '<S207>/STATE_1' */
  real_T TransferFcn_CSTATE;           /* '<S40>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S40>/Transfer Fcn1' */
} X_sm_vehicle_2axle_heave_roll_T;

/* State derivatives (default storage) */
typedef struct {
  real_T sm_vehicle_2axle_heave_rollVehi[2];/* '<S207>/INPUT_6_1_1' */
  real_T sm_vehicle_2axle_heave_rollVe_d[2];/* '<S207>/INPUT_5_1_1' */
  real_T sm_vehicle_2axle_heave_rollVe_o[29];/* '<S207>/STATE_1' */
  real_T TransferFcn_CSTATE;           /* '<S40>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S40>/Transfer Fcn1' */
} XDot_sm_vehicle_2axle_heave_r_T;

/* State disabled  */
typedef struct {
  boolean_T sm_vehicle_2axle_heave_rollVehi[2];/* '<S207>/INPUT_6_1_1' */
  boolean_T sm_vehicle_2axle_heave_rollVe_d[2];/* '<S207>/INPUT_5_1_1' */
  boolean_T sm_vehicle_2axle_heave_rollVe_o[29];/* '<S207>/STATE_1' */
  boolean_T TransferFcn_CSTATE;        /* '<S40>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S40>/Transfer Fcn1' */
} XDis_sm_vehicle_2axle_heave_r_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  sm_vehicle_2axle_heave_roll_U_t u;   /* '<Root>/u' */
} ExtU_sm_vehicle_2axle_heave_r_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  sm_vehicle_2axle_heave_roll_Y_t y;   /* '<Root>/y' */
} ExtY_sm_vehicle_2axle_heave_r_T;

/* Real-time Model Data Structure */
struct tag_RTM_sm_vehicle_2axle_heav_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  B_sm_vehicle_2axle_heave_roll_T *blockIO;
  X_sm_vehicle_2axle_heave_roll_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_sm_vehicle_2axle_heave_r_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][35];
  ODE1_IntgData intgData;
  DW_sm_vehicle_2axle_heave_rol_T *dwork;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* External data declarations for dependent source files */
extern const sm_vehicle_2axle_heave_roll_U_t sm_vehicle_2axle_heave_roll_rtZ;
                                    /* sm_vehicle_2axle_heave_roll_U_t ground */
extern const sm_vehicle_2axle_heave_roll_Y_t sm_vehicle_2axle_heave_roll_r_0;
                                    /* sm_vehicle_2axle_heave_roll_Y_t ground */

/* Model entry point functions */
extern void sm_vehicle_2axle_heave_roll_initialize
  (RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M,
   ExtU_sm_vehicle_2axle_heave_r_T *sm_vehicle_2axle_heave_roll_U,
   ExtY_sm_vehicle_2axle_heave_r_T *sm_vehicle_2axle_heave_roll_Y);
extern void sm_vehicle_2axle_heave_roll_step(RT_MODEL_sm_vehicle_2axle_hea_T *
  const sm_vehicle_2axle_heave_roll_M, ExtY_sm_vehicle_2axle_heave_r_T
  *sm_vehicle_2axle_heave_roll_Y);
extern void sm_vehicle_2axle_heave_roll_terminate
  (RT_MODEL_sm_vehicle_2axle_hea_T *const sm_vehicle_2axle_heave_roll_M);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<S119>/Constant6' : Unused code path elimination
 * Block '<S119>/Constant7' : Unused code path elimination
 * Block '<S121>/Constant6' : Unused code path elimination
 * Block '<S121>/Constant7' : Unused code path elimination
 * Block '<S123>/Constant6' : Unused code path elimination
 * Block '<S123>/Constant7' : Unused code path elimination
 * Block '<S125>/Constant6' : Unused code path elimination
 * Block '<S125>/Constant7' : Unused code path elimination
 * Block '<S3>/Gain' : Unused code path elimination
 * Block '<S3>/Gain1' : Unused code path elimination
 * Block '<S3>/Gain2' : Unused code path elimination
 * Block '<S3>/Gain3' : Unused code path elimination
 * Block '<S3>/Gain4' : Unused code path elimination
 * Block '<S3>/Gain5' : Unused code path elimination
 * Block '<S3>/Gain6' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope4' : Unused code path elimination
 * Block '<S42>/RESHAPE' : Reshape block reduction
 * Block '<S73>/RESHAPE' : Reshape block reduction
 * Block '<S67>/Reshape' : Reshape block reduction
 * Block '<S67>/Reshape1' : Reshape block reduction
 * Block '<S67>/Reshape2' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'sm_vehicle_2axle_heave_roll'
 * '<S1>'   : 'sm_vehicle_2axle_heave_roll/Driver Input'
 * '<S2>'   : 'sm_vehicle_2axle_heave_roll/Vehicle'
 * '<S3>'   : 'sm_vehicle_2axle_heave_roll/meas'
 * '<S4>'   : 'sm_vehicle_2axle_heave_roll/Driver Input/Step Steer'
 * '<S5>'   : 'sm_vehicle_2axle_heave_roll/Vehicle/Body'
 * '<S6>'   : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World'
 * '<S7>'   : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames'
 * '<S8>'   : 'sm_vehicle_2axle_heave_roll/Vehicle/Outputs'
 * '<S9>'   : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter'
 * '<S10>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter1'
 * '<S11>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter2'
 * '<S12>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter3'
 * '<S13>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FL'
 * '<S14>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FR'
 * '<S15>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RL'
 * '<S16>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RR'
 * '<S17>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Scene'
 * '<S18>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter'
 * '<S19>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter1'
 * '<S20>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter2'
 * '<S21>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter3'
 * '<S22>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front'
 * '<S23>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Rear'
 * '<S24>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL'
 * '<S25>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR'
 * '<S26>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL'
 * '<S27>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR'
 * '<S28>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/World'
 * '<S29>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor'
 * '<S30>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter1'
 * '<S31>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter11'
 * '<S32>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter2'
 * '<S33>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter3'
 * '<S34>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter4'
 * '<S35>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter5'
 * '<S36>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter6'
 * '<S37>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter7'
 * '<S38>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter8'
 * '<S39>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter9'
 * '<S40>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/Roll Pitch'
 * '<S41>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter1/EVAL_KEY'
 * '<S42>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter11/EVAL_KEY'
 * '<S43>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter2/EVAL_KEY'
 * '<S44>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter3/EVAL_KEY'
 * '<S45>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter4/EVAL_KEY'
 * '<S46>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter5/EVAL_KEY'
 * '<S47>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter6/EVAL_KEY'
 * '<S48>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter7/EVAL_KEY'
 * '<S49>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter8/EVAL_KEY'
 * '<S50>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/PS-Simulink Converter9/EVAL_KEY'
 * '<S51>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/Roll Pitch/Calculate aPitch'
 * '<S52>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body/Body Sensor/Roll Pitch/Calculate aRoll'
 * '<S53>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter'
 * '<S54>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter1'
 * '<S55>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter10'
 * '<S56>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter11'
 * '<S57>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter12'
 * '<S58>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter13'
 * '<S59>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter2'
 * '<S60>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter3'
 * '<S61>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter4'
 * '<S62>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter5'
 * '<S63>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter6'
 * '<S64>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter7'
 * '<S65>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter8'
 * '<S66>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter9'
 * '<S67>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/R to X-Y-Z Extrinsic'
 * '<S68>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter/EVAL_KEY'
 * '<S69>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter1/EVAL_KEY'
 * '<S70>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter10/EVAL_KEY'
 * '<S71>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter11/EVAL_KEY'
 * '<S72>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter12/EVAL_KEY'
 * '<S73>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter13/EVAL_KEY'
 * '<S74>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter2/EVAL_KEY'
 * '<S75>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter3/EVAL_KEY'
 * '<S76>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter4/EVAL_KEY'
 * '<S77>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter5/EVAL_KEY'
 * '<S78>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter6/EVAL_KEY'
 * '<S79>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter7/EVAL_KEY'
 * '<S80>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter8/EVAL_KEY'
 * '<S81>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Body to World/PS-Simulink Converter9/EVAL_KEY'
 * '<S82>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Dolly Camera'
 * '<S83>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FL'
 * '<S84>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FR'
 * '<S85>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO FL'
 * '<S86>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO FR'
 * '<S87>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO Front'
 * '<S88>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO Left'
 * '<S89>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO RL'
 * '<S90>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO RR'
 * '<S91>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO Rear'
 * '<S92>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO Right'
 * '<S93>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform FixO Top'
 * '<S94>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Front'
 * '<S95>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Left'
 * '<S96>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform RL'
 * '<S97>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform RR'
 * '<S98>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Rear'
 * '<S99>'  : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Right'
 * '<S100>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform SuspF'
 * '<S101>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform SuspR'
 * '<S102>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Top'
 * '<S103>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Seat FL'
 * '<S104>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Seat FR'
 * '<S105>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Susp FL'
 * '<S106>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Susp FR'
 * '<S107>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Susp RL'
 * '<S108>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform View Susp RR'
 * '<S109>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Wheel FL'
 * '<S110>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Wheel FR'
 * '<S111>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Wheel RL'
 * '<S112>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Transform Wheel RR'
 * '<S113>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Camera Frames/Dolly Camera/Off'
 * '<S114>' : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter/EVAL_KEY'
 * '<S115>' : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter1/EVAL_KEY'
 * '<S116>' : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter2/EVAL_KEY'
 * '<S117>' : 'sm_vehicle_2axle_heave_roll/Vehicle/PS-Simulink Converter3/EVAL_KEY'
 * '<S118>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FL/Flat'
 * '<S119>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FL/Flat/Road'
 * '<S120>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FR/Flat'
 * '<S121>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface FR/Flat/Road'
 * '<S122>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RL/Flat'
 * '<S123>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RL/Flat/Road'
 * '<S124>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RR/Flat'
 * '<S125>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Road Surface RR/Flat/Road'
 * '<S126>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Scene/Grid'
 * '<S127>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Scene/Grid/Mesh Lines x'
 * '<S128>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Scene/Grid/Mesh Lines y '
 * '<S129>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter/EVAL_KEY'
 * '<S130>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter1/EVAL_KEY'
 * '<S131>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter2/EVAL_KEY'
 * '<S132>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Simulink-PS Converter3/EVAL_KEY'
 * '<S133>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front/Steering Wheel to Wheel Angle'
 * '<S134>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter'
 * '<S135>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter1'
 * '<S136>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter/EVAL_KEY'
 * '<S137>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter1/EVAL_KEY'
 * '<S138>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/PS-Simulink Converter'
 * '<S139>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/PS-Simulink Converter1'
 * '<S140>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion'
 * '<S141>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/PS-Simulink Converter/EVAL_KEY'
 * '<S142>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/PS-Simulink Converter1/EVAL_KEY'
 * '<S143>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter'
 * '<S144>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter1'
 * '<S145>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter2'
 * '<S146>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter3'
 * '<S147>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter4'
 * '<S148>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter5'
 * '<S149>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S150>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S151>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S152>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S153>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S154>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S155>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/PS-Simulink Converter'
 * '<S156>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/PS-Simulink Converter1'
 * '<S157>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion'
 * '<S158>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/PS-Simulink Converter/EVAL_KEY'
 * '<S159>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/PS-Simulink Converter1/EVAL_KEY'
 * '<S160>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter'
 * '<S161>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter1'
 * '<S162>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter2'
 * '<S163>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter3'
 * '<S164>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter4'
 * '<S165>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter5'
 * '<S166>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S167>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S168>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S169>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S170>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S171>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S172>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/PS-Simulink Converter'
 * '<S173>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/PS-Simulink Converter1'
 * '<S174>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion'
 * '<S175>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/PS-Simulink Converter/EVAL_KEY'
 * '<S176>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/PS-Simulink Converter1/EVAL_KEY'
 * '<S177>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter'
 * '<S178>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter1'
 * '<S179>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter2'
 * '<S180>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter3'
 * '<S181>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter4'
 * '<S182>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter5'
 * '<S183>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S184>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S185>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S186>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S187>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S188>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S189>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/PS-Simulink Converter'
 * '<S190>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/PS-Simulink Converter1'
 * '<S191>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion'
 * '<S192>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/PS-Simulink Converter/EVAL_KEY'
 * '<S193>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/PS-Simulink Converter1/EVAL_KEY'
 * '<S194>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter'
 * '<S195>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter1'
 * '<S196>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter2'
 * '<S197>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter3'
 * '<S198>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter4'
 * '<S199>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter5'
 * '<S200>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S201>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S202>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S203>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S204>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S205>' : 'sm_vehicle_2axle_heave_roll/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S206>' : 'sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration'
 * '<S207>' : 'sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration/EVAL_KEY'
 */
#endif                           /* RTW_HEADER_sm_vehicle_2axle_heave_roll_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
