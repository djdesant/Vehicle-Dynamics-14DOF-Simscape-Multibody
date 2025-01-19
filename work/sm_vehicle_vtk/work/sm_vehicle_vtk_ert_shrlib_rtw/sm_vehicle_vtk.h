/*
 * File: sm_vehicle_vtk.h
 *
 * Code generated for Simulink model 'sm_vehicle_vtk'.
 *
 * Model version                  : 11.0
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Aug 19 21:56:35 2024
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: 32-bit Generic
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef sm_vehicle_vtk_h_
#define sm_vehicle_vtk_h_
#ifndef sm_vehicle_vtk_COMMON_INCLUDES_
#define sm_vehicle_vtk_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#include "nesl_rtw_rtp.h"
#include "sm_vehicle_vtk_836bb176_1_gateway.h"
#include "nesl_rtw.h"
#endif                                 /* sm_vehicle_vtk_COMMON_INCLUDES_ */

#include "sm_vehicle_vtk_types.h"
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
  real_T Ackermanleft;                 /* '<S137>/Ackerman left' */
  real_T INPUT_6_1_1[4];               /* '<S279>/INPUT_6_1_1' */
  real_T Ackermanright;                /* '<S137>/Ackerman right' */
  real_T INPUT_5_1_1[4];               /* '<S279>/INPUT_5_1_1' */
  real_T INPUT_7_1_1[4];               /* '<S279>/INPUT_7_1_1' */
  real_T INPUT_8_1_1[4];               /* '<S279>/INPUT_8_1_1' */
  real_T INPUT_9_1_1[4];               /* '<S279>/INPUT_9_1_1' */
  real_T INPUT_12_1_1[4];              /* '<S279>/INPUT_12_1_1' */
  real_T INPUT_10_1_1[4];              /* '<S279>/INPUT_10_1_1' */
  real_T INPUT_11_1_1[4];              /* '<S279>/INPUT_11_1_1' */
  real_T INPUT_13_1_1[4];              /* '<S279>/INPUT_13_1_1' */
  real_T INPUT_14_1_1[4];              /* '<S279>/INPUT_14_1_1' */
  real_T INPUT_15_1_1[4];              /* '<S279>/INPUT_15_1_1' */
  real_T INPUT_18_1_1[4];              /* '<S279>/INPUT_18_1_1' */
  real_T INPUT_16_1_1[4];              /* '<S279>/INPUT_16_1_1' */
  real_T INPUT_17_1_1[4];              /* '<S279>/INPUT_17_1_1' */
  real_T INPUT_19_1_1[4];              /* '<S279>/INPUT_19_1_1' */
  real_T INPUT_20_1_1[4];              /* '<S279>/INPUT_20_1_1' */
  real_T INPUT_21_1_1[4];              /* '<S279>/INPUT_21_1_1' */
  real_T INPUT_24_1_1[4];              /* '<S279>/INPUT_24_1_1' */
  real_T INPUT_22_1_1[4];              /* '<S279>/INPUT_22_1_1' */
  real_T INPUT_23_1_1[4];              /* '<S279>/INPUT_23_1_1' */
  real_T INPUT_25_1_1[4];              /* '<S279>/INPUT_25_1_1' */
  real_T INPUT_26_1_1[4];              /* '<S279>/INPUT_26_1_1' */
  real_T INPUT_27_1_1[4];              /* '<S279>/INPUT_27_1_1' */
  real_T INPUT_30_1_1[4];              /* '<S279>/INPUT_30_1_1' */
  real_T INPUT_28_1_1[4];              /* '<S279>/INPUT_28_1_1' */
  real_T INPUT_29_1_1[4];              /* '<S279>/INPUT_29_1_1' */
  real_T RTP_1;                        /* '<S278>/RTP_1' */
  real_T STATE_1[29];                  /* '<S279>/STATE_1' */
  real_T OUTPUT_1_0[221];              /* '<S279>/OUTPUT_1_0' */
  real_T INPUT_4_1_1[4];               /* '<S279>/INPUT_4_1_1' */
  real_T INPUT_1_1_1[4];               /* '<S279>/INPUT_1_1_1' */
  real_T INPUT_3_1_1[4];               /* '<S279>/INPUT_3_1_1' */
  real_T INPUT_2_1_1[4];               /* '<S279>/INPUT_2_1_1' */
  real_T OUTPUT_1_1[6];                /* '<S279>/OUTPUT_1_1' */
  real_T TrigonometricFunction1;       /* '<S53>/Trigonometric Function1' */
  real_T Flipsignforxaxis;             /* '<S41>/Flip sign for -x axis ' */
  real_T Selector1[3];                 /* '<S124>/Selector1' */
  real_T Atan2_e;                      /* '<S125>/Atan2' */
  real_T Atan1_b;                      /* '<S125>/Atan1' */
  real_T Asin_d;                       /* '<S125>/Asin' */
  real_T Selector1_i[3];               /* '<S126>/Selector1' */
  real_T Atan2_n;                      /* '<S127>/Atan2' */
  real_T Atan1_i;                      /* '<S127>/Atan1' */
  real_T Asin_j;                       /* '<S127>/Asin' */
  real_T Selector1_k[3];               /* '<S128>/Selector1' */
  real_T Atan2_k;                      /* '<S129>/Atan2' */
  real_T Atan1_g;                      /* '<S129>/Atan1' */
  real_T Asin_o;                       /* '<S129>/Asin' */
  real_T Selector1_l[3];               /* '<S130>/Selector1' */
  real_T Atan2_h;                      /* '<S131>/Atan2' */
  real_T Atan1_n;                      /* '<S131>/Atan1' */
  real_T Asin_f;                       /* '<S131>/Asin' */
} B_sm_vehicle_vtk_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T INPUT_6_1_1_Discrete;         /* '<S279>/INPUT_6_1_1' */
  real_T INPUT_6_1_1_FirstOutput;      /* '<S279>/INPUT_6_1_1' */
  real_T INPUT_5_1_1_Discrete;         /* '<S279>/INPUT_5_1_1' */
  real_T INPUT_5_1_1_FirstOutput;      /* '<S279>/INPUT_5_1_1' */
  real_T INPUT_7_1_1_Discrete[2];      /* '<S279>/INPUT_7_1_1' */
  real_T INPUT_8_1_1_Discrete[2];      /* '<S279>/INPUT_8_1_1' */
  real_T INPUT_9_1_1_Discrete[2];      /* '<S279>/INPUT_9_1_1' */
  real_T INPUT_12_1_1_Discrete[2];     /* '<S279>/INPUT_12_1_1' */
  real_T INPUT_10_1_1_Discrete[2];     /* '<S279>/INPUT_10_1_1' */
  real_T INPUT_11_1_1_Discrete[2];     /* '<S279>/INPUT_11_1_1' */
  real_T INPUT_13_1_1_Discrete[2];     /* '<S279>/INPUT_13_1_1' */
  real_T INPUT_14_1_1_Discrete[2];     /* '<S279>/INPUT_14_1_1' */
  real_T INPUT_15_1_1_Discrete[2];     /* '<S279>/INPUT_15_1_1' */
  real_T INPUT_18_1_1_Discrete[2];     /* '<S279>/INPUT_18_1_1' */
  real_T INPUT_16_1_1_Discrete[2];     /* '<S279>/INPUT_16_1_1' */
  real_T INPUT_17_1_1_Discrete[2];     /* '<S279>/INPUT_17_1_1' */
  real_T INPUT_19_1_1_Discrete[2];     /* '<S279>/INPUT_19_1_1' */
  real_T INPUT_20_1_1_Discrete[2];     /* '<S279>/INPUT_20_1_1' */
  real_T INPUT_21_1_1_Discrete[2];     /* '<S279>/INPUT_21_1_1' */
  real_T INPUT_24_1_1_Discrete[2];     /* '<S279>/INPUT_24_1_1' */
  real_T INPUT_22_1_1_Discrete[2];     /* '<S279>/INPUT_22_1_1' */
  real_T INPUT_23_1_1_Discrete[2];     /* '<S279>/INPUT_23_1_1' */
  real_T INPUT_25_1_1_Discrete[2];     /* '<S279>/INPUT_25_1_1' */
  real_T INPUT_26_1_1_Discrete[2];     /* '<S279>/INPUT_26_1_1' */
  real_T INPUT_27_1_1_Discrete[2];     /* '<S279>/INPUT_27_1_1' */
  real_T INPUT_30_1_1_Discrete[2];     /* '<S279>/INPUT_30_1_1' */
  real_T INPUT_28_1_1_Discrete[2];     /* '<S279>/INPUT_28_1_1' */
  real_T INPUT_29_1_1_Discrete[2];     /* '<S279>/INPUT_29_1_1' */
  real_T INPUT_4_1_1_Discrete[2];      /* '<S279>/INPUT_4_1_1' */
  real_T INPUT_1_1_1_Discrete[2];      /* '<S279>/INPUT_1_1_1' */
  real_T INPUT_3_1_1_Discrete[2];      /* '<S279>/INPUT_3_1_1' */
  real_T INPUT_2_1_1_Discrete[2];      /* '<S279>/INPUT_2_1_1' */
  real_T STATE_1_Discrete;             /* '<S279>/STATE_1' */
  real_T STATE_1_ZcValueStore;         /* '<S279>/STATE_1' */
  real_T OUTPUT_1_0_Discrete;          /* '<S279>/OUTPUT_1_0' */
  real_T OUTPUT_1_0_ZcValueStore;      /* '<S279>/OUTPUT_1_0' */
  real_T OUTPUT_1_1_Discrete;          /* '<S279>/OUTPUT_1_1' */
  real_T OUTPUT_1_1_ZcValueStore;      /* '<S279>/OUTPUT_1_1' */
  void* RTP_1_RtpManager;              /* '<S278>/RTP_1' */
  void* STATE_1_Simulator;             /* '<S279>/STATE_1' */
  void* STATE_1_SimData;               /* '<S279>/STATE_1' */
  void* STATE_1_DiagMgr;               /* '<S279>/STATE_1' */
  void* STATE_1_ZcLogger;              /* '<S279>/STATE_1' */
  void* STATE_1_TsInfo;                /* '<S279>/STATE_1' */
  void* OUTPUT_1_0_Simulator;          /* '<S279>/OUTPUT_1_0' */
  void* OUTPUT_1_0_SimData;            /* '<S279>/OUTPUT_1_0' */
  void* OUTPUT_1_0_DiagMgr;            /* '<S279>/OUTPUT_1_0' */
  void* OUTPUT_1_0_ZcLogger;           /* '<S279>/OUTPUT_1_0' */
  void* OUTPUT_1_0_TsInfo;             /* '<S279>/OUTPUT_1_0' */
  void* SINK_1_RtwLogger;              /* '<S279>/SINK_1' */
  void* SINK_1_RtwLogBuffer;           /* '<S279>/SINK_1' */
  void* SINK_1_RtwLogFcnManager;       /* '<S279>/SINK_1' */
  void* OUTPUT_1_1_Simulator;          /* '<S279>/OUTPUT_1_1' */
  void* OUTPUT_1_1_SimData;            /* '<S279>/OUTPUT_1_1' */
  void* OUTPUT_1_1_DiagMgr;            /* '<S279>/OUTPUT_1_1' */
  void* OUTPUT_1_1_ZcLogger;           /* '<S279>/OUTPUT_1_1' */
  void* OUTPUT_1_1_TsInfo;             /* '<S279>/OUTPUT_1_1' */
  int_T STATE_1_Modes;                 /* '<S279>/STATE_1' */
  int_T OUTPUT_1_0_Modes;              /* '<S279>/OUTPUT_1_0' */
  int_T OUTPUT_1_1_Modes;              /* '<S279>/OUTPUT_1_1' */
  uint8_T STATE_1_ZcSignalDir;         /* '<S279>/STATE_1' */
  uint8_T STATE_1_ZcStateStore;        /* '<S279>/STATE_1' */
  uint8_T OUTPUT_1_0_ZcSignalDir;      /* '<S279>/OUTPUT_1_0' */
  uint8_T OUTPUT_1_0_ZcStateStore;     /* '<S279>/OUTPUT_1_0' */
  uint8_T OUTPUT_1_1_ZcSignalDir;      /* '<S279>/OUTPUT_1_1' */
  uint8_T OUTPUT_1_1_ZcStateStore;     /* '<S279>/OUTPUT_1_1' */
  boolean_T RTP_1_SetParametersNeeded; /* '<S278>/RTP_1' */
  boolean_T STATE_1_FirstOutput;       /* '<S279>/STATE_1' */
  boolean_T OUTPUT_1_0_FirstOutput;    /* '<S279>/OUTPUT_1_0' */
  boolean_T OUTPUT_1_1_FirstOutput;    /* '<S279>/OUTPUT_1_1' */
} DW_sm_vehicle_vtk_T;

/* Continuous states (default storage) */
typedef struct {
  real_T sm_vehicle_vtkVehicleSuspension[2];/* '<S279>/INPUT_6_1_1' */
  real_T sm_vehicle_vtkVehicleSuspensi_i[2];/* '<S279>/INPUT_5_1_1' */
  real_T TransferFcn_CSTATE;           /* '<S124>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S124>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S124>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE;          /* '<S124>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE;          /* '<S124>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE;          /* '<S124>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_p;         /* '<S126>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_o;        /* '<S126>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_j;        /* '<S126>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_o;        /* '<S126>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_j;        /* '<S126>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_n;        /* '<S126>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_e;         /* '<S128>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_d;        /* '<S128>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_c;        /* '<S128>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_j;        /* '<S128>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_b;        /* '<S128>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_h;        /* '<S128>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_n;         /* '<S130>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_k;        /* '<S130>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_cy;       /* '<S130>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_f;        /* '<S130>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_f;        /* '<S130>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_g;        /* '<S130>/Transfer Fcn4' */
  real_T sm_vehicle_vtkVehicleCamera_Fra[29];/* '<S279>/STATE_1' */
  real_T TransferFcn_CSTATE_b;         /* '<S41>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_a;        /* '<S41>/Transfer Fcn1' */
} X_sm_vehicle_vtk_T;

/* State derivatives (default storage) */
typedef struct {
  real_T sm_vehicle_vtkVehicleSuspension[2];/* '<S279>/INPUT_6_1_1' */
  real_T sm_vehicle_vtkVehicleSuspensi_i[2];/* '<S279>/INPUT_5_1_1' */
  real_T TransferFcn_CSTATE;           /* '<S124>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S124>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S124>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE;          /* '<S124>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE;          /* '<S124>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE;          /* '<S124>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_p;         /* '<S126>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_o;        /* '<S126>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_j;        /* '<S126>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_o;        /* '<S126>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_j;        /* '<S126>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_n;        /* '<S126>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_e;         /* '<S128>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_d;        /* '<S128>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_c;        /* '<S128>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_j;        /* '<S128>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_b;        /* '<S128>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_h;        /* '<S128>/Transfer Fcn4' */
  real_T TransferFcn_CSTATE_n;         /* '<S130>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_k;        /* '<S130>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE_cy;       /* '<S130>/Transfer Fcn2' */
  real_T TransferFcn3_CSTATE_f;        /* '<S130>/Transfer Fcn3' */
  real_T TransferFcn5_CSTATE_f;        /* '<S130>/Transfer Fcn5' */
  real_T TransferFcn4_CSTATE_g;        /* '<S130>/Transfer Fcn4' */
  real_T sm_vehicle_vtkVehicleCamera_Fra[29];/* '<S279>/STATE_1' */
  real_T TransferFcn_CSTATE_b;         /* '<S41>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE_a;        /* '<S41>/Transfer Fcn1' */
} XDot_sm_vehicle_vtk_T;

/* State disabled  */
typedef struct {
  boolean_T sm_vehicle_vtkVehicleSuspension[2];/* '<S279>/INPUT_6_1_1' */
  boolean_T sm_vehicle_vtkVehicleSuspensi_i[2];/* '<S279>/INPUT_5_1_1' */
  boolean_T TransferFcn_CSTATE;        /* '<S124>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S124>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE;       /* '<S124>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE;       /* '<S124>/Transfer Fcn3' */
  boolean_T TransferFcn5_CSTATE;       /* '<S124>/Transfer Fcn5' */
  boolean_T TransferFcn4_CSTATE;       /* '<S124>/Transfer Fcn4' */
  boolean_T TransferFcn_CSTATE_p;      /* '<S126>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_o;     /* '<S126>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_j;     /* '<S126>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE_o;     /* '<S126>/Transfer Fcn3' */
  boolean_T TransferFcn5_CSTATE_j;     /* '<S126>/Transfer Fcn5' */
  boolean_T TransferFcn4_CSTATE_n;     /* '<S126>/Transfer Fcn4' */
  boolean_T TransferFcn_CSTATE_e;      /* '<S128>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_d;     /* '<S128>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_c;     /* '<S128>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE_j;     /* '<S128>/Transfer Fcn3' */
  boolean_T TransferFcn5_CSTATE_b;     /* '<S128>/Transfer Fcn5' */
  boolean_T TransferFcn4_CSTATE_h;     /* '<S128>/Transfer Fcn4' */
  boolean_T TransferFcn_CSTATE_n;      /* '<S130>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_k;     /* '<S130>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE_cy;    /* '<S130>/Transfer Fcn2' */
  boolean_T TransferFcn3_CSTATE_f;     /* '<S130>/Transfer Fcn3' */
  boolean_T TransferFcn5_CSTATE_f;     /* '<S130>/Transfer Fcn5' */
  boolean_T TransferFcn4_CSTATE_g;     /* '<S130>/Transfer Fcn4' */
  boolean_T sm_vehicle_vtkVehicleCamera_Fra[29];/* '<S279>/STATE_1' */
  boolean_T TransferFcn_CSTATE_b;      /* '<S41>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE_a;     /* '<S41>/Transfer Fcn1' */
} XDis_sm_vehicle_vtk_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: VehicleData_P
   * Referenced by: '<Root>/Constant'
   */
  VehicleData_t Constant_Value;
} ConstP_sm_vehicle_vtk_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  sm_vehicle_vtk_U_t u;                /* '<Root>/u' */
} ExtU_sm_vehicle_vtk_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  sm_vehicle_vtk_Y_t y;                /* '<Root>/y' */
  VehicleData_t p;                     /* '<Root>/p' */
} ExtY_sm_vehicle_vtk_T;

/* Real-time Model Data Structure */
struct tag_RTM_sm_vehicle_vtk_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  B_sm_vehicle_vtk_T *blockIO;
  X_sm_vehicle_vtk_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_sm_vehicle_vtk_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][59];
  ODE1_IntgData intgData;
  DW_sm_vehicle_vtk_T *dwork;

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
extern const sm_vehicle_vtk_U_t sm_vehicle_vtk_rtZsm_vehicle_vt;/* sm_vehicle_vtk_U_t ground */

/* Constant parameters (default storage) */
extern const ConstP_sm_vehicle_vtk_T sm_vehicle_vtk_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern struct_S57YbmiMjO6n4oTP6qUIC InitVehicle;/* Variable: InitVehicle
                                                 * Referenced by:
                                                 *   '<S2>/Subsystem_around_RTP_041AAD1B_VelocityTargetValue'
                                                 *   '<S2>/Subsystem_around_RTP_087BC81F_VelocityTargetValue'
                                                 *   '<S2>/Subsystem_around_RTP_5BD4A88C_VelocityTargetValue'
                                                 *   '<S2>/Subsystem_around_RTP_9CB22C5A_VelocityTargetValue'
                                                 *   '<S6>/Subsystem_around_RTP_2B721290_PxPositionTargetValue'
                                                 *   '<S6>/Subsystem_around_RTP_2B721290_PyPositionTargetValue'
                                                 *   '<S6>/Subsystem_around_RTP_2B721290_PzPositionTargetValue'
                                                 */

/* Model entry point functions */
extern void sm_vehicle_vtk_initialize(RT_MODEL_sm_vehicle_vtk_T *const
  sm_vehicle_vtk_M, ExtU_sm_vehicle_vtk_T *sm_vehicle_vtk_U,
  ExtY_sm_vehicle_vtk_T *sm_vehicle_vtk_Y);
extern void sm_vehicle_vtk_step(RT_MODEL_sm_vehicle_vtk_T *const
  sm_vehicle_vtk_M, ExtU_sm_vehicle_vtk_T *sm_vehicle_vtk_U,
  ExtY_sm_vehicle_vtk_T *sm_vehicle_vtk_Y);
extern void sm_vehicle_vtk_terminate(RT_MODEL_sm_vehicle_vtk_T *const
  sm_vehicle_vtk_M);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<S29>/Constant' : Unused code path elimination
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
 * Block '<Root>/Manual Switch' : Eliminated due to constant selection input
 * Block '<S43>/RESHAPE' : Reshape block reduction
 * Block '<S79>/RESHAPE' : Reshape block reduction
 * Block '<S73>/Reshape' : Reshape block reduction
 * Block '<S73>/Reshape1' : Reshape block reduction
 * Block '<S73>/Reshape2' : Reshape block reduction
 * Block '<S125>/Reshape' : Reshape block reduction
 * Block '<S125>/Reshape1' : Reshape block reduction
 * Block '<S125>/Reshape2' : Reshape block reduction
 * Block '<S127>/Reshape' : Reshape block reduction
 * Block '<S127>/Reshape1' : Reshape block reduction
 * Block '<S127>/Reshape2' : Reshape block reduction
 * Block '<S129>/Reshape' : Reshape block reduction
 * Block '<S129>/Reshape1' : Reshape block reduction
 * Block '<S129>/Reshape2' : Reshape block reduction
 * Block '<S131>/Reshape' : Reshape block reduction
 * Block '<S131>/Reshape1' : Reshape block reduction
 * Block '<S131>/Reshape2' : Reshape block reduction
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
 * '<Root>' : 'sm_vehicle_vtk'
 * '<S1>'   : 'sm_vehicle_vtk/Driver Input'
 * '<S2>'   : 'sm_vehicle_vtk/Vehicle'
 * '<S3>'   : 'sm_vehicle_vtk/meas'
 * '<S4>'   : 'sm_vehicle_vtk/Driver Input/External'
 * '<S5>'   : 'sm_vehicle_vtk/Vehicle/Body'
 * '<S6>'   : 'sm_vehicle_vtk/Vehicle/Body to World'
 * '<S7>'   : 'sm_vehicle_vtk/Vehicle/Camera Frames'
 * '<S8>'   : 'sm_vehicle_vtk/Vehicle/Outputs'
 * '<S9>'   : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter'
 * '<S10>'  : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter1'
 * '<S11>'  : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter2'
 * '<S12>'  : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter3'
 * '<S13>'  : 'sm_vehicle_vtk/Vehicle/Road Surface FL'
 * '<S14>'  : 'sm_vehicle_vtk/Vehicle/Road Surface FR'
 * '<S15>'  : 'sm_vehicle_vtk/Vehicle/Road Surface RL'
 * '<S16>'  : 'sm_vehicle_vtk/Vehicle/Road Surface RR'
 * '<S17>'  : 'sm_vehicle_vtk/Vehicle/Scene'
 * '<S18>'  : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter'
 * '<S19>'  : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter1'
 * '<S20>'  : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter2'
 * '<S21>'  : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter3'
 * '<S22>'  : 'sm_vehicle_vtk/Vehicle/Suspension Front'
 * '<S23>'  : 'sm_vehicle_vtk/Vehicle/Suspension Rear'
 * '<S24>'  : 'sm_vehicle_vtk/Vehicle/Wheel FL'
 * '<S25>'  : 'sm_vehicle_vtk/Vehicle/Wheel FR'
 * '<S26>'  : 'sm_vehicle_vtk/Vehicle/Wheel RL'
 * '<S27>'  : 'sm_vehicle_vtk/Vehicle/Wheel RR'
 * '<S28>'  : 'sm_vehicle_vtk/Vehicle/World'
 * '<S29>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor'
 * '<S30>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix'
 * '<S31>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter1'
 * '<S32>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter11'
 * '<S33>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter2'
 * '<S34>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter3'
 * '<S35>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter4'
 * '<S36>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter5'
 * '<S37>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter6'
 * '<S38>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter7'
 * '<S39>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter8'
 * '<S40>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter9'
 * '<S41>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/Roll Pitch'
 * '<S42>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter1/EVAL_KEY'
 * '<S43>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter11/EVAL_KEY'
 * '<S44>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter2/EVAL_KEY'
 * '<S45>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter3/EVAL_KEY'
 * '<S46>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter4/EVAL_KEY'
 * '<S47>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter5/EVAL_KEY'
 * '<S48>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter6/EVAL_KEY'
 * '<S49>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter7/EVAL_KEY'
 * '<S50>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter8/EVAL_KEY'
 * '<S51>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/PS-Simulink Converter9/EVAL_KEY'
 * '<S52>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/Roll Pitch/Calculate aPitch'
 * '<S53>'  : 'sm_vehicle_vtk/Vehicle/Body/Body Sensor/Roll Pitch/Calculate aRoll'
 * '<S54>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix/Concat T-Matrix'
 * '<S55>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix/PS-Simulink Converter1'
 * '<S56>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix/PS-Simulink Converter2'
 * '<S57>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix/PS-Simulink Converter1/EVAL_KEY'
 * '<S58>'  : 'sm_vehicle_vtk/Vehicle/Body/Trasformation Matrix/PS-Simulink Converter2/EVAL_KEY'
 * '<S59>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter'
 * '<S60>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter1'
 * '<S61>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter10'
 * '<S62>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter11'
 * '<S63>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter12'
 * '<S64>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter13'
 * '<S65>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter2'
 * '<S66>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter3'
 * '<S67>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter4'
 * '<S68>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter5'
 * '<S69>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter6'
 * '<S70>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter7'
 * '<S71>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter8'
 * '<S72>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter9'
 * '<S73>'  : 'sm_vehicle_vtk/Vehicle/Body to World/R to X-Y-Z Extrinsic'
 * '<S74>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter/EVAL_KEY'
 * '<S75>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter1/EVAL_KEY'
 * '<S76>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter10/EVAL_KEY'
 * '<S77>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter11/EVAL_KEY'
 * '<S78>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter12/EVAL_KEY'
 * '<S79>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter13/EVAL_KEY'
 * '<S80>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter2/EVAL_KEY'
 * '<S81>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter3/EVAL_KEY'
 * '<S82>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter4/EVAL_KEY'
 * '<S83>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter5/EVAL_KEY'
 * '<S84>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter6/EVAL_KEY'
 * '<S85>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter7/EVAL_KEY'
 * '<S86>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter8/EVAL_KEY'
 * '<S87>'  : 'sm_vehicle_vtk/Vehicle/Body to World/PS-Simulink Converter9/EVAL_KEY'
 * '<S88>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Dolly Camera'
 * '<S89>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FL'
 * '<S90>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FR'
 * '<S91>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO FL'
 * '<S92>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO FR'
 * '<S93>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO Front'
 * '<S94>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO Left'
 * '<S95>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO RL'
 * '<S96>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO RR'
 * '<S97>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO Rear'
 * '<S98>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO Right'
 * '<S99>'  : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform FixO Top'
 * '<S100>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Front'
 * '<S101>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Left'
 * '<S102>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform RL'
 * '<S103>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform RR'
 * '<S104>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Rear'
 * '<S105>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Right'
 * '<S106>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform SuspF'
 * '<S107>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform SuspR'
 * '<S108>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Top'
 * '<S109>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Seat FL'
 * '<S110>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Seat FR'
 * '<S111>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Susp FL'
 * '<S112>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Susp FR'
 * '<S113>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Susp RL'
 * '<S114>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform View Susp RR'
 * '<S115>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Wheel FL'
 * '<S116>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Wheel FR'
 * '<S117>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Wheel RL'
 * '<S118>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Transform Wheel RR'
 * '<S119>' : 'sm_vehicle_vtk/Vehicle/Camera Frames/Dolly Camera/Off'
 * '<S120>' : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter/EVAL_KEY'
 * '<S121>' : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter1/EVAL_KEY'
 * '<S122>' : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter2/EVAL_KEY'
 * '<S123>' : 'sm_vehicle_vtk/Vehicle/PS-Simulink Converter3/EVAL_KEY'
 * '<S124>' : 'sm_vehicle_vtk/Vehicle/Road Surface FL/External'
 * '<S125>' : 'sm_vehicle_vtk/Vehicle/Road Surface FL/External/R to X-Y-Z Extrinsic'
 * '<S126>' : 'sm_vehicle_vtk/Vehicle/Road Surface FR/External'
 * '<S127>' : 'sm_vehicle_vtk/Vehicle/Road Surface FR/External/R to X-Y-Z Extrinsic'
 * '<S128>' : 'sm_vehicle_vtk/Vehicle/Road Surface RL/External'
 * '<S129>' : 'sm_vehicle_vtk/Vehicle/Road Surface RL/External/R to X-Y-Z Extrinsic'
 * '<S130>' : 'sm_vehicle_vtk/Vehicle/Road Surface RR/External'
 * '<S131>' : 'sm_vehicle_vtk/Vehicle/Road Surface RR/External/R to X-Y-Z Extrinsic'
 * '<S132>' : 'sm_vehicle_vtk/Vehicle/Scene/None'
 * '<S133>' : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter/EVAL_KEY'
 * '<S134>' : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter1/EVAL_KEY'
 * '<S135>' : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter2/EVAL_KEY'
 * '<S136>' : 'sm_vehicle_vtk/Vehicle/Simulink-PS Converter3/EVAL_KEY'
 * '<S137>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Steering Wheel to Wheel Angle'
 * '<S138>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1'
 * '<S139>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2'
 * '<S140>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter'
 * '<S141>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter1'
 * '<S142>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter/EVAL_KEY'
 * '<S143>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Steering Wheel to Wheel Angle/Simulink-PS Converter1/EVAL_KEY'
 * '<S144>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1/Concat T-Matrix'
 * '<S145>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1/PS-Simulink Converter1'
 * '<S146>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1/PS-Simulink Converter2'
 * '<S147>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1/PS-Simulink Converter1/EVAL_KEY'
 * '<S148>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix1/PS-Simulink Converter2/EVAL_KEY'
 * '<S149>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2/Concat T-Matrix'
 * '<S150>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2/PS-Simulink Converter1'
 * '<S151>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2/PS-Simulink Converter2'
 * '<S152>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2/PS-Simulink Converter1/EVAL_KEY'
 * '<S153>' : 'sm_vehicle_vtk/Vehicle/Suspension Front/Trasformation Matrix2/PS-Simulink Converter2/EVAL_KEY'
 * '<S154>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1'
 * '<S155>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2'
 * '<S156>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1/Concat T-Matrix'
 * '<S157>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1/PS-Simulink Converter1'
 * '<S158>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1/PS-Simulink Converter2'
 * '<S159>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1/PS-Simulink Converter1/EVAL_KEY'
 * '<S160>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix1/PS-Simulink Converter2/EVAL_KEY'
 * '<S161>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2/Concat T-Matrix'
 * '<S162>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2/PS-Simulink Converter1'
 * '<S163>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2/PS-Simulink Converter2'
 * '<S164>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2/PS-Simulink Converter1/EVAL_KEY'
 * '<S165>' : 'sm_vehicle_vtk/Vehicle/Suspension Rear/Trasformation Matrix2/PS-Simulink Converter2/EVAL_KEY'
 * '<S166>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Concat T-Matrix'
 * '<S167>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter'
 * '<S168>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter1'
 * '<S169>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter2'
 * '<S170>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter3'
 * '<S171>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion'
 * '<S172>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix'
 * '<S173>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter/EVAL_KEY'
 * '<S174>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter1/EVAL_KEY'
 * '<S175>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter2/EVAL_KEY'
 * '<S176>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/PS-Simulink Converter3/EVAL_KEY'
 * '<S177>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter'
 * '<S178>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter1'
 * '<S179>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter2'
 * '<S180>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter3'
 * '<S181>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter4'
 * '<S182>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter5'
 * '<S183>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S184>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S185>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S186>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S187>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S188>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S189>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix/Concat T-Matrix'
 * '<S190>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix/PS-Simulink Converter1'
 * '<S191>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix/PS-Simulink Converter2'
 * '<S192>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix/PS-Simulink Converter1/EVAL_KEY'
 * '<S193>' : 'sm_vehicle_vtk/Vehicle/Wheel FL/Trasformation Matrix/PS-Simulink Converter2/EVAL_KEY'
 * '<S194>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Concat T-Matrix'
 * '<S195>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter'
 * '<S196>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter1'
 * '<S197>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter2'
 * '<S198>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter3'
 * '<S199>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion'
 * '<S200>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix'
 * '<S201>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter/EVAL_KEY'
 * '<S202>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter1/EVAL_KEY'
 * '<S203>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter2/EVAL_KEY'
 * '<S204>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/PS-Simulink Converter3/EVAL_KEY'
 * '<S205>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter'
 * '<S206>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter1'
 * '<S207>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter2'
 * '<S208>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter3'
 * '<S209>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter4'
 * '<S210>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter5'
 * '<S211>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S212>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S213>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S214>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S215>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S216>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S217>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix/Concat T-Matrix'
 * '<S218>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix/PS-Simulink Converter1'
 * '<S219>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix/PS-Simulink Converter2'
 * '<S220>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix/PS-Simulink Converter1/EVAL_KEY'
 * '<S221>' : 'sm_vehicle_vtk/Vehicle/Wheel FR/Trasformation Matrix/PS-Simulink Converter2/EVAL_KEY'
 * '<S222>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Concat T-Matrix'
 * '<S223>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter'
 * '<S224>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter1'
 * '<S225>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter2'
 * '<S226>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter3'
 * '<S227>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion'
 * '<S228>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix'
 * '<S229>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter/EVAL_KEY'
 * '<S230>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter1/EVAL_KEY'
 * '<S231>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter2/EVAL_KEY'
 * '<S232>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/PS-Simulink Converter3/EVAL_KEY'
 * '<S233>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter'
 * '<S234>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter1'
 * '<S235>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter2'
 * '<S236>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter3'
 * '<S237>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter4'
 * '<S238>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter5'
 * '<S239>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S240>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S241>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S242>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S243>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S244>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S245>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix/Concat T-Matrix'
 * '<S246>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix/PS-Simulink Converter1'
 * '<S247>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix/PS-Simulink Converter2'
 * '<S248>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix/PS-Simulink Converter1/EVAL_KEY'
 * '<S249>' : 'sm_vehicle_vtk/Vehicle/Wheel RL/Trasformation Matrix/PS-Simulink Converter2/EVAL_KEY'
 * '<S250>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Concat T-Matrix'
 * '<S251>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter'
 * '<S252>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter1'
 * '<S253>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter2'
 * '<S254>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter3'
 * '<S255>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion'
 * '<S256>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix'
 * '<S257>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter/EVAL_KEY'
 * '<S258>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter1/EVAL_KEY'
 * '<S259>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter2/EVAL_KEY'
 * '<S260>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/PS-Simulink Converter3/EVAL_KEY'
 * '<S261>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter'
 * '<S262>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter1'
 * '<S263>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter2'
 * '<S264>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter3'
 * '<S265>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter4'
 * '<S266>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter5'
 * '<S267>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter/EVAL_KEY'
 * '<S268>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter1/EVAL_KEY'
 * '<S269>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter2/EVAL_KEY'
 * '<S270>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter3/EVAL_KEY'
 * '<S271>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter4/EVAL_KEY'
 * '<S272>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Road Motion/Simulink-PS Converter5/EVAL_KEY'
 * '<S273>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix/Concat T-Matrix'
 * '<S274>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix/PS-Simulink Converter1'
 * '<S275>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix/PS-Simulink Converter2'
 * '<S276>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix/PS-Simulink Converter1/EVAL_KEY'
 * '<S277>' : 'sm_vehicle_vtk/Vehicle/Wheel RR/Trasformation Matrix/PS-Simulink Converter2/EVAL_KEY'
 * '<S278>' : 'sm_vehicle_vtk/Vehicle/World/Solver Configuration'
 * '<S279>' : 'sm_vehicle_vtk/Vehicle/World/Solver Configuration/EVAL_KEY'
 */
#endif                                 /* sm_vehicle_vtk_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
