/*
 * File: sm_vehicle_2axle_heave_roll_vtk_types.h
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

#ifndef RTW_HEADER_sm_vehicle_2axle_heave_roll_vtk_types_h_
#define RTW_HEADER_sm_vehicle_2axle_heave_roll_vtk_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_U_t_
#define DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_U_t_

typedef struct {
  real_T sWhl;
  real_T trqFL;
  real_T trqFR;
  real_T trqRL;
  real_T trqRR;
} sm_vehicle_2axle_heave_roll_U_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus1_Body_
#define DEFINED_TYPEDEF_FOR_slBus1_Body_

typedef struct {
  real_T FA;
  real_T RA;
  real_T CG[3];
  real_T Geometry[3];
  real_T Mass;
  real_T Inertias[3];
  real_T Color[3];
  real_T Color_2[3];
  real_T Opacity;
} slBus1_Body;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Heave_
#define DEFINED_TYPEDEF_FOR_Heave_

typedef struct {
  real_T Stiffness;
  real_T Damping;
  real_T EqPos;
  real_T Height;
} Heave;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Roll_
#define DEFINED_TYPEDEF_FOR_Roll_

typedef struct {
  real_T Stiffness;
  real_T Damping;
  real_T Height;
  real_T EqPos;
} Roll;

#endif

#ifndef DEFINED_TYPEDEF_FOR_UnsprungMass_
#define DEFINED_TYPEDEF_FOR_UnsprungMass_

typedef struct {
  real_T Mass;
  real_T Inertia[3];
  real_T Height;
  real_T Radius;
  real_T Length;
} UnsprungMass;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Hub_
#define DEFINED_TYPEDEF_FOR_Hub_

typedef struct {
  real_T Height;
} Hub;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SuspF_
#define DEFINED_TYPEDEF_FOR_SuspF_

typedef struct {
  Heave Heave;
  Roll Roll;
  real_T Track;
  UnsprungMass UnsprungMass;
  Hub Hub;
  real_T SteerRatio;
} SuspF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MDI_HEADER_
#define DEFINED_TYPEDEF_FOR_MDI_HEADER_

typedef struct {
  real_T FILE_VERSION;
} MDI_HEADER;

#endif

#ifndef DEFINED_TYPEDEF_FOR_DIMENSION_
#define DEFINED_TYPEDEF_FOR_DIMENSION_

typedef struct {
  real_T UNLOADED_RADIUS;
  real_T WIDTH;
  real_T RIM_RADIUS;
  real_T RIM_WIDTH;
  real_T ASPECT_RATIO;
} DIMENSION;

#endif

#ifndef DEFINED_TYPEDEF_FOR_OPERATING_CONDITIONS_
#define DEFINED_TYPEDEF_FOR_OPERATING_CONDITIONS_

typedef struct {
  real_T INFLPRES;
  real_T NOMPRES;
} OPERATING_CONDITIONS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_INERTIA_
#define DEFINED_TYPEDEF_FOR_INERTIA_

typedef struct {
  real_T MASS;
  real_T IXX;
  real_T IYY;
  real_T BELT_MASS;
  real_T BELT_IXX;
  real_T BELT_IYY;
  real_T GRAVITY;
} INERTIA;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VERTICAL_
#define DEFINED_TYPEDEF_FOR_VERTICAL_

typedef struct {
  real_T FNOMIN;
  real_T VERTICAL_STIFFNESS;
  real_T VERTICAL_DAMPING;
  real_T MC_CONTOUR_A;
  real_T MC_CONTOUR_B;
  real_T BREFF;
  real_T DREFF;
  real_T FREFF;
  real_T Q_RE0;
  real_T Q_V1;
  real_T Q_V2;
  real_T Q_FZ2;
  real_T Q_FCX;
  real_T Q_FCY;
  real_T Q_CAM;
  real_T PFZ1;
  real_T Q_FCY2;
  real_T Q_CAM1;
  real_T Q_CAM2;
  real_T Q_CAM3;
  real_T Q_FYS1;
  real_T Q_FYS2;
  real_T Q_FYS3;
  real_T BOTTOM_OFFST;
  real_T BOTTOM_STIFF;
} VERTICAL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_STRUCTURAL_
#define DEFINED_TYPEDEF_FOR_STRUCTURAL_

typedef struct {
  real_T LONGITUDINAL_STIFFNESS;
  real_T LATERAL_STIFFNESS;
  real_T YAW_STIFFNESS;
  real_T FREQ_LONG;
  real_T FREQ_LAT;
  real_T FREQ_YAW;
  real_T FREQ_WINDUP;
  real_T DAMP_LONG;
  real_T DAMP_LAT;
  real_T DAMP_YAW;
  real_T DAMP_WINDUP;
  real_T DAMP_RESIDUAL;
  real_T DAMP_VLOW;
  real_T Q_BVX;
  real_T Q_BVT;
  real_T PCFX1;
  real_T PCFX2;
  real_T PCFX3;
  real_T PCFY1;
  real_T PCFY2;
  real_T PCFY3;
  real_T PCMZ1;
} STRUCTURAL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CONTACT_PATCH_
#define DEFINED_TYPEDEF_FOR_CONTACT_PATCH_

typedef struct {
  real_T Q_RA1;
  real_T Q_RA2;
  real_T Q_RB1;
  real_T Q_RB2;
  real_T ELLIPS_SHIFT;
  real_T ELLIPS_LENGTH;
  real_T ELLIPS_HEIGHT;
  real_T ELLIPS_ORDER;
  real_T ELLIPS_MAX_STEP;
  real_T ELLIPS_NWIDTH;
  real_T ELLIPS_NLENGTH;
  real_T ENV_C1;
  real_T ENV_C2;
} CONTACT_PATCH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_INFLATION_PRESSURE_RANGE_
#define DEFINED_TYPEDEF_FOR_INFLATION_PRESSURE_RANGE_

typedef struct {
  real_T PRESMIN;
  real_T PRESMAX;
} INFLATION_PRESSURE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VERTICAL_FORCE_RANGE_
#define DEFINED_TYPEDEF_FOR_VERTICAL_FORCE_RANGE_

typedef struct {
  real_T FZMIN;
  real_T FZMAX;
} VERTICAL_FORCE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LONG_SLIP_RANGE_
#define DEFINED_TYPEDEF_FOR_LONG_SLIP_RANGE_

typedef struct {
  real_T KPUMIN;
  real_T KPUMAX;
} LONG_SLIP_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SLIP_ANGLE_RANGE_
#define DEFINED_TYPEDEF_FOR_SLIP_ANGLE_RANGE_

typedef struct {
  real_T ALPMIN;
  real_T ALPMAX;
} SLIP_ANGLE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_INCLINATION_ANGLE_RANGE_
#define DEFINED_TYPEDEF_FOR_INCLINATION_ANGLE_RANGE_

typedef struct {
  real_T CAMMIN;
  real_T CAMMAX;
} INCLINATION_ANGLE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SCALING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_SCALING_COEFFICIENTS_

typedef struct {
  real_T LFZO;
  real_T LCX;
  real_T LMUX;
  real_T LEX;
  real_T LKX;
  real_T LHX;
  real_T LVX;
  real_T LCY;
  real_T LMUY;
  real_T LEY;
  real_T LKY;
  real_T LKYC;
  real_T LKZC;
  real_T LHY;
  real_T LVY;
  real_T LTR;
  real_T LRES;
  real_T LXAL;
  real_T LYKA;
  real_T LVYKA;
  real_T LS;
  real_T LMX;
  real_T LVMX;
  real_T LMY;
  real_T LMP;
} SCALING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LONGITUDINAL_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_LONGITUDINAL_COEFFICIENTS_

typedef struct {
  real_T PCX1;
  real_T PDX1;
  real_T PDX2;
  real_T PDX3;
  real_T PEX1;
  real_T PEX2;
  real_T PEX3;
  real_T PEX4;
  real_T PKX1;
  real_T PKX2;
  real_T PKX3;
  real_T PHX1;
  real_T PHX2;
  real_T PVX1;
  real_T PVX2;
  real_T PPX1;
  real_T PPX2;
  real_T PPX3;
  real_T PPX4;
  real_T RBX1;
  real_T RBX2;
  real_T RBX3;
  real_T RCX1;
  real_T REX1;
  real_T REX2;
  real_T RHX1;
} LONGITUDINAL_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_OVERTURNING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_OVERTURNING_COEFFICIENTS_

typedef struct {
  real_T QSX1;
  real_T QSX2;
  real_T QSX3;
  real_T QSX4;
  real_T QSX5;
  real_T QSX6;
  real_T QSX7;
  real_T QSX8;
  real_T QSX9;
  real_T QSX10;
  real_T QSX11;
  real_T QSX12;
  real_T QSX13;
  real_T QSX14;
  real_T PPMX1;
} OVERTURNING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_LATERAL_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_LATERAL_COEFFICIENTS_

typedef struct {
  real_T PCY1;
  real_T PDY1;
  real_T PDY2;
  real_T PDY3;
  real_T PEY1;
  real_T PEY2;
  real_T PEY3;
  real_T PEY4;
  real_T PEY5;
  real_T PKY1;
  real_T PKY2;
  real_T PKY3;
  real_T PKY4;
  real_T PKY5;
  real_T PKY6;
  real_T PKY7;
  real_T PHY1;
  real_T PHY2;
  real_T PVY1;
  real_T PVY2;
  real_T PVY3;
  real_T PVY4;
  real_T PPY1;
  real_T PPY2;
  real_T PPY3;
  real_T PPY4;
  real_T PPY5;
  real_T RBY1;
  real_T RBY2;
  real_T RBY3;
  real_T RBY4;
  real_T RCY1;
  real_T REY1;
  real_T REY2;
  real_T RHY1;
  real_T RHY2;
  real_T RVY1;
  real_T RVY2;
  real_T RVY3;
  real_T RVY4;
  real_T RVY5;
  real_T RVY6;
} LATERAL_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ROLLING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_ROLLING_COEFFICIENTS_

typedef struct {
  real_T QSY1;
  real_T QSY2;
  real_T QSY3;
  real_T QSY4;
  real_T QSY5;
  real_T QSY6;
  real_T QSY7;
  real_T QSY8;
} ROLLING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ALIGNING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_ALIGNING_COEFFICIENTS_

typedef struct {
  real_T QBZ1;
  real_T QBZ2;
  real_T QBZ3;
  real_T QBZ4;
  real_T QBZ5;
  real_T QBZ9;
  real_T QBZ10;
  real_T QCZ1;
  real_T QDZ1;
  real_T QDZ2;
  real_T QDZ3;
  real_T QDZ4;
  real_T QDZ6;
  real_T QDZ7;
  real_T QDZ8;
  real_T QDZ9;
  real_T QDZ10;
  real_T QDZ11;
  real_T QEZ1;
  real_T QEZ2;
  real_T QEZ3;
  real_T QEZ4;
  real_T QEZ5;
  real_T QHZ1;
  real_T QHZ2;
  real_T QHZ3;
  real_T QHZ4;
  real_T PPZ1;
  real_T PPZ2;
  real_T SSZ1;
  real_T SSZ2;
  real_T SSZ3;
  real_T SSZ4;
} ALIGNING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TURNSLIP_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_TURNSLIP_COEFFICIENTS_

typedef struct {
  real_T PDXP1;
  real_T PDXP2;
  real_T PDXP3;
  real_T PKYP1;
  real_T PDYP1;
  real_T PDYP2;
  real_T PDYP3;
  real_T PDYP4;
  real_T PHYP1;
  real_T PHYP2;
  real_T PHYP3;
  real_T PHYP4;
  real_T PECP1;
  real_T PECP2;
  real_T QDTP1;
  real_T QCRP1;
  real_T QCRP2;
  real_T QBRP1;
  real_T QDRP1;
} TURNSLIP_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_MODEL__
#define DEFINED_TYPEDEF_FOR_MODEL__

typedef struct {
  real_T FITTYP;
  real_T LONGVL;
  real_T VXLOW;
  real_T ROAD_INCREMENT;
  real_T ROAD_DIRECTION;
  real_T N_TIRE_STATES;
  real_T USE_MODE;
  real_T HMAX_LOCAL;
  real_T SWITCH_INTEG;
} MODEL_;

#endif

#ifndef DEFINED_TYPEDEF_FOR_param_
#define DEFINED_TYPEDEF_FOR_param_

typedef struct {
  MDI_HEADER MDI_HEADER;
  DIMENSION DIMENSION;
  OPERATING_CONDITIONS OPERATING_CONDITIONS;
  INERTIA INERTIA;
  VERTICAL VERTICAL;
  STRUCTURAL STRUCTURAL;
  CONTACT_PATCH CONTACT_PATCH;
  INFLATION_PRESSURE_RANGE INFLATION_PRESSURE_RANGE;
  VERTICAL_FORCE_RANGE VERTICAL_FORCE_RANGE;
  LONG_SLIP_RANGE LONG_SLIP_RANGE;
  SLIP_ANGLE_RANGE SLIP_ANGLE_RANGE;
  INCLINATION_ANGLE_RANGE INCLINATION_ANGLE_RANGE;
  SCALING_COEFFICIENTS SCALING_COEFFICIENTS;
  LONGITUDINAL_COEFFICIENTS LONGITUDINAL_COEFFICIENTS;
  OVERTURNING_COEFFICIENTS OVERTURNING_COEFFICIENTS;
  LATERAL_COEFFICIENTS LATERAL_COEFFICIENTS;
  ROLLING_COEFFICIENTS ROLLING_COEFFICIENTS;
  ALIGNING_COEFFICIENTS ALIGNING_COEFFICIENTS;
  TURNSLIP_COEFFICIENTS TURNSLIP_COEFFICIENTS;
  MODEL_ MODEL_;
} param;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireDataF_
#define DEFINED_TYPEDEF_FOR_TireDataF_

typedef struct {
  param param;
} TireDataF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RimF_
#define DEFINED_TYPEDEF_FOR_RimF_

typedef struct {
  real_T Mass;
  real_T Inertia[2];
} RimF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus2_Heave_
#define DEFINED_TYPEDEF_FOR_slBus2_Heave_

typedef struct {
  real_T Stiffness;
  real_T Damping;
  real_T EqPos;
  real_T Height;
} slBus2_Heave;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus3_Roll_
#define DEFINED_TYPEDEF_FOR_slBus3_Roll_

typedef struct {
  real_T Stiffness;
  real_T Damping;
  real_T Height;
  real_T EqPos;
} slBus3_Roll;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus4_UnsprungMass_
#define DEFINED_TYPEDEF_FOR_slBus4_UnsprungMass_

typedef struct {
  real_T Mass;
  real_T Inertia[3];
  real_T Height;
  real_T Length;
  real_T Radius;
} slBus4_UnsprungMass;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus5_Hub_
#define DEFINED_TYPEDEF_FOR_slBus5_Hub_

typedef struct {
  real_T Height;
} slBus5_Hub;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SuspR_
#define DEFINED_TYPEDEF_FOR_SuspR_

typedef struct {
  slBus2_Heave Heave;
  slBus3_Roll Roll;
  real_T Track;
  slBus4_UnsprungMass UnsprungMass;
  slBus5_Hub Hub;
} SuspR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus6_MDI_HEADER_
#define DEFINED_TYPEDEF_FOR_slBus6_MDI_HEADER_

typedef struct {
  real_T FILE_VERSION;
} slBus6_MDI_HEADER;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus7_DIMENSION_
#define DEFINED_TYPEDEF_FOR_slBus7_DIMENSION_

typedef struct {
  real_T UNLOADED_RADIUS;
  real_T WIDTH;
  real_T RIM_RADIUS;
  real_T RIM_WIDTH;
  real_T ASPECT_RATIO;
} slBus7_DIMENSION;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus8_OPERATING_CONDITIONS_
#define DEFINED_TYPEDEF_FOR_slBus8_OPERATING_CONDITIONS_

typedef struct {
  real_T INFLPRES;
  real_T NOMPRES;
} slBus8_OPERATING_CONDITIONS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus9_INERTIA_
#define DEFINED_TYPEDEF_FOR_slBus9_INERTIA_

typedef struct {
  real_T MASS;
  real_T IXX;
  real_T IYY;
  real_T BELT_MASS;
  real_T BELT_IXX;
  real_T BELT_IYY;
  real_T GRAVITY;
} slBus9_INERTIA;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus10_VERTICAL_
#define DEFINED_TYPEDEF_FOR_slBus10_VERTICAL_

typedef struct {
  real_T FNOMIN;
  real_T VERTICAL_STIFFNESS;
  real_T VERTICAL_DAMPING;
  real_T MC_CONTOUR_A;
  real_T MC_CONTOUR_B;
  real_T BREFF;
  real_T DREFF;
  real_T FREFF;
  real_T Q_RE0;
  real_T Q_V1;
  real_T Q_V2;
  real_T Q_FZ2;
  real_T Q_FCX;
  real_T Q_FCY;
  real_T Q_CAM;
  real_T PFZ1;
  real_T Q_FCY2;
  real_T Q_CAM1;
  real_T Q_CAM2;
  real_T Q_CAM3;
  real_T Q_FYS1;
  real_T Q_FYS2;
  real_T Q_FYS3;
  real_T BOTTOM_OFFST;
  real_T BOTTOM_STIFF;
} slBus10_VERTICAL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus11_STRUCTURAL_
#define DEFINED_TYPEDEF_FOR_slBus11_STRUCTURAL_

typedef struct {
  real_T LONGITUDINAL_STIFFNESS;
  real_T LATERAL_STIFFNESS;
  real_T YAW_STIFFNESS;
  real_T FREQ_LONG;
  real_T FREQ_LAT;
  real_T FREQ_YAW;
  real_T FREQ_WINDUP;
  real_T DAMP_LONG;
  real_T DAMP_LAT;
  real_T DAMP_YAW;
  real_T DAMP_WINDUP;
  real_T DAMP_RESIDUAL;
  real_T DAMP_VLOW;
  real_T Q_BVX;
  real_T Q_BVT;
  real_T PCFX1;
  real_T PCFX2;
  real_T PCFX3;
  real_T PCFY1;
  real_T PCFY2;
  real_T PCFY3;
  real_T PCMZ1;
} slBus11_STRUCTURAL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus12_CONTACT_PATCH_
#define DEFINED_TYPEDEF_FOR_slBus12_CONTACT_PATCH_

typedef struct {
  real_T Q_RA1;
  real_T Q_RA2;
  real_T Q_RB1;
  real_T Q_RB2;
  real_T ELLIPS_SHIFT;
  real_T ELLIPS_LENGTH;
  real_T ELLIPS_HEIGHT;
  real_T ELLIPS_ORDER;
  real_T ELLIPS_MAX_STEP;
  real_T ELLIPS_NWIDTH;
  real_T ELLIPS_NLENGTH;
  real_T ENV_C1;
  real_T ENV_C2;
} slBus12_CONTACT_PATCH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus13_INFLATION_PRESSURE_RANGE_
#define DEFINED_TYPEDEF_FOR_slBus13_INFLATION_PRESSURE_RANGE_

typedef struct {
  real_T PRESMIN;
  real_T PRESMAX;
} slBus13_INFLATION_PRESSURE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus14_VERTICAL_FORCE_RANGE_
#define DEFINED_TYPEDEF_FOR_slBus14_VERTICAL_FORCE_RANGE_

typedef struct {
  real_T FZMIN;
  real_T FZMAX;
} slBus14_VERTICAL_FORCE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus15_LONG_SLIP_RANGE_
#define DEFINED_TYPEDEF_FOR_slBus15_LONG_SLIP_RANGE_

typedef struct {
  real_T KPUMIN;
  real_T KPUMAX;
} slBus15_LONG_SLIP_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus16_SLIP_ANGLE_RANGE_
#define DEFINED_TYPEDEF_FOR_slBus16_SLIP_ANGLE_RANGE_

typedef struct {
  real_T ALPMIN;
  real_T ALPMAX;
} slBus16_SLIP_ANGLE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus17_INCLINATION_ANGLE_RANGE_
#define DEFINED_TYPEDEF_FOR_slBus17_INCLINATION_ANGLE_RANGE_

typedef struct {
  real_T CAMMIN;
  real_T CAMMAX;
} slBus17_INCLINATION_ANGLE_RANGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus18_SCALING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus18_SCALING_COEFFICIENTS_

typedef struct {
  real_T LFZO;
  real_T LCX;
  real_T LMUX;
  real_T LEX;
  real_T LKX;
  real_T LHX;
  real_T LVX;
  real_T LCY;
  real_T LMUY;
  real_T LEY;
  real_T LKY;
  real_T LKYC;
  real_T LKZC;
  real_T LHY;
  real_T LVY;
  real_T LTR;
  real_T LRES;
  real_T LXAL;
  real_T LYKA;
  real_T LVYKA;
  real_T LS;
  real_T LMX;
  real_T LVMX;
  real_T LMY;
  real_T LMP;
} slBus18_SCALING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus19_LONGITUDINAL_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus19_LONGITUDINAL_COEFFICIENTS_

typedef struct {
  real_T PCX1;
  real_T PDX1;
  real_T PDX2;
  real_T PDX3;
  real_T PEX1;
  real_T PEX2;
  real_T PEX3;
  real_T PEX4;
  real_T PKX1;
  real_T PKX2;
  real_T PKX3;
  real_T PHX1;
  real_T PHX2;
  real_T PVX1;
  real_T PVX2;
  real_T PPX1;
  real_T PPX2;
  real_T PPX3;
  real_T PPX4;
  real_T RBX1;
  real_T RBX2;
  real_T RBX3;
  real_T RCX1;
  real_T REX1;
  real_T REX2;
  real_T RHX1;
} slBus19_LONGITUDINAL_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus20_OVERTURNING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus20_OVERTURNING_COEFFICIENTS_

typedef struct {
  real_T QSX1;
  real_T QSX2;
  real_T QSX3;
  real_T QSX4;
  real_T QSX5;
  real_T QSX6;
  real_T QSX7;
  real_T QSX8;
  real_T QSX9;
  real_T QSX10;
  real_T QSX11;
  real_T QSX12;
  real_T QSX13;
  real_T QSX14;
  real_T PPMX1;
} slBus20_OVERTURNING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus21_LATERAL_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus21_LATERAL_COEFFICIENTS_

typedef struct {
  real_T PCY1;
  real_T PDY1;
  real_T PDY2;
  real_T PDY3;
  real_T PEY1;
  real_T PEY2;
  real_T PEY3;
  real_T PEY4;
  real_T PEY5;
  real_T PKY1;
  real_T PKY2;
  real_T PKY3;
  real_T PKY4;
  real_T PKY5;
  real_T PKY6;
  real_T PKY7;
  real_T PHY1;
  real_T PHY2;
  real_T PVY1;
  real_T PVY2;
  real_T PVY3;
  real_T PVY4;
  real_T PPY1;
  real_T PPY2;
  real_T PPY3;
  real_T PPY4;
  real_T PPY5;
  real_T RBY1;
  real_T RBY2;
  real_T RBY3;
  real_T RBY4;
  real_T RCY1;
  real_T REY1;
  real_T REY2;
  real_T RHY1;
  real_T RHY2;
  real_T RVY1;
  real_T RVY2;
  real_T RVY3;
  real_T RVY4;
  real_T RVY5;
  real_T RVY6;
} slBus21_LATERAL_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus22_ROLLING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus22_ROLLING_COEFFICIENTS_

typedef struct {
  real_T QSY1;
  real_T QSY2;
  real_T QSY3;
  real_T QSY4;
  real_T QSY5;
  real_T QSY6;
  real_T QSY7;
  real_T QSY8;
} slBus22_ROLLING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus23_ALIGNING_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus23_ALIGNING_COEFFICIENTS_

typedef struct {
  real_T QBZ1;
  real_T QBZ2;
  real_T QBZ3;
  real_T QBZ4;
  real_T QBZ5;
  real_T QBZ9;
  real_T QBZ10;
  real_T QCZ1;
  real_T QDZ1;
  real_T QDZ2;
  real_T QDZ3;
  real_T QDZ4;
  real_T QDZ6;
  real_T QDZ7;
  real_T QDZ8;
  real_T QDZ9;
  real_T QDZ10;
  real_T QDZ11;
  real_T QEZ1;
  real_T QEZ2;
  real_T QEZ3;
  real_T QEZ4;
  real_T QEZ5;
  real_T QHZ1;
  real_T QHZ2;
  real_T QHZ3;
  real_T QHZ4;
  real_T PPZ1;
  real_T PPZ2;
  real_T SSZ1;
  real_T SSZ2;
  real_T SSZ3;
  real_T SSZ4;
} slBus23_ALIGNING_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus24_TURNSLIP_COEFFICIENTS_
#define DEFINED_TYPEDEF_FOR_slBus24_TURNSLIP_COEFFICIENTS_

typedef struct {
  real_T PDXP1;
  real_T PDXP2;
  real_T PDXP3;
  real_T PKYP1;
  real_T PDYP1;
  real_T PDYP2;
  real_T PDYP3;
  real_T PDYP4;
  real_T PHYP1;
  real_T PHYP2;
  real_T PHYP3;
  real_T PHYP4;
  real_T PECP1;
  real_T PECP2;
  real_T QDTP1;
  real_T QCRP1;
  real_T QCRP2;
  real_T QBRP1;
  real_T QDRP1;
} slBus24_TURNSLIP_COEFFICIENTS;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus25_MODEL__
#define DEFINED_TYPEDEF_FOR_slBus25_MODEL__

typedef struct {
  real_T FITTYP;
  real_T LONGVL;
  real_T VXLOW;
  real_T ROAD_INCREMENT;
  real_T ROAD_DIRECTION;
  real_T N_TIRE_STATES;
  real_T USE_MODE;
  real_T HMAX_LOCAL;
  real_T SWITCH_INTEG;
} slBus25_MODEL_;

#endif

#ifndef DEFINED_TYPEDEF_FOR_slBus26_param_
#define DEFINED_TYPEDEF_FOR_slBus26_param_

typedef struct {
  slBus6_MDI_HEADER MDI_HEADER;
  slBus7_DIMENSION DIMENSION;
  slBus8_OPERATING_CONDITIONS OPERATING_CONDITIONS;
  slBus9_INERTIA INERTIA;
  slBus10_VERTICAL VERTICAL;
  slBus11_STRUCTURAL STRUCTURAL;
  slBus12_CONTACT_PATCH CONTACT_PATCH;
  slBus13_INFLATION_PRESSURE_RANGE INFLATION_PRESSURE_RANGE;
  slBus14_VERTICAL_FORCE_RANGE VERTICAL_FORCE_RANGE;
  slBus15_LONG_SLIP_RANGE LONG_SLIP_RANGE;
  slBus16_SLIP_ANGLE_RANGE SLIP_ANGLE_RANGE;
  slBus17_INCLINATION_ANGLE_RANGE INCLINATION_ANGLE_RANGE;
  slBus18_SCALING_COEFFICIENTS SCALING_COEFFICIENTS;
  slBus19_LONGITUDINAL_COEFFICIENTS LONGITUDINAL_COEFFICIENTS;
  slBus20_OVERTURNING_COEFFICIENTS OVERTURNING_COEFFICIENTS;
  slBus21_LATERAL_COEFFICIENTS LATERAL_COEFFICIENTS;
  slBus22_ROLLING_COEFFICIENTS ROLLING_COEFFICIENTS;
  slBus23_ALIGNING_COEFFICIENTS ALIGNING_COEFFICIENTS;
  slBus24_TURNSLIP_COEFFICIENTS TURNSLIP_COEFFICIENTS;
  slBus25_MODEL_ MODEL_;
} slBus26_param;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireDataR_
#define DEFINED_TYPEDEF_FOR_TireDataR_

typedef struct {
  slBus26_param param;
} TireDataR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_RimR_
#define DEFINED_TYPEDEF_FOR_RimR_

typedef struct {
  real_T Mass;
  real_T Inertia[2];
} RimR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicleData_t_
#define DEFINED_TYPEDEF_FOR_VehicleData_t_

typedef struct {
  slBus1_Body Body;
  SuspF SuspF;
  TireDataF TireDataF;
  RimF RimF;
  SuspR SuspR;
  TireDataR TireDataR;
  RimR RimR;
} VehicleData_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Body_
#define DEFINED_TYPEDEF_FOR_Body_

typedef struct {
  real_T x;
  real_T y;
  real_T z;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T gx;
  real_T gy;
  real_T gz;
  real_T nRoll;
  real_T nPitch;
  real_T aRoll;
  real_T aPitch;
  real_T T[16];
} Body;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireFL_
#define DEFINED_TYPEDEF_FOR_TireFL_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
  real_T Tw[16];
  real_T Tc[16];
} TireFL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireFR_
#define DEFINED_TYPEDEF_FOR_TireFR_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
  real_T Tw[16];
  real_T Tc[16];
} TireFR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireRL_
#define DEFINED_TYPEDEF_FOR_TireRL_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
  real_T Tw[16];
  real_T Tc[16];
} TireRL;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TireRR_
#define DEFINED_TYPEDEF_FOR_TireRR_

typedef struct {
  real_T w;
  real_T Fx;
  real_T Fy;
  real_T Fz;
  real_T Tx;
  real_T Ty;
  real_T Tz;
  real_T Tw[16];
  real_T Tc[16];
} TireRR;

#endif

#ifndef DEFINED_TYPEDEF_FOR_World_
#define DEFINED_TYPEDEF_FOR_World_

typedef struct {
  real_T x;
  real_T y;
  real_T z;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T gx;
  real_T gy;
  real_T gz;
  real_T nRoll;
  real_T nPitch;
  real_T nYaw;
  real_T aRoll;
  real_T aPitch;
  real_T aYaw;
  real_T Q[4];
} World;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_Y_t_
#define DEFINED_TYPEDEF_FOR_sm_vehicle_2axle_heave_roll_Y_t_

typedef struct {
  Body Body;
  TireFL TireFL;
  TireFR TireFR;
  TireRL TireRL;
  TireRR TireRR;
  World World;
} sm_vehicle_2axle_heave_roll_Y_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_3WYYf58EhJNOmQlcLU9yKG_
#define DEFINED_TYPEDEF_FOR_struct_3WYYf58EhJNOmQlcLU9yKG_

typedef struct {
  real_T px;
  real_T py;
  real_T pz;
  real_T vx;
  real_T vy;
  real_T vz;
  real_T yaw;
  real_T pitch;
  real_T roll;
} struct_3WYYf58EhJNOmQlcLU9yKG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_STAvlAl9fAauyijRHWIxLF_
#define DEFINED_TYPEDEF_FOR_struct_STAvlAl9fAauyijRHWIxLF_

typedef struct {
  real_T wFL;
  real_T wFR;
  real_T wRL;
  real_T wRR;
} struct_STAvlAl9fAauyijRHWIxLF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_S57YbmiMjO6n4oTP6qUIC_
#define DEFINED_TYPEDEF_FOR_struct_S57YbmiMjO6n4oTP6qUIC_

typedef struct {
  struct_3WYYf58EhJNOmQlcLU9yKG Vehicle;
  struct_STAvlAl9fAauyijRHWIxLF Wheel;
} struct_S57YbmiMjO6n4oTP6qUIC;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_sm_vehicle_2axle_heav_T RT_MODEL_sm_vehicle_2axle_hea_T;

#endif                 /* RTW_HEADER_sm_vehicle_2axle_heave_roll_vtk_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
