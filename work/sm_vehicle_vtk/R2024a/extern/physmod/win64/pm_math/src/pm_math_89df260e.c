#include "pm_std.h"
#include "pm_std.h"
void pm_math_lin_alg_eig(uint32_T n,const real_T*
pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
real_T*pm_math_FFBCOFcrbF_Oc1OIGwRBc4,real_T*pm_math_VlJ4W5vD3kK5X99Lo9mn6_,
real_T*pm_math_FXqYP9M_eO_ybyvyG1U3_4,real_T*pm_math__0YXwStzi_WyXXlmJ2IHY2,
real_T*pm_math_FaY46KF_stWCXP_M9L46Rm,int32_T*pm_math_k_auWg11HRS2a54rbYevu2);
uint32_T pm_math_lin_alg_eigReqdRealWorkSize(uint32_T n);uint32_T
pm_math_lin_alg_eigReqdInt32WorkSize(uint32_T n);boolean_T
pm_math_lin_alg_orthonormalizeRealEigenvectorsWrtSpdB(uint32_T n,const real_T*
pm_math_Vqiy96WqvuhCaXm5e_vvT0,real_T*pm_math_kJeUz19e49pVaDwcttsZep,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D);
#include "pm_std.h"
extern real_T pm_math_V6hUjCc3PiCAauyMwJP5nb;extern real_T
pm_math_Vco4W7EuZu4k_1cAN9SDVG;extern real_T pm_math_k9HpTo0HuFt0gDRVyg17Wc;
extern real32_T pm_math__JOzYwXJOmSsiaZ6WMnnZZ;extern real32_T
pm_math_VaGr_igKLHx0bDW9g54I6v;extern real32_T pm_math_kZu0y5Pv4G8tg1EE_V6tGP;
extern void pm_math_F3OoZ1Weno85bujDCIyv_c(void);extern boolean_T
pm_math__LIYjt5pi54rVTuNKI6I5_(real_T pm_math_kg4CyRgbu6OdeewZOel7Qx);extern
boolean_T pm_math_VK2BuaMRCmOxjLjqE4tZMh(real32_T
pm_math_kg4CyRgbu6OdeewZOel7Qx);extern boolean_T pm_math__J9snrOx2Cp4dD_e3lGtRm
(real_T pm_math_kg4CyRgbu6OdeewZOel7Qx);extern boolean_T
pm_math_kwH13YYrpKtAbDLZ9Q_8I9(real32_T pm_math_kg4CyRgbu6OdeewZOel7Qx);void
pm_math_lin_alg_transposeColMajor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,double*
pm_math_kXii4Miq3Y_ShPn9EHS3D5);void pm_math_lin_alg_transposeRowMajor(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_kXii4Miq3Y_ShPn9EHS3D5);void
pm_math_lin_alg_convertColToRowMajor(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_k_pxeHD3zfxkcatRbNa25_,double*
pm_math_F4CqmafKSNK3_y4WQubdlT);void pm_math_lin_alg_convertRowToColMajor(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F4CqmafKSNK3_y4WQubdlT,double*pm_math_k_pxeHD3zfxkcatRbNa25_);void
pm_math_lin_alg_vectorAdd(uint32_T n,const double*
pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,
double*pm_math_Vzz7fHz6oElvjujXUEM3YM);void pm_math_lin_alg_vectorSubtract(
uint32_T n,const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*pm_math_Vo9I5RFcPal_bXJl4MT4zD);void
pm_math_lin_alg_vectorScale(uint32_T n,double pm_math_FQferGZUKft3_i5GvYy4Oy,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*
pm_math_kMXfgG5aJNtdgeZBD1F9s7);void pm_math_lin_alg_vectorNegate(uint32_T n,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ,double*
pm_math_Vmg5DyKiTS4YVaa_KIUPx2);void pm_math_lin_alg_matrixAssignStrided(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FnKRRzrQB6GPWacR5qZ_N1,uint32_T pm_math_VSkCOkBlf_lY_aOxP7FKDg,const
double*pm_math__Q_i1Z0_CMGpfPqpuLrnMT,double*pm_math_kyfq6L_eQbdYcPuOovpRDW);
void pm_math_lin_alg_matrixTransposeAdd(uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math__Q_i1Z0_CMGpfPqpuLrnMT);void
pm_math_lin_alg_matrixTransposeSubtract(uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_kyfq6L_eQbdYcPuOovpRDW);void
pm_math_lin_alg_matrixMultiply(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T pm_math_V2__YrimeI4E_yWnhKofpy,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixMultiplyStrided(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T pm_math_V2__YrimeI4E_yWnhKofpy,uint32_T n,uint32_T
pm_math__8PlegfDdmpxaD3yVABBka,uint32_T pm_math_khoq8zTqfel4cuvza1xV8U,
uint32_T pm_math__Cskocqf_YOea9zRlVhiAe,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixTransposeMultiply(uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void
pm_math_lin_alg_matrixMultiplyTranspose(uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
,uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_Vqiy96WqvuhCaXm5e_vvT0,
double*pm_math_FStFcQlyAJ_dVy4kGZXBPQ);void pm_math_lin_alg_scaleColumns(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_V5FB4OLCBN0eVqxcC6tIlV);void pm_math_lin_alg_scaleRows(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_knjWFknSIL0bYi3EKhrrJ4);void pm_math_lin_alg_inverseScaleColumns
(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_V5FB4OLCBN0eVqxcC6tIlV);void pm_math_lin_alg_inverseScaleRows(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*pm_math_FQferGZUKft3_i5GvYy4Oy,
double*pm_math_knjWFknSIL0bYi3EKhrrJ4);void pm_math_lin_alg_zeroMajor(uint32_T
pm_math_kRZY2SDSifK6ia_3EbLye7,uint32_T pm_math_FEXg1wwjA_S7cyqxiD_lWO,const
int32_T*pm_math_VU18hM_2GHSHWue20_mnXQ,double*pm_math_F2l4p_g4sn02huHNflQjMH);
void pm_math_lin_alg_reduceMatrix(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,uint32_T pm_math_FTSHHVQYO78NWDDPfQSm12,const int32_T*
pm_math_VLZhFoWjGJ0Q_DeI0FI6Gm,double*pm_math_F2l4p_g4sn02huHNflQjMH);void
pm_math_lin_alg_expandVector(uint32_T n,uint32_T pm_math_kZSgglHk8kWUemlk_e6TDr
,const int32_T*pm_math_VD0M3tgCkkSEiHjnw1wnK0,double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ);boolean_T pm_math_lin_alg_hasUniqueRows(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,double*pm_math_VRZCD_UL_ESThy75dC9J8D);void
pm_math_lin_alg_matrixVectorMultiply(uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,
uint32_T n,const double*pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_kcMuz7xytzhifPfhbjJNYL);void
pm_math_lin_alg_matrixVectorMultiplyStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_kcMuz7xytzhifPfhbjJNYL);void
pm_math_lin_alg_matrixTransposeVectorMultiply(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_F9DWOuzPFtdljqhs7NoH_Y);void pm_math_lin_alg_addMatrixVectorProduct(
uint32_T pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_addMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_addMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixVectorProductStrided(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,uint32_T
pm_math_FxajFA_1TahGZXVH9j9Gmx,const double*pm_math_F2l4p_g4sn02huHNflQjMH,
const double*x,double*pm_math_FzyLWRgau0pMYq2XSI3ETL);void
pm_math_lin_alg_subtractMatrixTransposeVectorProduct(uint32_T
pm_math_VLHhnPUiNQpve5VIL9P3O9,uint32_T n,const double*
pm_math_F2l4p_g4sn02huHNflQjMH,const double*x,double*
pm_math_FzyLWRgau0pMYq2XSI3ETL);double pm_math_lin_alg_dotProduct(uint32_T n,
const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,const double*
pm_math_VgJW5ZqpwPpuY1inYtaofQ);double pm_math_lin_alg_norm(uint32_T n,const
double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);double pm_math_lin_alg_inf_norm(
uint32_T n,const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);double
pm_math_lin_alg_computeQuadraticTerm(uint32_T n,const double*
pm_math__EIaXP4H4ZKuZmaTyPyymE,const double*pm_math_kEbBObcYFIxUZ5_77V3CO_,
const double*pm_math_VgJW5ZqpwPpuY1inYtaofQ);boolean_T
pm_math_lin_alg_choleskyFactor(real_T*pm_math_F2l4p_g4sn02huHNflQjMH,uint32_T n
);void pm_math_lin_alg_choleskySolve(const real_T*
pm_math_F8I2q9dciO0aXmFVhOX9ZQ,const real_T*b,uint32_T n,real_T*x,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D);void pm_math_VWv_QvuNRm4viyJPPMYOkS(const
real_T*pm_math__ut5UfJwzNlZ_XZC_yEgKo,const real_T*b,uint32_T n,boolean_T
pm_math_FEqGkTLIgMdKjaJ1lIV70b,real_T*x);void pm_math_VRlre4EWbUOCcXxk9GINjY(
const real_T*pm_math_Vi4Cp0qK964NYTFMGr9Ttn,const real_T*b,uint32_T n,
boolean_T pm_math_FEqGkTLIgMdKjaJ1lIV70b,real_T*x);void
pm_math_V4wY5inRPrGJ_q94UMd56c(const int pm_math_FdNKyaRLqeWhg5tGqF_VYT,const
int pm_math__jwySfHyx1SXgXJeVWoIzb,const real_T*pm_math_F2l4p_g4sn02huHNflQjMH
,const real_T*pm_math_Vqiy96WqvuhCaXm5e_vvT0,creal_T*
pm_math_kJeUz19e49pVaDwcttsZep,creal_T*pm_math_kyfq6L_eQbdYcPuOovpRDW,creal_T*
pm_math_kXii4Miq3Y_ShPn9EHS3D5,creal_T*pm_math_FYAUa2zJy8Cah1YsdPJok5,creal_T*
pm_math__Vu43oWQSkpLiuph6ye1KN,creal_T*pm_math_VHUYV1kdST0efLHvpiBBXm,creal_T*
pm_math_VRZCD_UL_ESThy75dC9J8D,creal_T*pm_math_Fo1tUR18sIdue1E1QXuOnL,creal_T*
pm_math___7SXc4Q_w8saDIIf2ecjp,real_T*pm_math_ktFLct2wbuxD_5iTvRiZEZ,real_T*
pm_math_kHM1aMQ_rJdeheUxTze_oV,int32_T*pm_math__qLHk6daYiGnXenMNt2kR_,int8_T*
pm_math_VS3xMd4pLil0i5cxPD0ErK);void pm_math_lin_alg_eig(uint32_T n,const
real_T*pm_math_F2l4p_g4sn02huHNflQjMH,const real_T*
pm_math_Vqiy96WqvuhCaXm5e_vvT0,real_T*pm_math_FFBCOFcrbF_Oc1OIGwRBc4,real_T*
pm_math_VlJ4W5vD3kK5X99Lo9mn6_,real_T*pm_math_FXqYP9M_eO_ybyvyG1U3_4,real_T*
pm_math__0YXwStzi_WyXXlmJ2IHY2,real_T*pm_math_FaY46KF_stWCXP_M9L46Rm,int32_T*
pm_math_k_auWg11HRS2a54rbYevu2){uint32_T pm_math_kf0Q9aAjyy0Uc5W69prr5z,
pm_math_kwrB3ZoKf7OufTHWaHJV7a;creal_T*pm_math_kJeUz19e49pVaDwcttsZep;creal_T*
pm_math_kyfq6L_eQbdYcPuOovpRDW;creal_T*pm_math_kXii4Miq3Y_ShPn9EHS3D5;creal_T*
pm_math_FYAUa2zJy8Cah1YsdPJok5;creal_T*pm_math__Vu43oWQSkpLiuph6ye1KN;creal_T*
pm_math_VHUYV1kdST0efLHvpiBBXm;creal_T*pm_math_VRZCD_UL_ESThy75dC9J8D;creal_T*
pm_math_Fo1tUR18sIdue1E1QXuOnL;creal_T*pm_math___7SXc4Q_w8saDIIf2ecjp;real_T*
pm_math_ktFLct2wbuxD_5iTvRiZEZ;real_T*pm_math_kHM1aMQ_rJdeheUxTze_oV;int32_T*
pm_math__qLHk6daYiGnXenMNt2kR_;int8_T*pm_math_VS3xMd4pLil0i5cxPD0ErK;
pm_math_kf0Q9aAjyy0Uc5W69prr5z=n*n;pm_math_kJeUz19e49pVaDwcttsZep=(creal_T*)
pm_math_FaY46KF_stWCXP_M9L46Rm;pm_math_FaY46KF_stWCXP_M9L46Rm+=2*
pm_math_kf0Q9aAjyy0Uc5W69prr5z;pm_math_kyfq6L_eQbdYcPuOovpRDW=(creal_T*)
pm_math_FaY46KF_stWCXP_M9L46Rm;pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;
pm_math_kXii4Miq3Y_ShPn9EHS3D5=(creal_T*)pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math_FaY46KF_stWCXP_M9L46Rm+=2*pm_math_kf0Q9aAjyy0Uc5W69prr5z;
pm_math_FYAUa2zJy8Cah1YsdPJok5=(creal_T*)pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math_FaY46KF_stWCXP_M9L46Rm+=2*pm_math_kf0Q9aAjyy0Uc5W69prr5z;
pm_math__Vu43oWQSkpLiuph6ye1KN=(creal_T*)pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;pm_math_VHUYV1kdST0efLHvpiBBXm=(creal_T*)
pm_math_FaY46KF_stWCXP_M9L46Rm;pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;
pm_math_VRZCD_UL_ESThy75dC9J8D=(creal_T*)pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;pm_math_Fo1tUR18sIdue1E1QXuOnL=(creal_T*)
pm_math_FaY46KF_stWCXP_M9L46Rm;pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;
pm_math___7SXc4Q_w8saDIIf2ecjp=(creal_T*)pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math_FaY46KF_stWCXP_M9L46Rm+=2*n;pm_math_ktFLct2wbuxD_5iTvRiZEZ=
pm_math_FaY46KF_stWCXP_M9L46Rm;pm_math_FaY46KF_stWCXP_M9L46Rm+=n;
pm_math_kHM1aMQ_rJdeheUxTze_oV=pm_math_FaY46KF_stWCXP_M9L46Rm;
pm_math__qLHk6daYiGnXenMNt2kR_=pm_math_k_auWg11HRS2a54rbYevu2;
pm_math_k_auWg11HRS2a54rbYevu2+=n;pm_math_VS3xMd4pLil0i5cxPD0ErK=(int8_T*)
pm_math_k_auWg11HRS2a54rbYevu2;pm_math_V4wY5inRPrGJ_q94UMd56c(n,n,
pm_math_F2l4p_g4sn02huHNflQjMH,pm_math_Vqiy96WqvuhCaXm5e_vvT0,
pm_math_kJeUz19e49pVaDwcttsZep,pm_math_kyfq6L_eQbdYcPuOovpRDW,
pm_math_kXii4Miq3Y_ShPn9EHS3D5,pm_math_FYAUa2zJy8Cah1YsdPJok5,
pm_math__Vu43oWQSkpLiuph6ye1KN,pm_math_VHUYV1kdST0efLHvpiBBXm,
pm_math_VRZCD_UL_ESThy75dC9J8D,pm_math_Fo1tUR18sIdue1E1QXuOnL,
pm_math___7SXc4Q_w8saDIIf2ecjp,pm_math_ktFLct2wbuxD_5iTvRiZEZ,
pm_math_kHM1aMQ_rJdeheUxTze_oV,pm_math__qLHk6daYiGnXenMNt2kR_,
pm_math_VS3xMd4pLil0i5cxPD0ErK);for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<pm_math_kf0Q9aAjyy0Uc5W69prr5z;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a){pm_math_FXqYP9M_eO_ybyvyG1U3_4[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]=pm_math_kJeUz19e49pVaDwcttsZep[
pm_math_kwrB3ZoKf7OufTHWaHJV7a].re;pm_math__0YXwStzi_WyXXlmJ2IHY2[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]=pm_math_kJeUz19e49pVaDwcttsZep[
pm_math_kwrB3ZoKf7OufTHWaHJV7a].im;}for(pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;
pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++pm_math_kwrB3ZoKf7OufTHWaHJV7a){
pm_math_FFBCOFcrbF_Oc1OIGwRBc4[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=
pm_math_kyfq6L_eQbdYcPuOovpRDW[pm_math_kwrB3ZoKf7OufTHWaHJV7a].re;
pm_math_VlJ4W5vD3kK5X99Lo9mn6_[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=
pm_math_kyfq6L_eQbdYcPuOovpRDW[pm_math_kwrB3ZoKf7OufTHWaHJV7a].im;}}uint32_T
pm_math_lin_alg_eigReqdRealWorkSize(uint32_T n){return 2*n*n+2*n+2*n*n+2*n*n+2
*n+2*n+2*n+2*n+2*n+n+n;}uint32_T pm_math_lin_alg_eigReqdInt32WorkSize(uint32_T
n){return n+n*n;}boolean_T
pm_math_lin_alg_orthonormalizeRealEigenvectorsWrtSpdB(uint32_T n,const real_T*
pm_math_Vqiy96WqvuhCaXm5e_vvT0,real_T*pm_math_kJeUz19e49pVaDwcttsZep,real_T*
pm_math_VRZCD_UL_ESThy75dC9J8D){uint32_T pm_math_kf0Q9aAjyy0Uc5W69prr5z,
pm_math_kwrB3ZoKf7OufTHWaHJV7a;real_T*pm_math_F5Ms7XlOr2OXjyocf4hGw2;real_T*
pm_math__ut5UfJwzNlZ_XZC_yEgKo;real_T*pm_math_kXlUJG1kmp_DZeXvK3s20a;boolean_T
pm_math_kjYqHZv9KmWAVe0FV70knd=false;boolean_T pm_math_FYXbPOwlJy0Tg9Zmhvc3ve=
false;pm_math_kf0Q9aAjyy0Uc5W69prr5z=n*n;pm_math_F5Ms7XlOr2OXjyocf4hGw2=
pm_math_VRZCD_UL_ESThy75dC9J8D;pm_math_VRZCD_UL_ESThy75dC9J8D+=
pm_math_kf0Q9aAjyy0Uc5W69prr5z;pm_math__ut5UfJwzNlZ_XZC_yEgKo=
pm_math_VRZCD_UL_ESThy75dC9J8D;pm_math_VRZCD_UL_ESThy75dC9J8D+=
pm_math_kf0Q9aAjyy0Uc5W69prr5z;pm_math_kXlUJG1kmp_DZeXvK3s20a=
pm_math_VRZCD_UL_ESThy75dC9J8D;pm_math_lin_alg_matrixTransposeMultiply(n,n,n,
pm_math_kJeUz19e49pVaDwcttsZep,pm_math_Vqiy96WqvuhCaXm5e_vvT0,
pm_math_F5Ms7XlOr2OXjyocf4hGw2);pm_math_lin_alg_matrixMultiply(n,n,n,
pm_math_F5Ms7XlOr2OXjyocf4hGw2,pm_math_kJeUz19e49pVaDwcttsZep,
pm_math__ut5UfJwzNlZ_XZC_yEgKo);pm_math_kjYqHZv9KmWAVe0FV70knd=
pm_math_lin_alg_choleskyFactor(pm_math__ut5UfJwzNlZ_XZC_yEgKo,n);if(!
pm_math_kjYqHZv9KmWAVe0FV70knd){pm_math_lin_alg_transposeColMajor(n,n,
pm_math_kJeUz19e49pVaDwcttsZep,pm_math_kXlUJG1kmp_DZeXvK3s20a);for(
pm_math_kwrB3ZoKf7OufTHWaHJV7a=0;pm_math_kwrB3ZoKf7OufTHWaHJV7a<n;++
pm_math_kwrB3ZoKf7OufTHWaHJV7a,pm_math_F5Ms7XlOr2OXjyocf4hGw2+=n,
pm_math_kXlUJG1kmp_DZeXvK3s20a+=n)pm_math_VWv_QvuNRm4viyJPPMYOkS(
pm_math__ut5UfJwzNlZ_XZC_yEgKo,pm_math_kXlUJG1kmp_DZeXvK3s20a,n,
pm_math_FYXbPOwlJy0Tg9Zmhvc3ve,pm_math_F5Ms7XlOr2OXjyocf4hGw2);
pm_math_F5Ms7XlOr2OXjyocf4hGw2-=pm_math_kf0Q9aAjyy0Uc5W69prr5z;
pm_math_lin_alg_transposeColMajor(n,n,pm_math_F5Ms7XlOr2OXjyocf4hGw2,
pm_math_kJeUz19e49pVaDwcttsZep);}return pm_math_kjYqHZv9KmWAVe0FV70knd;}
