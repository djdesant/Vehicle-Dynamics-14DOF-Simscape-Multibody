#ifndef __sm_contactForceSupport_h__
#define __sm_contactForceSupport_h__
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
void sm_core_compiler_computeSpatialContactWrenches(int32_T pointFlag,size_t
numProximityInfos,boolean_T isValid,const real_T*dists,const pm_math_Vector3*
extremePointsBase,const pm_math_Vector3*extremePointsFoll,const pm_math_Vector3
*normalsBase,const pm_math_Vector3*normalsFoll,const pm_math_Transform3*Bapp,
const pm_math_Transform3*Fapp,const pm_math_Transform3*Bfull,const
pm_math_Transform3*Ffull,const pm_math_SpatialVector*VbBase,const
pm_math_SpatialVector*VbFoll,int normalForceMode,int frictionalForceMode,
real_T springConstant,real_T damperConstant,real_T transRegWidth,real_T
coeffDynamicFric,real_T staticFricMult,real_T velocityDivisor,const real_T*
normalForceInput,const pm_math_Vector2*frictionalForceInput,
pm_math_SpatialVector*baseContactWrench,pm_math_SpatialVector*
follContactWrench);void sm_core_compiler_computeSpatialContactOutputs(int32_T
pointFlag,size_t numProximityInfos,const real_T*dists,const pm_math_Vector3*
extremePointsBase,const pm_math_Vector3*extremePointsFoll,const pm_math_Vector3
*normalsBase,const pm_math_Vector3*normalsFoll,const pm_math_Transform3*Bapp,
const pm_math_Transform3*Fapp,const pm_math_Transform3*Bfull,const
pm_math_Transform3*Ffull,const pm_math_SpatialVector*VbBase,const
pm_math_SpatialVector*VbFoll,int normalForceMode,int frictionalForceMode,
real_T springConstant,real_T damperConstant,real_T transRegWidth,real_T
coeffDynamicFric,real_T staticFricMult,real_T velocityDivisor,const real_T*
normalForceInput,const pm_math_Vector2*frictionalForceInput,pm_math_Vector3*
baseTranslation,pm_math_Vector3*follTranslation,pm_math_Matrix3x3*baseRotation
,pm_math_Matrix3x3*follRotation,real_T*normalVelocity,pm_math_Vector2*
tangentialVelocity,real_T*normalForce,real_T*frictionalForceMag);void
sm_core_compiler_computePlanarContactWrenches(int32_T pointFlag,size_t
numProximityInfos,boolean_T isValid,const real_T*dists,const pm_math_Vector2*
extremePointsBase,const pm_math_Vector2*extremePointsFoll,const pm_math_Vector2
*normalsBase,const pm_math_Vector2*normalsFoll,const pm_math_Transform3*Bapp,
const pm_math_Transform3*Fapp,const pm_math_Quaternion*QBfull,const
pm_math_Quaternion*QFfull,const pm_math_SpatialVector*VbBase,const
pm_math_SpatialVector*VbFoll,int normalForceMode,int frictionalForceMode,
real_T springConstant,real_T damperConstant,real_T transRegWidth,real_T
coeffDynamicFric,real_T staticFricMult,real_T velocityDivisor,const real_T*
normalForceInput,const real_T*frictionalForceInput,pm_math_SpatialVector*
baseContactWrench,pm_math_SpatialVector*follContactWrench);void
sm_core_compiler_computePlanarContactOutputs(int32_T pointFlag,size_t
numProximityInfos,const real_T*dists,const pm_math_Vector2*extremePointsBase,
const pm_math_Vector2*extremePointsFoll,const pm_math_Vector2*normalsBase,
const pm_math_Vector2*normalsFoll,const pm_math_Transform3*Bapp,const
pm_math_Transform3*Fapp,const pm_math_Quaternion*QBfull,const
pm_math_Quaternion*QFfull,const pm_math_SpatialVector*VbBase,const
pm_math_SpatialVector*VbFoll,int normalForceMode,int frictionalForceMode,
real_T springConstant,real_T damperConstant,real_T transRegWidth,real_T
coeffDynamicFric,real_T staticFricMult,real_T velocityDivisor,const real_T*
normalForceInput,const real_T*frictionalForceInput,pm_math_Vector3*
baseTranslation,pm_math_Vector3*follTranslation,pm_math_Matrix3x3*baseRotation
,pm_math_Matrix3x3*follRotation,real_T*normalVelocity,real_T*
tangentialVelocity,real_T*normalForce,real_T*frictionalForceMag);
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __sm_contactForceSupport_h__ */
