#include "pm_std.h"
#include "pm_std.h"
#include "pm_std.h"
void pm_math_lin_alg_jacobi3(const real_T*pm_math_kDh9Sdqtd1dhZLBqs4W1eB,const
real_T*pm_math__RbwWmDdK3dK_yyvubKIqt,real_T*pm_math_kHkjc8xugZ4Ji9gPTWP29s,
real_T*pm_math_FqUCZrSGGNOuePgRr82o_8);
#include "math.h"
static const uint32_T pm_math_VUZtz9wPFx4wWDr9w16kxh=100;void
pm_math_lin_alg_jacobi3(const real_T*pm_math__othH60mta_faevm1EMQjf,const
real_T*pm_math_kytvbxUigAOvVeWaEibCRI,real_T*pm_math_kDh9Sdqtd1dhZLBqs4W1eB,
real_T*pm_math_FqUCZrSGGNOuePgRr82o_8){uint32_T pm_math_V2__YrimeI4E_yWnhKofpy
;uint32_T pm_math_FbsbQRdIBwKmhqkBxxfakr;uint32_T
pm_math_FPwVUAP_do0ogeBnBjppr5;real_T pm_math__RbwWmDdK3dK_yyvubKIqt[3];real_T
pm_math_VKe8I84V6W8ac1Gw2wS_He;real_T pm_math_k9xXRhWDFj0BVql_df_Gt3;real_T
pm_math_Fk2O4u6vQUpibmbv8Kjgnn;const real_T pm_math__QgqlEW2f18scLxPhDEnMF=
1.0e-20*1.0e-20;pm_math_FqUCZrSGGNOuePgRr82o_8[1]=
pm_math_FqUCZrSGGNOuePgRr82o_8[2]=pm_math_FqUCZrSGGNOuePgRr82o_8[3]=
pm_math_FqUCZrSGGNOuePgRr82o_8[5]=pm_math_FqUCZrSGGNOuePgRr82o_8[6]=
pm_math_FqUCZrSGGNOuePgRr82o_8[7]=0.0;pm_math_FqUCZrSGGNOuePgRr82o_8[0]=
pm_math_FqUCZrSGGNOuePgRr82o_8[4]=pm_math_FqUCZrSGGNOuePgRr82o_8[8]=1.0;for(
pm_math_FbsbQRdIBwKmhqkBxxfakr=0;pm_math_FbsbQRdIBwKmhqkBxxfakr<3;++
pm_math_FbsbQRdIBwKmhqkBxxfakr){pm_math_kDh9Sdqtd1dhZLBqs4W1eB[
pm_math_FbsbQRdIBwKmhqkBxxfakr]=pm_math__othH60mta_faevm1EMQjf[
pm_math_FbsbQRdIBwKmhqkBxxfakr];pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_FbsbQRdIBwKmhqkBxxfakr]=pm_math_kytvbxUigAOvVeWaEibCRI[
pm_math_FbsbQRdIBwKmhqkBxxfakr];}pm_math_V2__YrimeI4E_yWnhKofpy=(fabs(
pm_math__RbwWmDdK3dK_yyvubKIqt[0])>fabs(pm_math__RbwWmDdK3dK_yyvubKIqt[1]))?0:
1;if(fabs(pm_math__RbwWmDdK3dK_yyvubKIqt[2])>fabs(
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_V2__YrimeI4E_yWnhKofpy]))
pm_math_V2__YrimeI4E_yWnhKofpy=2;pm_math_VKe8I84V6W8ac1Gw2wS_He=
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[0]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[0]+
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[1]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[1]+
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[2]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[2];
pm_math_k9xXRhWDFj0BVql_df_Gt3=pm_math__RbwWmDdK3dK_yyvubKIqt[0]*
pm_math__RbwWmDdK3dK_yyvubKIqt[0]+pm_math__RbwWmDdK3dK_yyvubKIqt[1]*
pm_math__RbwWmDdK3dK_yyvubKIqt[1]+pm_math__RbwWmDdK3dK_yyvubKIqt[2]*
pm_math__RbwWmDdK3dK_yyvubKIqt[2];for(pm_math_FPwVUAP_do0ogeBnBjppr5=0;
pm_math_FPwVUAP_do0ogeBnBjppr5<100&&pm_math_k9xXRhWDFj0BVql_df_Gt3>
pm_math_VKe8I84V6W8ac1Gw2wS_He*pm_math__QgqlEW2f18scLxPhDEnMF;++
pm_math_FPwVUAP_do0ogeBnBjppr5){const uint32_T pm_math_kwrB3ZoKf7OufTHWaHJV7a=
pm_math_V2__YrimeI4E_yWnhKofpy==2?0:(pm_math_V2__YrimeI4E_yWnhKofpy+1);const
uint32_T pm_math_kyp6uAyJE40UVuAQNEYzS1=pm_math_V2__YrimeI4E_yWnhKofpy==0?2:(
pm_math_V2__YrimeI4E_yWnhKofpy-1);const real_T pm_math_Flz54uVsi__UbP_LLGOB06=
0.5*atan2(2.0*pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_V2__YrimeI4E_yWnhKofpy],
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[pm_math_kyp6uAyJE40UVuAQNEYzS1]-
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[pm_math_kwrB3ZoKf7OufTHWaHJV7a]);const real_T
pm_math_FFZbGh27ya8eem_J_hUtAZ=cos(pm_math_Flz54uVsi__UbP_LLGOB06);const real_T
pm_math_FQferGZUKft3_i5GvYy4Oy=sin(pm_math_Flz54uVsi__UbP_LLGOB06);{real_T*
pm_Fr_bHKkQKFWbfi50VWd5Pw=pm_math_FqUCZrSGGNOuePgRr82o_8;for(
pm_math_FbsbQRdIBwKmhqkBxxfakr=0;pm_math_FbsbQRdIBwKmhqkBxxfakr<3;++
pm_math_FbsbQRdIBwKmhqkBxxfakr,pm_Fr_bHKkQKFWbfi50VWd5Pw+=3){
pm_math_Fk2O4u6vQUpibmbv8Kjgnn=pm_Fr_bHKkQKFWbfi50VWd5Pw[
pm_math_kwrB3ZoKf7OufTHWaHJV7a];pm_Fr_bHKkQKFWbfi50VWd5Pw[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]=pm_Fr_bHKkQKFWbfi50VWd5Pw[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]*pm_math_FFZbGh27ya8eem_J_hUtAZ-
pm_Fr_bHKkQKFWbfi50VWd5Pw[pm_math_kyp6uAyJE40UVuAQNEYzS1]*
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_Fr_bHKkQKFWbfi50VWd5Pw[
pm_math_kyp6uAyJE40UVuAQNEYzS1]=pm_Fr_bHKkQKFWbfi50VWd5Pw[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*pm_math_FFZbGh27ya8eem_J_hUtAZ+
pm_math_Fk2O4u6vQUpibmbv8Kjgnn*pm_math_FQferGZUKft3_i5GvYy4Oy;}}{const real_T
pm_math_FmQQKK7CeFlKc5_l03nvlx=pm_math_FFZbGh27ya8eem_J_hUtAZ*
pm_math_FFZbGh27ya8eem_J_hUtAZ;const real_T pm_math_Fl7GdOQqGEWCcqQK5l5ja4=
pm_math_FQferGZUKft3_i5GvYy4Oy*pm_math_FQferGZUKft3_i5GvYy4Oy;const real_T
pm_kpzAtHMD4_WnheH0UiioSE=2.0*pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_V2__YrimeI4E_yWnhKofpy]*pm_math_FFZbGh27ya8eem_J_hUtAZ*
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_math_Fk2O4u6vQUpibmbv8Kjgnn=
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[pm_math_kwrB3ZoKf7OufTHWaHJV7a];
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[pm_math_kwrB3ZoKf7OufTHWaHJV7a]*
pm_math_FmQQKK7CeFlKc5_l03nvlx+pm_math_kDh9Sdqtd1dhZLBqs4W1eB[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*pm_math_Fl7GdOQqGEWCcqQK5l5ja4-
pm_kpzAtHMD4_WnheH0UiioSE;pm_math_kDh9Sdqtd1dhZLBqs4W1eB[
pm_math_kyp6uAyJE40UVuAQNEYzS1]=pm_math_kDh9Sdqtd1dhZLBqs4W1eB[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*pm_math_FmQQKK7CeFlKc5_l03nvlx+
pm_math_Fk2O4u6vQUpibmbv8Kjgnn*pm_math_Fl7GdOQqGEWCcqQK5l5ja4+
pm_kpzAtHMD4_WnheH0UiioSE;}pm_math_Fk2O4u6vQUpibmbv8Kjgnn=
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kwrB3ZoKf7OufTHWaHJV7a];
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kwrB3ZoKf7OufTHWaHJV7a]=
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kwrB3ZoKf7OufTHWaHJV7a]*
pm_math_FFZbGh27ya8eem_J_hUtAZ+pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*pm_math_FQferGZUKft3_i5GvYy4Oy;
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kyp6uAyJE40UVuAQNEYzS1]=
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kyp6uAyJE40UVuAQNEYzS1]*
pm_math_FFZbGh27ya8eem_J_hUtAZ-pm_math_Fk2O4u6vQUpibmbv8Kjgnn*
pm_math_FQferGZUKft3_i5GvYy4Oy;pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_V2__YrimeI4E_yWnhKofpy]=0.0;pm_math_VKe8I84V6W8ac1Gw2wS_He=
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[0]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[0]+
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[1]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[1]+
pm_math_kDh9Sdqtd1dhZLBqs4W1eB[2]*pm_math_kDh9Sdqtd1dhZLBqs4W1eB[2];
pm_math_k9xXRhWDFj0BVql_df_Gt3=pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]*pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_kwrB3ZoKf7OufTHWaHJV7a]+pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_kyp6uAyJE40UVuAQNEYzS1]*pm_math__RbwWmDdK3dK_yyvubKIqt[
pm_math_kyp6uAyJE40UVuAQNEYzS1];pm_math_V2__YrimeI4E_yWnhKofpy=(fabs(
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kwrB3ZoKf7OufTHWaHJV7a])>fabs(
pm_math__RbwWmDdK3dK_yyvubKIqt[pm_math_kyp6uAyJE40UVuAQNEYzS1]))?
pm_math_kwrB3ZoKf7OufTHWaHJV7a:pm_math_kyp6uAyJE40UVuAQNEYzS1;}(void)0;;
pm_math_Fk2O4u6vQUpibmbv8Kjgnn=pm_math_FqUCZrSGGNOuePgRr82o_8[5];
pm_math_FqUCZrSGGNOuePgRr82o_8[5]=pm_math_FqUCZrSGGNOuePgRr82o_8[7];
pm_math_FqUCZrSGGNOuePgRr82o_8[7]=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;
pm_math_Fk2O4u6vQUpibmbv8Kjgnn=pm_math_FqUCZrSGGNOuePgRr82o_8[6];
pm_math_FqUCZrSGGNOuePgRr82o_8[6]=pm_math_FqUCZrSGGNOuePgRr82o_8[2];
pm_math_FqUCZrSGGNOuePgRr82o_8[2]=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;
pm_math_Fk2O4u6vQUpibmbv8Kjgnn=pm_math_FqUCZrSGGNOuePgRr82o_8[1];
pm_math_FqUCZrSGGNOuePgRr82o_8[1]=pm_math_FqUCZrSGGNOuePgRr82o_8[3];
pm_math_FqUCZrSGGNOuePgRr82o_8[3]=pm_math_Fk2O4u6vQUpibmbv8Kjgnn;}
