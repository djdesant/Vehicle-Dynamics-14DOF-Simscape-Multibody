/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'sm_vehicle_2axle_heave_roll/Vehicle/World/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"
#include "sm_vehicle_2axle_heave_roll_836bb176_1_geometries.h"

PmfMessageId sm_vehicle_2axle_heave_roll_836bb176_1_compOutputsKin(const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *input, const double *inputDot, const double *inputDdot, const
  double *discreteState, double *output, NeuDiagnosticManager *neDiagMgr)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  boolean_T bb[1];
  double xx[375];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) inputDdot;
  (void) discreteState;
  (void) neDiagMgr;
  xx[0] = 0.338430610991899;
  xx[1] = 0.1016374984694517;
  xx[2] = 0.03679784973285759;
  xx[3] = - 0.934762247153553;
  xx[4] = - xx[0];
  xx[5] = xx[1];
  xx[6] = xx[2];
  xx[7] = xx[3];
  xx[8] = state[6];
  xx[9] = state[7];
  xx[10] = state[8];
  xx[11] = state[9];
  xx[12] = 0.9999999997549381;
  xx[13] = 2.09302257320166e-5;
  xx[14] = 4.494946017906737e-6;
  xx[15] = 5.643148656246455e-6;
  xx[16] = xx[12];
  xx[17] = xx[13];
  xx[18] = - xx[14];
  xx[19] = - xx[15];
  pm_math_Quaternion_composeInverse_ra(xx + 8, xx + 16, xx + 20);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 20, xx + 8);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 8, xx + 4);
  xx[20] = - xx[4];
  xx[21] = - xx[5];
  xx[22] = - xx[6];
  xx[23] = - xx[7];
  xx[24] = 1.499988338565875;
  xx[25] = 2.645620204187505e-6;
  xx[26] = 0.6146812028982972;
  xx[27] = xx[24];
  xx[28] = xx[25];
  xx[29] = - xx[26];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 27, xx + 30);
  xx[27] = 1.795436775680343e-5;
  xx[28] = 9.726932096251812e-6;
  xx[29] = - 0.08530531197639897;
  pm_math_Quaternion_xform_ra(xx + 8, xx + 27, xx + 33);
  pm_math_Quaternion_xform_ra(xx + 0, xx + 33, xx + 36);
  xx[0] = xx[36] + state[0];
  xx[1] = - (0.7502690808970445 * state[3] + 0.62522423408087 * state[4] +
             0.2149208304652991 * state[5]);
  xx[2] = 0.6401843996644798 * state[3] - 0.7682212795973762 * state[4];
  xx[3] = 0.9766314743198204 * state[5] - (0.1651067553921828 * state[3] +
    0.1375889628268189 * state[4]);
  pm_math_Quaternion_inverseXform_ra(xx + 8, xx + 1, xx + 33);
  xx[1] = state[10];
  xx[2] = state[11];
  xx[3] = state[12];
  pm_math_Quaternion_xform_ra(xx + 16, xx + 1, xx + 8);
  pm_math_Vector3_cross_ra(xx + 8, xx + 27, xx + 1);
  xx[11] = xx[33] + xx[1];
  xx[27] = - xx[24];
  xx[28] = - xx[25];
  xx[29] = xx[26];
  pm_math_Vector3_cross_ra(xx + 8, xx + 27, xx + 24);
  xx[27] = xx[34] + xx[2];
  xx[1] = xx[35] + xx[3];
  xx[33] = xx[11] - xx[24];
  xx[34] = xx[27] - xx[25];
  xx[35] = xx[1] - xx[26];
  pm_math_Quaternion_xform_ra(xx + 20, xx + 33, xx + 24);
  xx[2] = 9.87654321;
  xx[3] = xx[37] + state[1];
  xx[28] = xx[38] + state[2];
  pm_math_Quaternion_compose_ra(xx + 20, xx + 16, xx + 33);
  xx[20] = - xx[33];
  xx[21] = - xx[34];
  xx[22] = - xx[35];
  xx[23] = - xx[36];
  pm_math_Quaternion_inverseXform_ra(xx + 16, xx + 8, xx + 33);
  pm_math_Quaternion_xform_ra(xx + 20, xx + 33, xx + 36);
  xx[29] = 0.5;
  xx[33] = xx[29] * input[9];
  xx[34] = cos(xx[33]);
  xx[35] = xx[29] * input[10];
  xx[39] = cos(xx[35]);
  xx[40] = xx[29] * input[11];
  xx[41] = cos(xx[40]);
  xx[42] = xx[39] * xx[41];
  xx[43] = sin(xx[35]);
  xx[35] = sin(xx[40]);
  xx[40] = xx[43] * xx[35];
  xx[44] = sin(xx[33]);
  xx[33] = xx[34] * xx[42] + xx[40] * xx[44];
  xx[45] = xx[42] * xx[44] - xx[34] * xx[40];
  xx[40] = xx[39] * xx[35];
  xx[42] = xx[41] * xx[43];
  xx[46] = xx[34] * xx[40] + xx[42] * xx[44];
  xx[47] = xx[34] * xx[42] - xx[40] * xx[44];
  xx[48] = xx[33];
  xx[49] = xx[45];
  xx[50] = xx[46];
  xx[51] = xx[47];
  xx[52] = input[6];
  xx[53] = input[7];
  xx[54] = input[8];
  xx[40] = 0.7071067811865476;
  xx[42] = xx[29] * input[4];
  xx[55] = cos(xx[42]);
  xx[56] = - xx[12];
  xx[57] = - xx[13];
  xx[58] = xx[14];
  xx[59] = xx[15];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 56, xx + 12);
  xx[60] = xx[29] * state[15];
  xx[61] = xx[29] * sin(xx[60]);
  xx[62] = xx[29] * cos(xx[60]);
  xx[60] = xx[61] - xx[62];
  xx[63] = xx[62] + xx[61];
  xx[64] = xx[62] + xx[61];
  xx[65] = xx[62] - xx[61];
  xx[66] = xx[60];
  xx[67] = xx[63];
  xx[68] = xx[64];
  xx[69] = xx[65];
  pm_math_Quaternion_compose_ra(xx + 12, xx + 66, xx + 70);
  xx[61] = sin(xx[42]);
  xx[74] = xx[55] * xx[70] - xx[71] * xx[61];
  xx[75] = xx[70] * xx[61] + xx[55] * xx[71];
  xx[76] = xx[55] * xx[72] + xx[73] * xx[61];
  xx[77] = xx[55] * xx[73] - xx[72] * xx[61];
  xx[42] = xx[29] * state[17];
  xx[62] = - (xx[40] * cos(xx[42]));
  xx[78] = - (xx[40] * sin(xx[42]));
  xx[79] = xx[62];
  xx[80] = xx[62];
  xx[81] = xx[78];
  xx[82] = xx[78];
  pm_math_Quaternion_compose_ra(xx + 74, xx + 79, xx + 83);
  xx[42] = xx[40] * xx[83];
  xx[62] = xx[40] * xx[84];
  xx[74] = xx[42] + xx[62];
  xx[75] = xx[62] - xx[42];
  xx[42] = xx[40] * xx[85];
  xx[62] = xx[40] * xx[86];
  xx[76] = xx[42] - xx[62];
  xx[77] = xx[62] + xx[42];
  xx[42] = 0.8;
  xx[62] = xx[42] * xx[70] * xx[72];
  xx[78] = xx[42] * xx[72];
  xx[83] = 5.000000000000004e-3;
  xx[84] = xx[83] * xx[73];
  xx[85] = xx[42] * xx[71];
  xx[86] = xx[84] + xx[85];
  xx[87] = xx[83] * xx[72];
  xx[88] = xx[78];
  xx[89] = - xx[86];
  xx[90] = xx[87];
  pm_math_Vector3_cross_ra(xx + 71, xx + 88, xx + 91);
  xx[88] = 2.0;
  xx[89] = 0.2903;
  xx[90] = xx[89] * xx[64];
  xx[94] = xx[89] * xx[65];
  xx[95] = - ((xx[90] * xx[64] + xx[94] * xx[65]) * xx[88] - xx[89]);
  xx[96] = (xx[60] * xx[94] + xx[63] * xx[90]) * xx[88];
  xx[97] = - (0.1 + xx[88] * (xx[60] * xx[90] - xx[63] * xx[94]));
  pm_math_Quaternion_xform_ra(xx + 12, xx + 95, xx + 63);
  xx[60] = 8.990128258360823e-6;
  xx[89] = 4.186040072247764e-5;
  xx[90] = 0.9999999990834422;
  xx[98] = 1.499986857891751 - xx[60] * state[13];
  xx[99] = - (4.248787794804562e-6 + xx[89] * state[13]);
  xx[100] = xx[90] * state[13] - 0.4499812030492543;
  pm_math_Quaternion_xform_ra(xx + 4, xx + 98, xx + 101);
  xx[94] = xx[63] + xx[101] + xx[0];
  xx[104] = (xx[62] + xx[91]) * xx[88] - xx[83] + xx[94];
  xx[105] = xx[64] + xx[102] + xx[3];
  xx[106] = (xx[92] - xx[70] * xx[86]) * xx[88] + xx[105];
  xx[86] = xx[70] * xx[87];
  xx[63] = xx[65] + xx[103] + xx[28];
  xx[64] = xx[42] + xx[88] * (xx[93] + xx[86]) + xx[63];
  xx[107] = xx[74];
  xx[108] = xx[75];
  xx[109] = xx[76];
  xx[110] = xx[77];
  xx[111] = xx[104];
  xx[112] = xx[106];
  xx[113] = xx[64];
  xx[65] = xx[35] * inputDot[10];
  xx[91] = inputDot[9] - xx[88] * xx[41] * xx[65];
  xx[92] = inputDot[10] - xx[88] * xx[65] * xx[35];
  xx[65] = xx[44] * xx[92];
  xx[93] = xx[44] * inputDot[11];
  xx[101] = inputDot[11] + xx[88] * (xx[34] * xx[65] - xx[93] * xx[44]);
  xx[102] = xx[92] - (xx[34] * xx[93] + xx[65] * xx[44]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 56, xx + 8, xx + 114);
  pm_math_Quaternion_inverseXform_ra(xx + 66, xx + 114, xx + 117);
  xx[65] = xx[118] - state[16];
  xx[92] = xx[119] * xx[61];
  xx[93] = xx[61] * xx[65];
  xx[120] = xx[117] + inputDot[4];
  xx[121] = xx[65] + xx[88] * (xx[55] * xx[92] - xx[93] * xx[61]);
  xx[122] = xx[119] - (xx[55] * xx[93] + xx[92] * xx[61]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 79, xx + 120, xx + 123);
  xx[92] = xx[124] + state[18];
  xx[93] = xx[40] * xx[40] * xx[125];
  xx[103] = xx[40] * xx[92] * xx[40];
  xx[118] = xx[92] - (xx[93] + xx[103]) * xx[88];
  xx[92] = xx[125] + xx[88] * (xx[103] - xx[93]);
  xx[93] = 1.0;
  xx[103] = 0.35;
  xx[120] = 2.4e5;
  xx[121] = 1.0e4;
  xx[122] = 1.5;
  xx[124] = 0.175;
  xx[125] = 0.0;
  xx[126] = 1.7e-3;
  xx[127] = 1.0e-4;
  xx[128] = - xx[93];
  xx[129] = 16.7;
  xx[130] = xx[93];
  xx[131] = xx[103];
  xx[132] = 0.245;
  xx[133] = 0.6;
  xx[134] = xx[120];
  xx[135] = xx[120];
  xx[136] = 5000.0;
  xx[137] = 2.3945e5;
  xx[138] = 8.390000000000001;
  xx[139] = 0.2587;
  xx[140] = 0.0736;
  xx[141] = 0.9974;
  xx[142] = 8.0e-4;
  xx[143] = 0.0466;
  xx[144] = 0.1078;
  xx[145] = - 0.4751;
  xx[146] = 15.416;
  xx[147] = 85.19;
  xx[148] = 257.4;
  xx[149] = 0.5119;
  xx[150] = - 2.04964e4;
  xx[151] = - 6.0e4;
  xx[152] = 8.82117e4;
  xx[153] = 0.7097;
  xx[154] = xx[121];
  xx[155] = 1.0e6;
  xx[156] = 100.0;
  xx[157] = xx[121];
  xx[158] = - xx[122];
  xx[159] = xx[122];
  xx[160] = - xx[103];
  xx[161] = xx[103];
  xx[162] = - xx[124];
  xx[163] = xx[124];
  xx[164] = 1.5752;
  xx[165] = 1.166;
  xx[166] = - 7.6e-3;
  xx[167] = xx[125];
  xx[168] = - 0.09660000000000001;
  xx[169] = xx[126];
  xx[170] = xx[125];
  xx[171] = - 0.9;
  xx[172] = 29.61;
  xx[173] = 4.205;
  xx[174] = 0.1389;
  xx[175] = - 2.4e-3;
  xx[176] = 5.0e-4;
  xx[177] = 0.0745;
  xx[178] = - 8.800000000000001e-3;
  xx[179] = 18.435;
  xx[180] = 16.5;
  xx[181] = xx[125];
  xx[182] = 1.009;
  xx[183] = - 0.8544;
  xx[184] = 0.8675;
  xx[185] = 8.399999999999999e-3;
  xx[186] = - 1.9068;
  xx[187] = 3.0;
  xx[188] = - 0.3383;
  xx[189] = - 0.4141;
  xx[190] = - 7.8e-3;
  xx[191] = 0.9557;
  xx[192] = 0.0159;
  xx[193] = 0.1244;
  xx[194] = 1.016;
  xx[195] = 20.0;
  xx[196] = 0.3059;
  xx[197] = - 0.0433;
  xx[198] = 0.6269;
  xx[199] = 0.1422;
  xx[200] = 0.3477;
  xx[201] = xx[125];
  xx[202] = xx[125];
  xx[203] = xx[125];
  xx[204] = xx[125];
  xx[205] = 1.5015;
  xx[206] = 0.8401;
  xx[207] = - 0.0563;
  xx[208] = xx[125];
  xx[209] = - 0.5276;
  xx[210] = 1.545;
  xx[211] = 0.0983;
  xx[212] = - 6.546;
  xx[213] = xx[125];
  xx[214] = - 26.8535;
  xx[215] = 1.676;
  xx[216] = 0.3699;
  xx[217] = 1.4902;
  xx[218] = xx[125];
  xx[219] = - 0.9004;
  xx[220] = - 0.2328;
  xx[221] = - 5.5e-3;
  xx[222] = xx[125];
  xx[223] = - 0.0282;
  xx[224] = 0.0145;
  xx[225] = - 0.1621;
  xx[226] = - 0.4908;
  xx[227] = 12.321;
  xx[228] = 4.305;
  xx[229] = xx[125];
  xx[230] = xx[125];
  xx[231] = 1.0321;
  xx[232] = - 0.1211;
  xx[233] = 0.131;
  xx[234] = 0.0116;
  xx[235] = xx[125];
  xx[236] = - 6.5e-3;
  xx[237] = - 1.1e-3;
  xx[238] = 0.1017;
  xx[239] = 81.1678;
  xx[240] = 2.2;
  xx[241] = 15.024;
  xx[242] = - 0.6249;
  xx[243] = - 0.0654;
  xx[244] = - 0.1668;
  xx[245] = 0.2834;
  xx[246] = xx[125];
  xx[247] = 8.200000000000001e-3;
  xx[248] = xx[125];
  xx[249] = 1.4e-3;
  xx[250] = xx[127];
  xx[251] = xx[125];
  xx[252] = xx[125];
  xx[253] = 1.0801;
  xx[254] = - 0.502;
  xx[255] = 9.167;
  xx[256] = - 0.534;
  xx[257] = - 2.5047;
  xx[258] = 0.0759;
  xx[259] = - 0.0605;
  xx[260] = 36.2424;
  xx[261] = - 3.3828;
  xx[262] = 1.083;
  xx[263] = 0.1451;
  xx[264] = 0.026;
  xx[265] = 0.2787;
  xx[266] = xx[125];
  xx[267] = - 4.7e-3;
  xx[268] = xx[126];
  xx[269] = - 0.1104;
  xx[270] = 8.1e-3;
  xx[271] = xx[125];
  xx[272] = xx[125];
  xx[273] = - 4.2987;
  xx[274] = 3.2383;
  xx[275] = 0.4625;
  xx[276] = - 0.1911;
  xx[277] = - 0.5441;
  xx[278] = - 4.3e-3;
  xx[279] = - 0.0272;
  xx[280] = 0.2379;
  xx[281] = - 0.2069;
  xx[282] = 9.2e-3;
  xx[283] = 0.0385;
  xx[284] = xx[125];
  xx[285] = xx[125];
  xx[286] = - 0.4402;
  xx[287] = xx[125];
  xx[288] = xx[125];
  xx[289] = xx[125];
  xx[290] = xx[125];
  xx[291] = xx[125];
  xx[292] = xx[125];
  xx[293] = xx[125];
  xx[294] = xx[125];
  xx[295] = xx[125];
  xx[296] = xx[125];
  xx[297] = xx[125];
  xx[298] = xx[125];
  xx[299] = xx[125];
  xx[300] = xx[125];
  xx[301] = xx[125];
  xx[302] = xx[125];
  xx[303] = xx[125];
  xx[304] = xx[125];
  xx[305] = xx[125];
  xx[306] = xx[125];
  xx[103] = sm_core_compiler_computeMagicFormulaTireFreeRadius(
    sqrt(xx[91] * xx[91] + xx[101] * xx[101] + xx[102] * xx[102]) + sqrt(xx[123]
    * xx[123] + xx[118] * xx[118] + xx[92] * xx[92]), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128));
  bb[0] = sm_core_compiler_computeProximityInfoPlaneTire(
    sm_vehicle_2axle_heave_roll_836bb176_1_geometry_1(NULL), (pm_math_Transform3
    *)(xx + 48), (pm_math_Transform3 *)(xx + 107), xx[103], xx + 120,
    (pm_math_Vector3 *)(xx + 307), (pm_math_Vector3 *)(xx + 310));
  xx[48] = xx[33];
  xx[49] = xx[45];
  xx[50] = xx[46];
  xx[51] = xx[47];
  pm_math_Quaternion_inverseCompose_ra(xx + 74, xx + 48, xx + 107);
  pm_math_Quaternion_xform_ra(xx + 107, xx + 310, xx + 45);
  xx[33] = sqrt(xx[45] * xx[45] + xx[46] * xx[46]);
  xx[48] = xx[33] == 0.0 ? 0.0 : - xx[46] / xx[33];
  xx[49] = xx[33] == 0.0 ? 0.0 : xx[45] / xx[33];
  xx[313] = bb[0] ? xx[48] : xx[93];
  xx[314] = bb[0] ? - (xx[47] * xx[49]) : xx[125];
  xx[315] = bb[0] ? xx[45] : xx[125];
  xx[316] = bb[0] ? xx[49] : xx[125];
  xx[317] = bb[0] ? xx[47] * xx[48] : xx[93];
  xx[318] = bb[0] ? xx[46] : xx[125];
  xx[319] = xx[125];
  xx[320] = bb[0] ? xx[45] * xx[49] - xx[46] * xx[48] : xx[125];
  xx[321] = bb[0] ? xx[47] : xx[93];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 313, xx + 45);
  pm_math_Quaternion_xform_ra(xx + 107, xx + 307, xx + 49);
  xx[52] = input[6] - xx[104];
  xx[53] = input[7] - xx[106];
  xx[54] = input[8] - xx[64];
  pm_math_Quaternion_inverseXform_ra(xx + 74, xx + 52, xx + 111);
  xx[307] = xx[45];
  xx[308] = xx[46];
  xx[309] = xx[47];
  xx[310] = xx[48];
  xx[311] = xx[49] + xx[111];
  xx[312] = xx[50] + xx[112];
  xx[313] = xx[51] + xx[113];
  xx[33] = xx[43] * inputDot[7];
  xx[45] = xx[43] * inputDot[6];
  xx[46] = inputDot[6] + xx[88] * (xx[39] * xx[33] - xx[45] * xx[43]);
  xx[47] = xx[35] * inputDot[8];
  xx[48] = xx[46] * xx[35];
  xx[49] = inputDot[7] - (xx[39] * xx[45] + xx[33] * xx[43]) * xx[88];
  xx[33] = inputDot[8] + xx[88] * (xx[41] * xx[48] - xx[47] * xx[35]);
  xx[39] = xx[33] * xx[44];
  xx[43] = xx[44] * xx[49];
  xx[314] = xx[91];
  xx[315] = xx[101];
  xx[316] = xx[102];
  xx[317] = xx[46] - (xx[41] * xx[47] + xx[48] * xx[35]) * xx[88];
  xx[318] = xx[49] + xx[88] * (xx[34] * xx[39] - xx[43] * xx[44]);
  xx[319] = xx[33] - (xx[34] * xx[43] + xx[39] * xx[44]) * xx[88];
  xx[33] = xx[42] * xx[65];
  pm_math_Vector3_cross_ra(xx + 114, xx + 95, xx + 43);
  pm_math_Vector3_cross_ra(xx + 8, xx + 98, xx + 46);
  xx[49] = xx[46] + xx[11];
  xx[50] = xx[47] + xx[27];
  xx[51] = xx[48] + xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 56, xx + 49, xx + 46);
  xx[49] = xx[43] + xx[46];
  xx[50] = xx[44] + xx[47];
  xx[51] = xx[45] + xx[48] + state[14];
  pm_math_Quaternion_inverseXform_ra(xx + 66, xx + 49, xx + 43);
  xx[34] = xx[83] * xx[119];
  xx[35] = xx[42] * xx[117];
  xx[39] = xx[44] - (xx[34] + xx[35]);
  xx[41] = xx[45] + 0.2903000000000001 * state[16] + xx[83] * xx[65];
  xx[45] = xx[61] * xx[41];
  xx[46] = xx[39] * xx[61];
  xx[47] = xx[33] + xx[43];
  xx[48] = xx[39] + xx[88] * (xx[55] * xx[45] - xx[46] * xx[61]);
  xx[49] = xx[41] - (xx[55] * xx[46] + xx[45] * xx[61]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 79, xx + 47, xx + 50);
  xx[39] = xx[40] * xx[40] * xx[52];
  xx[45] = xx[40] * xx[40] * xx[51];
  xx[95] = xx[123];
  xx[96] = xx[118];
  xx[97] = xx[92];
  xx[98] = xx[50];
  xx[99] = xx[51] - (xx[39] + xx[45]) * xx[88];
  xx[100] = xx[52] + xx[88] * (xx[45] - xx[39]);
  xx[320] = xx[93];
  xx[321] = xx[93];
  xx[322] = xx[93];
  xx[323] = xx[93];
  xx[324] = xx[93];
  xx[325] = xx[93];
  xx[326] = xx[93];
  xx[327] = xx[93];
  xx[328] = xx[93];
  xx[329] = xx[93];
  xx[330] = xx[93];
  xx[331] = xx[93];
  xx[332] = xx[93];
  xx[333] = xx[93];
  xx[334] = xx[93];
  xx[335] = xx[93];
  xx[336] = xx[93];
  xx[337] = xx[93];
  xx[338] = xx[93];
  xx[339] = xx[93];
  xx[340] = xx[93];
  xx[341] = xx[93];
  xx[342] = xx[93];
  xx[343] = xx[93];
  xx[344] = xx[93];
  xx[39] = 0.1;
  xx[45] = 1.0e-7;
  xx[345] = xx[39];
  xx[346] = xx[39];
  xx[347] = xx[39];
  xx[348] = xx[39];
  xx[349] = 5.0;
  xx[350] = xx[127];
  xx[351] = xx[127];
  xx[352] = xx[45];
  xx[353] = xx[45];
  xx[354] = xx[127];
  xx[355] = xx[127];
  xx[356] = xx[39];
  xx[357] = xx[127];
  xx[358] = xx[127];
  xx[359] = xx[127];
  xx[360] = xx[127];
  xx[361] = xx[127];
  xx[362] = xx[127];
  xx[363] = xx[127];
  xx[364] = xx[127];
  xx[365] = xx[127];
  sm_core_compiler_computeMagicFormulaTireWrench(
    bb[0], (const pm_math_Transform3 *)(xx + 307), (const pm_math_Transform3 *)
    (xx + 107), (const pm_math_SpatialVector *)(xx + 314), (const
    pm_math_SpatialVector *)(xx + 95), -1, 0, (const
    sm_core_compiler_MFTP_ScalingCoefficients *)(xx + 320), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128), (const
    sm_core_compiler_MagicFormulaNumericParameters *)(xx + 345),
    (pm_math_Vector3 *)(xx + 45),
    (pm_math_Vector3 *)(xx + 48),
    NULL);
  xx[39] = xx[29] * input[15];
  xx[51] = cos(xx[39]);
  xx[52] = xx[29] * input[16];
  xx[53] = cos(xx[52]);
  xx[54] = xx[29] * input[17];
  xx[55] = cos(xx[54]);
  xx[61] = xx[53] * xx[55];
  xx[64] = sin(xx[52]);
  xx[52] = sin(xx[54]);
  xx[54] = xx[64] * xx[52];
  xx[66] = sin(xx[39]);
  xx[39] = xx[51] * xx[61] + xx[54] * xx[66];
  xx[67] = xx[61] * xx[66] - xx[51] * xx[54];
  xx[54] = xx[53] * xx[52];
  xx[61] = xx[55] * xx[64];
  xx[68] = xx[51] * xx[54] + xx[61] * xx[66];
  xx[69] = xx[51] * xx[61] - xx[54] * xx[66];
  xx[95] = xx[39];
  xx[96] = xx[67];
  xx[97] = xx[68];
  xx[98] = xx[69];
  xx[99] = input[12];
  xx[100] = input[13];
  xx[101] = input[14];
  xx[54] = xx[29] * input[5];
  xx[61] = cos(xx[54]);
  xx[74] = sin(xx[54]);
  xx[79] = xx[61] * xx[70] - xx[71] * xx[74];
  xx[80] = xx[70] * xx[74] + xx[61] * xx[71];
  xx[81] = xx[61] * xx[72] + xx[73] * xx[74];
  xx[82] = xx[61] * xx[73] - xx[72] * xx[74];
  xx[54] = xx[29] * state[19];
  xx[75] = - (xx[40] * cos(xx[54]));
  xx[76] = - (xx[40] * sin(xx[54]));
  xx[106] = xx[75];
  xx[107] = xx[75];
  xx[108] = xx[76];
  xx[109] = xx[76];
  pm_math_Quaternion_compose_ra(xx + 79, xx + 106, xx + 110);
  xx[54] = xx[40] * xx[110];
  xx[75] = xx[40] * xx[111];
  xx[76] = xx[54] + xx[75];
  xx[77] = xx[75] - xx[54];
  xx[54] = xx[40] * xx[112];
  xx[75] = xx[40] * xx[113];
  xx[79] = xx[54] - xx[75];
  xx[80] = xx[75] + xx[54];
  xx[54] = xx[85] - xx[84];
  xx[102] = - xx[78];
  xx[103] = xx[54];
  xx[104] = xx[87];
  pm_math_Vector3_cross_ra(xx + 71, xx + 102, xx + 110);
  xx[71] = (xx[110] - xx[62]) * xx[88] - xx[83] + xx[94];
  xx[62] = (xx[70] * xx[54] + xx[111]) * xx[88] + xx[105];
  xx[54] = xx[88] * (xx[112] + xx[86]) - xx[42] + xx[63];
  xx[307] = xx[76];
  xx[308] = xx[77];
  xx[309] = xx[79];
  xx[310] = xx[80];
  xx[311] = xx[71];
  xx[312] = xx[62];
  xx[313] = xx[54];
  xx[63] = xx[52] * inputDot[16];
  xx[70] = inputDot[15] - xx[88] * xx[55] * xx[63];
  xx[72] = inputDot[16] - xx[88] * xx[63] * xx[52];
  xx[63] = xx[66] * xx[72];
  xx[73] = xx[66] * inputDot[17];
  xx[75] = inputDot[17] + xx[88] * (xx[51] * xx[63] - xx[73] * xx[66]);
  xx[78] = xx[72] - (xx[51] * xx[73] + xx[63] * xx[66]) * xx[88];
  xx[63] = xx[119] * xx[74];
  xx[72] = xx[74] * xx[65];
  xx[84] = xx[117] + inputDot[5];
  xx[85] = xx[65] + xx[88] * (xx[61] * xx[63] - xx[72] * xx[74]);
  xx[86] = xx[119] - (xx[61] * xx[72] + xx[63] * xx[74]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 106, xx + 84, xx + 102);
  xx[63] = xx[103] + state[20];
  xx[65] = xx[40] * xx[40] * xx[104];
  xx[72] = xx[40] * xx[63] * xx[40];
  xx[73] = xx[63] - (xx[65] + xx[72]) * xx[88];
  xx[63] = xx[104] + xx[88] * (xx[72] - xx[65]);
  xx[65] = sm_core_compiler_computeMagicFormulaTireFreeRadius(
    sqrt(xx[70] * xx[70] + xx[75] * xx[75] + xx[78] * xx[78]) + sqrt(xx[102] *
    xx[102] + xx[73] * xx[73] + xx[63] * xx[63]), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128));
  bb[0] = sm_core_compiler_computeProximityInfoPlaneTire(
    sm_vehicle_2axle_heave_roll_836bb176_1_geometry_1(NULL), (pm_math_Transform3
    *)(xx + 95), (pm_math_Transform3 *)(xx + 307), xx[65], xx + 72,
    (pm_math_Vector3 *)(xx + 84), (pm_math_Vector3 *)(xx + 103));
  xx[94] = xx[76];
  xx[95] = xx[77];
  xx[96] = xx[79];
  xx[97] = xx[80];
  xx[79] = xx[39];
  xx[80] = xx[67];
  xx[81] = xx[68];
  xx[82] = xx[69];
  pm_math_Quaternion_inverseCompose_ra(xx + 94, xx + 79, xx + 98);
  pm_math_Quaternion_xform_ra(xx + 98, xx + 103, xx + 67);
  xx[39] = sqrt(xx[67] * xx[67] + xx[68] * xx[68]);
  xx[65] = xx[39] == 0.0 ? 0.0 : - xx[68] / xx[39];
  xx[76] = xx[39] == 0.0 ? 0.0 : xx[67] / xx[39];
  xx[307] = bb[0] ? xx[65] : xx[93];
  xx[308] = bb[0] ? - (xx[69] * xx[76]) : xx[125];
  xx[309] = bb[0] ? xx[67] : xx[125];
  xx[310] = bb[0] ? xx[76] : xx[125];
  xx[311] = bb[0] ? xx[69] * xx[65] : xx[93];
  xx[312] = bb[0] ? xx[68] : xx[125];
  xx[313] = xx[125];
  xx[314] = bb[0] ? xx[67] * xx[76] - xx[68] * xx[65] : xx[125];
  xx[315] = bb[0] ? xx[69] : xx[93];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 307, xx + 79);
  pm_math_Quaternion_xform_ra(xx + 98, xx + 84, xx + 67);
  xx[84] = input[12] - xx[71];
  xx[85] = input[13] - xx[62];
  xx[86] = input[14] - xx[54];
  pm_math_Quaternion_inverseXform_ra(xx + 94, xx + 84, xx + 103);
  xx[117] = xx[79];
  xx[118] = xx[80];
  xx[119] = xx[81];
  xx[120] = xx[82];
  xx[121] = xx[67] + xx[103];
  xx[122] = xx[68] + xx[104];
  xx[123] = xx[69] + xx[105];
  xx[307] = xx[98];
  xx[308] = xx[99];
  xx[309] = xx[100];
  xx[310] = xx[101];
  xx[311] = xx[103];
  xx[312] = xx[104];
  xx[313] = xx[105];
  xx[39] = xx[64] * inputDot[13];
  xx[54] = xx[64] * inputDot[12];
  xx[62] = inputDot[12] + xx[88] * (xx[53] * xx[39] - xx[54] * xx[64]);
  xx[65] = xx[52] * inputDot[14];
  xx[67] = xx[62] * xx[52];
  xx[68] = inputDot[13] - (xx[53] * xx[54] + xx[39] * xx[64]) * xx[88];
  xx[39] = inputDot[14] + xx[88] * (xx[55] * xx[67] - xx[65] * xx[52]);
  xx[53] = xx[39] * xx[66];
  xx[54] = xx[66] * xx[68];
  xx[94] = xx[70];
  xx[95] = xx[75];
  xx[96] = xx[78];
  xx[97] = xx[62] - (xx[55] * xx[65] + xx[67] * xx[52]) * xx[88];
  xx[98] = xx[68] + xx[88] * (xx[51] * xx[53] - xx[54] * xx[66]);
  xx[99] = xx[39] - (xx[51] * xx[54] + xx[53] * xx[66]) * xx[88];
  xx[39] = xx[35] - xx[34] + xx[44];
  xx[34] = xx[74] * xx[41];
  xx[35] = xx[39] * xx[74];
  xx[51] = xx[43] - xx[33];
  xx[52] = xx[39] + xx[88] * (xx[61] * xx[34] - xx[35] * xx[74]);
  xx[53] = xx[41] - (xx[61] * xx[35] + xx[34] * xx[74]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 106, xx + 51, xx + 33);
  xx[39] = xx[40] * xx[40] * xx[35];
  xx[41] = xx[40] * xx[40] * xx[34];
  xx[64] = xx[102];
  xx[65] = xx[73];
  xx[66] = xx[63];
  xx[67] = xx[33];
  xx[68] = xx[34] - (xx[39] + xx[41]) * xx[88];
  xx[69] = xx[35] + xx[88] * (xx[41] - xx[39]);
  sm_core_compiler_computeMagicFormulaTireWrench(
    bb[0], (const pm_math_Transform3 *)(xx + 117), (const pm_math_Transform3 *)
    (xx + 307), (const pm_math_SpatialVector *)(xx + 94), (const
    pm_math_SpatialVector *)(xx + 64), 1, 0, (const
    sm_core_compiler_MFTP_ScalingCoefficients *)(xx + 320), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128), (const
    sm_core_compiler_MagicFormulaNumericParameters *)(xx + 345),
    (pm_math_Vector3 *)(xx + 33),
    (pm_math_Vector3 *)(xx + 51),
    NULL);
  xx[39] = xx[29] * input[21];
  xx[41] = cos(xx[39]);
  xx[43] = xx[29] * input[22];
  xx[44] = cos(xx[43]);
  xx[54] = xx[29] * input[23];
  xx[55] = cos(xx[54]);
  xx[61] = xx[44] * xx[55];
  xx[62] = sin(xx[43]);
  xx[43] = sin(xx[54]);
  xx[54] = xx[62] * xx[43];
  xx[63] = sin(xx[39]);
  xx[39] = xx[41] * xx[61] + xx[54] * xx[63];
  xx[64] = xx[61] * xx[63] - xx[41] * xx[54];
  xx[54] = xx[44] * xx[43];
  xx[61] = xx[55] * xx[62];
  xx[65] = xx[41] * xx[54] + xx[61] * xx[63];
  xx[66] = xx[41] * xx[61] - xx[54] * xx[63];
  xx[67] = xx[39];
  xx[68] = xx[64];
  xx[69] = xx[65];
  xx[70] = xx[66];
  xx[71] = input[18];
  xx[72] = input[19];
  xx[73] = input[20];
  xx[54] = xx[29] * state[23];
  xx[61] = xx[29] * sin(xx[54]);
  xx[74] = xx[29] * cos(xx[54]);
  xx[54] = xx[61] - xx[74];
  xx[75] = xx[74] + xx[61];
  xx[76] = xx[74] + xx[61];
  xx[77] = xx[74] - xx[61];
  xx[78] = xx[54];
  xx[79] = xx[75];
  xx[80] = xx[76];
  xx[81] = xx[77];
  pm_math_Quaternion_compose_ra(xx + 12, xx + 78, xx + 84);
  xx[61] = xx[29] * state[25];
  xx[74] = - (xx[40] * cos(xx[61]));
  xx[82] = - (xx[40] * sin(xx[61]));
  xx[94] = xx[74];
  xx[95] = xx[74];
  xx[96] = xx[82];
  xx[97] = xx[82];
  pm_math_Quaternion_compose_ra(xx + 84, xx + 94, xx + 98);
  xx[61] = xx[40] * xx[98];
  xx[74] = xx[40] * xx[99];
  xx[82] = xx[61] + xx[74];
  xx[91] = xx[74] - xx[61];
  xx[61] = xx[40] * xx[100];
  xx[74] = xx[40] * xx[101];
  xx[92] = xx[61] - xx[74];
  xx[98] = xx[74] + xx[61];
  xx[61] = xx[42] * xx[84] * xx[86];
  xx[74] = xx[42] * xx[86];
  xx[99] = xx[83] * xx[87];
  xx[100] = xx[42] * xx[85];
  xx[101] = xx[99] + xx[100];
  xx[102] = xx[83] * xx[86];
  xx[103] = xx[74];
  xx[104] = - xx[101];
  xx[105] = xx[102];
  pm_math_Vector3_cross_ra(xx + 85, xx + 103, xx + 106);
  xx[103] = 0.2403;
  xx[104] = xx[103] * xx[76];
  xx[105] = xx[103] * xx[77];
  xx[109] = - ((xx[104] * xx[76] + xx[105] * xx[77]) * xx[88] - xx[103]);
  xx[110] = (xx[54] * xx[105] + xx[75] * xx[104]) * xx[88];
  xx[111] = - (0.05000000000000002 + xx[88] * (xx[54] * xx[104] - xx[75] * xx
    [105]));
  pm_math_Quaternion_xform_ra(xx + 12, xx + 109, xx + 75);
  xx[12] = - (1.500013141795951 + xx[60] * state[21]);
  xx[13] = 2.961066861578549e-5 - xx[89] * state[21];
  xx[14] = xx[90] * state[21] - 0.4500081720166809;
  pm_math_Quaternion_xform_ra(xx + 4, xx + 12, xx + 103);
  xx[15] = xx[75] + xx[103] + xx[0];
  xx[54] = (xx[61] + xx[106]) * xx[88] - xx[83] + xx[15];
  xx[60] = xx[76] + xx[104] + xx[3];
  xx[89] = (xx[107] - xx[84] * xx[101]) * xx[88] + xx[60];
  xx[90] = xx[84] * xx[102];
  xx[75] = xx[77] + xx[105] + xx[28];
  xx[76] = xx[42] + xx[88] * (xx[108] + xx[90]) + xx[75];
  xx[117] = xx[82];
  xx[118] = xx[91];
  xx[119] = xx[92];
  xx[120] = xx[98];
  xx[121] = xx[54];
  xx[122] = xx[89];
  xx[123] = xx[76];
  xx[77] = xx[43] * inputDot[22];
  xx[101] = inputDot[21] - xx[88] * xx[55] * xx[77];
  xx[103] = inputDot[22] - xx[88] * xx[77] * xx[43];
  xx[77] = xx[63] * xx[103];
  xx[104] = xx[63] * inputDot[23];
  xx[105] = inputDot[23] + xx[88] * (xx[41] * xx[77] - xx[104] * xx[63]);
  xx[106] = xx[103] - (xx[41] * xx[104] + xx[77] * xx[63]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 114, xx + 307);
  xx[77] = xx[308] - state[24];
  xx[310] = xx[307];
  xx[311] = xx[77];
  xx[312] = xx[309];
  pm_math_Quaternion_inverseXform_ra(xx + 94, xx + 310, xx + 313);
  xx[103] = xx[314] + state[26];
  xx[104] = xx[40] * xx[40] * xx[315];
  xx[107] = xx[40] * xx[103] * xx[40];
  xx[108] = xx[103] - (xx[104] + xx[107]) * xx[88];
  xx[103] = xx[315] + xx[88] * (xx[107] - xx[104]);
  xx[104] = sm_core_compiler_computeMagicFormulaTireFreeRadius(
    sqrt(xx[101] * xx[101] + xx[105] * xx[105] + xx[106] * xx[106]) + sqrt(xx
    [313] * xx[313] + xx[108] * xx[108] + xx[103] * xx[103]), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128));
  bb[0] = sm_core_compiler_computeProximityInfoPlaneTire(
    sm_vehicle_2axle_heave_roll_836bb176_1_geometry_1(NULL), (pm_math_Transform3
    *)(xx + 67), (pm_math_Transform3 *)(xx + 117), xx[104], xx + 107,
    (pm_math_Vector3 *)(xx + 314), (pm_math_Vector3 *)(xx + 317));
  xx[67] = xx[82];
  xx[68] = xx[91];
  xx[69] = xx[92];
  xx[70] = xx[98];
  xx[117] = xx[39];
  xx[118] = xx[64];
  xx[119] = xx[65];
  xx[120] = xx[66];
  pm_math_Quaternion_inverseCompose_ra(xx + 67, xx + 117, xx + 121);
  pm_math_Quaternion_xform_ra(xx + 121, xx + 317, xx + 64);
  xx[39] = sqrt(xx[64] * xx[64] + xx[65] * xx[65]);
  xx[71] = xx[39] == 0.0 ? 0.0 : - xx[65] / xx[39];
  xx[72] = xx[39] == 0.0 ? 0.0 : xx[64] / xx[39];
  xx[366] = bb[0] ? xx[71] : xx[93];
  xx[367] = bb[0] ? - (xx[66] * xx[72]) : xx[125];
  xx[368] = bb[0] ? xx[64] : xx[125];
  xx[369] = bb[0] ? xx[72] : xx[125];
  xx[370] = bb[0] ? xx[66] * xx[71] : xx[93];
  xx[371] = bb[0] ? xx[65] : xx[125];
  xx[372] = xx[125];
  xx[373] = bb[0] ? xx[64] * xx[72] - xx[65] * xx[71] : xx[125];
  xx[374] = bb[0] ? xx[66] : xx[93];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 366, xx + 117);
  pm_math_Quaternion_xform_ra(xx + 121, xx + 314, xx + 64);
  xx[71] = input[18] - xx[54];
  xx[72] = input[19] - xx[89];
  xx[73] = input[20] - xx[76];
  pm_math_Quaternion_inverseXform_ra(xx + 67, xx + 71, xx + 314);
  xx[67] = xx[117];
  xx[68] = xx[118];
  xx[69] = xx[119];
  xx[70] = xx[120];
  xx[71] = xx[64] + xx[314];
  xx[72] = xx[65] + xx[315];
  xx[73] = xx[66] + xx[316];
  xx[366] = xx[121];
  xx[367] = xx[122];
  xx[368] = xx[123];
  xx[369] = xx[124];
  xx[370] = xx[314];
  xx[371] = xx[315];
  xx[372] = xx[316];
  xx[39] = xx[62] * inputDot[19];
  xx[54] = xx[62] * inputDot[18];
  xx[64] = inputDot[18] + xx[88] * (xx[44] * xx[39] - xx[54] * xx[62]);
  xx[65] = xx[43] * inputDot[20];
  xx[66] = xx[64] * xx[43];
  xx[76] = inputDot[19] - (xx[44] * xx[54] + xx[39] * xx[62]) * xx[88];
  xx[39] = inputDot[20] + xx[88] * (xx[55] * xx[66] - xx[65] * xx[43]);
  xx[44] = xx[39] * xx[63];
  xx[54] = xx[63] * xx[76];
  xx[117] = xx[101];
  xx[118] = xx[105];
  xx[119] = xx[106];
  xx[120] = xx[64] - (xx[55] * xx[65] + xx[66] * xx[43]) * xx[88];
  xx[121] = xx[76] + xx[88] * (xx[41] * xx[44] - xx[54] * xx[63]);
  xx[122] = xx[39] - (xx[41] * xx[54] + xx[44] * xx[63]) * xx[88];
  xx[39] = xx[42] * xx[77];
  pm_math_Vector3_cross_ra(xx + 114, xx + 109, xx + 62);
  pm_math_Vector3_cross_ra(xx + 8, xx + 12, xx + 104);
  xx[12] = xx[104] + xx[11];
  xx[13] = xx[105] + xx[27];
  xx[14] = xx[106] + xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 56, xx + 12, xx + 104);
  xx[12] = xx[62] + xx[104];
  xx[13] = xx[63] + xx[105];
  xx[14] = xx[64] + xx[106] + state[22];
  pm_math_Quaternion_inverseXform_ra(xx + 78, xx + 12, xx + 54);
  xx[12] = xx[83] * xx[309];
  xx[13] = xx[42] * xx[307];
  xx[14] = xx[56] + 0.2403 * state[24] + xx[83] * xx[77];
  xx[56] = xx[39] + xx[54];
  xx[57] = xx[55] - (xx[12] + xx[13]);
  xx[58] = xx[14];
  pm_math_Quaternion_inverseXform_ra(xx + 94, xx + 56, xx + 62);
  xx[41] = xx[40] * xx[40] * xx[64];
  xx[43] = xx[40] * xx[40] * xx[63];
  xx[76] = xx[313];
  xx[77] = xx[108];
  xx[78] = xx[103];
  xx[79] = xx[62];
  xx[80] = xx[63] - (xx[41] + xx[43]) * xx[88];
  xx[81] = xx[64] + xx[88] * (xx[43] - xx[41]);
  sm_core_compiler_computeMagicFormulaTireWrench(
    bb[0], (const pm_math_Transform3 *)(xx + 67), (const pm_math_Transform3 *)
    (xx + 366), (const pm_math_SpatialVector *)(xx + 117), (const
    pm_math_SpatialVector *)(xx + 76), -1, 0, (const
    sm_core_compiler_MFTP_ScalingCoefficients *)(xx + 320), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128), (const
    sm_core_compiler_MagicFormulaNumericParameters *)(xx + 345),
    (pm_math_Vector3 *)(xx + 56),
    (pm_math_Vector3 *)(xx + 62),
    NULL);
  xx[41] = xx[29] * input[27];
  xx[43] = cos(xx[41]);
  xx[44] = xx[29] * input[28];
  xx[59] = cos(xx[44]);
  xx[65] = xx[29] * input[29];
  xx[66] = cos(xx[65]);
  xx[67] = xx[59] * xx[66];
  xx[68] = sin(xx[44]);
  xx[44] = sin(xx[65]);
  xx[65] = xx[68] * xx[44];
  xx[69] = sin(xx[41]);
  xx[41] = xx[43] * xx[67] + xx[65] * xx[69];
  xx[70] = xx[67] * xx[69] - xx[43] * xx[65];
  xx[65] = xx[59] * xx[44];
  xx[67] = xx[66] * xx[68];
  xx[71] = xx[43] * xx[65] + xx[67] * xx[69];
  xx[72] = xx[43] * xx[67] - xx[65] * xx[69];
  xx[76] = xx[41];
  xx[77] = xx[70];
  xx[78] = xx[71];
  xx[79] = xx[72];
  xx[80] = input[24];
  xx[81] = input[25];
  xx[82] = input[26];
  xx[65] = xx[29] * state[27];
  xx[29] = - (xx[40] * cos(xx[65]));
  xx[67] = - (xx[40] * sin(xx[65]));
  xx[94] = xx[29];
  xx[95] = xx[29];
  xx[96] = xx[67];
  xx[97] = xx[67];
  pm_math_Quaternion_compose_ra(xx + 84, xx + 94, xx + 103);
  xx[29] = xx[40] * xx[103];
  xx[65] = xx[40] * xx[104];
  xx[67] = xx[29] + xx[65];
  xx[73] = xx[65] - xx[29];
  xx[29] = xx[40] * xx[105];
  xx[65] = xx[40] * xx[106];
  xx[89] = xx[29] - xx[65];
  xx[91] = xx[65] + xx[29];
  xx[29] = xx[100] - xx[99];
  xx[98] = - xx[74];
  xx[99] = xx[29];
  xx[100] = xx[102];
  pm_math_Vector3_cross_ra(xx + 85, xx + 98, xx + 101);
  xx[65] = (xx[101] - xx[61]) * xx[88] - xx[83] + xx[15];
  xx[15] = (xx[84] * xx[29] + xx[102]) * xx[88] + xx[60];
  xx[29] = xx[88] * (xx[103] + xx[90]) - xx[42] + xx[75];
  xx[98] = xx[67];
  xx[99] = xx[73];
  xx[100] = xx[89];
  xx[101] = xx[91];
  xx[102] = xx[65];
  xx[103] = xx[15];
  xx[104] = xx[29];
  xx[42] = xx[44] * inputDot[28];
  xx[60] = inputDot[27] - xx[88] * xx[66] * xx[42];
  xx[61] = inputDot[28] - xx[88] * xx[42] * xx[44];
  xx[42] = xx[69] * xx[61];
  xx[74] = xx[69] * inputDot[29];
  xx[75] = inputDot[29] + xx[88] * (xx[43] * xx[42] - xx[74] * xx[69]);
  xx[83] = xx[61] - (xx[43] * xx[74] + xx[42] * xx[69]) * xx[88];
  pm_math_Quaternion_inverseXform_ra(xx + 94, xx + 310, xx + 84);
  xx[42] = xx[85] + state[28];
  xx[61] = xx[40] * xx[40] * xx[86];
  xx[74] = xx[40] * xx[42] * xx[40];
  xx[85] = xx[42] - (xx[61] + xx[74]) * xx[88];
  xx[42] = xx[86] + xx[88] * (xx[74] - xx[61]);
  xx[61] = sm_core_compiler_computeMagicFormulaTireFreeRadius(
    sqrt(xx[60] * xx[60] + xx[75] * xx[75] + xx[83] * xx[83]) + sqrt(xx[84] *
    xx[84] + xx[85] * xx[85] + xx[42] * xx[42]), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128));
  bb[0] = sm_core_compiler_computeProximityInfoPlaneTire(
    sm_vehicle_2axle_heave_roll_836bb176_1_geometry_1(NULL), (pm_math_Transform3
    *)(xx + 76), (pm_math_Transform3 *)(xx + 98), xx[61], xx + 74,
    (pm_math_Vector3 *)(xx + 105), (pm_math_Vector3 *)(xx + 108));
  xx[76] = xx[67];
  xx[77] = xx[73];
  xx[78] = xx[89];
  xx[79] = xx[91];
  xx[89] = xx[41];
  xx[90] = xx[70];
  xx[91] = xx[71];
  xx[92] = xx[72];
  pm_math_Quaternion_inverseCompose_ra(xx + 76, xx + 89, xx + 70);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 108, xx + 80);
  xx[41] = sqrt(xx[80] * xx[80] + xx[81] * xx[81]);
  xx[61] = xx[41] == 0.0 ? 0.0 : - xx[81] / xx[41];
  xx[67] = xx[41] == 0.0 ? 0.0 : xx[80] / xx[41];
  xx[111] = bb[0] ? xx[61] : xx[93];
  xx[112] = bb[0] ? - (xx[82] * xx[67]) : xx[125];
  xx[113] = bb[0] ? xx[80] : xx[125];
  xx[114] = bb[0] ? xx[67] : xx[125];
  xx[115] = bb[0] ? xx[82] * xx[61] : xx[93];
  xx[116] = bb[0] ? xx[81] : xx[125];
  xx[117] = xx[125];
  xx[118] = bb[0] ? xx[80] * xx[67] - xx[81] * xx[61] : xx[125];
  xx[119] = bb[0] ? xx[82] : xx[93];
  pm_math_Quaternion_Matrix3x3Ctor_ra(xx + 111, xx + 89);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 105, xx + 80);
  xx[98] = input[24] - xx[65];
  xx[99] = input[25] - xx[15];
  xx[100] = input[26] - xx[29];
  pm_math_Quaternion_inverseXform_ra(xx + 76, xx + 98, xx + 101);
  xx[104] = xx[89];
  xx[105] = xx[90];
  xx[106] = xx[91];
  xx[107] = xx[92];
  xx[108] = xx[80] + xx[101];
  xx[109] = xx[81] + xx[102];
  xx[110] = xx[82] + xx[103];
  xx[76] = xx[70];
  xx[77] = xx[71];
  xx[78] = xx[72];
  xx[79] = xx[73];
  xx[80] = xx[101];
  xx[81] = xx[102];
  xx[82] = xx[103];
  xx[15] = xx[68] * inputDot[25];
  xx[29] = xx[68] * inputDot[24];
  xx[41] = inputDot[24] + xx[88] * (xx[59] * xx[15] - xx[29] * xx[68]);
  xx[61] = xx[44] * inputDot[26];
  xx[65] = xx[41] * xx[44];
  xx[67] = inputDot[25] - (xx[59] * xx[29] + xx[15] * xx[68]) * xx[88];
  xx[15] = inputDot[26] + xx[88] * (xx[66] * xx[65] - xx[61] * xx[44]);
  xx[29] = xx[15] * xx[69];
  xx[59] = xx[69] * xx[67];
  xx[98] = xx[60];
  xx[99] = xx[75];
  xx[100] = xx[83];
  xx[101] = xx[41] - (xx[66] * xx[61] + xx[65] * xx[44]) * xx[88];
  xx[102] = xx[67] + xx[88] * (xx[43] * xx[29] - xx[59] * xx[69]);
  xx[103] = xx[15] - (xx[43] * xx[59] + xx[29] * xx[69]) * xx[88];
  xx[59] = xx[54] - xx[39];
  xx[60] = xx[13] - xx[12] + xx[55];
  xx[61] = xx[14];
  pm_math_Quaternion_inverseXform_ra(xx + 94, xx + 59, xx + 12);
  xx[15] = xx[40] * xx[40] * xx[14];
  xx[29] = xx[40] * xx[40] * xx[13];
  xx[65] = xx[84];
  xx[66] = xx[85];
  xx[67] = xx[42];
  xx[68] = xx[12];
  xx[69] = xx[13] - (xx[15] + xx[29]) * xx[88];
  xx[70] = xx[14] + xx[88] * (xx[29] - xx[15]);
  sm_core_compiler_computeMagicFormulaTireWrench(
    bb[0], (const pm_math_Transform3 *)(xx + 104), (const pm_math_Transform3 *)
    (xx + 76), (const pm_math_SpatialVector *)(xx + 98), (const
    pm_math_SpatialVector *)(xx + 65), 1, 0, (const
    sm_core_compiler_MFTP_ScalingCoefficients *)(xx + 320), (const
    sm_core_compiler_MagicFormulaTireParameters *)(xx + 128), (const
    sm_core_compiler_MagicFormulaNumericParameters *)(xx + 345),
    (pm_math_Vector3 *)(xx + 12),
    (pm_math_Vector3 *)(xx + 39),
    NULL);
  pm_math_Quaternion_compose_ra(xx + 4, xx + 16, xx + 65);
  xx[15] = xx[65] * xx[65];
  xx[29] = (xx[15] + xx[66] * xx[66]) * xx[88] - xx[93];
  xx[42] = xx[66] * xx[67];
  xx[43] = xx[65] * xx[68];
  xx[44] = (xx[42] + xx[43]) * xx[88];
  xx[54] = xx[66] * xx[68];
  xx[55] = xx[65] * xx[67];
  xx[59] = xx[88] * (xx[54] - xx[55]);
  xx[60] = xx[88] * (xx[42] - xx[43]);
  xx[42] = (xx[15] + xx[67] * xx[67]) * xx[88] - xx[93];
  xx[43] = xx[67] * xx[68];
  xx[61] = xx[65] * xx[66];
  xx[69] = (xx[43] + xx[61]) * xx[88];
  xx[70] = (xx[54] + xx[55]) * xx[88];
  xx[54] = xx[88] * (xx[43] - xx[61]);
  xx[43] = (xx[15] + xx[68] * xx[68]) * xx[88] - xx[93];
  xx[71] = - xx[65];
  xx[72] = - xx[66];
  xx[73] = - xx[67];
  xx[74] = - xx[68];
  xx[65] = - 1.718750981636525e-5;
  xx[66] = - 6.156239914624473e-6;
  xx[67] = 5.312054581417384e-6;
  pm_math_Quaternion_xform_ra(xx + 4, xx + 65, xx + 75);
  xx[4] = xx[75] + xx[0];
  xx[5] = xx[76] + xx[3];
  xx[6] = xx[77] + xx[28];
  pm_math_Quaternion_inverseXform_ra(xx + 71, xx + 4, xx + 75);
  pm_math_Vector3_cross_ra(xx + 8, xx + 65, xx + 4);
  xx[7] = xx[4] + xx[11];
  xx[8] = xx[5] + xx[27];
  xx[9] = xx[6] + xx[1];
  pm_math_Quaternion_inverseXform_ra(xx + 16, xx + 7, xx + 4);
  output[0] = xx[30] + xx[0];
  output[1] = xx[24];
  output[3] = xx[31] + xx[3];
  output[4] = xx[25];
  output[6] = xx[32] + xx[28];
  output[7] = xx[26];
  output[9] = xx[20];
  output[10] = xx[21];
  output[11] = xx[22];
  output[12] = xx[23];
  output[13] = xx[36];
  output[14] = xx[37];
  output[15] = xx[38];
  output[16] = state[18];
  output[17] = state[20];
  output[18] = state[26];
  output[19] = state[28];
  output[20] = xx[45];
  output[21] = xx[46];
  output[22] = xx[47];
  output[23] = xx[48];
  output[24] = xx[49];
  output[25] = xx[50];
  output[26] = xx[33];
  output[27] = xx[34];
  output[28] = xx[35];
  output[29] = xx[51];
  output[30] = xx[52];
  output[31] = xx[53];
  output[32] = xx[56];
  output[33] = xx[57];
  output[34] = xx[58];
  output[35] = xx[62];
  output[36] = xx[63];
  output[37] = xx[64];
  output[38] = xx[12];
  output[39] = xx[13];
  output[40] = xx[14];
  output[41] = xx[39];
  output[42] = xx[40];
  output[43] = xx[41];
  output[44] = xx[29];
  output[45] = xx[44];
  output[46] = xx[59];
  output[47] = xx[60];
  output[48] = xx[42];
  output[49] = xx[69];
  output[50] = xx[70];
  output[51] = xx[54];
  output[52] = xx[43];
  output[53] = xx[75];
  output[54] = xx[76];
  output[55] = xx[77];
  output[56] = xx[4];
  output[57] = xx[5];
  output[58] = xx[6];
  output[62] = xx[29];
  output[63] = xx[44];
  output[64] = xx[59];
  output[65] = xx[60];
  output[66] = xx[42];
  output[67] = xx[69];
  output[68] = xx[70];
  output[69] = xx[54];
  output[70] = xx[43];
  return NULL;
}
