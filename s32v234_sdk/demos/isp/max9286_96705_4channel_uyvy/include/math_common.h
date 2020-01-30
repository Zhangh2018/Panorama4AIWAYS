/* 
 * Basic matrix operation functions
 * add by Shuo.Yan on 2017/10/31 InteSight
 */
#ifndef __MATH_COMMON_H_
#define __MATH_COMMON_H_
#include "data_type.h"

#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

#ifndef SIGN
#define SIGN(a)  ((a) >= (0.0) ? (1) : (-1))
#endif

#ifndef FABS
#define FABS(x) (((x)>=0.0)? (x) : -(x))
#endif // !FABS




#ifdef __cplusplus
extern "C" {
#endif
/******************Matrix Operation*******************/
//determinant of a 3by3 matrix
Float32_t Mat_Determinant_3by3(IN Float32_t A[9]);
Float64_t Mat_Determinant_3by3_DB(IN Float64_t A[9]);

//inverse of 3*3 matrix
unsigned char Inv_Mat_3by3(OUT Float32_t pOut[9], IN Float32_t pIn[9]);
unsigned char Inv_Mat_3by3_DB(OUT Float64_t pOut[9], IN Float64_t pIn[9]);

//transpose Matrix A[m][n] --> AT [n][m]
void Transpose_Mat(OUT Float32_t At[9], IN Float32_t A[9], 
	IN Int32_t row, IN Int32_t col);
void Transpose_Mat_DB(OUT Float64_t At[9], IN Float64_t A[9], 
	IN Int32_t row, IN Int32_t col);

//in-place transpose A by A'
void Transpose_Mat_3by3(OUT Float32_t At[9], IN Float32_t A[9]);
void Transpose_Mat_3by3_DB(OUT Float64_t At[9], IN Float64_t A[9]);

void Transpose_Mat_3by3_InPlace(OUT IN Float32_t A[9]);
void Transpose_Mat_3by3_InPlace_DB(OUT IN Float64_t A[9]);

//mat multi C=A*B, all are 3*3 matrix
void Mat_Mul_3by3(OUT Float32_t Cout[9], 
	IN Float32_t Ain[9], IN Float32_t Bin[9]);
void Mat_Mul_3by3_DB(OUT Float64_t Cout[9], 
	IN Float64_t Ain[9], IN Float64_t Bin[9]);

//mat multi vector: C=A*B, C_31, A_33, B_31
void Mat_Mul_Vec_3by3(OUT Float32_t Cout[3], 
	IN Float32_t Ain[9], IN Float32_t Bin[3]);
void Mat_Mul_Vec_3by3_DB(OUT Float64_t Cout[3], 
	IN Float64_t Ain[9], IN Float64_t Bin[3]);

//R, T will be in-place updated to R', and -R'T
void Inverse_RT_InPlace(OUT IN Float32_t R[9], OUT IN Float32_t T[3]);
void Inverse_RT_InPlace_DB(OUT IN Float64_t R[9], OUT IN Float64_t T[3]);

void Inverse_RT(OUT Float32_t Rout[9], OUT Float32_t Tout[3], 
	IN Float32_t Rin[9], IN Float32_t Tin[3]);
void Inverse_RT_DB(OUT Float64_t Rout[9], OUT Float64_t Tout[3], 
	IN Float64_t Rin[9], IN Float64_t Tin[3]);

//convert from angles into rotation matrix, R=Rot(Z)*Rot(Y)*Rot(X), angles in rad
void Cvt_Rotation_To_Angles(IN const Float32_t R[9], OUT Float32_t angles[3]);
void Cvt_Rotation_To_Angles_DB(IN const Float64_t R[9], OUT Float64_t angles[3]);

//convert from rotation matrix into angles, R=Rot(Z)*Rot(Y)*Rot(X), angles in rad
void Cvt_Angles_To_Rotation(OUT Float32_t R[9], IN const Float32_t angles[3]);
void Cvt_Angles_To_Rotation_DB(OUT Float64_t R[9], IN const Float64_t angles[3]);

// invert angles and trans
void Inverse_Angles(OUT Float32_t angles_out[3], OUT Float32_t trans_out[3], 
	IN Float32_t angles_in[3], IN Float32_t trans_in[3]);
void Inverse_Angles_DB(OUT Float64_t angles_out[3], OUT Float64_t trans_out[3], 
	IN Float64_t angles_in[3], IN Float64_t trans_in[3]);

// invert angles and trans in place
void Inverse_Angles_InPlace(
	OUT IN Float32_t angles[3], OUT IN Float32_t trans[3]);
void Inverse_Angles_InPlace_DB(
	OUT IN Float64_t angles[3], OUT IN Float64_t trans[3]);

Int32_t svd(IN Float64_t a[][9], IN Int32_t m, IN Int32_t n, 
	OUT Float64_t w[], OUT Float64_t v[][9]);
#ifdef __cplusplus
}
#endif
#endif
