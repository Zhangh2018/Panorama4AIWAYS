/*------------------------------------------------------------------------------
	camera_model.h define camera model structure and some basic operating functions
	base on this model.
	Add by Shuo.Yan on 2017/10/31 InteSight
------------------------------------------------------------------------------*/
#ifndef __CAMERA_MODEL_H__
#define __CAMERA_MODEL_H__
#include <stdio.h>
#include "math.h"


#define IN
#define OUT
#define MAX_POL_LENGTH			(16)
#define DEF_NUM4				(4)
#define MAX_ARRAY_SIZE			(1024)
typedef signed int			Int32_t;
typedef double				Float64_t;

// In image coordinate system, the original point is image up-left point,
//     to right is x positive direction, to down is y positive direction;
// the camera coordinate system in camera model, to down is x positive direction,
//     to right is y positive direction, to inner is z positive direction;
// the camera coordinate system to calculate camera extrinsic param,
//     to right is x positive direction, to down is y positive direction,
//     to outer is z positive direction.
typedef struct Camera_Model_Int_Struct
{
	Float64_t    pol[MAX_POL_LENGTH];       // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
	Int32_t      length_pol;                // length of polynomial
	Float64_t    inv_pol[MAX_POL_LENGTH];   // the coefficients of the inverse polynomial
	Int32_t      length_inv_pol;            // length of inverse polynomial
	Float64_t    xc;                        // column coordinate of the center
	Float64_t    yc;                        // row coordinate of the center
	Float64_t    c;                         // affine parameter
	Float64_t    d;                         // affine parameter
	Float64_t    e;                         // affine parameter
	Float64_t	 fov_h;						// camera horizontal field of view
	Float64_t	 fov_v;						// camera vertical field of view
	Int32_t      width;                     // image width
	Int32_t      height;                    // image height
	Float64_t	 lut[1000];					// lens distortion lut, one of lut or (pol and inv_pol) must be chosen to describe lens distortion
	bool		 is_lut_used;				// whether choose to use lut represents lens distortion
}Camera_Model_Int_S, *P_Camera_Model_Int_S;

typedef struct Camera_Model_Ext_Struct
{
	Float64_t	pose[6];			        // the Euler angle and translation vector of camera in vehicle coordinate system
	Float64_t	R[9];                       // the rotation matrix from camera coordinate system to world coordinate system
	Float64_t	inv_pose[6];			    // the vehicle coordinate system's Euler angle and translation in camera coordinate system
	Float64_t	inv_R[9];                   // the rotation matrix from vehicle coordinate system to camera coordinate system
	bool		is_inverted;				// whether the camera is mounted rotated
}Camera_Model_Ext_S, *P_Camera_Model_Ext_S;

typedef struct Camera_Model_Struct
{
	Camera_Model_Int_S camera_model_int;    // camera intrinsic param structure
	Camera_Model_Ext_S camera_model_ext;    // camera extrinsic param structure
}Camera_Model_S, *P_Camera_Model_S;


extern P_Camera_Model_S Camera_Model[DEF_NUM4];
extern double g_camPos[4][3];
/*------------------------------------------------------------------------------
	Read camera model from files
------------------------------------------------------------------------------*/
Int32_t Load_Camera_Model_Int(
	OUT P_Camera_Model_Int_S p_camera_model_int, IN const char* file_name);

void Image_Point_To_Cam_Ray(OUT Float64_t cam_ray[3], IN Float64_t image_point[2],
	IN P_Camera_Model_Int_S p_camera_model_int);

void Cam_Ray_To_Image_Point(OUT Float64_t image_point[2],
	IN Float64_t cam_ray[3], IN P_Camera_Model_Int_S p_camera_model_int);

void World_Ray_To_Cam_Ray(OUT Float64_t cam_ray[3],
	IN Float64_t world_ray[3], IN P_Camera_Model_Ext_S p_camera_model_ext);

void Cam_Ray_To_World_Ray(OUT Float64_t world_ray[3],
	IN Float64_t cam_ray[3], IN P_Camera_Model_Ext_S p_camera_model_ext);

void World_Ray_To_Image_Point(OUT Float64_t image_point[2],
                              IN Float64_t world_ray[3], IN P_Camera_Model_S p_camera_model);

Int32_t Load_Camera_Model_Ext(
	OUT P_Camera_Model_Ext_S p_camera_model_ext, IN const char* file_name);

void Sync_Camera_Model_Ext(OUT IN P_Camera_Model_Ext_S p_camera_model_ext);
void load_ocam_params();

// 读取另一类内外参，其中外参文件中只包含了欧拉角和平移向量
void load_ocam_params2();

#endif /*__CAMERA_MODEL_H__*/
