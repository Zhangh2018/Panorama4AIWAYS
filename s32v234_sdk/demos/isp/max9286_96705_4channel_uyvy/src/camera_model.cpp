/*------------------------------------------------------------------------------
	camera_model.cpp realize the basic operating functions defined in camera_model.h
	Add by Shuo.Yan on 2017/10/31 InteSight
------------------------------------------------------------------------------*/
#include "camera_model.h"
#include <iostream>
#include <memory.h>
using namespace std;
P_Camera_Model_S Camera_Model[DEF_NUM4];
double g_camPos[4][3];
/*
 * Function Name:				Load_Camera_Model_Int
 * Function Function:			load camera intrinsic param from file(*.txt)
 * Input:
 *     p_camera_model_int:		camera intrinsic param structure
 *     file_name:				file name
 * Note:
 *     In camera coordinate system, to down is x positive direction, to right is y positive direction, to back is z positive z direction
 * Revision:
 *     Rename by Shuo.Yan on 2017/11/23
 */
Int32_t Load_Camera_Model_Int(
	OUT P_Camera_Model_Int_S p_camera_model_int, IN const char* file_name)
{
	Int32_t ret = 0;
	FILE *f;
	char buf[MAX_ARRAY_SIZE];

	Float64_t	*pol = p_camera_model_int->pol;
	Float64_t	*inv_pol = p_camera_model_int->inv_pol;
	Float64_t	*xc = &(p_camera_model_int->xc);
	Float64_t	*yc = &(p_camera_model_int->yc);
	Float64_t	*c = &(p_camera_model_int->c);
	Float64_t	*d = &(p_camera_model_int->d);
	Float64_t	*e = &(p_camera_model_int->e);
	Int32_t		*width = &(p_camera_model_int->width);
	Int32_t		*height = &(p_camera_model_int->height);
	Int32_t		*length_pol = &(p_camera_model_int->length_pol);
	Int32_t		*length_invpol = &(p_camera_model_int->length_inv_pol);

	Int32_t i;

	//Open file
	if (!(f = fopen(file_name, "r")))
	{
		printf("File %s cannot be opened\n", file_name);
		ret = 1;
		return ret;
	}
	//Read polynomial coefficients
	fgets(buf, MAX_ARRAY_SIZE, f);
	fscanf(f, "\n");
	fscanf(f, "%d", length_pol);
	for (i = 0; i < *length_pol; i++)
	{
		fscanf(f, " %lf", &pol[i]);
	}

	//Read inverse polynomial coefficients
	fscanf(f, "\n");
	fgets(buf, MAX_ARRAY_SIZE, f);
	fscanf(f, "\n");
	fscanf(f, "%d", length_invpol);
	for (i = 0; i < *length_invpol; i++)
	{
		fscanf(f, " %lf", &inv_pol[i]);
	}

	//Read center coordinates
	fscanf(f, "\n");
	fgets(buf, MAX_ARRAY_SIZE, f);
	fscanf(f, "\n");
	fscanf(f, "%lf %lf\n", yc, xc);
    fprintf(stdout,"(xc,yc) = %lf %lf\n", *yc, *xc);
	//Read affine coefficients
	fgets(buf, MAX_ARRAY_SIZE, f);
	fscanf(f, "\n");
	fscanf(f, "%lf %lf %lf\n", c, d, e);

	//Read image size
	fgets(buf, MAX_ARRAY_SIZE, f);
	fscanf(f, "\n");
	fscanf(f, "%d %d", height, width);

	fclose(f);
	return ret;
}

/*
 * Function Name:				Image_Point_To_Cam_Ray
 * Function Function:			calculate image point's coordinate in camera coordinate system
 * Input:
 *     point3D:					incline ray vector in camera coordinate system(x to down(the 1st element), y to right(the 2nd element), z to back(the 3rd element))
 *     point2D:					image point's coordinate in image coordinate system(x to right(the 1st element), y to down(the 2nd element))
 *     p_camera_model_int:		fisheye camera model
 * Note:
 *     In camera coordinate system, to down is x positive direction, to right is y positive direction, to back is z positive z direction
 * Revision:
 */
void Image_Point_To_Cam_Ray(OUT Float64_t cam_ray[3],
	IN Float64_t image_point[2],
	IN P_Camera_Model_Int_S p_camera_model_int)
{
	Float64_t* pol = p_camera_model_int->pol;
	Float64_t  xc = (p_camera_model_int->xc);
	Float64_t  yc = (p_camera_model_int->yc);
	Float64_t  c = (p_camera_model_int->c);
	Float64_t  d = (p_camera_model_int->d);
	Float64_t  e = (p_camera_model_int->e);
	Int32_t    length_pol = (p_camera_model_int->length_pol);

	// 1/det(A), where A = [c,d;e,1] as in the Matlab file
	Float64_t  invdet = 1 / (c - d * e);

	Float64_t  xp = invdet
		* ((image_point[0] - yc) - d * (image_point[1] - xc));
	Float64_t  yp = invdet
		* (-e * (image_point[0] - yc) + c * (image_point[1] - xc));

	//distance [pixels] of  the point from the image center
	Float64_t  r = sqrt(xp*xp + yp*yp);
	Float64_t  zp = pol[0];
	Float64_t  r_i = 1;
	Int32_t i;

	for (i = 1; i < length_pol; i++)
	{
		r_i *= r;
		zp += r_i * pol[i];
	}

	//normalize to unit norm
	Float64_t invnorm = 1 / sqrt(xp*xp + yp*yp + zp*zp);

	cam_ray[0] = invnorm * xp;
	cam_ray[1] = invnorm * yp;
	cam_ray[2] = invnorm * zp;
}

/*
 * Function Name :			Cam_Ray_To_Image_Point
 * Function Function :		calculate image point's coordinate in camera coordinate system
 * Input :
 *     point2D :			image point's coordinate in image coordinate system(y to down(the 1st element, x to right(the 2nd element) ))
 * 	   point3D :			incline ray vector in camera coordinate system(x to up(the 1st element), y to right(the 2nd element), z to back(the 3rd element))
 * 	   p_camera_model_int : fisheye camera model
 * Note :
 *     In camera coordinate system, to down is x positive direction, to right is y positive direction, to back is z positive z direction
 * Revision :
 *     Rename by Shuo.Yan on 2017/11/23
 */
void Cam_Ray_To_Image_Point(OUT Float64_t image_point[2],
	IN Float64_t cam_ray[3], IN P_Camera_Model_Int_S p_camera_model_int)
{
	Float64_t* inv_pol = p_camera_model_int->inv_pol;
	Float64_t  xc = (p_camera_model_int->xc);
	Float64_t  yc = (p_camera_model_int->yc);
	Float64_t  c = (p_camera_model_int->c);
	Float64_t  d = (p_camera_model_int->d);
	Float64_t  e = (p_camera_model_int->e);
	Int32_t    width = (p_camera_model_int->width);
	Int32_t    height = (p_camera_model_int->height);
	Int32_t    length_invpol = (p_camera_model_int->length_inv_pol);
	Float64_t  norm = sqrt(cam_ray[0] * cam_ray[0] + cam_ray[1] * cam_ray[1]);
	Float64_t  theta = atan(cam_ray[2] / norm);
	Float64_t  t, t_i;
	Float64_t  rho, x, y;
	Float64_t  invnorm;
	Int32_t i;

	if (norm != 0)
	{
		invnorm = 1 / norm;
		t = theta;
		rho = inv_pol[0];
		t_i = 1;

		for (i = 1; i < length_invpol; i++)
		{
			t_i *= t;
			rho += t_i * inv_pol[i];
		}

		x = cam_ray[0] * invnorm * rho;
		y = cam_ray[1] * invnorm * rho;

		image_point[0] = x * c + y * d + yc;
		image_point[1] = x * e + y * 1 + xc;
	}
	else
	{
		image_point[0] = yc;
		image_point[1] = xc;
	}
}

void World_Ray_To_Cam_Ray(OUT Float64_t cam_ray[3],
	IN Float64_t world_ray[3], IN P_Camera_Model_Ext_S p_camera_model_ext)
{
	cam_ray[1] = (p_camera_model_ext->inv_R[0] * world_ray[0]
		+ p_camera_model_ext->inv_R[1] * world_ray[1]
		+ p_camera_model_ext->inv_R[2] * world_ray[2]
		+ p_camera_model_ext->inv_pose[3]);
	cam_ray[0] = (p_camera_model_ext->inv_R[3] * world_ray[0]
		+ p_camera_model_ext->inv_R[4] * world_ray[1]
		+ p_camera_model_ext->inv_R[5] * world_ray[2]
		+ p_camera_model_ext->inv_pose[4]);
	cam_ray[2] = -(p_camera_model_ext->inv_R[6] * world_ray[0]
		+ p_camera_model_ext->inv_R[7] * world_ray[1]
		+ p_camera_model_ext->inv_R[8] * world_ray[2]
		+ p_camera_model_ext->inv_pose[5]);
}

void Cam_Ray_To_World_Ray(OUT Float64_t world_ray[3],
	IN Float64_t cam_ray[3], IN P_Camera_Model_Ext_S p_camera_model_ext)
{
	Float64_t coef;

	world_ray[0] = (p_camera_model_ext->R[0] * cam_ray[0]
		+ p_camera_model_ext->R[1] * cam_ray[1]
		+ p_camera_model_ext->R[2] * cam_ray[2]
		);
	world_ray[1] = (p_camera_model_ext->R[3] * cam_ray[0]
		+ p_camera_model_ext->R[4] * cam_ray[1]
		+ p_camera_model_ext->R[5] * cam_ray[2]
		);
	world_ray[2] = (p_camera_model_ext->R[6] * cam_ray[0]
		+ p_camera_model_ext->R[7] * cam_ray[1]
		+ p_camera_model_ext->R[8] * cam_ray[2]
		);

	coef = p_camera_model_ext->pose[5] / (-world_ray[2]);
	world_ray[0] = world_ray[0] * coef + p_camera_model_ext->pose[3];
	world_ray[1] = world_ray[1] * coef + p_camera_model_ext->pose[4];
	world_ray[2] = world_ray[2] * coef + p_camera_model_ext->pose[5];
}

void World_Ray_To_Image_Point(OUT Float64_t image_point[2],
                              IN Float64_t world_ray[3], IN P_Camera_Model_S p_camera_model)
{
	Float64_t cam_ray[3] = {0};
	World_Ray_To_Cam_Ray(cam_ray, world_ray, &p_camera_model->camera_model_ext);
	Cam_Ray_To_Image_Point(image_point, cam_ray, &p_camera_model->camera_model_int);
}

/*
 * Function Name :			Load_Camera_Model_Ext
 * Function Function :		Load camera extrinsic param(Euler angle & trans vector from camera coordinate system to vehicle coordinate system)
 * Input :
 *     p_camera_model_ext:	camera extrinsic param structure
 * 	   file_name :			extrinsic file name for one camera
 * Note :
 *     the function read the Euler angle and translation vector from camera to vehicle coordinate system to camera extrinsic param structure
 * Revision :
 *     Add by Shuo.Yan on 2017/11/23
 */
Int32_t Load_Camera_Model_Ext(
	OUT P_Camera_Model_Ext_S p_camera_model_ext, IN const char* file_name)
{
	Int32_t ret = 0;
	FILE* f;
	Int32_t param_id = 0;
	char buf[MAX_ARRAY_SIZE];

	if (!(f = fopen(file_name, "r")))
	{
		printf("Extrinsic param file %s cannot be opened\n", file_name);
		ret = 1;
		return ret;
	}

	fgets(buf, MAX_ARRAY_SIZE, f);
	for (param_id = 0; param_id < 6; param_id++)
	{
		fscanf(f, "%lf ", &p_camera_model_ext->pose[param_id]);
	}

	fscanf(f, "\n");
	fgets(buf, MAX_ARRAY_SIZE, f);
	for (param_id = 0; param_id < 9; param_id++)
	{
		fscanf(f, "%lf ", &p_camera_model_ext->R[param_id]);
	}

	fscanf(f, "\n");
	fgets(buf, MAX_ARRAY_SIZE, f);
	for (param_id = 0; param_id < 6; param_id++)
	{
		fscanf(f, "%lf ", &p_camera_model_ext->inv_pose[param_id]);
	}

	fscanf(f, "\n");
	fgets(buf, MAX_ARRAY_SIZE, f);
	for (param_id = 0; param_id < 9; param_id++)
	{
		fscanf(f, "%lf ", &p_camera_model_ext->inv_R[param_id]);
	}
	fclose(f);

	return ret;
}

/*
* Function Name:		Cvt_Angles_To_Rotation_DB
* Function Function:	convert Euler angles to rotation matrix
* Input:
*     R:				rotation matrix
*     angles:		    Euler angle
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/
void Cvt_Angles_To_Rotation_DB(OUT Float64_t R[9], IN const Float64_t angles[3])
{
	Float64_t cx = cos(angles[0]), cy = cos(angles[1]), cz = cos(angles[2]);
	Float64_t sx = sin(angles[0]), sy = sin(angles[1]), sz = sin(angles[2]);
	R[0] = cy*cz;	R[1] = -cx*sz + sx*sy*cz;	R[2] = sx*sz + cx*sy*cz;
	R[3] = cy*sz;	R[4] = cx*cz + sx*sy*sz;	R[5] = -sx*cz + cx*sy*sz;
	R[6] = -sy;		R[7] = sx*cy;			R[8] = cx*cy;
}

/*
* Function Name:		Cvt_Rotation_To_Angles_DB
* Function Function:	convert rotation matrix to Euler angles
* Input:
*     R:				rotation matrix
*     angles:		    Euler angle
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/
void Cvt_Rotation_To_Angles_DB(IN const Float64_t R[9], OUT Float64_t angles[3])
{
	angles[0] = atan2(R[7], R[8]);
	angles[1] = atan2(-R[6], sqrt(R[7] * R[7] + R[8] * R[8]));
	angles[2] = atan2(R[3], R[0]);
}

/*
* Function Name:		Transpose_Mat_3by3_DB
* Function Function:	transpose matrix
* Input:
*     At:               the transposed matrix
*     A:				the input vector
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/
void Transpose_Mat_3by3_DB(OUT Float64_t At[9], IN Float64_t A[9])
{
	//swap A[1] A[3]
	At[0] = A[0]; At[1] = A[3]; At[2] = A[6];
	//swap A[2] A[6]
	At[3] = A[1]; At[4] = A[4]; At[5] = A[7];
	//swap A[5] A[7]
	At[6] = A[2]; At[7] = A[5]; At[8] = A[8];
}

/*
* Function Name:		Mat_Mul_Vec_3by3_DB
* Function Function:	calculate a 3*3 matrix multiply a 3*1 vector
* Input:
*     Cout:				result vector
*     Ain:				the input matrix
*     Bin:				the input vector
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/
void Mat_Mul_Vec_3by3_DB(
	OUT Float64_t Cout[3], IN Float64_t Ain[9], IN Float64_t Bin[3])
{
	Cout[0] = Ain[0] * Bin[0] + Ain[1] * Bin[1] + Ain[2] * Bin[2];
	Cout[1] = Ain[3] * Bin[0] + Ain[4] * Bin[1] + Ain[5] * Bin[2];
	Cout[2] = Ain[6] * Bin[0] + Ain[7] * Bin[1] + Ain[8] * Bin[2];
}

/*
* Function Name:		Inverse_RT_DB
* Function Function:	inverse rotation matrix and trans vector
* Input:
*     Rout:				inversed rotation matrix
*     Tout:				inversed transition vector
*     Rin:				origin rotation matrix
*     Tout:				origin transition vector
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/
void Inverse_RT_DB(OUT Float64_t Rout[9], OUT Float64_t Tout[3],
	IN Float64_t Rin[9], IN Float64_t Tin[3])
{
	Transpose_Mat_3by3_DB(Rout, Rin);
	Mat_Mul_Vec_3by3_DB(Tout, Rout, Tin);
	Tout[0] = -Tout[0]; Tout[1] = -Tout[1]; Tout[2] = -Tout[2];
}
/*
 * Function Name :			Sync_Camera_Model_Ext
 * Function Function :		Update all extrinsic params by given angle and translation vector from camera to vehicle coordinate system
 * Input :
 *     p_camera_model_ext:	camera extrinsic param structure
 * Note :
 *     The function keep each camera extrinsic parameter' representation format in consistent with each other. only pose array is correctly
 *     in this function by default, once this array is updated, all other extrinsic param should by updated correspondingly to keep consistency.
 * Revision :
 *     Add by Shuo.Yan on 2017/11/23
 */
void Sync_Camera_Model_Ext(OUT IN P_Camera_Model_Ext_S p_camera_model_ext)
{
	// Update rotation matrix by Euler angle
	Cvt_Angles_To_Rotation_DB(p_camera_model_ext->R, p_camera_model_ext->pose);

	// Update inv_R and translation vector in inv_pose by R and t
	Inverse_RT_DB(p_camera_model_ext->inv_R, &p_camera_model_ext->inv_pose[3],
		p_camera_model_ext->R, &p_camera_model_ext->pose[3]);

	// Update Euler angle vector in inv_pose by inv_R
	Cvt_Rotation_To_Angles_DB(p_camera_model_ext->inv_R,
		p_camera_model_ext->inv_pose);
}
void load_ocam_params()
{
	for (Int32_t cam_id = 0; cam_id < DEF_NUM4; cam_id++)
	{
		Camera_Model[cam_id] = (Camera_Model_S*)malloc(sizeof(Camera_Model_S));
		if (NULL == Camera_Model[cam_id])
		{
			cout << "can not load camera_model!\n" << endl;
			return;
		}
		memset(&Camera_Model[cam_id]->camera_model_int, 0, sizeof(Camera_Model_Int_S));
		string file_name_int, file_name_ext;
		if (0 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_front.txt";
			file_name_ext = "/data/opengl_new/param/camera_model_ext_front.txt";
		}
		else if (1 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_back.txt";
			file_name_ext = "/data/opengl_new/param/camera_model_ext_back.txt";
		}
		else if (2 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_left.txt";
			file_name_ext = "/data/opengl_new/param/camera_model_ext_left.txt";
		}
		else if (3 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_right.txt";
			file_name_ext = "/data/opengl_new/param/camera_model_ext_right.txt";
		}
		/*读取内参*/
		Load_Camera_Model_Int(&Camera_Model[cam_id]->camera_model_int, file_name_int.c_str());
                //printf("cameraId = %d,xc = %lf,yc = %lf\n",cam_id,Camera_Model[cam_id]->camera_model_int.xc,Camera_Model[cam_id]->camera_model_int.yc);
		/*读取外参*/
		Load_Camera_Model_Ext(&Camera_Model[cam_id]->camera_model_ext, file_name_ext.c_str());
        //Float64_t camera_orgin[3] = {0,0,0};
        //Cam_Ray_To_World_Ray(g_camPos[cam_id],camera_orgin, &Camera_Model[cam_id]->camera_model_ext);
        memcpy(g_camPos[cam_id],Camera_Model[cam_id]->camera_model_ext.pose + 3,sizeof(Float64_t) * 3);
        printf("cameraPos%d,(%8.2f,%8.2f,%8.2f)\n",cam_id,g_camPos[cam_id][0],g_camPos[cam_id][1],g_camPos[cam_id][2]);
	}
}


/*
 * Function Name :			Load_Camera_Model_Ext2
 * Function Function :		Load camera extrinsic param(Euler angle & trans vector from camera coordinate system to vehicle coordinate system)
 * Input :
 *     p_camera_model_ext:	camera extrinsic param structure
 * 	   file_name :			extrinsic file name for one camera
 * Note :
 *     the function read the Euler angle and translation vector from camera to vehicle coordinate system to camera extrinsic param structure
 * Revision :
 *     Add by Shuo.Yan on 2017/11/23
 */
Int32_t Load_Camera_Model_Ext2(
	OUT P_Camera_Model_Ext_S p_camera_model_ext, IN const char* file_name)
{
	Int32_t ret = 0;
	FILE* f;
	Int32_t param_id = 0;

	if (!(f = fopen(file_name, "r")))
	{
		printf("Extrinsic param file %s cannot be opened\n", file_name);
		ret = 1;
		return ret;
	}
	/*for (param_id = 0; param_id < 3; param_id++)
	{
		fscanf(f, "%f ", &p_camera_model_ext->inv_R[param_id]);
	}
	fscanf(f, "\n");
	for (param_id = 3; param_id < 6; param_id++)
	{
		fscanf(f, "%f ", &p_camera_model_ext->inv_R[param_id]);
	}
	fscanf(f, "\n");
	for (param_id = 6; param_id < 9; param_id++)
	{
		fscanf(f, "%f ", &p_camera_model_ext->inv_R[param_id]);
	}
	fscanf(f, "\n");
	for (param_id = 3; param_id < 6; param_id++)
	{
		fscanf(f, "%f ", &p_camera_model_ext->inv_pose[param_id]);
	}*/

	for (param_id = 0; param_id < 6; param_id++)
	{
		fscanf(f, "%lf ", &p_camera_model_ext->pose[param_id]);
	}
	/*for (param_id = 0; param_id < 6; param_id++)
	{
		printf("gg%f\t ", p_camera_model_ext->inv_pose[param_id]);
	}*/
	//while (1);
	fclose(f);

	return ret;
}

void load_ocam_params2()
{
	for (Int32_t cam_id = 0; cam_id < DEF_NUM4; cam_id++)
	{
		Camera_Model[cam_id] = (Camera_Model_S*)malloc(sizeof(Camera_Model_S));
		if (NULL == Camera_Model[cam_id])
		{
			cout << "can not load camera_model!\n" << endl;
			return;
		}
		memset(&Camera_Model[cam_id]->camera_model_int, 0, sizeof(Camera_Model_Int_S));
		string file_name_int, file_name_ext;
		if (0 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_front.txt";
			file_name_ext = "/data/opengl_new/param/ocam_extrinsic_front.txt";
		}
		else if (1 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_rear.txt";
			file_name_ext = "/data/opengl_new/param/ocam_extrinsic_rear.txt";
		}
		else if (2 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_left.txt";
			file_name_ext = "/data/opengl_new/param/ocam_extrinsic_left.txt";
		}
		else if (3 == cam_id){
			file_name_int = "/data/opengl_new/param/ocam_intrinsic_right.txt";
			file_name_ext = "/data/opengl_new/param/ocam_extrinsic_right.txt";
		}
		/*读取内参*/
		Load_Camera_Model_Int(&Camera_Model[cam_id]->camera_model_int, file_name_int.c_str());
		//printf("cameraId = %d,xc = %lf,yc = %lf\n",cam_id,Camera_Model[cam_id]->camera_model_int.xc,Camera_Model[cam_id]->camera_model_int.yc);
		/*读取外参*/
		Load_Camera_Model_Ext2(&Camera_Model[cam_id]->camera_model_ext, file_name_ext.c_str());

		Sync_Camera_Model_Ext(&Camera_Model[cam_id]->camera_model_ext);
		//Float64_t camera_orgin[3] = {0,0,0};
		//Cam_Ray_To_World_Ray(g_camPos[cam_id],camera_orgin, &Camera_Model[cam_id]->camera_model_ext);
		memcpy(g_camPos[cam_id],Camera_Model[cam_id]->camera_model_ext.pose + 3,sizeof(Float64_t) * 3);
		printf("cameraPos%d,(%8.2f,%8.2f,%8.2f)\n",cam_id,g_camPos[cam_id][0],g_camPos[cam_id][1],g_camPos[cam_id][2]);
	}
}
