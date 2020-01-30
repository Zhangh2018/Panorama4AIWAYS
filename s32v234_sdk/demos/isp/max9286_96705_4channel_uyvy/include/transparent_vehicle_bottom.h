

//#include "opencv.h"
#include "data_type.h"
//#include "eol_config.h"
///#include "ImageStitching.h"
typedef struct CvPoint2D64f1
{
    double x;
    double y;
}
CvPoint2D64f1;


 typedef struct CvPoint2
{
	int x;
 	int y;
 } 
CvPoint2;

typedef enum Gear_Enum
{
	//GEAR_BETWEEN = 0,
	//GEAR_FORWARD = 1,
	//GEAR_REVERSE = 2,
	//GEAR_NEUTRAL = 3,	
	//GEAR_PARKING = 4	
	
  	GEAR_BETWEEN = 0,
	GEAR_FORWARD = 4,
	GEAR_REVERSE = 2,
	GEAR_NEUTRAL = 3,	
	GEAR_PARKING = 1	
	
}Gear_E;


void Mat_Mul_Vec_3by3_DB(OUT Float64_t Cout[3], IN Float64_t Ain[9], IN Float64_t Bin[3]);

static Int32_t convert_steering_wheel_angle(
	IN const Float64_t steering_wheel_angle,
	IN const Float64_t axis_length,
	OUT Float64_t& turning_radius,
	OUT Float64_t& wheel_turning_angle);

Int32_t fill_vehicle_bottom(uchar* result_uyvy_image,
	uchar* prev_uyvy_image,
	Int32_t result_image_width,
	Int32_t result_image_height,
	int  x1,
	int  x2,
	Float64_t mm_per_pixel_hori,
	Float64_t mm_per_pixel_vert,
	Float64_t vehicle_length,
	Float64_t vehicle_width,
	Float64_t front_bumper_length,
	Float64_t axis_length,
	Float64_t rear_bumper_length,
	Float64_t steering_angle,
	Float64_t speed,
	Float64_t time_elapsed,
	Gear_E gear);

