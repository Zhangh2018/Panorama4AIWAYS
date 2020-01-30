/* data_type.h provides basic datatype and 
 * structure's definition of InteSight.
 * Create by Shuo.Yan on 2017/10/31 InteSight
 */
#ifndef __DATA_TYPE_H_
#define __DATA_TYPE_H_


#define CHESS_BOARD_NUM 80

#ifndef PI
#define PI (3.14159265358979323846264338327950288)
#endif



#ifdef __cplusplus 
#define INTESIGHT_API extern "C"
#else
#define INTESIGHT_API
#endif


#define  IN
#define  OUT

/* ================================================================= *\
 * define uniformed type name
\* ================================================================= */
#ifndef __DATA_TYPE_

#define Uint32  unsigned int
#define Uint16  unsigned short
#define Uint8   unsigned char
#define S08 char
#define S32 int
#define U08 unsigned S08
#define U16 unsigned short
#define U32 unsigned int

typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef signed short	Int16_t;

typedef unsigned int 			UInt32_t; 
typedef unsigned long  int 		UInt64_t; 
typedef  int 					Int32_t; 
typedef double				Float64_t;
typedef  unsigned char uchar;

typedef signed char			Int8_t;
typedef unsigned char		UInt8_t;
///typedef signed short		Int16_t;
typedef unsigned short		UInt16_t;
///typedef signed int			Int32_t;
///typedef unsigned int		UInt32_t;
///typedef long long			Int64_t;
//typedef unsigned long long	UInt64_t;
typedef float				Float32_t;
///typedef double				Float64_t;
typedef void*				Handle;
#endif

#define FREE_POINTER(p)		{ if(p != NULL) {free(p); p = NULL;} }

typedef enum Image_Type_Enum
{
	IMAGE_TYPE_YUV420 = 0,
	IMAGE_TYPE_YUV422,
	IMAGE_TYPE_YUV444,
	IMAGE_TYPE_NV12,
	IMAGE_TYPE_UYVY,
	IMAGE_TYPE_RGB,
	IMAGE_TYPE_RGBA,
}Image_Type_E;

typedef struct Intesight_Image_Struct
{
	Image_Type_E	image_type;			// image type
	Int32_t			height;				// image height
	Int32_t			width;				// image width
	Int32_t			channels;			// image channels
	UInt8_t*		p_data;				// image data's init address
}Intesight_Image_S, *P_Intesight_Image_S;

typedef enum Camera_Id_Enum
{
	CAM_FRONT = 0,
	CAM_REAR,
	CAM_LEFT,
	CAM_RIGHT
}Camera_Id_E;


typedef enum View_Angle
{
	_4_VIEW_BIG = 0,
	_4_VIEW_SMALL,
	_3_VIEW_FRONT,
	_3_VIEW_REAR
}View_Ang;

#define CAMERA_NUM					(4)

#define MAX_ARRAY_SIZE				(1024)

#define SOURCE_IMAGE_TYPE				IMAGE_TYPE_YUV420
#define RESULT_IMAGE_TYPE				IMAGE_TYPE_YUV420
#define LOGO_IMAGE_TYPE					IMAGE_TYPE_YUV420
#define LINE_WIDTH						6
#endif