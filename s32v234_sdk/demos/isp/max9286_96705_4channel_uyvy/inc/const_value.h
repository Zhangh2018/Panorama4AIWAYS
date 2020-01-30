#ifndef _CONST_VALUE_H__
#define _CONST_VALUE_H__

namespace const_value
{
    const int SRC_IMAGE_WIDTH = 1280;
    const int SRC_IMAGE_HEIGHT = 720;
    const int SRC_IMAGE_RGB_CHANNELS = 3;
    const int SRC_IMAGE_YUV_CHANNELS = 2;

    const int SRC_FRAME_BUFFERS = 3;

	const float SF = 5.0;
    const float Pi = 3.141592653;
	const int FRONT_REGION = 0;
	const int REAR_REGION = 1;
	const int LEFT_REGION = 2;
	const int RIGHT_REGION = 3;
	const int FRONT_LEFT_REGION = 4;
	const int FRONT_RIGHT_REGION = 5;
	const int REAR_LEFT_REGION = 6;
	const int REAR_RIGHT_REGION = 7;
	const int REGION_NUM = 8;
    const float ESP = 0.00001;

    const int FRONT_CAM	= 0;
    const int REAR_CAM	= 1;
    const int LEFT_CAM	= 2;
    const int RIGHT_CAM	= 3;
    const int CAMERANUM = 4;
};
#endif
