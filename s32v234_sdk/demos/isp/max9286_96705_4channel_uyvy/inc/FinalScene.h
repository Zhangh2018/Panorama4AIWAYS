#ifndef _FINAL_SCENE_H__
#define _FINAL_SCENE_H__

#include <memory>
#include "nanovg.h"

#include "const_value.h"
#include "GLES2/gl2.h"
#include "opencv2/core.hpp"
#include "DrawScene.h"
#include "OGLWin.h"
//代表最终展示的界面类，初始展示时调用onStart，结束展示时调用onStop，并可以根据用户的输入执行不同的控制代码
//使用renderToSreen渲染界面

#include "LUT.h"
#include "utils.hpp"

#include "RadarView.hpp"
#include "DistanceLine.hpp"
#include "TransBot.h"
#include "BirdEyeView.hpp"
#include "PlaneExpandView.hpp"
#include "BoxExpandView.hpp"
#include "imgproc/smooth.hpp"

#include <thread>
#include <mutex>

enum class SceneMode
{
	Scene2D,
	Scene3D,
	SceneSingle,
	Scene2DWith3D,
	Scene2DWithSingle,
	Scene2DWithZoomFront,
	Scene2DWithZoomRear,
	Scene2DWithZoomLeft,
	Scene2DWithZoomRight,
	Scene2DWithZoomLeftRight
};

class CFinalScene
{
public:
	CFinalScene();
	~CFinalScene(void);
public:
    void changeView();
    void onStart();
    void onStop();
    void renderToSreen();              //渲染界面
	void onMessage(int message);       //响应消息
	void onButtonUp(int times);
	void onButtonDown(int times);
	void onButtonLeft(int times);
	void onButtonRight(int times);
	void onButtonOK();
	void endPreviousMode(int mode);
	void beginCurrentMode(int mode);
	void switchMode();

	void onButtonZoomIn(int times);
	void onButtonZoomOut(int times);

	void onButtonMoveLeft(int times);
	void onButtonMoveRight(int times);
	void onButtonMoveUp(int times);
	void onButtonMoveDown(int times);

	void onButtonTurnLeft(int times)
	{
		steeringAngle = clamp(steeringAngle - steeringStep, minAngle, maxAngle);
	}
	void onButtonTurnRight(int times)
	{
		steeringAngle = clamp(steeringAngle + steeringStep, minAngle, maxAngle);
	}

	void onButtonSwitchMode(SceneMode mode);

	inline void setBirdEyeViewport(int x, int y, int width, int height)
	{
		birdEyeViewX = x;
		birdEyeViewY = y;
		birdEyeViewWidth = width;
		birdEyeViewHeight = height;
		setBirdEyeViewParams(*lut);
	}

	inline void setSingleViewport(int x, int y, int width, int height)
	{
		singleViewX = x;
		singleViewY = y;
		singleViewWidth = width;
		singleViewHeight = height;
		//setSingleViewParams(singleViewWidth, singleViewHeight);
		setSingleViewParams(1110, 810);
	}

	inline void set3DViewport(int x, int y, int width, int height)
	{
		threeDViewX = x;
		threeDViewY = y;
		threeDViewWidth = width;
		threeDViewHeight = height;
	}

	void render2DView(int x, int y, int width, int height,void* data, GLuint fbo = 0);
	void render2DViewCrop(int x, int y, int width, int height, void* data, const glm::mat4& transform = glm::mat4(1.0), GLuint fbo = 0);
	void render2DViewCropViewport(int cropWidth, int cropHeight, int viewX, int viewY, int viewWidth, int viewHeight, void* data, const glm::mat4& transform = glm::mat4(1.0), GLuint fbo = 0);
	void render3DView(int x, int y, int width, int height, void* data, GLuint fbo = 0);
	void renderSingleView(int camID,int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));
	void renderImage(const char* path, int x, int y, int width, int height, const glm::mat4& transform = glm::mat4(1.0));


	void renderTrail(int x, int y, int width, int height, bool applyTransform = false, GLuint fbo = 0);
	void renderTexView(GLint textureID, int x, int y, int width, int height, glm::mat4 transMatrix = glm::mat4(1.0));
	void copyFBOTexture(GLsizei srcWidth, GLsizei srcHeight, GLsizei srcDepth);

/**
	 * 渲染文本框
	 * 
	 * @param text 要渲染的文字，编码必须为 UTF-8
	 * @param x 文本框左上角位置 (坐标原点在屏幕左上角)
	 * @param y 
	 * @param rowWidth 行宽，文本超过行宽会换行。(默认为当前屏幕宽度)
	 * @param fontSize 字体大小
	 * @param textColor 字体颜色 (RGBA) (默认不透明白色)
	 * @param boxColor 文本框的填充色 (RGBA) (默认全透明色)
	 */
	void renderTextBox(const char* text, int x, int y, int fontSize,
					  int rowWidth = getScreenWidth(),
					  const NVGcolor& textColor = nvgRGBAf(1, 1, 1, 1),
					  const NVGcolor& boxColor = nvgRGBAf(1, 1, 1, 0));

	void setRegionHighlight(int regionId);
	void unsetRegionHighlight(int regionId);
	// 透明车底
	// 2. 里程计数据
	Encoder encoderData;
	// 3. 透明车底类
    bool reset;
private:
    int m_endView;
    int m_curView;
    float m_phei[4];
    float m_theta[4];
    GLuint m_imageTextureID[const_value::CAMERANUM];
    GLvoid *m_dataBuffer[const_value::CAMERANUM];
    static const int ADJUST_TIMES = 8;

    GLuint fbo, rbo, dstTex;

	static const int lumEqualRate = 15;

	std::shared_ptr<CalibLUT> lut;

	RadarView radar;
	float radarSignals[16] = {0};
	std::mutex radarSignalMutex;


	DistanceLineView disLine;
	std::shared_ptr<NVGcontext> vgCtx;

	//SceneMode mode = SceneMode::Scene2DWith3D;
	SceneMode mode = SceneMode::Scene2DWithSingle;

	const float steeringStep = M_PI / 180.0;
	const float minAngle = steeringStep * -31.0;
	const float maxAngle = steeringStep * 31.0;

	float steeringAngle = minAngle;


	bool isRegionHighlighted[const_value::REGION_NUM];
	static float highlightColor[4];	// RGBA
	static float blankColor[4];

	int dynamicLineFlag = 31;

    GLuint Calib_BirdView_TextureID[1];

	int birdEyeViewX, birdEyeViewY, birdEyeViewWidth, birdEyeViewHeight;
	int singleViewX, singleViewY, singleViewWidth, singleViewHeight;
	int threeDViewX, threeDViewY, threeDViewWidth, threeDViewHeight;

	BirdEyeView bev;
	PlaneExpandView pevs[const_value::CAMERANUM];
	BoxExpandView boxViews[2];
	ExposureCompensation exposureComp;

	int vgFontId;

	std::map<std::string, int> images;

	
	// 透明车底
	// 1. 前一帧的帧缓存
	GLuint fboPre, dstTexPre;
	// 2. 里程计数据
	//Encoder encoderData;
	// 3. 透明车底类
	TransBot *transBotC;
    //bool reset;
};

#endif
