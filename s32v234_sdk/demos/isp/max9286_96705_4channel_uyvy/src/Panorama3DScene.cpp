#include "Panorama3DScene.h"
#include "TextureIL.h"
#include "MatrixState.h"
#include "ImageUtilStruct.h"
#include "VertexArrayIO.h"
#include "CModifyableParam.h"
#include "TextureIL.h"
#include "ImageBilt.h"
#include "TextureIL.h"
#include "const_value.h"
#include "PointGrid.h"
#include "PointMap.h"
#include <iostream>
#include "DrawScene.h"
Panorama3DScene::Panorama3DScene()
{
    m_trackBall = NULL;
    m_isInit = false;
    m_isLoadVetex = false;
    m_curCameraPosition = 1;
}

void create3DPanoramaVertexArray(std::vector<float> (&positionArray)[8],std::vector<float> (&weightArray)[4],
         std::vector<float> (&unMergeTextArray)[4],std::vector<float> (&mergeTextArray1)[4],std::vector<float> (&mergeTextArray2)[4],std::vector<unsigned short> (&indexArray)[8])
{
	//定义3D全景模型生成类，3D全景模型是一个以小车为中心的碗状结构，碗状结构的内半径为m_innerRadius，外半径为m_outerRadius，三角剖分的密度由m_sizePerGrid表示，
	//即每个剖分格子的大小,m_carWidth为车宽，m_carLength为车长，m_mergeMinAngle为起始融合角，m_mergeMaxAngle为结束融合角
    PointGrid pointGrid(g_3dParam.m_innerRadius,g_3dParam.m_outerRadius,g_3dParam.m_sizePerGrid,g_carParam.m_carWidth,
                        g_carParam.m_carLength,g_3dParam.m_mergeMinAngle * const_value::Pi / 180,g_3dParam.m_mergeMaxAngle * const_value::Pi / 180);
    pointGrid.generate2DGrid();
    pointGrid.generate3DVertexArray(positionArray,weightArray);
    pointGrid.generate3DIndexArray(indexArray);
    PointMap pointMap;
    pointMap.generateMergeTextCoord(&positionArray[4],weightArray,mergeTextArray1,mergeTextArray2);
    pointMap.generateUnmergeTextCoord(positionArray,unMergeTextArray);
    //VertexArrayIO::generateLutStatisLut("param/lumin_3d.bin",unMergeTextArray,mergeTextArray1,mergeTextArray2,positionArray);
}
Panorama3DScene::~Panorama3DScene(void)
{
    delete m_trackBall;
}
void Panorama3DScene::setNextCameraViewByThetaPhei(float theta,float phei,int motionLen)
{
	//定义旋转动画，由当前视角到(theta,phei)所表示的结束节点，motionLen为动画所需经过的帧数
    float curPhei = m_trackBall->getCurCameraPhei();
    float curTheta = m_trackBall->getCurCameraTheta();
    //printf("curPhei = %f,curTheta = %f\n",curPhei * 180 / const_value::PI,curTheta * 180 / const_value::PI);
    //printf("phei = %f,theta = %f\n",phei * 180 / const_value::PI,theta * 180 / const_value::PI);
    m_intervalPhei = (phei - curPhei);// / motionLen;
    m_intervalTheta = (theta - curTheta);// / motionLen;
	//保证旋转动画是逆时针旋转
    if(m_intervalTheta < 0.0001){
        m_intervalTheta += 2 * const_value::Pi / motionLen;
    }
    m_endPhei = phei;
    m_endTheta = theta;
}
bool Panorama3DScene::isAnimationCompleted()
{
    float curPhei = m_trackBall->getCurCameraPhei();
    float curTheta = m_trackBall->getCurCameraTheta();
    if(fabs(curPhei - m_endPhei) > 0.0001 || fabs(curTheta - m_endTheta) > 0.0001){
        return false;
    }
    return true;
}
MyMatrix Panorama3DScene::getCameraViewMatrix()
{
    float curPhei = m_trackBall->getCurCameraPhei();
    float curTheta = m_trackBall->getCurCameraTheta();
    if(!isAnimationCompleted()){  //如果旋转没有结束，则修改当前的视角，即修改theta和phei
        curPhei += m_intervalPhei;
        curTheta += m_intervalTheta;
		//保证旋转角度是(0,2*pi)之间
        if(curTheta > 2 * const_value::Pi - 0.0001) curTheta -= 2 * const_value::Pi;
        m_trackBall->setCurViewByAngle(curPhei,curTheta);
    }
    MyMatrix cameraMatrix = m_trackBall->getMatrixByThetaPhei(curPhei,curTheta);
    return cameraMatrix;
}

extern float view2DScale;
extern float hOffset;
extern float vOffset;

void Panorama3DScene::drawPanorama3D(double x,double y,double width,double height, float* delta)
{
    int screenWidth = getScreenWidth();
    int screenHeight = getScreenHeight();
    setGlobalLuminParam(g_globalCompensate);
    glEnable(GL_DEPTH_TEST);
    //glViewport (int(x * screenWidth + 0.5),int(y * screenHeight + 0.5),int(width * screenWidth + 0.5),int(height * screenHeight + 0.5));
	glViewport(x, y, width, height);

	printf("3D: x = %d, y = %d, width = %d, height = %d\n", viewX, viewY, viewWidth, viewHeight);
    int mergedIndex1[4] = {0,0,1,1};
    int mergedIndex2[4] = {2,3,2,3};
    MyMatrix cameraMatrix = getCameraViewMatrix();
	cameraMatrix.m12 = hOffset;
	cameraMatrix.m13 = vOffset;
	cameraMatrix.m14 += (view2DScale - 1.0);
    MatrixState::setCamera(cameraMatrix);
    MatrixState::loadMatrix(m_panoramaMatrix);
    for(int i = 0; i < 4; i++)
    {
        m_unmergedPanorama3D[i].drawSelf(m_imageTextureID[i],MatrixState::getFinalMatrix(), delta[i]);
        m_mergedPanorama3D[i].drawSelf(m_imageTextureID[mergedIndex1[i]],m_imageTextureID[mergedIndex2[i]],MatrixState::getFinalMatrix(), delta[mergedIndex1[i]], delta[mergedIndex2[i]]);
    }
    MatrixState::loadMatrix(m_modelMatrix);
    m_modelRender.drawModel(m_carModel);
    glViewport(0,0,getScreenWidth(), getScreenHeight());
    glDisable(GL_DEPTH_TEST);
}
void Panorama3DScene::setCurCameraPosition(int position)
{
    m_curCameraPosition = position;
}
void Panorama3DScene::draw3DScene(float* delta)
{
    glClear(GL_DEPTH_BUFFER_BIT);
    drawPanorama3D(0,0,0.60,1, delta);
}
void Panorama3DScene::draw(float* delta, bool isFullScreen)
{

    if(isFullScreen)
    {
		printf("fullscreen\n");
        glClear(GL_DEPTH_BUFFER_BIT);
		//右侧提示栏，显示位置为(0.792,0.035),大小为(0.167,0.935)
        //imageBilt(m_warningTextureID[m_curCameraPosition],0.9,0.0,0.1,1.0);
		//3D全景的显示位置
        //drawPanorama3D(0.4,0.2,0.6,0.8, delta);
        drawPanorama3D(viewX, viewY, viewWidth, viewHeight, delta);
    }
    else
    {
        glClear(GL_DEPTH_BUFFER_BIT);
        drawPanorama3D(0.4444,0.125,0.5,0.75, delta);
    }
}
void Panorama3DScene::initDrawScene(GLuint *imageTextureId)
{
    /*********************************初始化场景*****************************************/
    glClearDepthf(1.0);
    if(m_isInit == true)
    {
        return;
    }
    /*********************************载入纹理******************************************/
    for(int i = 0; i < 4; i++)
    {
        m_imageTextureID[i] = imageTextureId[i];
    }
    m_carModel.loadModel("/data/opengl_new/image/car.3DS");
    char *warningImage[] = {"/data/opengl_new/image/right_3d.bmp","/data/opengl_new/image/left_3d.bmp","/data/opengl_new/image/right_3d.bmp"};
    for(int i = 0; i < sizeof(warningImage) / sizeof(char *); i++)
        m_warningTextureID[i] = createStaticTexture(warningImage[i]);
    m_modelRender.initRender();
    /********************************初始化3D全景着色器**********************************/
    for(int i = 0; i < 4; i++)
    {
        m_mergedPanorama3D[i].initMergePanorama();
        m_unmergedPanorama3D[i].initUnMergePanorama();
    }
    setLocalLuminParam(g_luminParam);
    m_isInit = true;
}
void Panorama3DScene::setLocalLuminParam(float *luminParam)
{
    for(int i = 0; i < 4; i++)
    {
        m_unmergedPanorama3D[i].setLocalLuminParam(luminParam);
        m_mergedPanorama3D[i].setLocalLuminParam(luminParam);
    }
}
void Panorama3DScene::setGlobalLuminParam(float *luminParam)
{
    int index1[4] = {0,0,1,1};
    int index2[4] = {2,3,2,3};
    for(int i = 0; i < 4; i++)
    {
        m_unmergedPanorama3D[i].setGlobalLuminDiff(luminParam[i]);
        m_mergedPanorama3D[i].setGlobalLuminDiff(luminParam[index1[i]],luminParam[index2[i]]);
    }
}
//修改，其中theta和phei表示起始的视角
void Panorama3DScene::setPanorama3dParam(float theta,float phei,bool m_forceReload)
{
    if(m_isLoadVetex && !m_forceReload)
    {
        printf("not setPanorama3dParam!\n");
        return;
    }
    /**********************************初始化矩阵***************************************/
    MatrixState::setInitStack();
    MatrixState::pushMatrix();
    float scale = g_3dParam.m_outerRadius * 2 + std::max(g_carParam.m_carWidth / 2,g_carParam.m_carLength / 2);
    MatrixState::setProjectFrustum(-0.01,0.01,-0.01,0.01,0.01,10);
    MatrixState::applyScale(1.0 / scale,1.0 / scale,1.0 / scale);
    memcpy(m_panoramaMatrix,MatrixState::getCurMatrix(),sizeof(float) * 16);
    MatrixState::applyScale(g_carParam.m_carWidth,g_carParam.m_carLength,g_carParam.m_carHeight);
    MatrixState::applyTranslate(0,0,0.5);
    MatrixState::applyRotate(90,1,0,0);
    MatrixState::applyRotate(180,0,1,0);
    memcpy(m_modelMatrix,MatrixState::getCurMatrix(),sizeof(float) * 16);
    MatrixState::popMatrix();
    /*********************************载入网格******************************************/
    int mergedIndex1[4] = {0,0,1,1};
    int mergedIndex2[4] = {2,3,2,3};
    std::vector<float> positionArray[8],weightArray[4],unMergeTextArray[4],mergeTextArray1[4],mergeTextArray2[4];
    std::vector<unsigned short> indexArray[8];
	//如果第一次运行3D全景，没有生成模型的顶点、纹理坐标，则设置m_reGenerateModel为1，表示需要重新生成顶点文件，如果为0，则直接从文件中导入
    if(g_3dParam.m_reGenerateModel == 1){
        create3DPanoramaVertexArray(positionArray,weightArray,unMergeTextArray,mergeTextArray1,mergeTextArray2,indexArray);
        VertexArrayIO::saveMergeVertexArray(positionArray + 4,weightArray,mergeTextArray1,mergeTextArray2,indexArray + 4);
        VertexArrayIO::saveUnmergeVertexArray(positionArray,unMergeTextArray,indexArray);
    }else{
        VertexArrayIO::loadMergeVertexArray(positionArray + 4,weightArray,mergeTextArray1,mergeTextArray2,indexArray + 4);
        VertexArrayIO::loadUnmergeVertexArray(positionArray,unMergeTextArray,indexArray);
    }
    for(int i = 0; i < 4; i++)
    {
        m_mergedPanorama3D[i].createVertex(positionArray[i + 4],weightArray[i],mergeTextArray1[i],mergeTextArray2[i],indexArray[i + 4]);
        m_unmergedPanorama3D[i].createVertex(positionArray[i],unMergeTextArray[i],indexArray[i]);
    }
    float cameraHeight = g_3dParam.m_cameraHeight / scale;
    float observerHeight = g_3dParam.m_observerHeight / scale;
    m_trackBall = new TrackBall(getScreenWidth(), getScreenHeight(),observerHeight,cameraHeight);
    m_trackBall->setCurViewByAngle(phei,theta);
    m_endPhei = phei;
    m_endTheta = theta;
    //printf("m_endPhei = %f,m_endTheta = %f\n",phei * 180 / const_value::PI,theta * 180 / const_value::PI);
    m_isLoadVetex = true;
}
Panorama3DScene &getPanorama3DScene()
{
    static Panorama3DScene panorama3DScene;
    return panorama3DScene;
}
