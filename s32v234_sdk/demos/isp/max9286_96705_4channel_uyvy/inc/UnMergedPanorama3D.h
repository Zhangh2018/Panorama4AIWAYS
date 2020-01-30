#ifndef _UNMERGED_PANORAMA3D_H__
#define _UNMERGED_PANORAMA3D_H__
#include <GLES2/gl2.h>
#include <vector>
#include <memory.h>
class UnMergedPanorama3D
{
public:
	UnMergedPanorama3D(void);
	~UnMergedPanorama3D(void);
	 int createVertex(const std::vector<float> &vertexArray,const std::vector<float> &textArray,const std::vector<unsigned short> &indexArray);
	 bool initUnMergePanorama();
	 void drawSelf(GLuint textureID,float *projMatrix, float delta);
	 void setGlobalLuminDiff(float luminParam){m_luminParam = luminParam;}
	 void setLocalLuminParam(float *luminParam){memcpy(m_localLuminParam,luminParam,sizeof(m_localLuminParam));}
private:
         float m_luminParam;
	int m_vertexNum;
	GLuint m_compensationHandle;
	GLuint m_shaderProgram;
	GLuint m_positionHandle;     //顶点数组引用
	GLuint m_textCoordHandle;
	GLuint m_samplerHandle;
	GLuint m_projMatrixHandle;
	GLuint m_deltaHandle;

	float m_localLuminParam[3];
	GLuint m_localLuminHandle;
	GLuint m_vbo[3];
};

class UnMergedPanorama2D
{
public:
	UnMergedPanorama2D(void);
	~UnMergedPanorama2D(void);
	 int createVertex();
	 bool initUnMergePanorama();
	 void drawSelf(GLuint textureID,float *projMatrix);
	 void setGlobalLuminDiff(float luminParam){m_luminParam = luminParam;}
	 void setLocalLuminParam(float *luminParam){memcpy(m_localLuminParam,luminParam,sizeof(m_localLuminParam));}
private:
    float m_luminParam;
	int m_vertexNum;
	GLuint m_compensationHandle;
	GLuint m_shaderProgram;
	GLuint m_positionHandle;     //顶点数组引用
	GLuint m_textCoordHandle;
	GLuint m_samplerHandle;
	GLuint m_projMatrixHandle;
	float m_localLuminParam[3];
	GLuint m_localLuminHandle;
	GLuint m_vbo[3];
};

#endif // _UNMERGED_PANORAMA_H__
