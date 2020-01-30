#ifndef _MERGED_PANORAMA3D_H__
#define _MERGED_PANORAMA3D_H__
#include <vector>
#include <GLES2/gl2.h>
#include <memory.h>
class MergedPanorama3D
{
public:
	MergedPanorama3D(void);
	~MergedPanorama3D(void);
	int createVertex(const std::vector<float> &vertexArray,const std::vector<float> &weightArray,const std::vector<float> &textArray1,const std::vector<float> &textArray2,const std::vector<unsigned short> &indexArray);
	bool initMergePanorama();
	void drawSelf(GLuint textureID1,GLuint textureID2,float *projMatrix, float delta1, float delta2);
	void setGlobalLuminDiff(float luminParam1,float luminParam2){m_luminParam1 = luminParam1;m_luminParam2 = luminParam2;}
	void setLocalLuminParam(float *luminParam){memcpy(m_localLuminParam,luminParam,sizeof(m_localLuminParam));}
private:
         float m_luminParam1;
         float m_luminParam2;
         GLuint m_compensation1Handle;
         GLuint m_compensation2Handle;
         float m_localLuminParam[3];
         GLuint m_localLuminHandle;

	int m_vertexNum;
	GLuint m_shaderProgram;
	GLuint m_positionHandle;      //顶点数组引用
	GLuint m_texCoord1Handle;     //纹理数组1引用
	GLuint m_texCoord2Handle;     //纹理数组1引用
	GLuint m_weightHandle;

	GLuint m_sampler1Handle;
	GLuint m_sampler2Handle;

	GLuint m_projMatrixHandle;

	GLuint m_delta1Handle;
	GLuint m_delta2Handle;

	GLuint m_vbo[5];
};
#endif // _MERGED_PANORAMA_H__
