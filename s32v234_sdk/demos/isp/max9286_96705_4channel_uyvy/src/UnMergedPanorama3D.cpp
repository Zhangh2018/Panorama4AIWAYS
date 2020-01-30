#include "UnMergedPanorama3D.h"
#include "ShaderUtil.h"
#include <memory.h>
#include <stdio.h>
UnMergedPanorama3D::UnMergedPanorama3D(void)
{
    memset(m_localLuminParam,0,sizeof(m_localLuminParam));
    m_localLuminHandle = 0;
    m_compensationHandle = 0;
    m_luminParam = 0;

	m_projMatrixHandle = 0;
	m_vertexNum = 0;
	m_positionHandle = 0;
	m_textCoordHandle = 0;
	m_samplerHandle = 0;
	m_shaderProgram = 0;
	m_deltaHandle = 0;
	memset(m_vbo,0,sizeof(GLuint) * 3);
}
UnMergedPanorama3D::~UnMergedPanorama3D(void)
{
}
int UnMergedPanorama3D::createVertex(const std::vector<float> &vertexArray,const std::vector<float> &textArray,const std::vector<unsigned short> &indexArray)
{

    //printf("vertexArray.size = %d,textArray.size = %d,indexArray.size = %d\n",
	//      vertexArray.size(),textArray.size(),indexArray.size());
	glGenBuffers(3, m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertexArray.size(),&vertexArray[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * textArray.size(),&textArray[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * indexArray.size(), &indexArray[0], GL_STATIC_DRAW);
	m_vertexNum = indexArray.size();
	    //printf("m_localLuminParam createVertex\t%f\t%f\t%f\n",m_localLuminParam[0],m_localLuminParam[1],m_localLuminParam[2]);
	return 0;
}
bool UnMergedPanorama3D::initUnMergePanorama()
{

	if(loadShaders("/data/opengl_new/shader/unmerge3d.vert","/data/opengl_new/shader/unmerge3d.frag",m_shaderProgram) != 0)
		return false;
//属性参数
	m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
	m_textCoordHandle = glGetAttribLocation(m_shaderProgram, "aTexCoord");
//纹理采样参数
	m_samplerHandle = glGetUniformLocation(m_shaderProgram,"sTexture");
//观察视角参数
	m_projMatrixHandle = glGetUniformLocation(m_shaderProgram,"uProjMatrix");

	m_compensationHandle = glGetUniformLocation(m_shaderProgram,"uCompensation");
	m_localLuminHandle = glGetUniformLocation(m_shaderProgram,"uLocalLuminParam");

	m_deltaHandle = glGetUniformLocation(m_shaderProgram, "uDelta");
	if (m_deltaHandle == -1)
	{
		auto err = glGetError();
		printf("get uDelta location failed: %d (0x%x)\n", err, err);
	}
	//printf("m_localLuminParam initUnMergePanorama\t%f\t%f\t%f\n",m_localLuminParam[0],m_localLuminParam[1],m_localLuminParam[2]);
	return true;
}
void UnMergedPanorama3D::drawSelf(GLuint textureID,float *projMatrix, float delta)
{
    //printf("m_localLuminParam drawSelf\t%f\t%f\t%f\n",m_localLuminParam[0],m_localLuminParam[1],m_localLuminParam[2]);
	glUseProgram(m_shaderProgram);
//传递观察视角参数
	glUniformMatrix4fv(m_projMatrixHandle,1,GL_FALSE,projMatrix);
//global lumin param
	glUniform1f(m_compensationHandle, m_luminParam);
	glUniform3f(m_localLuminHandle,m_localLuminParam[0],m_localLuminParam[1],m_localLuminParam[2]);
//传递纹理采样图像
	glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glUniform1i(m_samplerHandle, 0);

	//printf("delta = %f, delta handle = %d\n", delta, m_deltaHandle);

	glUniform1f(m_deltaHandle, delta);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glVertexAttribPointer(m_textCoordHandle, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[2]);
	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_textCoordHandle);
	glDrawElements(GL_TRIANGLES, m_vertexNum, GL_UNSIGNED_SHORT, 0);
	//printf("test m_vertexNum:::%d\n",m_vertexNum);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_textCoordHandle);
}
