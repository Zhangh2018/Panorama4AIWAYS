#include "MergedPanorama3D.h"
#include "ShaderUtil.h"
#include <memory.h>
#include <stdio.h>
MergedPanorama3D::MergedPanorama3D(void)
{

         m_luminParam1 = 0;
         m_luminParam2 = 0;
         m_compensation1Handle = 0;
         m_compensation2Handle = 0;
         memset(m_localLuminParam,0,sizeof(m_localLuminParam));
         m_localLuminHandle = 0;

	m_projMatrixHandle = 0;
	m_vertexNum = 0;
	m_positionHandle = 0;
	m_weightHandle = 0;
	m_texCoord1Handle = 0;
	m_texCoord2Handle = 0;

	m_sampler1Handle = 0;
	m_sampler2Handle = 0;

	m_delta1Handle = 0;
	m_delta2Handle = 0;

	m_shaderProgram = 0;
	memset(m_vbo,0,sizeof(GLuint) * 5);
}


MergedPanorama3D::~MergedPanorama3D(void)
{

}
int MergedPanorama3D::createVertex(const std::vector<float> &vertexArray,const std::vector<float> &weightArray,const std::vector<float> &textArray1,const std::vector<float> &textArray2,const std::vector<unsigned short> &indexArray)
{
	glGenBuffers(5, m_vbo);
	//printf("vertexArray.size = %d,weightArray.size = %d,textArray1.size = %d,textArray2.size = %d,indexArray.size = %d\n",
	//      vertexArray.size(),weightArray.size(),textArray1.size(),textArray2.size(),indexArray.size());
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertexArray.size(),&vertexArray[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * weightArray.size(),&weightArray[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[2]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * textArray1.size(),&textArray1[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[3]);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * textArray2.size(),&textArray2[0], GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[4]);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * indexArray.size(), &indexArray[0], GL_STATIC_DRAW);
	m_vertexNum = indexArray.size();
	return 0;
}
bool MergedPanorama3D::initMergePanorama()
{
	if(loadShaders("/data/opengl_new/shader/merge3d.vert","/data/opengl_new/shader/merge3d.frag",m_shaderProgram) != 0)
	{
		fprintf(stderr,"the merge shader compile error!\n");
		return false;
	}
//属性参数,分别为位置，距离和权重
	m_positionHandle = glGetAttribLocation(m_shaderProgram, "aPosition");
	m_weightHandle = glGetAttribLocation(m_shaderProgram,"aWeight");
	m_texCoord1Handle = glGetAttribLocation(m_shaderProgram, "aTexCoord1");
	m_texCoord2Handle = glGetAttribLocation(m_shaderProgram, "aTexCoord2");
//纹理采样参数
	m_sampler1Handle = glGetUniformLocation(m_shaderProgram,"sTexture1");
	m_sampler2Handle = glGetUniformLocation(m_shaderProgram,"sTexture2");

	m_compensation1Handle = glGetUniformLocation(m_shaderProgram,"uCompensation1");
	m_compensation2Handle = glGetUniformLocation(m_shaderProgram,"uCompensation2");
	m_localLuminHandle = glGetUniformLocation(m_shaderProgram,"uLocalLuminParam");
//观察视角参数
	m_projMatrixHandle = glGetUniformLocation(m_shaderProgram,"uProjMatrix");

	m_delta1Handle = glGetUniformLocation(m_shaderProgram, "uDelta1");
	m_delta2Handle = glGetUniformLocation(m_shaderProgram, "uDelta2");

	return true;
}
void MergedPanorama3D::drawSelf(GLuint textureID1,GLuint textureID2,float *projMatrix, float delta1, float delta2)
{
	glUseProgram(m_shaderProgram);
//传递观察视角参数
	glUniformMatrix4fv(m_projMatrixHandle,1,GL_FALSE,projMatrix);

	glUniform1f(m_compensation1Handle, m_luminParam1);
	glUniform1f(m_compensation2Handle, m_luminParam2);
         glUniform3f(m_localLuminHandle,m_localLuminParam[0],m_localLuminParam[1],m_localLuminParam[2]);
//传递纹理采样图像
	glActiveTexture (GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID1);
	glUniform1i(m_sampler1Handle, 0);
	glActiveTexture (GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, textureID2);
	glUniform1i(m_sampler2Handle, 1);

	//printf("delta1 = %f, delta2 = %f\n", delta1, delta2);
	glUniform1f(m_delta1Handle, delta1);
	glUniform1f(m_delta2Handle, delta2);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[0]);
	glVertexAttribPointer(m_positionHandle, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[1]);
	glVertexAttribPointer(m_weightHandle, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[2]);
	glVertexAttribPointer(m_texCoord1Handle, 2, GL_FLOAT, GL_FALSE, 0, 0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vbo[3]);
	glVertexAttribPointer(m_texCoord2Handle, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo[4]);
	glEnableVertexAttribArray(m_positionHandle);
	glEnableVertexAttribArray(m_weightHandle);
	glEnableVertexAttribArray(m_texCoord1Handle);
	glEnableVertexAttribArray(m_texCoord2Handle);
	glDrawElements(GL_TRIANGLES, m_vertexNum, GL_UNSIGNED_SHORT, 0);
	glDisableVertexAttribArray(m_positionHandle);
	glDisableVertexAttribArray(m_weightHandle);
	glDisableVertexAttribArray(m_texCoord1Handle);
	glDisableVertexAttribArray(m_texCoord2Handle);
}
