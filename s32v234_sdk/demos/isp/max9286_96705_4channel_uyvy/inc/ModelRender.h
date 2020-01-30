#ifndef _MODEL_RENDER_H__
#define _MODEL_RENDER_H__
#include "Model3D.h"
#include "UniformLocation.h"
class ModelRender
{
public:
	ModelRender(void);
	~ModelRender(void);
public:
	void initRender();
	void drawModel(const Model3D &model3D);
	void recursive_render(const aiNode* nd,const Model3D &model3D);
private:
	GLuint m_shaderProg;                 //×ÅÉ«Æ÷
	MatrialLoc m_materialLoc;
	MatrixLoc  m_matrixLoc;
	SamplerLoc m_samplerLoc;
	LightLoc   m_lightLoc;
	VertexLoc  m_vertexLoc;
};
#endif // _MODEL_RENDER_H__
