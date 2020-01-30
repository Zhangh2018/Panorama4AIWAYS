#include "ModelRender.h"
#include "ShaderUtil.h"
#include "MatrixState.h"
#include <math.h>
#include <stack>
ModelRender::ModelRender(void)
{
}
ModelRender::~ModelRender(void)
{
}
void ModelRender::initRender()
{
	loadShaders("/data/opengl_new/shader/model.vert","/data/opengl_new/shader/model.frag",m_shaderProg);
	m_vertexLoc.m_positionLoc = glGetAttribLocation(m_shaderProg , "aPosition");
	m_vertexLoc.m_normalLoc = glGetAttribLocation(m_shaderProg , "aNormal");

	m_matrixLoc.m_finalMatrixLoc = glGetUniformLocation(m_shaderProg, "uMVPMatrix");
	m_matrixLoc.m_worldMatrixLoc = glGetUniformLocation(m_shaderProg, "uMMatirx");
	m_matrixLoc.m_cameraPositionLoc = glGetUniformLocation(m_shaderProg,"uCamera");

	m_materialLoc.m_ambientColorLoc = glGetUniformLocation(m_shaderProg, "uAmbientColor");
	m_materialLoc.m_diffuseColorLoc = glGetUniformLocation(m_shaderProg, "uDiffuseColor");
	m_materialLoc.m_emissiveColorLoc = glGetUniformLocation(m_shaderProg, "uEmissiveColor");
	m_materialLoc.m_specularColorLoc = glGetUniformLocation(m_shaderProg, "uSpecularColor");
	m_materialLoc.m_shinessLoc = glGetUniformLocation(m_shaderProg, "uShiness");
	m_materialLoc.m_opacityLoc = glGetUniformLocation(m_shaderProg, "uOpacity");

	m_lightLoc.m_positionLoc = glGetUniformLocation(m_shaderProg, "uLightState.position");
	m_lightLoc.m_ambientColorLoc = glGetUniformLocation(m_shaderProg, "uLightState.ambientColor");
	m_lightLoc.m_diffuseColorLoc = glGetUniformLocation(m_shaderProg, "uLightState.diffuseColor");
	m_lightLoc.m_specularColorLoc = glGetUniformLocation(m_shaderProg, "uLightState.specularColor");
}
void ModelRender::drawModel(const Model3D &model3D)
{
    glEnable( GL_CULL_FACE );
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glUseProgram(m_shaderProg);
	float scalar[3] = {1,1,1};
	float translator[3] = {0,0,0};
	model3D.normalizeModel(scalar,translator);
	/***************************¹âÕÕ*******************/
	GLfloat Ambient[4] = { 0.6, 0.6, 0.6, 0.6 };
	GLfloat Diffuse[4] = { 0.7, 0.7, 0.7, 1.0 };
	GLfloat Position[4] = { 0.0, 1.0, 0.0, 0.0 };
	GLfloat Specular[4] = {0.8,0.8,0.8,1.0};
	glUniform4fv( m_lightLoc.m_ambientColorLoc, 1, Ambient);
	glUniform4fv( m_lightLoc.m_diffuseColorLoc, 1, Diffuse);
	glUniform4fv( m_lightLoc.m_positionLoc, 1, Position);
	glUniform4fv( m_lightLoc.m_specularColorLoc, 1, Specular);
	MatrixState::applyScale(scalar[0],scalar[1],scalar[2]);
	MatrixState::applyTranslate(translator[0], translator[1], translator[2]);
	glUniform3fv(m_matrixLoc.m_cameraPositionLoc,1,MatrixState::getCameraPosition());
	recursive_render(model3D.getScene()->mRootNode,model3D);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
}
void ModelRender::recursive_render(const aiNode* nd,const Model3D &model3D)
{
	MatrixState::pushMatrix();
	aiMatrix4x4 m = nd->mTransformation;
	m.Transpose();
	MatrixState::applyMatrix(MyMatrix(m[0]));
	glUniformMatrix4fv(m_matrixLoc.m_finalMatrixLoc, 1, 0, MatrixState::getFinalMatrix());
	glUniformMatrix4fv(m_matrixLoc.m_worldMatrixLoc, 1, 0, MatrixState::getCurMatrix());
	for (unsigned int n=0; n < nd->mNumMeshes; ++n){

		glUniform4fv( m_materialLoc.m_emissiveColorLoc, 1, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.emissive);
		glUniform4fv( m_materialLoc.m_diffuseColorLoc, 1, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.diffuse);
		glUniform4fv( m_materialLoc.m_ambientColorLoc, 1, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.ambient);
		glUniform4fv( m_materialLoc.m_specularColorLoc, 1, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.specular);
		glUniform1f( m_materialLoc.m_shinessLoc, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.shininess);
		glUniform1f( m_materialLoc.m_opacityLoc, model3D.m_myMeshes[nd->mMeshes[n]].matInfo.opacity);

		// bind VBOs
		glBindBuffer(GL_ARRAY_BUFFER, model3D.m_myMeshes[nd->mMeshes[n]].VBO[VBO_vertices]);
		glVertexAttribPointer(m_vertexLoc.m_positionLoc, 3, GL_FLOAT, 0, 0, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model3D.m_myMeshes[nd->mMeshes[n]].VBO[VBO_faces]);
		glEnableVertexAttribArray(m_vertexLoc.m_positionLoc);

		glBindBuffer(GL_ARRAY_BUFFER, model3D.m_myMeshes[nd->mMeshes[n]].VBO[VBO_normals]);
		glVertexAttribPointer(m_vertexLoc.m_normalLoc, 3, GL_FLOAT, 0, 0, 0);
		glEnableVertexAttribArray(m_vertexLoc.m_normalLoc);

		glDrawElements(GL_TRIANGLES, model3D.m_myMeshes[nd->mMeshes[n]].numFaces*3, GL_UNSIGNED_SHORT, 0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		glDisableVertexAttribArray(m_vertexLoc.m_positionLoc);
		glDisableVertexAttribArray(m_vertexLoc.m_normalLoc);
	}
	for (unsigned int n=0; n < nd->mNumChildren; ++n){
		recursive_render(nd->mChildren[n],model3D);
	}
	MatrixState::popMatrix();
}
