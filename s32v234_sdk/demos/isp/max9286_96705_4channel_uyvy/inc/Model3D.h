#ifndef _MODEL_3D_H__
#define _MODEL_3D_H__
//#include <ASSIMP/assimp.h>
//#include <ASSIMP/aiPostProcess.h>
//#include <ASSIMP/aiScene.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <vector>
#include <map>

#include <GLES2/gl2.h>
#include <EGL/egl.h>

#include <stdio.h>
struct MyMaterial{
	float diffuse[4];
	float ambient[4];
	float specular[4];
	float emissive[4];
	float shininess;
	float opacity;
};
struct MyMesh{
	GLuint VBO[4];
	MyMaterial matInfo;
	int numFaces;
	MyMesh()
	{
		memset(VBO,0,sizeof(GLuint) * 4);
		numFaces = 0;
	}
};
enum vboNames { VBO_faces,
				VBO_vertices,
				VBO_normals,
				VBO_texCoords
				};
class Model3D{
public:
	std::vector<struct MyMesh> m_myMeshes;
	std::map<std::string, GLuint> m_textureIdMap;
	const aiScene *m_scene;
public:
	Model3D();
	void loadModel(const char *filename);
	void loadAsset(const char * path);
	void normalizeModel(float scale[],float translate[]) const;
	const aiScene * getScene(void) const{return m_scene;};
	void set_float4(float f[4], float a, float b, float c, float d);
	void color4_to_float4(const aiColor4D *c, float f[4]);
	void getBoundingBoxForNode (const aiNode* nd,aiVector3D* minVal, aiVector3D* maxVal,aiMatrix4x4* trafo);
private:
	aiVector3D m_sceneMin;
	aiVector3D m_sceneMax;
	aiVector3D m_sceneCenter;
};
#endif
