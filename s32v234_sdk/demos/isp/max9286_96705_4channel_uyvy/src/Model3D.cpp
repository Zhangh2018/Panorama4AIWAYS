#include "Model3D.h"
#include "IL/il.h"
Model3D::Model3D(){
}

void Model3D::set_float4(float f[4], float a, float b, float c, float d)
{
	f[0] = a;
	f[1] = b;
	f[2] = c;
	f[3] = d;
}

void Model3D::color4_to_float4(const aiColor4D *c, float f[4])
{
	f[0] = c->r;
	f[1] = c->g;
	f[2] = c->b;
	f[3] = c->a;
}

void Model3D::normalizeModel(float scale[],float translate[]) const
{
	scale[0] = 1.f / (m_sceneMax.x - m_sceneMin.x);
	scale[1] = 1.f / (m_sceneMax.y - m_sceneMin.y);
	scale[2] = 1.f / (m_sceneMax.z - m_sceneMin.z);
	translate[0] = -m_sceneCenter.x;
	translate[1] = -m_sceneCenter.y;
	translate[2] = -m_sceneCenter.z;
}
void Model3D::loadModel(const char *filename)
{
	loadAsset(filename);
	aiNode *rootNode = m_scene->mRootNode;
	aiMatrix4x4 itentityM = aiMatrix4x4(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
	getBoundingBoxForNode(rootNode,&m_sceneMin,&m_sceneMax,&itentityM);
	m_sceneCenter.x = (m_sceneMin.x + m_sceneMax.x) / 2.0f;
	m_sceneCenter.y = (m_sceneMin.y + m_sceneMax.y) / 2.0f;
	m_sceneCenter.z = (m_sceneMin.z + m_sceneMax.z) / 2.0f;
}
/***************************************************************************************
* loads asset
***************************************************************************************/
void Model3D::loadAsset(const char * path)
{
	m_scene = aiImportFile(path,aiProcessPreset_TargetRealtime_Quality);
	if (m_scene)
	{
		struct MyMesh aMesh;
		struct MyMaterial aMat;
		unsigned short shortFaces[3];
		for (unsigned int n = 0; n < m_scene->mNumMeshes; ++n)
		{
			const aiMesh* mesh = m_scene->mMeshes[n];
			unsigned short *faceArray;
			faceArray = (unsigned short *)malloc(sizeof(unsigned short) * mesh->mNumFaces * 3);
			unsigned int faceIndex = 0;
			for (unsigned int t = 0; t < mesh->mNumFaces; ++t){
				const aiFace* face = &mesh->mFaces[t];
				shortFaces[0] = (unsigned short)face->mIndices[0];
				shortFaces[1] = (unsigned short)face->mIndices[1];
				shortFaces[2] = (unsigned short)face->mIndices[2];
				memcpy(&faceArray[faceIndex], shortFaces, 3 * sizeof(unsigned short));
				faceIndex += 3;
			}
			aMesh.numFaces = m_scene->mMeshes[n]->mNumFaces;

			// buffer for faces
			glGenBuffers(1, &aMesh.VBO[VBO_faces]);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, aMesh.VBO[VBO_faces]);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * mesh->mNumFaces * 3, faceArray, GL_STATIC_DRAW);
			free(faceArray);

			// buffer for vertex positions
			if (mesh->HasPositions()) {
				glGenBuffers(1, &aMesh.VBO[VBO_vertices]);
				glBindBuffer(GL_ARRAY_BUFFER, aMesh.VBO[VBO_vertices]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*mesh->mNumVertices, mesh->mVertices, GL_STATIC_DRAW);

			}

			// buffer for vertex normals
			if (mesh->HasNormals()) {
				glGenBuffers(1, &aMesh.VBO[VBO_normals]);
				glBindBuffer(GL_ARRAY_BUFFER, aMesh.VBO[VBO_normals]);
				glBufferData(GL_ARRAY_BUFFER, sizeof(float)*3*mesh->mNumVertices, mesh->mNormals, GL_STATIC_DRAW);

			}
			// buffer for vertex texture coordinates
			glBindBuffer(GL_ARRAY_BUFFER,0);
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
			aiMaterial *mtl = m_scene->mMaterials[mesh->mMaterialIndex];
			//导入材料漫反射光分量
			set_float4(aMat.diffuse, 0.8f, 0.8f, 0.8f, 1.0f);
			aiColor4D diffuse;
			if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) //(color * intensity) = diffuse
				color4_to_float4(&diffuse, aMat.diffuse);
			//导入材料环境光分量
			set_float4(aMat.ambient, 0.2f, 0.2f, 0.2f, 1.0f);
			aiColor4D ambient;
			if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_AMBIENT, &ambient))
			{
				if(fabs(ambient[0]) > 0.0001 || fabs(ambient[1]) > 0.0001 || fabs(ambient[2]) > 0.0001)
					color4_to_float4(&ambient, aMat.ambient);
				else
					color4_to_float4(&diffuse, aMat.ambient);
			}
			//导入材料镜面反射分量
			set_float4(aMat.specular, 0.0f, 0.0f, 0.0f, 1.0f);
			aiColor4D specular;
			if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_SPECULAR, &specular))
				color4_to_float4(&specular, aMat.specular);
			//导入材料自身光照分量
			set_float4(aMat.emissive, 0.0f, 0.0f, 0.0f, 1.0f);
			aiColor4D emission;
			if(AI_SUCCESS == aiGetMaterialColor(mtl, AI_MATKEY_COLOR_EMISSIVE, &emission))
				color4_to_float4(&emission, aMat.emissive);

			aMat.shininess = 0.0;
			unsigned int max;
			aiGetMaterialFloatArray(mtl, AI_MATKEY_SHININESS, &aMat.shininess, &max);

			aMat.opacity = 0.0;
			aiGetMaterialFloat(mtl, AI_MATKEY_OPACITY, &aMat.opacity);

			aMesh.matInfo = aMat;
			m_myMeshes.push_back(aMesh);
		}
	}
	else
	{
		printf ("could not load model. misstype?\n");
	}
}
void Model3D::getBoundingBoxForNode(const aiNode* nd, aiVector3D* minVal, aiVector3D* maxVal,aiMatrix4x4* trafo)
{
		aiMatrix4x4 prev;
		unsigned int n = 0, t;
		prev = *trafo;
		aiMultiplyMatrix4(trafo,&nd->mTransformation);
		for (; n < nd->mNumMeshes; ++n) {
			const aiMesh* mesh = m_scene->mMeshes[nd->mMeshes[n]];
			for (t = 0; t < mesh->mNumVertices; ++t) {

				aiVector3D tmp = mesh->mVertices[t];
				aiTransformVecByMatrix4(&tmp,trafo);
				minVal->x = std::min(minVal->x,tmp.x);
				minVal->y = std::min(minVal->y,tmp.y);
				minVal->z = std::min(minVal->z,tmp.z);
				maxVal->x = std::max(maxVal->x,tmp.x);
				maxVal->y = std::max(maxVal->y,tmp.y);
				maxVal->z = std::max(maxVal->z,tmp.z);
			}
		}
		for (n = 0; n < nd->mNumChildren; ++n) {
			getBoundingBoxForNode(nd->mChildren[n],minVal,maxVal,trafo);
		}
		*trafo = prev;
}
