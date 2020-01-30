#ifndef _UNIFORM_LOCATION_H__
#define _UNIFORM_LOCATION_H__
#include <GLES2/gl2.h>
struct Light
{
	bool  m_enableLight;                             //是否允许光照
	float m_position[4];                             //光源位置
	float m_ambientColor[4];                         //光源环境光分量 
	float m_diffuseColor[4];                         //光源漫反射光分量
	float m_specularColor[4];                        //光源镜面反射光分量  
	float m_spotDirection[3];                        //聚光灯照射方向 
	float m_attenuationFactors[3];                   //光强与距离关系参数
	float m_spotExponent;                            //光强随偏离中心角度的衰减系数                  
	float m_spotCutoffAngle;                         //聚光光源照射幅角           
	bool  m_computeDistanceAttenuation;              //是否计算点光源与物体距离   
	Light()
	{
		for(int i = 0;i < 4;i++)
		{
			m_position[i] = 0;
			m_ambientColor[i] = 0;
			m_diffuseColor[i] = 0;
			m_specularColor[i] = 0;
		}
		for(int i = 0;i < 3;i++)
		{
			m_spotDirection[i] = 0;
			m_attenuationFactors[i] = 0;
		}
		m_spotExponent = 0;
		m_spotCutoffAngle = 0;
		m_enableLight = false;
		m_computeDistanceAttenuation = true;
	}
};
struct LightLoc
{
	GLuint m_enableLightLoc;                          //是否允许光照
	GLuint m_positionLoc;                             //光源位置
	GLuint m_ambientColorLoc;                         //光源环境光分量  
	GLuint m_diffuseColorLoc;                         //光源漫反射光分量
	GLuint m_specularColorLoc;                        //光源镜面反射光分量  
	GLuint m_spotDirectionLoc;                        //聚光灯照射方向  
	GLuint m_attenuationFactorsLoc;                   //光强与距离关系参数
	GLuint m_spotExponentLoc;                         //光强随偏离中心角度的衰减系数 
	GLuint m_spotCutoffAngleLoc;                      //聚光光源照射幅角
	GLuint m_computeDistanceAttenuationLoc;           // 是否计算点光源与物体距离 
	LightLoc()
	{
		m_enableLightLoc = 0;
		m_positionLoc = 0;
		m_ambientColorLoc = 0;
		m_diffuseColorLoc = 0;
		m_specularColorLoc = 0;
		m_spotDirectionLoc = 0;
		m_attenuationFactorsLoc = 0;
		m_spotExponentLoc = 0;
		m_spotCutoffAngleLoc = 0;
		m_computeDistanceAttenuationLoc = 0;
	}
};
struct Matrial
{
	float m_ambientColor[4];                         //材质环境光分量 
	float m_diffuseColor[4];                         //材质漫反射光分量
	float m_specularColor[4];                        //材质镜面反射光分量 
	float m_emissiveColor[4];                        //材质自身光强
	float m_shiness;                                 //材质反光系数
	bool  m_enableColorMatrial;                      //是否设置材料环境光分量和漫反射分量等于当前顶点颜色
	Matrial()
	{
		for(int i = 0;i < 4;i++)
		{
			m_ambientColor[i] = 0;
			m_diffuseColor[i] = 0;
			m_specularColor[i] = 0;
			m_emissiveColor[i] = 0;
		}
		m_shiness = 0;
		m_enableColorMatrial = false;
	}

};
struct MatrialLoc
{
	GLuint m_ambientColorLoc;                         //材质环境光分量 
	GLuint m_diffuseColorLoc;                         //材质漫反射光分量
	GLuint m_specularColorLoc;                        //材质镜面反射光分量 
	GLuint m_emissiveColorLoc;                        //材质自身光强
	GLuint m_shinessLoc;                              //材质反光系数
	GLuint m_opacityLoc;
	GLuint m_enableColorMatrialLoc;                   //是否设置材料环境光分量和漫反射分量等于当前顶点颜色
	MatrialLoc()
	{
		m_ambientColorLoc = 0;
		m_diffuseColorLoc = 0;
		m_specularColorLoc = 0;
		m_shinessLoc = 0;
		m_enableColorMatrialLoc = 0;
		m_emissiveColorLoc = 0;
		m_opacityLoc = 0;
	}
};
struct VertexLoc
{
	GLuint m_positionLoc;                               //顶点数组
	GLuint m_normalLoc;                               //法向量数组
	GLuint m_colorLoc;                                //颜色数组
	GLuint m_textCoordLoc;                            //纹理坐标数组
	VertexLoc()
	{
		m_positionLoc = 0;
		m_normalLoc = 0;
		m_colorLoc = 0;
		m_textCoordLoc = 0;
	}
};
struct MatrixLoc
{
	GLuint m_cameraPositionLoc;                       //摄像机坐标
	GLuint m_finalMatrixLoc;                          //总变换矩阵
	GLuint m_worldMatrixLoc;                          //世界坐标系下模型矩阵
	GLuint m_cameraViewLoc;                           //相机坐标系下模型矩阵
	GLuint m_projMatrixLoc;                           //投影矩阵
	MatrixLoc()
	{
		m_projMatrixLoc = 0;
		m_cameraPositionLoc = 0;
		m_cameraViewLoc = 0;
		m_finalMatrixLoc = 0;
		m_worldMatrixLoc = 0;
	}
};
struct SamplerLoc
{
	static const int MAX_SAMPLER_NUM = 8;
	GLuint m_existTextureLoc;
	GLuint m_texSamplerLoc[MAX_SAMPLER_NUM];
	SamplerLoc()
	{
		for(int i = 0;i < MAX_SAMPLER_NUM;i++)
			m_texSamplerLoc[i] = 0;
	}
};
#endif