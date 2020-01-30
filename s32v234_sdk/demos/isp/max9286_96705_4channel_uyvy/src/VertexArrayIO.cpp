#include "VertexArrayIO.h"
#include <stdio.h>
#include "const_value.h"
using namespace const_value;
void VertexArrayIO::saveSingleMergeVertexArray(const char *filename,const std::vector<float> &positionArray,const std::vector<float> &weightArray,const std::vector<float> &mergeTextArray1,const std::vector<float> &mergeTextArray2,const std::vector<unsigned short> &indexArray)
{
    FILE *ofp = fopen(filename,"w");
    fprintf(ofp,"vertex amount: %d\n",positionArray.size() / 3);

    for(int i = 0; i < positionArray.size() / 3; i++)
    {
        fprintf(ofp,"%8.3f\t%8.3f\t%8.3f\t%5.3f\t%5.3f\t%5.3f\t%5.3f\t%5.3f\n",positionArray[3 * i],positionArray[3 * i + 1],positionArray[3 * i + 2],weightArray[i],
                mergeTextArray1[2 * i],mergeTextArray1[2 * i + 1],mergeTextArray2[2 * i],mergeTextArray2[2 * i + 1]);
    }
    fprintf(ofp,"\n");

    fprintf(ofp,"index amount: %d\n",indexArray.size());
    for(int i = 0; i < indexArray.size() / 3; i++)
    {
        fprintf(ofp,"%d\t%d\t%d\n",indexArray[3 * i],indexArray[3 * i + 1],indexArray[3 * i + 2]);
    }
    fprintf(ofp,"\n");
    fclose(ofp);
}
void VertexArrayIO::saveSingleUnmergeVertexArray(const char *filename,const std::vector<float> &positionArray,const std::vector<float> &unmergeTextArray,const std::vector<unsigned short> &indexArray)
{
    FILE *ofp = fopen(filename,"w");
    fprintf(ofp,"vertex amount: %d\n",positionArray.size() / 3);

    for(int i = 0; i < positionArray.size() / 3; i++)
    {
        fprintf(ofp,"%8.3f\t%8.3f\t%8.3f\t%5.3f\t%5.3f\n",positionArray[3 * i],positionArray[3 * i + 1],positionArray[3 * i + 2],unmergeTextArray[2 * i],unmergeTextArray[2 * i + 1]);
    }
    fprintf(ofp,"\n");

    fprintf(ofp,"index amount: %d\n",indexArray.size());
    for(int i = 0; i < indexArray.size() / 3; i++)
    {
        fprintf(ofp,"%d\t%d\t%d\n",indexArray[3 * i],indexArray[3 * i + 1],indexArray[3 * i + 2]);
    }
    fprintf(ofp,"\n");
    fclose(ofp);
}

void savePosVertex2XYZ(const char* filename, const std::vector<float>& positionArray)
{
	FILE* fp = fopen(filename, "w");
	for (int i = 0; i < positionArray.size() / 3; ++i)
	{
		fprintf(fp, "%8.3f\t%8.3f\t%8.3f\n", positionArray[3 * i], positionArray[3 * i + 1], positionArray[3 * i + 2]);
	}
	fclose(fp);
}

void savePosVertex2XYZ(const std::vector<float>& positionArray)
{
	static FILE* fp = fopen("/data/opengl_new/param/position_vertex.xyz", "w");
	static int counter = 0;

	if (fp == nullptr)
	{
		return;
	}

	for (int i = 0; i < positionArray.size() / 3; ++i)
	{
		fprintf(fp, "%8.3f\t%8.3f\t%8.3f\n", positionArray[3 * i], positionArray[3 * i + 1], positionArray[3 * i + 2]);
	}

	counter++;

	printf("counter = %d\n", counter);
	if (counter >= 8)
	{
		counter = 0;
		if (fp != nullptr)
		{
			fclose(fp);
		}
	}
}

void saveTexVertex2XY(const std::vector<float>& texVertex, const char* filename, int tag)
{
	FILE* fp = fopen(filename, "a+");
	if (fp == NULL)
	{
		return;
	}
	for (int i = 0; i < texVertex.size() - 1; i += 2)
	{
		fprintf(fp, "%8.3f\t%8.3f\t%d\n", texVertex[i], texVertex[i + 1], tag);
	}
	fclose(fp);
}

static const char* texXYFilenames[] = {"/data/opengl_new/front_tex_vertex.xy", "/data/opengl_new/back_tex_vertex.xy", "/data/opengl_new/left_tex_vertex.xy", "/data/opengl_new/right_tex_vertex.xy"};
static const char* mergedTexXYFilenames1[] = {"/data/opengl_new/front_tex_vertex.xy", "/data/opengl_new/back_tex_vertex.xy", "/data/opengl_new/left_tex_vertex.xy", "/data/opengl_new/right_tex_vertex.xy"};

void VertexArrayIO::saveMergeVertexArray(const std::vector<float> positionArray[],const std::vector<float> weightArray[],const std::vector<float> mergeTextArray1[],const std::vector<float> mergeTextArray2[],const std::vector<unsigned short> indexArray[])
{
    char *filename[4] = {"/data/opengl_new/param/front_left_vertex.txt","/data/opengl_new/param/front_right_vertex.txt","/data/opengl_new/param/rear_left_vertex.txt","/data/opengl_new/param/rear_right_vertex.txt"};
    int texIdx1[] = {0, 0, 1, 1};
    int texIdx2[] = {2, 3, 2, 3};
    for(int i = 0; i < 4; i++)
    {
        saveSingleMergeVertexArray(filename[i],positionArray[i],weightArray[i],mergeTextArray1[i],mergeTextArray2[i],indexArray[i]);
		//savePosVertex2XYZ(posFilename[i], positionArray[i]);
		savePosVertex2XYZ(positionArray[i]);
		saveTexVertex2XY(mergeTextArray1[i], texXYFilenames[texIdx1[i]], 1);
		saveTexVertex2XY(mergeTextArray2[i], texXYFilenames[texIdx2[i]], 1);

	}
//	saveTexVertex2XY(mergeTextArray1[0], texXYFilenames[0], 1);
//	saveTexVertex2XY(mergeTextArray1[1], texXYFilenames[0], 1);
}

void VertexArrayIO::saveUnmergeVertexArray(const std::vector<float> positionArray[],const std::vector<float> unmergeTextArray[],const std::vector<unsigned short> indexArray[])
{
    char *filename[4] = {"/data/opengl_new/param/front_vertex.txt","/data/opengl_new/param/rear_vertex.txt","/data/opengl_new/param/left_vertex.txt","/data/opengl_new/param/right_vertex.txt"};
	char *posFilename[4] = {"/data/opengl_new/param/front_pos_vertex.xyz","/data/opengl_new/param/rear_pos_vertex.xyz","/data/opengl_new/param/left_pos_vertex.xyz","/data/opengl_new/param/right_pos_vertex.xyz"};
    for(int i = 0; i < 4; i++)
    {
        saveSingleUnmergeVertexArray(filename[i],positionArray[i],unmergeTextArray[i],indexArray[i]);
		//savePosVertex2XYZ(posFilename[i], positionArray[i]);
		savePosVertex2XYZ(positionArray[i]);
		saveTexVertex2XY(unmergeTextArray[i], texXYFilenames[i], 0);
    }
}
void VertexArrayIO::loadSingleMergeVertexArray(const char *filename,std::vector<float> &positionArray,std::vector<float> &weightArray,std::vector<float> &mergeTextArray1,std::vector<float> &mergeTextArray2,std::vector<unsigned short> &indexArray)
{
    char tipStr[100];
    unsigned int vertexNum = 0;
    FILE *ifp = fopen(filename,"r");
    fscanf(ifp,"%s%s%u",tipStr,tipStr,&vertexNum);
    mergeTextArray1.resize(2 * vertexNum);
    mergeTextArray2.resize(2 * vertexNum);
    positionArray.resize(3 * vertexNum);
    weightArray.resize(vertexNum);
    for(int i = 0; i < vertexNum; i++)
    {
        fscanf(ifp,"%f%f%f%f%f%f%f%f",&positionArray[3 * i],&positionArray[3 * i + 1],&positionArray[3 * i + 2],&weightArray[i],
               &mergeTextArray1[2 * i],&mergeTextArray1[2 * i + 1],&mergeTextArray2[2 * i],&mergeTextArray2[2 * i + 1]);
    }
    unsigned int indexNum = 0;
    fscanf(ifp,"%s%s%u",tipStr,tipStr,&indexNum);
    indexArray.resize(indexNum);
    for(int i = 0; i < indexNum; i++)
    {
        fscanf(ifp,"%hu",&indexArray[i]);
    }
    fclose(ifp);
}
void VertexArrayIO::loadSingleUnmergeVertexArray(const char *filename,std::vector<float> &positionArray,std::vector<float> &unmergeTextArray,std::vector<unsigned short> &indexArray)
{
    char tipStr[100];
    unsigned int vertexNum = 0;
    FILE *ifp = fopen(filename,"r");
    fscanf(ifp,"%s%s%u",tipStr,tipStr,&vertexNum);
    unmergeTextArray.resize(2 * vertexNum);
    positionArray.resize(3 * vertexNum);
    for(int i = 0; i < vertexNum; i++)
    {
        fscanf(ifp,"%f%f%f%f%f",&positionArray[3 * i],&positionArray[3 * i + 1],&positionArray[3 * i + 2],&unmergeTextArray[2 * i],&unmergeTextArray[2 * i + 1]);
    }

    unsigned int indexNum = 0;
    fscanf(ifp,"%s%s%u",tipStr,tipStr,&indexNum);
    indexArray.resize(indexNum);
    for(int i = 0; i < indexNum; i++)
    {
        fscanf(ifp,"%hu",&indexArray[i]);
    }
    fclose(ifp);
}
void VertexArrayIO::loadMergeVertexArray(std::vector<float> positionArray[],std::vector<float> weightArray[],std::vector<float> mergeTextArray1[],std::vector<float> mergeTextArray2[],std::vector<unsigned short> indexArray[])
{
    char *filename[4] = {"/data/opengl_new/param/front_left_vertex.txt","/data/opengl_new/param/front_right_vertex.txt","/data/opengl_new/param/rear_left_vertex.txt","/data/opengl_new/param/rear_right_vertex.txt"};
    for(int i = 0; i < 4; i++)
    {
        loadSingleMergeVertexArray(filename[i],positionArray[i],weightArray[i],mergeTextArray1[i],mergeTextArray2[i],indexArray[i]);
    }

}
void VertexArrayIO::loadUnmergeVertexArray(std::vector<float> positionArray[],std::vector<float> unmergeTextArray[],std::vector<unsigned short> indexArray[])
{
    char *filename[4] = {"/data/opengl_new/param/front_vertex.txt","/data/opengl_new/param/rear_vertex.txt","/data/opengl_new/param/left_vertex.txt","/data/opengl_new/param/right_vertex.txt"};
    for(int i = 0; i < 4; i++)
    {
        loadSingleUnmergeVertexArray(filename[i],positionArray[i],unmergeTextArray[i],indexArray[i]);
    }
}
static float clamp(float realVal,float minVal,float maxVal)
{
	if(realVal < minVal)
	{
		return minVal;
	}
	if(realVal > maxVal)
	{
		return maxVal;
	}
	return realVal;
}
void getGroundPointTexture(const std::vector<float> &positionArray,const std::vector<float> &unmergeArray,std::vector<float> &groundTextArray)
{
    for(int i = 0;i < positionArray.size() / 3;i++)
    {
         if(positionArray[3 * i + 2] <= 0.001){
               groundTextArray.push_back(unmergeArray[2 * i]);
               groundTextArray.push_back(unmergeArray[2 * i + 1]);
         }
    }
}
void getGroundPointTexture(const std::vector<float> &positionArray,const std::vector<float> &mergeArray1,const std::vector<float> &mergeArray2,std::vector<float> &groundTextArray1,std::vector<float> &groundTextArray2)
{
    for(int i = 0;i < positionArray.size() / 3;i++)
    {
         if(positionArray[3 * i + 2] <= 0.001){
               groundTextArray1.push_back(mergeArray1[2 * i]);
               groundTextArray1.push_back(mergeArray1[2 * i + 1]);
               groundTextArray2.push_back(mergeArray2[2 * i]);
               groundTextArray2.push_back(mergeArray2[2 * i + 1]);
         }
    }
}
void VertexArrayIO::generateLutStatisLut(const char *filename,std::vector<float> unmergeTextArray[],std::vector<float> mergeTextArray1[],std::vector<float> mergeTextArray2[],std::vector<float> positionArray[])
{
    FILE *ofp = fopen(filename,"wb");
    char version[2] = {0,0};
    fwrite(version,2,sizeof(char),ofp);
    std::vector<float> groundTexArray[4],groundTexArray1[4],groundTexArray2[4];
    for(int i = 0;i < 4;i++)
    {
        getGroundPointTexture(positionArray[i],unmergeTextArray[i],groundTexArray[i]);
        getGroundPointTexture(positionArray[i + 4],mergeTextArray1[i],mergeTextArray2[i],groundTexArray1[i],groundTexArray2[i]);
    }
    for(int i = 0;i < 4;i++)
    {
         int vecSize = groundTexArray[i].size() / 2;
         fwrite(&vecSize,1,sizeof(int),ofp);
    }
    for(int i = 0;i < 4;i++)
    {
         int vecSize = groundTexArray1[i].size() / 2;
         fwrite(&vecSize,1,sizeof(int),ofp);
    }
    for(int i = 0;i < 4;i++)
    {
         std::vector<int> positionVec(groundTexArray[i].size());
         for(int j = 0;j < groundTexArray[i].size() / 2;j++)
         {
             float textCoordX = clamp(groundTexArray[i][2 * j],0,1);
             float textCoordY = 1 - clamp(groundTexArray[i][2 * j + 1],0,1);
             int x = int(textCoordX * (SRC_IMAGE_WIDTH - 1));
             int y = int(textCoordY * (SRC_IMAGE_HEIGHT - 1));
             positionVec[2 * j] = x;
             positionVec[2 * j + 1] = y;
         }
         fwrite(&positionVec[0],positionVec.size(),sizeof(int),ofp);
    }
    for(int i = 0;i < 4;i++)
    {
         std::vector<int> positionVec1(groundTexArray1[i].size());
         for(int j = 0;j < groundTexArray1[i].size() / 2;j++)
         {
             float textCoordX = clamp(groundTexArray1[i][2 * j],0,1);
             float textCoordY = 1 - clamp(groundTexArray1[i][2 * j + 1],0,1);
             int x = int(textCoordX * (SRC_IMAGE_WIDTH - 1));
             int y = int(textCoordY * (SRC_IMAGE_HEIGHT - 1));
             positionVec1[2 * j] = x;
             positionVec1[2 * j + 1] = y;
         }
         fwrite(&positionVec1[0],positionVec1.size(),sizeof(int),ofp);
         std::vector<int> positionVec2(groundTexArray2[i].size());
         for(int j = 0;j < groundTexArray2[i].size() / 2;j++)
         {
             float textCoordX = clamp(groundTexArray2[i][2 * j],0,1);
             float textCoordY = 1 - clamp(groundTexArray2[i][2 * j + 1],0,1);
             int x = int(textCoordX * (SRC_IMAGE_WIDTH - 1));
             int y = int(textCoordY * (SRC_IMAGE_HEIGHT - 1));
             positionVec2[2 * j] = x;
             positionVec2[2 * j + 1] = y;
         }
         fwrite(&positionVec2[0],positionVec2.size(),sizeof(int),ofp);
    }
    fclose(ofp);
}
