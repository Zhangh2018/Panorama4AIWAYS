#pragma once
#include <vector>
class VertexArrayIO
{
public:
	static void saveSingleMergeVertexArray(const char *filename,const std::vector<float> &positionArray,const std::vector<float> &weightArray,const std::vector<float> &mergeTextArray1,const std::vector<float> &mergeTextArray2,const std::vector<unsigned short> &indexArray);
	static void saveMergeVertexArray(const std::vector<float> positionArray[],const std::vector<float> weightArray[],const std::vector<float> mergeTextArray1[],const std::vector<float> mergeTextArray2[],const std::vector<unsigned short> indexArray[]);
	static void saveSingleUnmergeVertexArray(const char *filename,const std::vector<float> &positionArray,const std::vector<float> &unmergeTextArray,const std::vector<unsigned short> &indexArray);
	static void saveUnmergeVertexArray(const std::vector<float> positionArray[],const std::vector<float> unmergeTextArray[],const std::vector<unsigned short> indexArray[]);

	static void loadSingleMergeVertexArray(const char *filename,std::vector<float> &positionArray,std::vector<float> &weightArray,std::vector<float> &mergeTextArray1,std::vector<float> &mergeTextArray2,std::vector<unsigned short> &indexArray);
	static void loadSingleUnmergeVertexArray(const char *filename,std::vector<float> &positionArray,std::vector<float> &unmergeTextArray,std::vector<unsigned short> &indexArray);
	static void loadMergeVertexArray(std::vector<float> positionArray[],std::vector<float> weightArray[],std::vector<float> mergeTextArray1[],std::vector<float> mergeTextArray2[],std::vector<unsigned short> indexArray[]);
	static void loadUnmergeVertexArray(std::vector<float> positionArray[],std::vector<float> unmergeTextArray[],std::vector<unsigned short> indexArray[]);
	static void generateLutStatisLut(const char *filename,std::vector<float> unmergeTextArray[],std::vector<float> mergeTextArray1[],std::vector<float> mergeTextArray2[],std::vector<float> positionArray[]);
};

