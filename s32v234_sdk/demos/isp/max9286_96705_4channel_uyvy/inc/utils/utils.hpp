#pragma once

#include <string>
#include <map>

static std::string cvtype2str(unsigned int cvtype)
{
	static std::map<unsigned int, std::string> cvtype2strTable = {
		{CV_8UC1,  "CV_8UC1"},
		{CV_8UC2,  "CV_8UC2"},
		{CV_8UC3,  "CV_8UC3"},
		{CV_8UC4,  "CV_8UC4"},


		{CV_8SC1,  "CV_8SC1"},
		{CV_8SC2,  "CV_8SC2"},
		{CV_8SC3,  "CV_8SC3"},
		{CV_8SC4,  "CV_8SC4"},


		{CV_16UC1, "CV_16UC1"},
		{CV_16UC2, "CV_16UC2"},
		{CV_16UC3, "CV_16UC3"},
		{CV_16UC4, "CV_16UC4"},


		{CV_16SC1, "CV_16SC1"},
		{CV_16SC2, "CV_16SC2"},
		{CV_16SC3, "CV_16SC3"},
		{CV_16SC4, "CV_16SC4"},


		{CV_32SC1, "CV_32SC1"},
		{CV_32SC2, "CV_32SC2"},
		{CV_32SC3, "CV_32SC3"},
		{CV_32SC4, "CV_32SC4"},


		{CV_32FC1, "CV_32FC1"},
		{CV_32FC2, "CV_32FC2"},
		{CV_32FC3, "CV_32FC3"},
		{CV_32FC4, "CV_32FC4"},


		{CV_64FC1, "CV_64FC1"},
		{CV_64FC2, "CV_64FC2"},
		{CV_64FC3, "CV_64FC3"},
		{CV_64FC4, "CV_64FC4"}
	};

	if (cvtype2strTable.find(cvtype) == cvtype2strTable.end())
	{
		return "null";
	}
	return cvtype2strTable[cvtype];
}

template<class T>
static inline T clamp(T v, T min, T max)
{
	return (v < min ? min : (v > max ? max : v));
}