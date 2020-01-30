#include <cstdio>

#include "LUT.h"

using namespace panorama;
bool CalibLUT::load(const char* filename)
{
	bool ret = true;
	FILE* fp = fopen(filename, "rb");
	if (fp != nullptr)
	{
		if (fread(&header, sizeof(LUTHeader), 1, fp) != 1)
		{
			ret = false;
			goto End;
		}
		int tableSize = header.bev_img_width * header.bev_img_height;
		table = new LUTPoint[tableSize];
		if (fread(table, sizeof(LUTPoint), tableSize, fp) != tableSize)
		{
			ret = false;
			goto End;
		}
	}
End:
	if (fp != nullptr)
	{
		fclose(fp);
	}
	return ret;
}

bool CalibLUT::save(const char* filename)
{
	auto ret = true;
	FILE* fp = fopen(filename, "wb");
	if (fp != nullptr)
	{
		if (fwrite(&header, sizeof(header), 1, fp) != 1)
		{
			ret = false;
			goto End;
		}
		if (table != nullptr)
		{
			int tableSize = header.bev_img_height * header.bev_img_width;
			if (fwrite(table, sizeof(LUTPoint), tableSize, fp) != tableSize)
			{
				ret = false;
				goto End;
			}
		}
	}

End:
	if (fp != nullptr)
	{
		fclose(fp);
	}
	return ret;
}

LUTPoint& CalibLUT::operator()(int y, int x)
{
	return table[y  * header.bev_img_width + x];
}

const LUTPoint& CalibLUT::operator()(int y, int x) const
{
	return table[y * header.bev_img_width + x];
}

cv::Mat_<LUTPoint> CalibLUT::operator()(const cv::Rect& roi) const
{
	cv::Mat_<LUTPoint> mat(header.bev_img_height, header.bev_img_width, table);
	return mat(roi);
}
