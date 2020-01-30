#ifndef _IMAGE_UTIL_STRUCT_H__
#define _IMAGE_UTIL_STRUCT_H__
template <typename T>
struct ImageSize
{
	T width;
	T height;
	ImageSize(T width = 0,T height = 0)
	{
		this->width = width;
		this->height = height;
	}
	ImageSize (const ImageSize<T>& curSize)
	{
		this->width = curSize.width;
		this->height = curSize.height;
	}
	ImageSize &operator= (const ImageSize<T>& curSize)
	{
		this->width = curSize.width;
		this->height = curSize.height;
		return *this;
	}
};
template <typename T>
struct ImagePoint
{
	T x;
	T y;
	ImagePoint(T x = 0,T y = 0)
	{
		this->x = x;
		this->y = y;
	}
	ImagePoint(const ImagePoint<T>& curPoint)
	{
		this->x = curPoint.x;
		this->y = curPoint.y;
	}
	ImagePoint &operator= (const ImagePoint<T>& curPoint)
	{
		this->x = curPoint.x;
		this->y = curPoint.y;
		return *this;
	}
};
template <typename T>
struct ImageRect
{
	T x,y;
	T width,height;
	ImageRect(T x = 0,T y = 0,T width = 0,T height = 0)
	{
		this->x = x;
		this->y = y;
		this->width = width;
		this->height = height;
	}
	ImageRect(const ImageRect<T>& curRect)
	{
		this->x = curRect.x;
		this->y = curRect.y;
		this->width = curRect.width;
		this->height = curRect.height;
	}
	ImageRect  &operator= (const ImageRect<T>& curRect)
	{
		this->x = curRect.x;
		this->y = curRect.y;
		this->width = curRect.width;
		this->height = curRect.height;
		return *this;
	}
	int left(){return x;}
	int top(){return y;}
	int right(){return x + width - 1;}
	int bottom(){return y + height - 1;}
};
template <typename T>
struct ImageMergeLine
{
	T frontLeft;
	T frontRight;
	T rearLeft;
	T rearRight;
	T leftFront;
	T leftRear;
	T rightFront;
	T rightRear;
};
template <typename T>
struct ImageMergePoint
{
	ImagePoint<T> frontLeft;
	ImagePoint<T> frontRight;
	ImagePoint<T> rearLeft;
	ImagePoint<T> rearRight;
	ImagePoint<T> leftFront;
	ImagePoint<T> leftRear;
	ImagePoint<T> rightFront;
	ImagePoint<T> rightRear;
};
#endif
