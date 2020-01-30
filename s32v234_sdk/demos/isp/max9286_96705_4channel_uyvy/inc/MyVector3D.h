#ifndef _MY_VECTOR_H__
#define _MY_VECTOR_H__
#include <GLES2/gl2.h>
class MyVector3D
{
public:
	typedef GLfloat DataType;
	~MyVector3D(void){}
	MyVector3D () : x(0.0f), y(0.0f), z(0.0f) {}
	MyVector3D (DataType _x, DataType _y, DataType _z) : x(_x), y(_y), z(_z) {}
	MyVector3D (DataType _xyz) : x(_xyz), y(_xyz), z(_xyz) {}
	MyVector3D (const MyVector3D& o) : x(o.x), y(o.y), z(o.z) {}
public:
	const MyVector3D& operator += (const MyVector3D& o);
	const MyVector3D& operator -= (const MyVector3D& o);
	const MyVector3D& operator *= (DataType f);
	const MyVector3D& operator /= (DataType f);
	DataType operator[](unsigned int i) const;
	DataType& operator[](unsigned int i);
	bool operator== (const MyVector3D& other) const;
	bool operator!= (const MyVector3D& other) const;
public:
	void Set( DataType pX, DataType pY, DataType pZ = 0.f);
	DataType SquareLength() const;
	DataType Length() const;
	MyVector3D& Normalize();
public:
	DataType x,y,z;
};
MyVector3D operator + (const MyVector3D& v1, const MyVector3D& v2);
MyVector3D operator - (const MyVector3D& v1, const MyVector3D& v2);
MyVector3D::DataType operator * (const MyVector3D& v1, const MyVector3D& v2);
MyVector3D operator * ( MyVector3D::DataType f, const MyVector3D& v);
MyVector3D operator * ( const MyVector3D& v, MyVector3D::DataType f);
MyVector3D operator / ( const MyVector3D& v, MyVector3D::DataType f);
MyVector3D operator / ( const MyVector3D& v, const MyVector3D& v2);
MyVector3D operator ^(const MyVector3D& v1, const MyVector3D& v2);
MyVector3D operator - ( const MyVector3D& v);
#endif // _MY_VECTOR_H__
