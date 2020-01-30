#include "MyMatrix.h"
#include <cmath>
#include <ostream>
#include "MyVector3D.h"
//构造函数和析构函数
MyMatrix::MyMatrix(void)
{
}
MyMatrix::MyMatrix(const DataType val)
{
	m0=m1=m2=m3=m4=m5=m6=m7=m8=m9=m10=m11=m12=m13=m14=m15=val;
}
MyMatrix::MyMatrix(const DataType* const val)
{
	memmove(m,val,16*sizeof(DataType));
}
MyMatrix::MyMatrix(const DataType* const v0, const DataType* const v1, const DataType* const v2, const DataType* const v3)
{
	memmove(m,v0,4*sizeof(DataType)); memmove(m+4,v1,4*sizeof(DataType));
	memmove(m+8,v2,4*sizeof(DataType)); memmove(m+12,v3,4*sizeof(DataType));
}
MyMatrix::MyMatrix(const MyMatrix& mat)
{
	memmove(m,mat.m,16*sizeof(DataType));
}
MyMatrix::~MyMatrix(void)
{
}

MyMatrix& MyMatrix::operator= (const MyMatrix& mat)
{
	memmove(m,mat.m,16*sizeof(DataType)); return *this;
}
bool MyMatrix::operator== (const MyMatrix& mat)
{
	return memcmp(m,mat.m,16*sizeof(DataType))==0;
}
MyMatrix::DataType& MyMatrix::operator[] (const int ind)
{
	assert(ind > -1 && ind < 16);
	return m[ind];
}
MyMatrix& MyMatrix::operator*= (const DataType& val)
{
	for(register unsigned i = 0; i < 16; ++i)
		m[i] *= val;
	return *this;
}
MyMatrix& MyMatrix::operator/= (const DataType& val)
{
	for(register unsigned i = 0; i < 16; ++i)
		m[i] /= val;
	return *this;
}
MyMatrix& MyMatrix::operator+= (const MyMatrix& mat)
{
	for(register unsigned i = 0; i < 16; ++i)
		m[i] += mat.m[i];
	return *this;
}
MyMatrix MyMatrix::operator+ (const MyMatrix& mat)
{
	MyMatrix ret(*this);
	ret += mat;
	return ret;
}
MyMatrix& MyMatrix::operator-= (const MyMatrix& mat)
{
	for(register unsigned i = 0; i < 16; ++i)
		m[i] -= mat.m[i];
	return *this;
}
MyMatrix MyMatrix::operator- (const MyMatrix& mat)
{
	MyMatrix ret(*this);
	ret -= mat;
	return ret;
}
MyMatrix MyMatrix::operator* (const MyMatrix& mat)
{
	MyMatrix ret;
	ret.m[0]    = m[0]*mat.m0  + m[0+4]*mat.m1  + m[0+8]*mat.m2  + m[0+12]*mat.m3;
	ret.m[1]    = m[1]*mat.m0  + m[1+4]*mat.m1  + m[1+8]*mat.m2  + m[1+12]*mat.m3;
	ret.m[2]    = m[2]*mat.m0  + m[2+4]*mat.m1  + m[2+8]*mat.m2  + m[2+12]*mat.m3;
	ret.m[3]    = m[3]*mat.m0  + m[3+4]*mat.m1  + m[3+8]*mat.m2  + m[3+12]*mat.m3;
	ret.m[0+4]  = m[0]*mat.m4  + m[0+4]*mat.m5  + m[0+8]*mat.m6  + m[0+12]*mat.m7;
	ret.m[1+4]  = m[1]*mat.m4  + m[1+4]*mat.m5  + m[1+8]*mat.m6  + m[1+12]*mat.m7;
	ret.m[2+4]  = m[2]*mat.m4  + m[2+4]*mat.m5  + m[2+8]*mat.m6  + m[2+12]*mat.m7;
	ret.m[3+4]  = m[3]*mat.m4  + m[3+4]*mat.m5  + m[3+8]*mat.m6  + m[3+12]*mat.m7;
	ret.m[0+8]  = m[0]*mat.m8  + m[0+4]*mat.m9  + m[0+8]*mat.m10 + m[0+12]*mat.m11;
	ret.m[1+8]  = m[1]*mat.m8  + m[1+4]*mat.m9  + m[1+8]*mat.m10 + m[1+12]*mat.m11;
	ret.m[2+8]  = m[2]*mat.m8  + m[2+4]*mat.m9  + m[2+8]*mat.m10 + m[2+12]*mat.m11;
	ret.m[3+8]  = m[3]*mat.m8  + m[3+4]*mat.m9  + m[3+8]*mat.m10 + m[3+12]*mat.m11;
	ret.m[0+12] = m[0]*mat.m12 + m[0+4]*mat.m13 + m[0+8]*mat.m14 + m[0+12]*mat.m15;
	ret.m[1+12] = m[1]*mat.m12 + m[1+4]*mat.m13 + m[1+8]*mat.m14 + m[1+12]*mat.m15;
	ret.m[2+12] = m[2]*mat.m12 + m[2+4]*mat.m13 + m[2+8]*mat.m14 + m[2+12]*mat.m15;
	ret.m[3+12] = m[3]*mat.m12 + m[3+4]*mat.m13 + m[3+8]*mat.m14 + m[3+12]*mat.m15;
	return ret;
}
MyMatrix& MyMatrix::operator*= (const MyMatrix& mat)
{
	MyMatrix temp(*this);
	m[0]    = temp.m[0]*mat.m0  + temp.m[0+4]*mat.m1  + temp.m[0+8]*mat.m2  + temp.m[0+12]*mat.m3;
	m[1]    = temp.m[1]*mat.m0  + temp.m[1+4]*mat.m1  + temp.m[1+8]*mat.m2  + temp.m[1+12]*mat.m3;
	m[2]    = temp.m[2]*mat.m0  + temp.m[2+4]*mat.m1  + temp.m[2+8]*mat.m2  + temp.m[2+12]*mat.m3;
	m[3]    = temp.m[3]*mat.m0  + temp.m[3+4]*mat.m1  + temp.m[3+8]*mat.m2  + temp.m[3+12]*mat.m3;
	m[0+4]  = temp.m[0]*mat.m4  + temp.m[0+4]*mat.m5  + temp.m[0+8]*mat.m6  + temp.m[0+12]*mat.m7;
	m[1+4]  = temp.m[1]*mat.m4  + temp.m[1+4]*mat.m5  + temp.m[1+8]*mat.m6  + temp.m[1+12]*mat.m7;
	m[2+4]  = temp.m[2]*mat.m4  + temp.m[2+4]*mat.m5  + temp.m[2+8]*mat.m6  + temp.m[2+12]*mat.m7;
	m[3+4]  = temp.m[3]*mat.m4  + temp.m[3+4]*mat.m5  + temp.m[3+8]*mat.m6  + temp.m[3+12]*mat.m7;
	m[0+8]  = temp.m[0]*mat.m8  + temp.m[0+4]*mat.m9  + temp.m[0+8]*mat.m10 + temp.m[0+12]*mat.m11;
	m[1+8]  = temp.m[1]*mat.m8  + temp.m[1+4]*mat.m9  + temp.m[1+8]*mat.m10 + temp.m[1+12]*mat.m11;
	m[2+8]  = temp.m[2]*mat.m8  + temp.m[2+4]*mat.m9  + temp.m[2+8]*mat.m10 + temp.m[2+12]*mat.m11;
	m[3+8]  = temp.m[3]*mat.m8  + temp.m[3+4]*mat.m9  + temp.m[3+8]*mat.m10 + temp.m[3+12]*mat.m11;
	m[0+12] = temp.m[0]*mat.m12 + temp.m[0+4]*mat.m13 + temp.m[0+8]*mat.m14 + temp.m[0+12]*mat.m15;
	m[1+12] = temp.m[1]*mat.m12 + temp.m[1+4]*mat.m13 + temp.m[1+8]*mat.m14 + temp.m[1+12]*mat.m15;
	m[2+12] = temp.m[2]*mat.m12 + temp.m[2+4]*mat.m13 + temp.m[2+8]*mat.m14 + temp.m[2+12]*mat.m15;
	m[3+12] = temp.m[3]*mat.m12 + temp.m[3+4]*mat.m13 + temp.m[3+8]*mat.m14 + temp.m[3+12]*mat.m15;
	return *this;
}
MyMatrix& MyMatrix::mult3by3(const MyMatrix& mat)
{
	MyMatrix temp(*this);
	m0 = temp.m0*mat.m0+temp.m4*mat.m1+temp.m8*mat.m2 + temp.m12*mat.m3;
	m4 = temp.m0*mat.m4+temp.m4*mat.m5+temp.m8*mat.m6 + temp.m12*mat.m7;
	m8 = temp.m0*mat.m8+temp.m4*mat.m9+temp.m8*mat.m10 + temp.m12*mat.m11;
	m12 = temp.m0*mat.m12 + temp.m4*mat.m13 + temp.m8*mat.m14 + temp.m12*mat.m15;

	m1 = temp.m1*mat.m0+temp.m5*mat.m1+temp.m9*mat.m2 + temp.m13*mat.m3;
	m5 = temp.m1*mat.m4+temp.m5*mat.m5+temp.m9*mat.m6 + temp.m13*mat.m7;
	m9 = temp.m1*mat.m8+temp.m5*mat.m9+temp.m9*mat.m10 + temp.m13*mat.m11;
	m13 = temp.m1*mat.m12 + temp.m5*mat.m13 + temp.m9*mat.m14 + temp.m13*mat.m15;

	m2 = temp.m2*mat.m0+temp.m6*mat.m1+temp.m10*mat.m2 + temp.m14*mat.m3;
	m6 = temp.m2*mat.m4+temp.m6*mat.m5+temp.m10*mat.m6 + temp.m14*mat.m7;
	m10 = temp.m2*mat.m8+temp.m6*mat.m9+temp.m10*mat.m10 + temp.m14*mat.m11;
	m14 = temp.m2*mat.m12 + temp.m6*mat.m13 + temp.m10*mat.m14 + temp.m14*mat.m15;

	return *this;
}
MyMatrix MyMatrix::getTranspose(void)
{
	return (MyMatrix(*this).transpose());
}
MyMatrix& MyMatrix::transpose(void)
{
	swap(m1,m4); swap(m2,m8); swap(m3,m12);
	swap(m6,m9); swap(m7,m13); swap(m11,m14);
	return *this;
}
MyMatrix MyMatrix::identity1(void)
{
	MyMatrix ret;
	ret.m0 = 1;   ret.m4 = 0;   ret.m8  = 0;  ret.m12 = 0;
	ret.m1 = 0;   ret.m5 = 1;   ret.m9  = 0;  ret.m13 = 0;
	ret.m2 = 0;   ret.m6 = 0;   ret.m10 = 1;  ret.m14 = 0;
	ret.m3 = 0;   ret.m7 = 0;   ret.m11 = 0;  ret.m15 = 1;
	return ret;
}
MyMatrix& MyMatrix::loadIdentity(void)
{
	m0 = 1;   m4 = 0;   m8  = 0;  m12 = 0;
	m1 = 0;   m5 = 1;   m9  = 0;  m13 = 0;
	m2 = 0;   m6 = 0;   m10 = 1;  m14 = 0;
	m3 = 0;   m7 = 0;   m11 = 0;  m15 = 1;
	return *this;
}
MyMatrix MyMatrix::zero(void)
{
	return MyMatrix(DataType(0));
}
MyMatrix& MyMatrix::loadZero(void)
{
	m0 = 0;   m4 = 0;   m8  = 0;  m12 = 0;
	m1 = 0;   m5 = 0;   m9  = 0;  m13 = 0;
	m2 = 0;   m6 = 0;   m10 = 0;  m14 = 0;
	m3 = 0;   m7 = 0;   m11 = 0;  m15 = 0;

	return *this;
}
MyMatrix MyMatrix::glScale(const DataType x, const DataType y, const DataType z)
{
	MyMatrix ret;
	ret.m0 = x;     ret.m4 = 0;     ret.m8 = 0;     ret.m12 = 0;
	ret.m1 = 0;     ret.m5 = y;     ret.m9 = 0;     ret.m13 = 0;
	ret.m2 = 0;     ret.m6 = 0;     ret.m10 = z;    ret.m14 = 0;
	ret.m3 = 0;     ret.m7 = 0;     ret.m11 = 0;    ret.m15 = 1;
	return ret;
}

//!Make this an OpenGL scale matrix
MyMatrix& MyMatrix::loadScale(const DataType x, const DataType y, const DataType z)
{
	m0 = x;   m4 = 0;   m8  = 0;  m12 = 0;
	m1 = 0;   m5 = y;   m9  = 0;  m13 = 0;
	m2 = 0;   m6 = 0;   m10 = z;  m14 = 0;
	m3 = 0;   m7 = 0;   m11 = 0;  m15 = 1;
	return *this;
}

//!Apply an OpenGL scale matrix to this
MyMatrix& MyMatrix::applyScale(const DataType x, const DataType y)
{
	/*improved version*/
	//assuming z = 1
	m0 *= x;    m1 *= x;    m2 *= x;    m3 *= x;
	m4 *= y;    m5 *= y;    m6 *= y;    m7 *= y;
	return *this;
}

//!Apply an OpenGL scale matrix to this
MyMatrix& MyMatrix::applyScale(const DataType x, const DataType y, const DataType z)
{
	/*improved version*/
	m0 *= x;    m1 *= x;    m2 *= x;    m3 *= x;
	m4 *= y;    m5 *= y;    m6 *= y;    m7 *= y;
	m8 *= z;    m9 *= z;    m10 *= z;   m11 *= z;
	return *this;
}
MyMatrix& MyMatrix::loadTranslate(const DataType x, const DataType y, const DataType z)
{
	m0 = 1;   m4 = 0;   m8  = 0;  m12 = x;
	m1 = 0;   m5 = 1;   m9  = 0;  m13 = y;
	m2 = 0;   m6 = 0;   m10 = 1;  m14 = z;
	m3 = 0;   m7 = 0;   m11 = 0;  m15 = 1;
	return *this;
}
MyMatrix& MyMatrix::applyTranslate(const DataType x, const DataType y)
{
	/*improved version*/
	//assuming z = 0
	m12 += m0*x + m4*y;
	m13 += m1*x + m5*y;
	m14 += m2*x + m6*y;
	return *this;
}

//!Apply an OpenGL translate matrix to this
MyMatrix& MyMatrix::applyTranslate(const DataType x, const DataType y, const DataType z)
{
	/*improved version*/
	m12 += m0*x + m4*y + m8*z;
	m13 += m1*x + m5*y + m9*z;
	m14 += m2*x + m6*y + m10*z;
	return *this;
}
MyMatrix MyMatrix::glTranslate(const DataType x, const DataType y, const DataType z)
{
	MyMatrix ret;
	ret.m0 = 1; ret.m4 = 0; ret.m8 = 0;  ret.m12 = x;
	ret.m1 = 0; ret.m5 = 1; ret.m9 = 0;  ret.m13 = y;
	ret.m2 = 0; ret.m6 = 0; ret.m10 = 1; ret.m14 = z;
	ret.m3 = 0; ret.m7 = 0; ret.m11 = 0; ret.m15 = 1;
	return ret;
}
MyMatrix& MyMatrix::loadRotate(const DataType angle, DataType x, DataType y, DataType z)
{
	register DataType magSqr = x*x + y*y + z*z;
	if(magSqr != 1.0)
	{
		const DataType mag = sqrt(magSqr);
		x /= mag;
		y /= mag;
		z /= mag;
	}
	const DataType c = DataType(cos(angle * M_PI / 180));
	const DataType s = DataType(sin(angle * M_PI / 180));
	m0 = x*x*(1-c)+c;
	m1 = y*x*(1-c)+z*s;
	m2 = z*x*(1-c)-y*s;
	m3 = 0;

	m4 = x*y*(1-c)-z*s;
	m5 = y*y*(1-c)+c;
	m6 = z*y*(1-c)+x*s;
	m7 = 0;

	m8 = x*z*(1-c)+y*s;
	m9 = y*z*(1-c)-x*s;
	m10 = z*z*(1-c)+c;
	m11 = 0;

	m12 = 0;
	m13 = 0;
	m14 = 0;
	m15 = 1;

	return *this;
}
MyMatrix& MyMatrix::loadRotateX(const DataType angle)
{
	const DataType c = DataType(cos(angle * M_PI / 180));
	const DataType s = DataType(sin(angle * M_PI / 180));
	m0 = 1;
	m1 = 0;
	m2 = 0;
	m3 = 0;

	m4 = 0;
	m5 = c;
	m6 = s;
	m7 = 0;

	m8 = 0;
	m9 = -s;
	m10 = c;
	m11 = 0;

	m12 = 0;
	m13 = 0;
	m14 = 0;
	m15 = 1;

	return *this;
}
MyMatrix& MyMatrix::loadRotateY(const DataType angle)
{
	const DataType c = DataType(cos(angle * M_PI / 180));
	const DataType s = DataType(sin(angle * M_PI / 180));
	m0 = c;
	m1 = 0;
	m2 = -s;
	m3 = 0;

	m4 = 0;
	m5 = 1;
	m6 = 0;
	m7 = 0;

	m8 = s;
	m9 = 0;
	m10 = c;
	m11 = 0;

	m12 = 0;
	m13 = 0;
	m14 = 0;
	m15 = 1;

	return *this;
}

//!Make this an OpenGL rotation matrix: same as loadRotateZ(angle,0,0,1)
MyMatrix& MyMatrix::loadRotateZ(const DataType angle)
{
	const DataType c = DataType(cos(angle * M_PI / 180));
	const DataType s = DataType(sin(angle * M_PI / 180));
	m0 = c;
	m1 = s;
	m2 = 0;
	m3 = 0;

	m4 = -s;
	m5 = c;
	m6 = 0;
	m7 = 0;

	m8 = 0;
	m9 = 0;
	m10 = 1;
	m11 = 0;

	m12 = 0;
	m13 = 0;
	m14 = 0;
	m15 = 1;

	return *this;
}
MyMatrix& MyMatrix::applyRotate(const DataType angle, const DataType x, const DataType y, const DataType z)
{
	static MyMatrix temp;
	temp.loadRotate(angle, x, y, z);
	return mult3by3(temp);
}
MyMatrix& MyMatrix::applyRotateX(const DataType angle)
{
	static MyMatrix temp;
	temp.loadRotateX(angle);
	return mult3by3(temp);
}

//!Apply an OpenGL rotation matrix to this
MyMatrix& MyMatrix::applyRotateY(const DataType angle)
{
	static MyMatrix temp;
	temp.loadRotateY(angle);
	return mult3by3(temp);
}

//!Apply an OpenGL rotation matrix to this
MyMatrix& MyMatrix::applyRotateZ(const DataType angle)
{
	static MyMatrix temp;
	temp.loadRotateZ(angle);
	return mult3by3(temp);
}
MyMatrix& MyMatrix::applyRotateXYZ(const DataType x, const DataType y, const DataType z)
{
	static MyMatrix temp;
	temp.loadRotateX(x);
	mult3by3(temp);
	temp.loadRotateY(y);
	mult3by3(temp);
	temp.loadRotateZ(z);
	return mult3by3(temp);
}
MyMatrix MyMatrix::glRotate(const DataType angle, DataType x, DataType y, DataType z)
{
	MyMatrix ret;
	DataType mag = std::sqrt(x*x + y*y + z*z);
	x /= mag;
	y /= mag;
	z /= mag;
	const DataType c = cos(angle*M_PI/180);
	const DataType s = sin(angle*M_PI/180);
	ret.m0 = x*x*(1-c)+c;
	ret.m1 = y*x*(1-c)+z*s;
	ret.m2 = z*x*(1-c)-y*s;
	ret.m3 = 0;
	ret.m4 = x*y*(1-c)-z*s;
	ret.m5 = y*y*(1-c)+c;
	ret.m6 = z*y*(1-c)+x*s;
	ret.m7 = 0;
	ret.m8 = x*z*(1-c)+y*s;
	ret.m9 = y*z*(1-c)-x*s;
	ret.m10 = z*z*(1-c)+c;
	ret.m11 = 0;
	ret.m12 = 0;
	ret.m13 = 0;
	ret.m14 = 0;
	ret.m15 = 1;
	return ret;
}
MyMatrix MyMatrix::glFrustum(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear, const DataType zFar)
{
	MyMatrix ret;
	ret.m0 = DataType(2 * zNear / (right - left));
	ret.m1 = 0;
	ret.m2 = 0;
	ret.m3 = 0;

	ret.m4 = 0;
	ret.m5 = DataType(2 * zNear / (top - bottom));
	ret.m6 = 0;
	ret.m7 = 0;

	ret.m8 = DataType((right + left) / (right - left));
	ret.m9 = DataType((top + bottom) / (top - bottom));
	ret.m10 = -DataType((zFar + zNear) / (zFar - zNear));
	ret.m11 = -1;

	ret.m12 = 0;
	ret.m13 = 0;
	ret.m14 = -DataType(2 * zFar * zNear / (zFar - zNear));
	ret.m15 = 0;

	return ret;
}
MyMatrix& MyMatrix::loadFrustum(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear, const DataType zFar)
{
	m0 = DataType(2 * zNear / (right-left));
	m1 = 0;
	m2 = 0;
	m3 = 0;

	m4 = 0;
	m5 = DataType(2 * zNear / (top-bottom));
	m6 = 0;
	m7 = 0;

	m8 = DataType((right + left) / (right - left));
	m9 = DataType((top + bottom) / (top - bottom));
	m10 = -DataType((zFar + zNear) / (zFar - zNear));
	m11 = -1;

	m12 = 0;
	m13 = 0;
	m14 = DataType(-2 * zFar * zNear / (zFar - zNear));
	m15 = 0;

	return *this;
}
MyMatrix MyMatrix::glOrtho(const DataType left, const DataType right,
	const DataType bottom, const DataType top,
	const DataType zNear, const DataType zFar)
{
	MyMatrix ret;
	ret.m0 = DataType(2 / (right - left));
	ret.m1 = 0;
	ret.m2 = 0;
	ret.m3 = 0;

	ret.m4 = 0;
	ret.m5 = DataType(2 / (top - bottom));
	ret.m6 = 0;
	ret.m7 = 0;

	ret.m8 = 0;
	ret.m9 = 0;
	ret.m10 = DataType(-2 / (zFar - zNear));
	ret.m11 = 0;

	ret.m12 = -DataType((right + left) / (right - left));
	ret.m13 = -DataType((top + bottom) / (top - bottom));
	ret.m14 = -DataType((zFar + zNear) / (zFar - zNear));
	ret.m15 = 1;

	return ret;
}

//!OpenGL orthogonal matrix
MyMatrix& MyMatrix::loadOrtho(const DataType left, const DataType right,
	const DataType bottom, const DataType top,
	const DataType zNear, const DataType zFar)
{
	m0 = DataType(2 / (right - left));
	m1 = 0;
	m2 = 0;
	m3 = 0;

	m4 = 0;
	m5 = DataType(2 / (top - bottom));
	m6 = 0;
	m7 = 0;

	m8 = 0;
	m9 = 0;
	m10 = -DataType(2 / (zFar - zNear));
	m11 = 0;

	m12 = -DataType((right + left) / (right - left));
	m13 = -DataType((top + bottom) / (top - bottom));
	m14 = -DataType((zFar + zNear) / (zFar - zNear));
	m15 = 1;

	return *this;
}
MyMatrix& MyMatrix::loadView(const DataType cx, const DataType cy, const DataType cz,
	const DataType tx, const DataType ty, const DataType tz,
	const DataType upx, const DataType upy, const DataType upz)
{
    MyMatrix inverseMatrix;
	inverseMatrix.m[12] = cx;  inverseMatrix.m[13] = cy;  inverseMatrix.m[14] = cz;  inverseMatrix.m[15] = 1;
	MyVector3D vecz(cx - tx,cy - ty,cz - tz); vecz.Normalize();
	MyVector3D vecy(upx,upy,upz);   vecy.Normalize();
	MyVector3D vecx = vecy ^ vecz;
	inverseMatrix.m[0] = vecx[0]; inverseMatrix.m[1] = vecx[1];  inverseMatrix.m[2] = vecx[2];  inverseMatrix.m[3] = 0;
	inverseMatrix.m[4] = vecy[0]; inverseMatrix.m[5] = vecy[1];  inverseMatrix.m[6] = vecy[2];  inverseMatrix.m[7] = 0;
	inverseMatrix.m[8] = vecz[0]; inverseMatrix.m[9] = vecz[1];  inverseMatrix.m[10] = vecz[2];  inverseMatrix.m[11] = 0;
	*this = inverseMatrix.inverse();
	return *this;
}

//!OpenGL View Matrix.
MyMatrix& MyMatrix::loadView(const DataType* const c, const DataType* const t, const DataType* const up)
{
	return loadView(c[0],c[1],c[2],t[0],t[1],t[2],up[0],up[1],up[2]);
}
MyMatrix MyMatrix::adjoint() const
{
	MyMatrix ret;
	ret[0] = cofactorm0();
	ret[1] = -cofactorm4();
	ret[2] = cofactorm8();
	ret[3] = -cofactorm12();
	ret[4] = -cofactorm1();
	ret[5] = cofactorm5();
	ret[6] = -cofactorm9();
	ret[7] = cofactorm13();
	ret[8] = cofactorm2();
	ret[9] = -cofactorm6();
	ret[10] = cofactorm10();
	ret[11] = -cofactorm14();
	ret[12] = -cofactorm3();
	ret[13] = cofactorm7();
	ret[14] = -cofactorm11();
	ret[15] = cofactorm15();
	return ret;
}
MyMatrix MyMatrix::inverse() const
{
	MyMatrix ret(adjoint());
	const DataType determinant = m0 * ret[0] + m1 * ret[4] + m2 * ret[8] + m3 * ret[12];
	assert(determinant!=0 && "Singular matrix has no inverse");
	ret /= determinant;
	return ret;
}
std::ostream& operator <<(std::ostream &out,MyMatrix &matrix)
{
	out<<matrix[0]<<' '<<matrix[4]<<' '<<matrix[8]<<' '<<matrix[12]<<std::endl;
	out<<matrix[1]<<' '<<matrix[5]<<' '<<matrix[9]<<' '<<matrix[13]<<std::endl;
	out<<matrix[2]<<' '<<matrix[6]<<' '<<matrix[10]<<' '<<matrix[14]<<std::endl;
	out<<matrix[3]<<' '<<matrix[7]<<' '<<matrix[11]<<' '<<matrix[15]<<std::endl;
	return out;
}
MyVector3D MyMatrix::multiplyByVector(const MyVector3D &myVector)
{
    MyVector3D retVec;
    retVec.x = m0 * myVector.x + m4 * myVector.y + m8 * myVector.z + m12;
    retVec.y = m1 * myVector.x + m5 * myVector.y + m9 * myVector.z + m13;
    retVec.z = m2 * myVector.x + m6 * myVector.y + m10 * myVector.z + m14;
    return retVec;
}
