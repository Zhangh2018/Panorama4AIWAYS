#include "MyVector3D.h"
#include <math.h>
const MyVector3D& MyVector3D::operator += (const MyVector3D& o)
{
	x += o.x; y += o.y; z += o.z; 
	return *this; 
}
const MyVector3D& MyVector3D::operator -= (const MyVector3D& o)
{
	x -= o.x; y -= o.y; z -= o.z; 
	return *this;
}
const MyVector3D& MyVector3D::operator *= (DataType f)
{
	x *= f; y *= f; z *= f; 
	return *this; 
}
const MyVector3D& MyVector3D::operator /= (DataType f)
{
	x /= f; y /= f; z /= f; 
	return *this; 
}
float MyVector3D::operator[](unsigned int i) const
{
	return *(&x + i);
}
float& MyVector3D::operator[](unsigned int i)
{
	return *(&x + i);
}
bool MyVector3D::operator== (const MyVector3D& other) const {
	return x == other.x && y == other.y && z == other.z;
}
bool MyVector3D::operator!= (const MyVector3D& other) const {
	return x != other.x || y != other.y || z != other.z;
}
void MyVector3D::Set( DataType pX, DataType pY, DataType pZ) { 
	x = pX; y = pY; z = pZ; 
}
MyVector3D::DataType MyVector3D::SquareLength() const {
	return x*x + y*y + z*z; 
}
MyVector3D::DataType MyVector3D::Length() const {
	return sqrt( SquareLength()); 
}
MyVector3D& MyVector3D::Normalize() { 
	*this /= Length(); return *this;
}
MyVector3D operator + (const MyVector3D& v1, const MyVector3D& v2){
	return MyVector3D( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
MyVector3D operator - (const MyVector3D& v1, const MyVector3D& v2){
	return MyVector3D( v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}
MyVector3D::DataType operator * (const MyVector3D& v1, const MyVector3D& v2){
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
MyVector3D operator * ( MyVector3D::DataType f, const MyVector3D& v){
	return MyVector3D( f*v.x, f*v.y, f*v.z);
}
MyVector3D operator * ( const MyVector3D& v, MyVector3D::DataType f){
	return MyVector3D( f*v.x, f*v.y, f*v.z);
}
MyVector3D operator / ( const MyVector3D& v, MyVector3D::DataType f){
	return v * (1/f);
}
MyVector3D operator / ( const MyVector3D& v, const MyVector3D& v2){
	return MyVector3D(v.x / v2.x,v.y / v2.y,v.z / v2.z);
}
MyVector3D operator ^(const MyVector3D& v1, const MyVector3D& v2){
	return MyVector3D(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}
MyVector3D operator - ( const MyVector3D& v){
	return MyVector3D( -v.x, -v.y, -v.z);
}