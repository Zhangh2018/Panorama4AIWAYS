#ifndef _MY_MATRIX_H__
#define _MY_MATRIX_H__
#include <GLES2/gl2.h>
#include <string.h>
#include <cassert>
#include <math.h>
#include <ostream>
#define M_PI 3.14159265358979323846
#include "MyVector3D.h"
class MyMatrix
{
public:
	typedef GLfloat DataType;
	static inline void swap(DataType& a, DataType& b) { DataType temp = a; a = b; b = temp; }
	MyMatrix(void);
	MyMatrix(const DataType val);
	MyMatrix(const DataType* const val);
	MyMatrix(const DataType* const v0, const DataType* const v1, const DataType* const v2, const DataType* const v3);
	MyMatrix(const MyMatrix& mat);
	~MyMatrix(void);
public:
	friend std::ostream& operator <<(std::ostream &out,MyMatrix &matrix);
	MyMatrix& operator= (const MyMatrix& mat);
	bool operator== (const MyMatrix& mat);
	DataType& operator[] (const int ind);
	inline operator const DataType*(void) const { return m; }
	inline operator DataType*(void) { return m; }
	MyMatrix& operator*= (const DataType& val);
	MyMatrix& operator/= (const DataType& val);
	MyMatrix& operator+= (const MyMatrix& mat);
	MyMatrix operator+ (const MyMatrix& mat);
	MyMatrix& operator-= (const MyMatrix& mat);
	MyMatrix operator- (const MyMatrix& mat);
	MyMatrix operator* (const MyMatrix& mat);
	MyMatrix& operator*= (const MyMatrix& mat);
	DataType *getMatrixData(){ return m;};
public:
	MyMatrix& mult3by3(const MyMatrix& mat);
	MyMatrix& transpose(void);
	MyMatrix getTranspose(void);
	static MyMatrix identity1(void);
	MyMatrix& loadIdentity(void);
	static MyMatrix zero(void);
	MyMatrix& loadZero(void);
	MyMatrix adjoint() const;
	MyMatrix inverse() const;
	MyVector3D multiplyByVector(const MyVector3D &vector);
public:
    #define cofactor_maker(f1,mj1,mi1, f2,mj2,mi2, f3,mj3,mi3) \
	    f1*(mj1*mi1-mj2*mi3) + f2*(mj2*mi2-mj3*mi1) + f3*(mj3*mi3-mj1*mi2)

	//inline T cofactorm0()const { return m5*(m10*m15-m11*m14)+m6*(m11*m13-m8*m15)+m7*(m8*m14*m10*m13);  }
	//!Cofactor of m0
	inline const DataType cofactorm0() const { return cofactor_maker(m5,m10,m15, m6,m11,m13, m7,m9,m14); }
	//!Cofactor of m1
	inline const DataType cofactorm1() const { return cofactor_maker(m6,m11,m12, m7,m8,m14, m4,m10,m15); }
	//!Cofactor of m2
	inline const DataType cofactorm2() const { return cofactor_maker(m7,m8,m13, m4,m9,m15, m5,m11,m12); }
	//!Cofactor of m3
	inline const DataType cofactorm3() const { return cofactor_maker(m4,m9,m14, m5,m10,m12, m6,m8,m13); }

	//!Cofactor of m4
	inline const DataType cofactorm4() const { return cofactor_maker(m9,m14,m3, m10,m15,m1, m11,m13,m2); }
	//!Cofactor of m5
	inline const DataType cofactorm5() const { return cofactor_maker(m10,m15,m0, m11,m12,m2, m8,m14,m3); }
	//!Cofactor of m6
	inline const DataType cofactorm6() const { return cofactor_maker(m11,m12,m1, m8,m13,m3, m9,m15,m0); }
	//!Cofactor of m7
	inline const DataType cofactorm7() const { return cofactor_maker(m8,m13,m2, m9,m14,m0, m10,m12,m1); }

	//!Cofactor of m8
	inline const DataType cofactorm8() const { return cofactor_maker(m13,m2,m7, m14,m3,m5, m15,m1,m6); }
	//!Cofactor of m9
	inline const DataType cofactorm9() const { return cofactor_maker(m14,m13,m4, m15,m0,m6, m12,m2,m7); }
	//!Cofactor of m10
	inline const DataType cofactorm10() const { return cofactor_maker(m15,m0,m5, m12,m1,m7, m13,m3,m4); }
	//!Cofactor of m11
	inline const DataType cofactorm11() const { return cofactor_maker(m12,m1,m6, m13,m2,m4, m14,m0,m5); }

	//!Cofactor of m12
	inline const DataType cofactorm12() const { return cofactor_maker(m1,m6,m11, m2,m7,m9, m3,m5,m10); }
	//!Cofactor of m13
	inline const DataType cofactorm13() const { return cofactor_maker(m2,m7,m8, m3,m4,m10, m10,m6,m11); }
	//!Cofactor of m14
	inline const DataType cofactorm14() const { return cofactor_maker(m3,m4,m9, m0,m5,m11, m1,m7,m8); }
	//!Cofactor of m15
	inline const DataType cofactorm15() const { return cofactor_maker(m0,m5,m10, m1,m6,m8, m2,m4,m9); }
	inline const DataType det() const
	{ return m0 * cofactorm0() - m1 * cofactorm1() + m2 * cofactorm2() - m3 * cofactorm3(); }
public:
	union
	{
		//!Straight column major vector layout
		DataType m[16];
		//!Column-Row matrix layout
		DataType m_c_r[4][4];
		struct
		{
			//!Fast individual element access
			DataType m0,m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,m13,m14,m15;
		};
		struct
		{
			//!Fast individual element access
			DataType _a,_b,_c,_d,_e,_f,_g,_h,_i,_j,_k,_l,_m,_n,_o,_p;
		};
	};
public:
	static MyMatrix glRotate(const DataType angle, DataType x, DataType y, DataType z);
	MyMatrix& loadRotate(const DataType angle, DataType x, DataType y, DataType z);
	MyMatrix& loadRotateX(const DataType angle);
	MyMatrix& loadRotateY(const DataType angle);
	MyMatrix& loadRotateZ(const DataType angle);
	MyMatrix& applyRotate(const DataType angle, const DataType x, const DataType y, const DataType z);
	MyMatrix& applyRotateX(const DataType angle);
	MyMatrix& applyRotateY(const DataType angle);
	MyMatrix& applyRotateZ(const DataType angle);
	MyMatrix& applyRotateXYZ(const DataType x, const DataType y, const DataType z);
	static MyMatrix glScale(const DataType x, const DataType y, const DataType z);
	MyMatrix& loadScale(const DataType x, const DataType y, const DataType z);
	MyMatrix& applyScale(const DataType x, const DataType y);
	MyMatrix& applyScale(const DataType x, const DataType y, const DataType z);
	static MyMatrix glTranslate(const DataType x, const DataType y, const DataType z);
	MyMatrix& loadTranslate(const DataType x, const DataType y, const DataType z);
	MyMatrix& applyTranslate(const DataType x, const DataType y);
	MyMatrix& applyTranslate(const DataType x, const DataType y, const DataType z);
	static MyMatrix glFrustum(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear, const DataType zFar);
	MyMatrix& loadFrustum(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear,const DataType zFar);
	static MyMatrix glOrtho(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear, const DataType zFar);
	MyMatrix& loadOrtho(const DataType left, const DataType right,const DataType bottom, const DataType top,const DataType zNear, const DataType zFar);
	MyMatrix& loadView(const DataType cx, const DataType cy, const DataType cz,const DataType tx, const DataType ty, const DataType tz,const DataType upx, const DataType upy, const DataType upz);
	MyMatrix& loadView(const DataType* const c, const DataType* const t, const DataType* const up);
};
#endif // _MY_MATRIX_H__
