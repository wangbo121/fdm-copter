/*
 * matrix3.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef MATRIX3_H_
#define MATRIX3_H_

#include "vector3.h"

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3 {
public:

	// Vectors comprising the rows of the matrix
	Vector3<T>	a, b, c;

	// trivial ctor
	// note that the Vector3 ctor will zero the vector elements
	Matrix3<T>() {}

	// setting ctor
	Matrix3<T>(const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0): a(a0), b(b0), c(c0) {}

	// setting ctor
	Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz): a(ax,ay,az), b(bx,by,bz), c(cx,cy,cz) {}

	// function call operator
	void operator () (const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0)
	{	a = a0; b = b0; c = c0;  }

	// test for equality
	bool operator == (const Matrix3<T> &m)
	{	return (a==m.a && b==m.b && c==m.c);	}

	// test for inequality
	bool operator != (const Matrix3<T> &m)
	{	return (a!=m.a || b!=m.b || c!=m.c);	}

	// negation
	Matrix3<T> operator - (void) const
	{	return Matrix3<T>(-a,-b,-c);	}

	// addition
	Matrix3<T> operator + (const Matrix3<T> &m) const
	{   return Matrix3<T>(a+m.a, b+m.b, c+m.c);	 }
	Matrix3<T> &operator += (const Matrix3<T> &m)
	{	return *this = *this + m;	}

	// subtraction
	Matrix3<T> operator - (const Matrix3<T> &m) const
	{   return Matrix3<T>(a-m.a, b-m.b, c-m.c);	 }
	Matrix3<T> &operator -= (const Matrix3<T> &m)
	{	return *this = *this - m;	}

	// uniform scaling
	Matrix3<T> operator * (const T num) const
	{	return Matrix3<T>(a*num, b*num, c*num);	}
	Matrix3<T> &operator *= (const T num)
	{	return *this = *this * num;	}
	 Matrix3<T> operator / (const T num) const
	{	return Matrix3<T>(a/num, b/num, c/num);	}
	Matrix3<T> &operator /= (const T num)
	{	return *this = *this / num;	}


	// multiplication by a vector
	Vector3<T> operator *(const Vector3<T> &v) const
	{
		return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
						  b.x * v.x + b.y * v.y + b.z * v.z,
						  c.x * v.x + c.y * v.y + c.z * v.z);
	}
	// multiplication by another Matrix3<T>
		Matrix3<T> operator *(const Matrix3<T> &m) const;
#if 0
	// multiplication by another Matrix3<T>
	Matrix3<T> operator *(const Matrix3<T> &m) const
	{
		Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
									a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
									a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
						 Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
									b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
									b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
						 Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
									c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
									c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
		return temp;
	}
	Matrix3<T> &operator *=(const Matrix3<T> &m)
	{	return *this = *this * m;	}
#endif
#if 0
	// transpose the matrix
	Matrix3<T> transposed(void) const
	{
		return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
						  Vector3<T>(a.y, b.y, c.y),
						  Vector3<T>(a.z, b.z, c.z));
	}

	Matrix3<T> transpose(void)
	{	return *this = transposed();	}
#endif
	// transpose the matrix
	    Matrix3<T>          transposed(void) const;

	    void transpose(void)
	    {
	        *this = transposed();
	    }
	/*
	 * 20170813添加
	 */

	/**
	 * Calculate the determinant of this matrix.
	 *
	 * @return The value of the determinant.
	 */
	T det() const;

	/**
	 * Calculate the inverse of this matrix.
	 *
	 * @param inv[in] Where to store the result.
	 *
	 * @return If this matrix is invertible, then true is returned. Otherwise,
	 * \p inv is unmodified and false is returned.
	 */
	bool inverse(Matrix3<T>& inv) const;

	/**
	 * Invert this matrix if it is invertible.
	 *
	 * @return Return true if this matrix could be successfully inverted and
	 * false otherwise.
	 */
	bool invert();

	// zero the matrix
	void        zero(void);


	// setup the identity matrix
	void        identity(void) {
		a.x = b.y = c.z = 1;
		a.y = a.z = 0;
		b.x = b.z = 0;
		c.x = c.y = 0;
	}

	// check if any elements are NAN
	bool        is_nan(void)
	{
		return a.is_nan() || b.is_nan() || c.is_nan();
	}

	// create a rotation matrix from Euler angles
	void        from_euler(float roll, float pitch, float yaw);

	// create eulers from a rotation matrix
	void        to_euler(float *roll, float *pitch, float *yaw) const;

	/*
	  calculate Euler angles (312 convention) for the matrix.
	  See http://www.atacolorado.com/eulersequences.doc
	  vector is returned in r, p, y order
	*/
	Vector3<T> to_euler312() const;

	/*
	  fill the matrix from Euler angles in radians in 312 convention
	*/
	void from_euler312(float roll, float pitch, float yaw);

	// apply an additional rotation from a body frame gyro vector
	// to a rotation matrix.
	void        rotate(const Vector3<T> &g);

	// create rotation matrix for rotation about the vector v by angle theta
	// See: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
	// "Rotation matrix from axis and angle"
	void        from_axis_angle(const Vector3<T> &v, float theta);

	// normalize a rotation matrix
	void        normalize(void);





};



typedef Matrix3<int>			Matrix3i;
typedef Matrix3<unsigned int>	Matrix3ui;
typedef Matrix3<long>			Matrix3l;
typedef Matrix3<unsigned long>	Matrix3ul;
typedef Matrix3<float>			Matrix3f;



#endif /* MATRIX3_H_ */
