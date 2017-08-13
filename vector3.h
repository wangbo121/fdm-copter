/*
 * vector3.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include <math.h>
#include <string.h>

template <typename T>
class Vector3
{
public:
	T x, y, z;

	// trivial ctor
	Vector3<T>() { x = y = z = 0; }

	// setting ctor
	Vector3<T>(const T x0, const T y0, const T z0): x(x0), y(y0), z(z0) {}

	// function call operator
	void operator ()(const T x0, const T y0, const T z0)
	{	x= x0; y= y0; z= z0;  }

	// test for equality
	bool operator==(const Vector3<T> &v)
	{	return (x==v.x && y==v.y && z==v.z);	}

	// test for inequality
	bool operator!=(const Vector3<T> &v)
	{	return (x!=v.x || y!=v.y || z!=v.z);	}

	// negation
	Vector3<T> operator -(void) const
	{	return Vector3<T>(-x,-y,-z);	}

	// addition
	Vector3<T> operator +(const Vector3<T> &v) const
	{   return Vector3<T>(x+v.x, y+v.y, z+v.z);	 }

	// subtraction
	Vector3<T> operator -(const Vector3<T> &v) const
	{   return Vector3<T>(x-v.x, y-v.y, z-v.z);	 }

	// uniform scaling
	Vector3<T> operator *(const T num) const
	{
		Vector3<T> temp(*this);
		return temp*=num;
	}

	// uniform scaling
	Vector3<T> operator /(const T num) const
	{
		Vector3<T> temp(*this);
		return temp/=num;
	}

	// addition
	Vector3<T> &operator +=(const Vector3<T> &v)
	{
		x+=v.x;	y+=v.y;	z+=v.z;
		return *this;
	}

	// subtraction
	Vector3<T> &operator -=(const Vector3<T> &v)
	{
		x-=v.x;	y-=v.y;	z-=v.z;
		return *this;
	}

	// uniform scaling
	Vector3<T> &operator *=(const T num)
	{
		x*=num; y*=num; z*=num;
		return *this;
	}

	// uniform scaling
	Vector3<T> &operator /=(const T num)
	{
		x/=num; y/=num; z/=num;
		return *this;
	}

	// dot product
	T operator *(const Vector3<T> &v) const
	{	return x*v.x + y*v.y + z*v.z;	}

	// cross product
	Vector3<T> operator %(const Vector3<T> &v) const
	{
		Vector3<T> temp(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
		return temp;
	}

	// gets the length of this vector squared
	T length_squared() const
	{	return (T)(*this * *this);   }

	// gets the length of this vector
	float length() const
	{	return (T)sqrt(*this * *this);   }

	// normalizes this vector
	void normalize()
	{	*this/=length();	}

	// returns the normalized version of this vector
	Vector3<T> normalized() const
	{   return  *this/length();  }

	// reflects this vector about n
	void reflect(const Vector3<T> &n)
	{
		Vector3<T> orig(*this);
		project(n);
		*this= *this*2 - orig;
	}

	// projects this vector onto v
	void project(const Vector3<T> &v)
	{	*this= v * (*this * v)/(v*v);	}

	// returns this vector projected onto v
	Vector3<T> projected(const Vector3<T> &v)
	{   return v * (*this * v)/(v*v);	}

	// computes the angle between 2 arbitrary vectors
	T angle(const Vector3<T> &v1, const Vector3<T> &v2)
	{   return (T)acosf((v1*v2) / (v1.length()*v2.length()));  }

	// computes the angle between 2 arbitrary normalized vectors
	T angle_normalized(const Vector3<T> &v1, const Vector3<T> &v2)
	{   return (T)acosf(v1*v2);  }


	// check if all elements are zero
	  //  bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON) && (fabsf(z) < FLT_EPSILON); }

};

typedef Vector3<int>			Vector3i;
typedef Vector3<unsigned int>	Vector3ui;
typedef Vector3<long>			Vector3l;
typedef Vector3<unsigned long>	Vector3ul;
typedef Vector3<float>			Vector3f;



#endif /* VECTOR3_H_ */
