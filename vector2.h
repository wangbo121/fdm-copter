/*
 * vector2.h
 *
 *  Created on: 2017-7-31
 *      Author: wangbo
 */

#ifndef VECTOR2_H_
#define VECTOR2_H_

#include <math.h>

template <typename T>
struct Vector2
{
	T x, y;

	// trivial ctor ctor:constructor wangbo20170731
	Vector2<T>() { x = y = 0; }

	// setting ctor
	Vector2<T>(const T x0, const T y0): x(x0), y(y0) {}

	// function call operator
	void operator ()(const T x0, const T y0)
	{	x= x0; y= y0;	}

	// test for equality
	bool operator==(const Vector2<T> &v)
	{	return (x==v.x && y==v.y);	}

	// test for inequality
	bool operator!=(const Vector2<T> &v)
	{	return (x!=v.x || y!=v.y);	}

	// negation
	Vector2<T> operator -(void) const
	{	return Vector2<T>(-x, -y);	}

	// addition
	Vector2<T> operator +(const Vector2<T> &v) const
	{	return Vector2<T>(x+v.x, y+v.y);	}

	// subtraction
	Vector2<T> operator -(const Vector2<T> &v) const
	{   return Vector2<T>(x-v.x, y-v.y);	}

	// uniform scaling
	Vector2<T> operator *(const T num) const
	{
		Vector2<T> temp(*this);
		return temp*=num;
	}

	// uniform scaling
	Vector2<T> operator /(const T num) const
	{
		Vector2<T> temp(*this);
		return temp/=num;
	}

	// addition
	Vector2<T> &operator +=(const Vector2<T> &v)
	{
		x+=v.x;	y+=v.y;
		return *this;
	}

	// subtraction
	Vector2<T> &operator -=(const Vector2<T> &v)
	{
		x-=v.x;	y-=v.y;
		return *this;
	}

	// uniform scaling
	Vector2<T> &operator *=(const T num)
	{
		x*=num;	y*=num;
		return *this;
	}

	// uniform scaling
	Vector2<T> &operator /=(const T num)
	{
		x/=num;	y/=num;
		return *this;
	}

	// dot product
	T operator *(const Vector2<T> &v) const
	{	return x*v.x + y*v.y;	}

	// gets the length of this vector squared
	T length_squared() const
	{	return (T)(*this * *this);   }

	// gets the length of this vector
	T length() const
	{	return (T)sqrt(*this * *this);   }

	// normalizes this vector
	void normalize()
	{	*this/=length();	}

	// returns the normalized vector
	Vector2<T> normalized() const
	{   return  *this/length();  }

	// reflects this vector about n
	void reflect(const Vector2<T> &n)
	{
		Vector2<T> orig(*this);
		project(n);
		*this= *this*2 - orig;
	}

	// projects this vector onto v
	void project(const Vector2<T> &v)
	{	*this= v * (*this * v)/(v*v);	}

	// returns this vector projected onto v
	Vector2<T> projected(const Vector2<T> &v)
	{   return v * (*this * v)/(v*v);	}

	// computes the angle between 2 arbitrary vectors
	T angle(const Vector2<T> &v1, const Vector2<T> &v2)
	{   return (T)acosf((v1*v2) / (v1.length()*v2.length()));  }

	// computes the angle between 2 normalized arbitrary vectors
	T angle_normalized(const Vector2<T> &v1, const Vector2<T> &v2)
	{   return (T)acosf(v1*v2);  }

};

typedef Vector2<int>			Vector2i;
typedef Vector2<unsigned int>	Vector2ui;
typedef Vector2<long>			Vector2l;
typedef Vector2<unsigned long>	Vector2ul;
typedef Vector2<float>			Vector2f;



#endif /* VECTOR2_H_ */
