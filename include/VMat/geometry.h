#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <iostream>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <type_traits>

#include "vmattype.h"

namespace vm
{
template <typename T>
bool IsNaN( const T &t )
{
	return std::isnan( t );
}

template <>
inline bool
  IsNaN( const int &t )
{
	return false;
}

template <>
inline bool
  IsNaN( const std::size_t &t )
{
	return false;
}

template <typename T>
class Vector2;
template <typename T>
class Vector3;
template <typename T>
class Point3;
template <typename T>
class Point2;
template <typename T>
class Normal3;
template <typename T>
class Vector4;

using Vector3f = Vector3<Float>;
using Vector3i = Vector3<int>;
using Point3f = Point3<Float>;
using Point3i = Point3<int>;
using Vector4f = Vector4<Float>;

using Normal3f = Normal3<Float>;
using Normal3i = Normal3<int>;

using Vector2f = Vector2<Float>;
using Vector2i = Vector2<int>;
using Point2f = Point2<Float>;
using Point2i = Point2<int>;
using Vector4i = Vector4<int>;

using Size3 = Vector3<std::size_t>;
using Size2 = Vector2<std::size_t>;

using Vec3f = Vector3f;
using Vec3i = Vector3i;
using Vec2f = Vector2f;
using Vec2i = Vector2i;
using Vec4f = Vector4f;
using Vec4i = Vector4i;
using Nor3f = Normal3f;
using Nor3i = Normal3i;

template <typename T>
class Grid;

template <typename T>
class Vector2
{
public:
	T x, y;
	constexpr Vector2() :
	  x( T() ),
	  y( T() )
	{
		assert( !HasNaN() );
	}
	constexpr Vector2( const T &x, const T &y ) :
	  x( x ),
	  y( y )
	{
		assert( !HasNaN() );
	}
	explicit constexpr Vector2( const T &a ) :
	  x( a ),
	  y( a )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector2( const Vector3<T> &v ) :
	  x( v.x ),
	  y( v.y )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector2( const Vector4<T> &v ) :
	  x( v.x ),
	  y( v.y )
	{
		assert( !HasNaN() );
	}

	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y );
	}

	constexpr Vector2<T> operator+( const Vector2<T> &p ) const
	{
		assert( !p.HasNaN() );
		return Vector2<T>( x + p.x, y + p.y );
	}

	constexpr Point2<T> operator+( const Point2<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Point2<T>( x + v.x, y + v.y );
	}

	constexpr Vector2<T> &operator+=( const Vector2<T> &v )
	{
		assert( !v.HasNaN() );
		x += v.x;
		y += v.y;
		return *this;
	}

	Vector2<T> operator-( const Vector2<T> &v )
	{
		assert( !v.HasNaN() );
		return Vector2<T>( x - v.x, y - v.y );
	}

	Vector2<T> &operator-=( const Vector2<T> &v )
	{
		assert( !v.HasNaN() );
		x -= v.x;
		y -= v.y;
		return *this;
	}

	template <typename U>
	Vector2<T> operator/( const U &s ) const
	{
		assert( !IsNaN( s ) );
		const Float inv = static_cast<Float>( 1 ) / s;
		return Vector2<T>( x * inv, y * inv );
	}

	template <typename U>
	Vector2<T> &operator/=( const U &s )
	{
		assert( !IsNaN( s ) );
		const Float inv = static_cast<Float>( 1 ) / s;
		x *= inv;
		y *= inv;
		return *this;
	}

	template <typename U>
	Vector2<decltype( T{} / U{} )> operator/( const Vector2<U> &s ) const
	{
		assert( !s.HasNaN() );
		return Vector2<decltype( T{} / U{} )>( x / s.x, y / s.y );
	}

	template <typename U>
	Vector2<T> &operator/=( const Vector2<U> &s )
	{
		assert( !s.HasNaN() );
		x /= s.x;
		y /= s.y;
		return *this;
	}

	template <typename U>
	Vector2<T>
	  operator*( const U &s )
	{
		assert( !IsNaN( s ) );
		return Vector2<T>( s * x, s * y );
	}

	template <typename U>
	Vector2<T> &operator*=( const U &s )
	{
		assert( !IsNaN( s ) );
		x *= s;
		y *= s;
		return *this;
	}

	template <typename U>
	Vector2<decltype( T{} / U{} )> operator*( const Vector2<U> &s )
	{
		assert( !s.HasNaN() );
		return Vector2<decltype( T{} / U{} )>( s.x * x, s.y * y );
	}

	template <typename U>
	Vector2<T> &operator*=( const Vector2<U> &s )
	{
		assert( !s.HasNaN() );
		x *= s.x;
		y *= s.y;
		return *this;
	}

	Vector2<T> operator-()
	{
		return Vector2<T>( -x, -y );
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 2 );
		return *( &x + i );
	}

	T &operator[]( int i )
	{
		assert( i >= 0 && i < 2 );
		return *( &x + i );
	}

	bool operator==( const Vector2<T> &v )
	{
		return x == v.x && y == v.y;
	}
	bool operator!=( const Vector2<T> &v )
	{
		return !( *this == v );
	}

	template <typename U>
	explicit operator Vector2<U>() const
	{
		return Vector2<U>( U( x ), U( y ) );
	}

	Float
	  LengthSquared() const
	{
		return x * x + y * y;
	}

	Float
	  Length() const
	{
		return std::sqrt( LengthSquared() );
	}

	Vector2<T> Normalized() const
	{
		const Float len = LengthSquared();
		return *this / len;
	}

	void Normalize()
	{
		const Float len = LengthSquared();
		*this /= len;
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}

	typename std::conditional<std::is_integral<T>::value, size_t, Float>::type Prod() const
	{
		return static_cast<typename std::conditional<std::is_integral<T>::value, size_t, Float>::type>( x ) * y;
	}

	template <typename X>
	friend class Point2;  //Vector2D can be accessed by all instances of Point2D
};

template <typename T>
std::ostream &operator<<( std::ostream &os, const Vector2<T> &v )
{
	os << "[" << v.x << ", " << v.y << "]";
	return os;
}

/*
	* Point2D
	*/
template <typename T>
class Point2
{
public:
	T x, y;
	constexpr Point2() :
	  x( T() ),
	  y( T() ) {}
	constexpr Point2( const T &x, const T &y ) :
	  x( x ),
	  y( y )
	{
		assert( !HasNaN() );
	}

	constexpr Point2( const T &a ) :
	  x( a ),
	  y( a )
	{
	}

	constexpr Point2( const Point3<T> &p ) :
	  x( p.x ),
	  y( p.y )
	{
		assert( !HasNaN() );
	}

	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y );
	}

	constexpr Point2<T> operator+( const Vector2<T> &v ) const
	{
		return Point2<T>( x + v.x, y + v.y );
	}

	constexpr Point2<T> operator+( const Point2<T> &p ) const
	{
		return Point2<T>( x + p.x, y + p.y );
	}

	constexpr Point2<T> operator+=( const Point2<T> &p ) const
	{
		return Point2<T>( x + p.x, y + p.y );
	}

	Point2<T> &operator+=( const Vector2<T> &v )
	{
		x += v.x;
		y += v.y;
		return *this;
	}

	Point2<T> operator-( const Vector2<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Point2<T>( x - v.x, y - v.y );
	}
	Vector2<T> operator-( const Point2<T> &p ) const
	{
		assert( !p.HasNaN() );
		return Vector2<T>( x - p.x, y - p.y );
	}

	Point2<T> operator-()
	{
		return Point2<T>( -x, -y );
	}

	Point2<T> &operator-=( const Vector2<T> &v )
	{
		assert( !v.HasNaN() );
		x -= v.x;
		y -= v.y;
		return *this;
	}

	template <typename U>
	Point2<T> operator*( const U &s )
	{
		assert( !IsNaN( s ) );

		return Point2<T>( s * x, s * y );
	}
	template <typename U>
	Point2<T> &operator*=( const U &s )
	{
		assert( !IsNaN( s ) );
		x *= s;
		y *= s;
		return *this;
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 2 );
		return *( &x + i );
	}

	T &operator[]( int i )
	{
		assert( i >= 0 && i < 2 );
		return *( &x + i );
	}

	bool operator==( const Point2<T> &p )
	{
		return x == p.x && y == p.y;
	}
	bool operator!=( const Point2<T> &p )
	{
		return !( *this == p );
	}

	explicit operator Vector2<T>() const
	{
		return Vector2<T>{ x, y };
	}

	template <typename U>
	explicit operator Point2<U>() const
	{
		return Point2<U>( U( x ), U( y ) );
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}
};

template <typename T>
std::ostream &operator<<( std::ostream &os, const Point2<T> &p )
{
	os << "[" << p.x << "," << p.y << "]";
	return os;
}

template <typename T, typename U>
Point2<T> operator*( const U &s, const Point2<T> &v )
{
	return Point2<T>( s * v.x, s * v.y );
}

template <typename T>
class Vector3
{
public:
	T x, y, z;
	constexpr Vector3() :
	  x( T() ),
	  y( T() ),
	  z( T() ) {}
	constexpr Vector3( const T &x, const T &y, const T &z ) :
	  x( x ),
	  y( y ),
	  z( z )
	{
		assert( !HasNaN() );
	}

	constexpr Vector3( const Vector2<T> &v, const T &a ) :
	  x( v.x ),
	  y( v.y ),
	  z( a )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector3( const T &a ) :
	  x( a ),
	  y( a ),
	  z( a )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector3( const Vector4<T> &v ) :
	  x( v.x ),
	  y( v.y ),
	  z( v.z )
	{
		assert( !HasNaN() );
	}

	//constexpr Vector3(const Normal3<T> & n):x(n.x),y(n.y),z(n.z){}
	//Vector3D(const Vector3D<T> & v) :x(T(v.x)), y(T(v.y)), z(T(v.z)) {}
	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y ) || IsNaN( z );
	}

	constexpr Vector3<T> operator+( const Vector3<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Vector3<T>( x + v.x, y + v.y, z + v.z );
	}

	constexpr Point3<T> operator+( const Point3<T> &p ) const
	{
		//assert(!p.HasNaN());
		return Point3<T>{ x + p.x, y + p.y, z + p.z };
	}

	constexpr Vector3<T> &operator+=( const Vector3<T> &v )
	{
		assert( !v.HasNaN() );
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	constexpr Vector3<T> operator-( const Vector3<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Vector3<T>( x - v.x, y - v.y, z - v.z );
	}

	constexpr Vector3<T> &operator-=( const Vector3<T> &v )
	{
		assert( !v.HasNaN() );
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	template <typename U>
	explicit operator Vector3<U>() const
	{
		return Vector3<U>( U( x ), U( y ), U( z ) );
	}

	//unary operator
	constexpr Vector3<T> operator-() const
	{
		return Vector3<T>( -x, -y, -z );
	}

	constexpr Vector3<T> operator*( const Vector3<T> &v ) const
	{
		return { x * v.x, y * v.y, z * v.z };
	}

	constexpr Vector3<T> operator*=( const Vector3<T> &v )
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
		return *this;
	}

	template <typename U>
	constexpr Vector3<T> operator*( const U &s ) const
	{
		assert( !IsNaN( s ) );
		return Vector3<T>( s * x, s * y, s * z );
	}
	template <typename U>
	constexpr Vector3<T> &operator*=( const U &s )
	{
		assert( !IsNaN( s ) );
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	constexpr Vector3<T> operator/( Float s ) const
	{
		//assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		return Vector3<T>( x * inv, y * inv, z * inv );
	}

	constexpr Vector3<T> &operator/=( Float s )
	{
		//	assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		x *= inv;
		y *= inv;
		z *= inv;
		return *this;
	}

	constexpr bool operator==( const Vector3<T> &v ) const
	{
		return x == v.x && y == v.y && z == v.z;
	}

	constexpr bool operator!=( const Vector3<T> &v ) const
	{
		return !( *this == v );
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}
	T &operator[]( int i )
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}

	static Float Dot( const Vector3f &v1, const Vector3f &v2 )
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	}

	static Vector3f Cross( const Vector3f &v1, const Vector3f &v2 )
	{
		return Vector3<T>{
			v1.y * v2.z - v1.z * v2.y,
			v1.z * v2.x - v1.x * v2.z,
			v1.x * v2.y - v1.y * v2.x
		};
	}

	typename std::conditional<std::is_integral<T>::value, size_t, Float>::type Prod() const
	{
		using type = typename std::conditional<std::is_integral<T>::value, size_t, Float>::type;
		return type( x ) * type( y ) * type( z );
	}

	constexpr Float LengthSquared() const { return x * x + y * y + z * z; }

	constexpr Float Length() const { return std::sqrt( LengthSquared() ); }

	constexpr Vector3<T> Normalized() const
	{
		//if len is too small?
		const auto len = Length();
		return ( *this ) / len;
	}

	constexpr void Normalize()
	{
		//if len is too small?
		const auto len = Length();
		( *this ) /= len;
	}

	constexpr bool IsNull() const
	{
		return x == 0 && y == 0 && z == 0;
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}

	constexpr Point3<T> ToPoint3() const
	{
		return Point3<T>( x, y, z );
	}
};

template <typename T>
std::ostream &operator<<( std::ostream &os, const Vector3<T> &v )
{
	os << "[" << v.x << "," << v.y << "," << v.z << "]";
	return os;
}

template <typename T>
constexpr Vector3<T> operator*( Float s, const Vector3<T> &v )
{
	return v * s;
}

template <typename T>
constexpr Vector3<T> Abs( const Vector3<T> &v )
{
	return Vector3<T>( std::abs( v.x ), std::abs( v.y ), std::abs( v.z ) );
}
/*
	* Point3D
	*/

template <typename T>
class Point3
{
public:
	T x, y, z;
	constexpr Point3() :
	  x( 0 ),
	  y( 0 ),
	  z( 0 ) {}
	constexpr Point3( const T &x, const T &y, const T &z ) :
	  x( x ),
	  y( y ),
	  z( z )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Point3( const T &a ) :
	  x( a ),
	  y( a ),
	  z( a )
	{
		assert( !HasNaN() );
	}

	constexpr Point3( const Point2<T> &p, const T &a ) :
	  x( p.x ),
	  y( p.y ),
	  z( a )
	{
		assert( !HasNaN() );
	}

	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y ) || IsNaN( z );
	}

	template <typename U>
	explicit operator Vector3<U>() const
	{
		return Vector3<U>( x, y, z );
	}

	template <typename U>
	explicit operator Point3<U>() const
	{
		return Point3<U>( U( x ), U( y ), U( z ) );
	}

	constexpr Point3<T> operator+( const Point3<T> &p ) const
	{
		assert( !p.HasNaN() );
		return Point3<T>( x + p.x, y + p.y, z + p.z );
	}

	constexpr Point3<T> operator+( const Vector3<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Point3<T>( x + v.x, y + v.y, z + v.z );
	}

	constexpr Point3<T> &operator+=( const Vector3<T> &v )
	{
		assert( !v.HasNaN() );
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	constexpr Point3<T> &operator+=( const Point3<T> &p )
	{
		assert( !p.HasNaN() );
		x += p.x;
		y += p.y;
		z += p.z;
		return *this;
	}

	template <typename U>
	constexpr Point3<T> operator*( const U &s ) const
	{
		assert( !IsNaN( s ) );
		return Point3<T>( s * x, s * y, s * z );
	}

	//Point3<T> operator*(const Point3<T> & p)const
	//{
	//	assert(!IsNaN(s));
	//	return Point3<T>{x*p.x,y*p.y,z*p.z};
	//}

	template <typename U>
	constexpr Point3<T> &operator*=( const U &s )
	{
		assert( !IsNaN( s ) );
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	template <typename U>
	constexpr Point3<T> operator/( const U &s ) const
	{
		assert( !IsNaN( s ) );
		const Float inv = static_cast<Float>( 1 ) / s;
		return Point3<T>( inv * x, inv * y, inv * z );
	}

	template <typename U>
	constexpr Point3<T> &operator/=( const U &s )
	{
		assert( !IsNaN( s ) );
		const Float inv = static_cast<Float>( 1 ) / s;
		x *= inv;
		y *= inv;
		z *= inv;
		return *this;
	}

	constexpr Vector3<T> operator-( const Point3<T> &p ) const
	{
		assert( !p.HasNaN() );
		return Vector3<T>( x - p.x, y - p.y, z - p.z );
	}

	constexpr Point3<T> operator-( const Vector3<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Point3<T>( x - v.x, y - v.y, z - v.z );
	}

	constexpr Point3<T> operator-() const
	{
		return Point3<T>( -x, -y, -z );
	}

	constexpr Point3<T> &operator-=( const Vector3<T> &v )
	{
		assert( !v.HasNaN() );
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	constexpr bool operator==( const Point3<T> &p ) const
	{
		return x == p.x && y == p.y && z == p.z;
	}

	constexpr bool operator!=( const Point3<T> &p ) const
	{
		return !( *this == p );
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}

	T &operator[]( int i )
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}

	bool IsNull() const
	{
		return x == 0 && y == 0 && z == 0;
	}

	constexpr Vector3<T> ToVector3() const
	{
		return Vector3<T>( x, y, z );
	}

	Vector3<T> Abs( const Vector3<T> &v )
	{
		return Vector3<T>( std::abs( v.x ), std::abs( v.y ), std::abs( v.z ) );
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}
};

template <typename T>
Point3<T> operator*( const Float &s, const Point3<T> &p )
{
	return p * s;
}

template <typename T>
std::ostream &operator<<( std::ostream &os, const Point3<T> &p )
{
	os << "[" << p.x << "," << p.y << "," << p.z << "]";
	return os;
}

template <typename T>
class Normal3
{
public:
	T x, y, z;
	explicit constexpr Normal3<T>( const Vector3<T> &v ) :
	  x( v.x ),
	  y( v.y ),
	  z( v.z )
	{
		assert( !v.HasNaN() );
	}
	constexpr Normal3<T>( T x, T y, T z ) :
	  x( x ),
	  y( y ),
	  z( z )
	{
	}

	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y ) || IsNaN( z );
	}

	constexpr Normal3<T> operator+( const Normal3<T> &v ) const
	{
		//assert(!v.HasNaN());
		return Vector3<T>( x + v.x, y + v.y, z + v.z );
	}

	constexpr Normal3<T> &operator+=( const Normal3<T> &v )
	{
		//assert(!v.HasNaN());
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	constexpr Normal3<T> operator-( const Normal3<T> &v ) const
	{
		//assert(!v.HasNaN());
		return Normal3<T>( x - v.x, y - v.y, z - v.z );
	}

	constexpr Normal3<T> &operator-=( const Normal3<T> &v )
	{
		//assert(!v.HasNaN());
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	constexpr Normal3<T> operator*( const Normal3<T> &v ) const
	{
		return { x * v.x, y * v.y, z * v.z };
	}

	constexpr Normal3<T> operator*=( const Normal3<T> &v )
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
		return *this;
	}

	template <typename U>
	constexpr Normal3<T> operator*( const U &s ) const
	{
		//assert(!IsNaN(s));
		return Normal3<T>( s * x, s * y, s * z );
	}
	template <typename U>
	constexpr Normal3<T> &operator*=( const U &s )
	{
		//assert(!IsNaN(s));
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	template <typename U>
	constexpr Normal3<T> operator/( U s ) const
	{
		//assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		return Normal3<T>( x * inv, y * inv, z * inv );
	}
	template <typename U>
	constexpr Normal3<T> &operator/=( U s )
	{
		//assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		x *= inv;
		y *= inv;
		z *= inv;
		return *this;
	}

	constexpr bool operator==( const Normal3<T> &n ) const
	{
		return x == n.x && y == n.y && z == n.z;
	}

	constexpr bool operator!=( const Normal3<T> &n ) const
	{
		return !( *this == n );
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}
	T &operator[]( int i )
	{
		assert( i >= 0 && i < 3 );
		return *( &x + i );
	}

	constexpr Float LengthSquared() const { return x * x + y * y + z * z; }

	constexpr Float Length() const { return std::sqrt( LengthSquared() ); }

	constexpr Normal3<T> Normalized() const
	{
		//if len is too small?
		const auto len = Length();
		return ( *this ) / len;
	}

	constexpr void Normalize()
	{
		//if len is too small?
		const auto len = Length();
		( *this ) /= len;
	}

	constexpr bool IsNull() const
	{
		return x == 0 && y == 0 && z == 0;
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}
};

template <typename T>
class Vector4
{
public:
	T x, y, z, w;
	constexpr Vector4() :
	  x( 0 ),
	  y( 0 ),
	  z( 0 ),
	  w( 0 )
	{
		assert( !HasNaN() );
	}
	constexpr Vector4( const T &x, const T &y, const T &z, const T &w ) :
	  x( x ),
	  y( y ),
	  z( z ),
	  w( w )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector4( const Vector3<T> &v ) :
	  x( v.x ),
	  y( v.y ),
	  z( v.z ),
	  w( 0.0 )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector4( const Vector2<T> &v ) :
	  x( v.x ),
	  y( v.y ),
	  z( 0.0 ),
	  w( 0.0 )
	{
		assert( !HasNaN() );
	}

	constexpr Vector4( const Vector3<T> &v, const T &a ) :
	  x( v.x ),
	  y( v.y ),
	  z( v.z ),
	  w( a )
	{
		assert( !HasNaN() );
	}

	constexpr Vector4( const Vector2<T> &v, const T &a1 ) :
	  x( v.x ),
	  y( v.y ),
	  z( a1 ),
	  w( T() )
	{
		assert( !HasNaN() );
	}

	constexpr Vector4( const Vector2<T> &v, const T &a1, const T &a2 ) :
	  x( v.x ),
	  y( v.y ),
	  z( a1 ),
	  w( a2 )
	{
		assert( !HasNaN() );
	}

	explicit constexpr Vector4( const T &a ) :
	  x( a ),
	  y( a ),
	  z( a ),
	  w( a )
	{
		assert( !HasNaN() );
	}

	bool HasNaN() const
	{
		return IsNaN( x ) || IsNaN( y ) || IsNaN( z ) || IsNaN( w );
	}

	constexpr Vector4<T> operator+( const Vector4<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Vector4<T>( x + v.x, y + v.y, z + v.z, w + v.w );
	}

	constexpr Vector4<T> &operator+=( const Vector4<T> &v )
	{
		assert( !v.HasNaN() );
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
		return *this;
	}

	constexpr Vector4<T> operator-( const Vector4<T> &v ) const
	{
		assert( !v.HasNaN() );
		return Vector4<T>( x - v.x, y - v.y, z - v.z, w - v.w );
	}

	constexpr Vector4<T> &operator-=( const Vector4<T> &v )
	{
		assert( !v.HasNaN() );
		x -= v.x;
		y -= v.y;
		z -= v.z;
		w -= v.w;
		return *this;
	}

	template <typename U>
	constexpr explicit operator Vector4<U>() const
	{
		return Vector4<U>( U( x ), U( y ), U( z ), U( w ) );
	}

	template <typename U>
	constexpr explicit operator Vector3<U>() const
	{
		return Vector3<U>( U( x ), U( y ), U( z ) );
	}

	template <typename U>
	constexpr explicit operator Vector2<U>() const
	{
		return Vector2<U>( U( x ), U( y ) );
	}

	//unary operator
	constexpr Vector4<T> operator-() const
	{
		return Vector4<T>( -x, -y, -z, -w );
	}

	constexpr Vector4<T> operator*( const Vector4<T> &v ) const
	{
		return { x * v.x, y * v.y, z * v.z, w * v.w };
	}

	constexpr Vector4<T>& operator*=( const Vector4<T> &v )
	{
		x *= v.x;
		y *= v.y;
		z *= v.z;
		w *= v.w;
		return *this;
	}

	constexpr Vector4<T> operator*( const Float s ) const
	{
		assert( !IsNaN( s ) );
		return Vector4<T>( s * x, s * y, s * z, s * w );
	}

	constexpr Vector4<T> &operator*=( const Float s )
	{
		assert( !IsNaN( s ) );
		x *= s;
		y *= s;
		z *= s;
		w *= s;
		return *this;
	}

	constexpr Vector4<T> operator/( Float s ) const
	{
		//assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		return Vector4<T>( x * inv, y * inv, z * inv, w * inv );
	}

	constexpr Vector4<T> &operator/=( Float s )
	{
		//	assert(!IsNaN(s));
		const auto inv = static_cast<Float>( 1 ) / s;
		x *= inv;
		y *= inv;
		z *= inv;
		;
		w *= inv;
		return *this;
	}

	constexpr bool operator==( const Vector4<T> &v ) const
	{
		return x == v.x && y == v.y && z == v.z && w == v.w;
	}

	constexpr bool operator!=( const Vector4<T> &v ) const
	{
		return !( *this == v );
	}

	const T &operator[]( int i ) const
	{
		assert( i >= 0 && i < 4 );
		return *( &x + i );
	}
	T &operator[]( int i )
	{
		assert( i >= 0 && i < 4 );
		return *( &x + i );
	}

	static Float Dot( const Vector4f &v1, const Vector4f &v2 )
	{
		return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
	}

	typename std::conditional<std::is_integral<T>::value, size_t, Float>::type Prod() const
	{
		using type = typename std::conditional<std::is_integral<T>::value, size_t, Float>::type;
		return type( x ) * type( y ) * type( z ) * type( w );
	}

	constexpr Float LengthSquared() const
	{
		return x * x + y * y + z * z + w * w;
	}

	constexpr Float Length() const
	{
		return std::sqrt( LengthSquared() );
	}

	constexpr Vector4<T> Normalized() const
	{
		//if len is too small?
		const auto len = Length();
		return ( *this ) / len;
	}

	constexpr void Normalize()
	{
		//if len is too small?
		const auto len = Length();
		( *this ) /= len;
	}

	constexpr bool IsNull() const
	{
		return x == 0 && y == 0 && z == 0 && w == 0;
	}

	T *Data()
	{
		return &x;
	}

	const T *ConstData() const
	{
		return &x;
	}
};

template <typename T>
std::ostream &operator<<( std::ostream &os, const Vector4<T> &v )
{
	os << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]";
	return os;
}

template <typename T>
constexpr Vector4<T> operator*( Float s, const Vector4<T> &v )
{
	return v * s;
}

template <typename T>
constexpr Vector4<T> Abs( const Vector4<T> &v )
{
	return Vector4<T>( std::abs( v.x ), std::abs( v.y ), std::abs( v.z ), std::abs( v.w ) );
}

//Geometry Inline Functions

//For Vector3<T>

template <typename T>
Vector3<T> Cross( const Vector3<T> &v1, const Vector3<T> &v2 )
{
	double v1x = v1.x, v1y = v1.y, v1z = v1.z;
	double v2x = v2.x, v2y = v2.y, v2z = v2.z;
	return Vector3<T>( ( v1y * v2z ) - ( v1z * v2y ), ( v1z * v2x ) - ( v1x * v2z ),
					   ( v1x * v2y ) - ( v1y * v2x ) );
}

template <typename T>
Vector3<T> Cross( const Vector3<T> &v1, const Normal3<T> &v2 )
{
	double v1x = v1.x, v1y = v1.y, v1z = v1.z;
	double v2x = v2.x, v2y = v2.y, v2z = v2.z;
	return Vector3<T>( ( v1y * v2z ) - ( v1z * v2y ), ( v1z * v2x ) - ( v1x * v2z ),
					   ( v1x * v2y ) - ( v1y * v2x ) );
}

template <typename T>
Vector3<T> Cross( const Normal3<T> &v1, const Vector3<T> &v2 )
{
	double v1x = v1.x, v1y = v1.y, v1z = v1.z;
	double v2x = v2.x, v2y = v2.y, v2z = v2.z;
	return Vector3<T>( ( v1y * v2z ) - ( v1z * v2y ), ( v1z * v2x ) - ( v1x * v2z ),
					   ( v1x * v2y ) - ( v1y * v2x ) );
}

template <typename T>
Float Dot( const Vector3<T> &v1, const Vector3<T> &v2 )
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T>
Float Dot( const Point3<T> &v1, const Point3<T> &v2 )
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T>
Float Dot( const Vector3<T> &v1, const Point3<T> &v2 )
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

template <typename T>
T Prod( const Vector2<T> &v )
{
	return v.x * v.y;
}

template <typename T>
T Prod( const Vector3<T> &v )
{
	return v.x * v.y * v.z;
}

template <typename T>
T Prod( const Vector4<T> &v )
{
	return v.x * v.y * v.z * v.w;
}

template <typename T>
Vector3<T> Normalize( const Vector3<T> &v )
{
	return v / v.Length();
}

template <typename T>
T MinComponent( const Vector3<T> &v )
{
	return ( std::min )( v.x, ( std::min )( v.x, v.y ) );
}

template <typename T>
T MaxComponent( const Vector3<T> &v )
{
	return ( std::max )( v.x, ( std::max )( v.x, v.y ) );
}

template <typename T>
int MaxDimension( const Vector3<T> &v )
{
	return ( v.x > v.y ) ? ( v.x > v.z ? 0 : 2 ) : ( v.y > v.z ? 1 : 2 );
}

template <typename T>
int MinDimension( const Vector3<T> &v )
{
	return ( v.x < v.y ) ? ( v.x < v.z ? 0 : 2 ) : ( v.y < v.z ? 1 : 2 );
}

template <typename T>
Vector3<T> Min( const Vector3<T> &p1, const Vector3<T> &p2 )
{
	return Vector3<T>( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ),
					   std::min( p1.z, p2.z ) );
}

template <typename T>
Vector3<T> Max( const Vector3<T> &p1, const Vector3<T> &p2 )
{
	return Vector3<T>( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ),
					   std::max( p1.z, p2.z ) );
}

template <typename T>
Vector3<T> Permute( const Vector3<T> &v, int x, int y, int z )
{
	return Vector3<T>( v[ x ], v[ y ], v[ z ] );
}

// For Point3<T>

template <typename T>
Point3<T> Min( const Point3<T> &p1, const Point3<T> &p2 )
{
	return Point3<T>( std::min( p1.x, p2.x ), std::min( p1.y, p2.y ),
					  std::min( p1.z, p2.z ) );
}

template <typename T>
Point3<T> Max( const Point3<T> &p1, const Point3<T> &p2 )
{
	return Point3<T>( std::max( p1.x, p2.x ), std::max( p1.y, p2.y ),
					  std::max( p1.z, p2.z ) );
}

template <typename T>
Point3<T> Permute( const Point3<T> &p, int x, int y, int z )
{
	return Point3<T>( p[ x ], p[ y ], p[ z ] );
}

template <typename T>
Point3<T> Normalize( const Point3<T> &v )
{
	return v / v.Length();
}

template <typename T>
T MinComponent( const Point3<T> &v )
{
	return ( std::min )( v.x, ( std::min )( v.x, v.y ) );
}

template <typename T>
T MaxComponent( const Point3<T> &v )
{
	return ( std::max )( v.x, ( std::max )( v.x, v.y ) );
}

template <typename T>
int MaxDimension( const Point3<T> &v )
{
	return ( v.x > v.y ) ? ( v.x > v.z ? 0 : 2 ) : ( v.y > v.z ? 1 : 2 );
}

template <typename T>
int MinDimension( const Point3<T> &v )
{
	return ( v.x < v.y ) ? ( v.x < v.z ? 0 : 2 ) : ( v.y < v.z ? 1 : 2 );
}

template <typename T>
class Bound2
{
public:
	Point2<T> min;
	Point2<T> max;
	Bound2() :
	  min( std::numeric_limits<T>::max(), std::numeric_limits<T>::max() ),
	  max( std::numeric_limits<T>::lowest(), std::numeric_limits<T>::lowest() )
	{
	}

	explicit Bound2( const Point2<T> &p ) :
	  min( p ),
	  max( p ) {}

	Bound2( const Point2<T> &p0, const Point2<T> &p1 ) :
	  min( ( std::min )( p0.x, p1.x ), ( std::min )( p0.y, p1.y ) ),
	  max( ( std::max )( p0.x, p1.x ), ( std::max )( p0.y, p1.y ) ) {}

	template <typename U>
	explicit operator Bound2<U>() const
	{
		return Bound2<U>( U( min ), U( max ) );
	}

	Vector2<T> Diagonal() const
	{
		return max - min;
	}

	const Point2<T> &operator[]( int i ) const
	{
		assert( i >= 0 && i < 2 );
		return *( &min + i );
	}

	Point2<T> &operator[]( int i )
	{
		return const_cast<Point2<T> &>( static_cast<const Bound2<T> &>( &this )[ i ] );
	}

	bool operator==( const Bound2<T> &b ) const
	{
		return b.min == min && b.max == max;
	}

	bool operator!=( const Bound2<T> &b ) const
	{
		return !( *this == b );
	}

	T Area() const
	{
		const auto d = Diagonal();
		return d.x * d.y;
	}

	int MaximumExtent() const
	{
		const auto d = Diagonal();
		if ( d.x > d.y )
			return 0;
		else
			return 1;
	}
};

class Ray
{
	///TODO:: Add Medium class pointer later
	//const Medium *medium;

public:
	Point3f o;
	Vector3f d;
	Float tMax;
	Float Time;
	bool negDirection[ 3 ];
	Ray() = default;
	Ray( const Vector3f &d, const Point3f &o, Float t = ( std::numeric_limits<float>::max )(), Float time = 0.f ) :
	  o( o ),
	  d( d.Normalized() ),
	  tMax( t ),
	  Time( time )
	{
		negDirection[ 0 ] = d.x < 0;
		negDirection[ 1 ] = d.y < 0;
		negDirection[ 2 ] = d.z < 0;
	}
	Point3f operator()( float t ) const noexcept { return o + t * d; }
	const Point3f &Original() const { return o; }
	const Vector3f &Direction() const { return d; }
	void SetMaxLength( Float t ) { tMax = t; }

	//friend class Bound3f;
	friend class Triangle;
	friend class Shape;
	friend class BVHTreeAccelerator;
};

class DifferentialRay : public Ray
{
public:
	DifferentialRay() = default;
	DifferentialRay( const Ray &ray ) :
	  Ray( ray ), Differential( false )
	{
	}
	DifferentialRay( const Vec3f &d, const Point3f &o, Float t = ( ( std::numeric_limits<Float>::max )() ), Float time = 0.f ) :
	  Ray( d, o, t, time ), Differential( false )
	{
	}
	void ScaleDifferentials( Float s )
	{
		Ox = o + ( Ox - o ) * s;
		Oy = o + ( Oy - o ) * s;
		Dx = d + ( Dx - d ) * s;
		Dy = d + ( Dy - d ) * s;
	}
	bool Differential = false;
	Point3f Ox, Oy;
	Vec3f Dx, Dy;
};

/**
	 * \brief A 3-dimension axis-aligned bounding box class
	 * \tparam T 
	 */
template <typename T>
class Bound3
{
public:
	Point3<T> min;
	Point3<T> max;

	constexpr Bound3() :
	  min( MAX_VALUE, MAX_VALUE, MAX_VALUE ),
	  max( LOWEST_FLOAT, LOWEST_FLOAT, LOWEST_FLOAT ){};

	constexpr Bound3( const Point3<T> &p0, const Point3<T> &p1 ) noexcept :
	  min( ( std::min )( p0.x, p1.x ), ( std::min )( p0.y, p1.y ), ( std::min )( p0.z, p1.z ) ),
	  max( ( std::max )( p0.x, p1.x ), ( std::max )( p0.y, p1.y ), ( std::max )( p0.z, p1.z ) )
	{
	}

	explicit Bound3( const Point3<T> &p ) :
	  min( p.min ),
	  max( p.max )
	{
	}

	const Point3f &operator[]( int i ) const
	{
		assert( i >= 0 && i < 2 );
		//if (i == 0)return m_min;
		//if (i == 1)return m_max;
		return *( &min + i );
	}

	Point3<T> &operator[]( int i )
	{
		return const_cast<Point3<T> &>( static_cast<const Bound3<T> &>( *this )[ i ] );
	}

	Point3<T> Corner( int i ) const
	{
		return Point3<T>{ ( *this )[ i & 1 ].x, ( *this )[ i & 2 ? 1 : 0 ].y, ( *this )[ i & 4 ? 1 : 0 ].z };
	}

	/*
		* Check whether it is intersected with a ray.
		* hit0 stores the nearest ray
		* parameter and hit1 stores the farthest ray parameter
		*/

	/*inline function definitions for AABB*/

	bool Intersect( const Ray &ray, Float *hit0 = nullptr, Float *hit1 = nullptr ) const noexcept
	{
		auto t1 = ray.tMax;
		auto t0 = 0.0;
		for ( auto i = 0; i < 3; i++ ) {
			const auto inv = 1 / ray.d[ i ];
			auto tNear = ( min[ i ] - ray.o[ i ] ) * inv;
			auto tFar = ( max[ i ] - ray.o[ i ] ) * inv;
			if ( tNear > tFar ) std::swap( tNear, tFar );
			t0 = tNear > t0 ? tNear : t0;
			t1 = tFar < t1 ? tFar : t1;
			if ( t0 > t1 ) return false;
		}
		if ( hit0 != nullptr ) *hit0 = t0;
		if ( hit1 != nullptr ) *hit1 = t1;
		return true;
	}

	template <typename U>
	explicit operator Bound3<U>() const
	{
		return Bound3<U>( (Point3<U>)min, (Point3<U>)max );
	}

	Point3<T> Center() const
	{
		return ( min + max ) / 2.0;
	}

	Vector3<T> Diagonal() const
	{
		return max - min;
	}

	T SurfaceArea() const
	{
		const auto d = Diagonal();
		if ( d[ 0 ] < 0 || d[ 1 ] < 0 || d[ 2 ] < 0 ) return Float( 0 );
		const auto area = ( d[ 0 ] * d[ 1 ] + d[ 1 ] * d[ 2 ] + d[ 2 ] * d[ 0 ] ) * 2;
		return area;
	}

	bool IsNull() const
	{
		return ( max.x <= min.x || max.y <= min.y || max.z <= min.z );
	}

	/**
	 * \brief 

	 * \note This function maybe return a non-positive value when \a IsNull() returns \a true 

	 * \sa IsNull()
	 */
	T Volume() const
	{
		const auto d = Diagonal();
		return d.x * d.y * d.z;
	}

	int MaximumExtent() const
	{
		const auto d = Diagonal();
		if ( d.x > d.y && d.x > d.z )
			return 0;
		else if ( d.y > d.z )
			return 1;
		else
			return 2;
	}

	/*
		*Check whether a point is in the bound
		*/

	bool Inside( const Point3<T> &p ) const
	{
		return ( p.x >= min.x && p.y <= max.x &&
				 p.y >= min.y && p.y <= max.y &&
				 p.z >= min.z && p.z <= max.z );
	}

	bool InsideEx( const Point3<T> &p ) const
	{
		return ( p.x >= min.x && p.y < max.x &&
				 p.y >= min.y && p.y < max.y &&
				 p.z >= min.z && p.z < max.z );
	}

	/**
		 * \brief Check whether the given \a bound is inside the bound
		 */
	bool Inside( const Bound3<T> &bound ) const
	{
		return (
		  bound.min.x >= min.x && bound.max.x < max.x && bound.min.y >= min.y && bound.max.y < max.y && bound.min.z >= min.z && bound.max.z < max.z );
	}

	bool InsideEx( const Bound3<T> &bound ) const
	{
		return (
		  bound.min.x >= min.x && bound.max.x <= max.x && bound.min.y >= min.y && bound.max.y <= max.y && bound.min.z >= min.z && bound.max.z <= max.z );
	}

	/*
		* Check whether the bounding box is
		* intersected with another bounding box
		*/

	bool IsIntersectWith( const Bound3<T> &b ) const
	{
		return ( max.x >= b.min.x && b.max.x >= min.x ) &&
			   ( max.y >= b.min.y && b.max.y >= min.y ) &&
			   ( max.z >= b.min.z && b.max.z >= min.z );
	}

	/*
		* return the common part of two bounding box
		*/

	Bound3<T> IntersectWidth( const Bound3<T> &b ) const
	{
		const auto pMin = Point3<T>{ ( std::max )( min.x, b.min.x ),
									 ( std::max )( min.y, b.min.y ),
									 ( std::max )( min.z, b.min.z ) };
		const auto pMax = Point3<T>{
			( std::min )( max.x, b.max.x ),
			( std::min )( max.y, b.max.y ),
			( std::min )( max.z, b.max.z )
		};
		Bound3<T> ret;
		ret.min = pMin;
		ret.max = pMax;
		return ret;
	}

	/*
		* Return a minimum bounding box containing the two bounding boxes
		*/
	Bound3<T> UnionWith( const Bound3<T> &b ) const
	{
		Bound3<T> ret;
		ret.min = Point3<T>{
			( std::min )( min.x, b.min.x ),
			( std::min )( min.y, b.min.y ),
			( std::min )( min.z, b.min.z )
		};
		ret.max = Point3<T>{
			( std::max )( max.x, b.max.x ),
			( std::max )( max.y, b.max.y ),
			( std::max )( max.z, b.max.z )
		};	// For fucking min/max defined in windows.h
		return ret;
	}

	/*
		* Return a minimun bounding box containing the
		* bounding box and the point
		*/
	Bound3<T> UnionWith( const Point3<T> &p ) const
	{
		Bound3<T> a;
		a.min = Point3<T>(
		  ( std::min )( min.x, p.x ),
		  ( std::min )( min.y, p.y ),
		  ( std::min )( min.z, p.z ) );
		a.max = Point3<T>(
		  ( std::max )( max.x, p.x ),
		  ( std::max )( max.y, p.y ),
		  ( std::max )( max.z, p.z ) );
		return a;
	}

	Grid<T> GenGrid( const Vec3i &grid ) const;

	friend class BVHTreeAccelerator;
};

using Bound3f = Bound3<Float>;
using Bound3i = Bound3<int>;
using Bound2f = Bound2<Float>;
using Bound2i = Bound2<int>;

class RayIntervalIter
{
	Vec3f deltaT, accumT;
	Vec3i grid;
	bool negRayDir[ 3 ];
	RayIntervalIter( const Vec3f &rayDirection,	 // normalized
					 const Vec3f &cellDimension,
					 const Vec3f &rayOrigGrid,
					 const Point3i &initCellIndex,
					 const Vec3i &grid,
					 float &tMin ,
					 float tMax) :
	  Pos( tMin ), grid( grid ),Max(tMax)
	{
		// The ray-grid intersection algorithm is modified from 
		// https://www.scratchapixel.com/lessons/advanced-rendering/introduction-acceleration-structure/grid. See it for more detail
		for ( int i = 0; i < 3; i++ ) {
			if ( rayDirection[ i ] < 0 ) {
				deltaT[ i ] = -cellDimension[ i ] / rayDirection[ i ];
				accumT[ i ] = ( floor( rayOrigGrid[ i ] / cellDimension[ i ] ) * cellDimension[ i ] - rayOrigGrid[ i ] ) / rayDirection[ i ];
				negRayDir[ i ] = true;
			} else {
				deltaT[ i ] = cellDimension[ i ] / rayDirection[ i ];
				accumT[ i ] = ( ( floor( rayOrigGrid[ i ] / cellDimension[ i ] ) + 1 ) * cellDimension[ i ] - rayOrigGrid[ i ] ) / rayDirection[ i ];
				negRayDir[ i ] = false;
			}
			accumT[i] += tMin; 
			// we assume that rayOrigGrid is begin from the intersection of the ray and bound box, 
			// so the global ray parameter is yield by adding the offset onto the local ray parameters
			assert( accumT[ i ] >= 0 );
			assert( deltaT[ i ] >= 0 );
		}
		CellIndex = initCellIndex;
	}

	inline void _next()
	{
		int mini[ 3 ];
		int cnt = 0;
		if ( accumT[ 0 ] < accumT[ 1 ] ) {
			if ( accumT[ 0 ] < accumT[ 2 ] ) {
				mini[ cnt++ ] = 0;	// 0
			} else {				// 2<=0<1
				mini[ cnt++ ] = 2;	// 2
				if ( accumT[ 2 ] == accumT[ 0 ] ) {
					mini[ cnt++ ] = 0;	// 2, 0
				}
			}
		} else {  // 1<=0
			if ( accumT[ 1 ] < accumT[ 2 ] ) {
				mini[ cnt++ ] = 1;	// 1
				if ( accumT[ 0 ] == accumT[ 1 ] ) {
					mini[ cnt++ ] = 0;	// 1, 0
				}
			} else {  // 2<=1<=0,
				mini[ cnt++ ] = 2;
				if ( accumT[ 2 ] == accumT[ 1 ] ) {
					mini[ cnt++ ] = 1;	// 1,2
					if ( accumT[ 1 ] == accumT[ 0 ] ) {
						mini[ cnt++ ] = 0;	// 2, 1, 0
					}
				}
			}
		}
		assert(cnt != 0);
		for ( int c = 0; c < cnt; c++ ) {
			auto i = mini[c];
			Pos = accumT[i];
			accumT[ i ] += deltaT[ i ];
			if ( negRayDir[ i ] )
				CellIndex[ i ] -= 1;
			else
				CellIndex[ i ] += 1;
		}
	}

	RayIntervalIter empty()
	{
		RayIntervalIter iter;
		return iter;
	}

	template <typename T>
	friend class Grid;

public:
	float Pos = 0.0f, Max;
	Point3i CellIndex = {};
	RayIntervalIter() = default;
	RayIntervalIter operator++( int )
	{
		RayIntervalIter itr = *this;
		_next();
		return itr;
	}

	RayIntervalIter &operator++()
	{
		_next();
		return *this;
	}

	RayIntervalIter &Next()
	{
		_next();
		return *this;
	}

	bool Valid() const
	{
		for ( int i = 0; i < 3; i++ ) {
			if ( CellIndex[ i ] < 0 || CellIndex[ i ] >= grid[ i ] ) {
				return false;
			}
		}
		return true;
	}
};

template <typename T>
class Grid
{
public:
	Bound3<T> Bound;
	Vec3f Cell;
	Vec3i GridDimension;
	Grid( const Bound3<T> &bound, const Vec3i &grid ) :
	  Bound( bound ), GridDimension( grid )
	{
		auto diag = Vec3f( bound.Diagonal() );
		Cell = Vec3f( diag.x / grid.x, diag.y / grid.y, diag.z / grid.z );
	}

	RayIntervalIter IntersectWith( const Ray &ray ) const
	{
		float hit0, hit1;
		if ( Bound.Intersect( ray, &hit0, &hit1 ) ) {
			const auto hit = ray( hit0 + 0.001 );
			const auto v = hit - Vec3f( Bound.min );
			Vec3f rayOrigGrid = hit - Point3f( Bound.min );
			const Point3i initCell( v.x / Cell.x, v.y / Cell.y, v.z / Cell.z );
			return RayIntervalIter( ray.Direction().Normalized(), Cell, rayOrigGrid, initCell, GridDimension, hit0, hit1 );
		}
		return RayIntervalIter();
	};

	void IntersectWith(const Ray & ray, Point3i*res, int* count)const{

	}

};

template <typename T>
Grid<T> Bound3<T>::GenGrid( const Vec3i &grid ) const
{
	return Grid<T>( *this, grid );
}

}  // namespace vm

#endif	// GEOMETRY_H
