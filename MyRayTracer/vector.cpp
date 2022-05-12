#include <cmath>
#include "vector.h"

Vector::Vector(float a_x, float a_y, float a_z) : x(a_x), y(a_y), z(a_z)
{

}


float Vector::length()
{
	return sqrt( x * x + y * y + z * z );
}

float Vector::getAxisValue(int axis) {
	return (axis == 0) ? x : (axis == 1) ? y : z;
}

float Vector::distance(const Vector& v, const Vector& w) {
	return sqrt(pow(w.x - v.x, 2) + pow(w.y - v.y, 2) * 1.0);
}

// --------------------------------------------------------------------- copy constructor
Vector::Vector(const Vector& v)
{
	x = v.x; y = v.y; z = v.z;
}

// --------------------------------------------------------------------- assignment operator
Vector Vector::operator= (const Vector& rhs) {
	if (this == &rhs)
		return (*this);
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
	return (*this);
}

Vector Vector::operator+(const  Vector& v )
{
	return Vector( x + v.x, y + v.y, z + v.z );
}


Vector Vector::operator-(const Vector& v )
{
	return Vector( x - v.x, y - v.y, z - v.z );
}


Vector Vector::operator*( float f )
{
	return Vector( x * f, y * f, z * f );
}

float Vector::operator*(const  Vector& v)
{
	return x * v.x + y * v.y + z * v.z;
}

Vector Vector::operator/( float f )
{
	return Vector( x / f, y / f, z / f );
}

Vector&	Vector::normalize	()
{
				   float l=1.0/this->length();
				   x *= l; y *= l; z *= l;
				   return *this;
}

Vector&	Vector::operator-=(const Vector& v)
{ x-=v.x; y-=v.y; z-=v.z; return *this; }

Vector&	Vector::operator-=(const float v)
{ x-=v; y-=v; z-=v; return *this; }

Vector&	Vector::operator+=(const float v)
{ x+=v; y+=v; z+=v; return *this; }

Vector&	Vector::operator*=(const float v)
{ x*=v; y*=v; z*=v; return *this; }

Vector Vector::operator%( const Vector& v)
{
	float uX = x;
	float uY = y;
	float uZ = z;

	float vX = v.x;
	float vY = v.y;
	float vZ = v.z;

	float sX = uY * vZ - uZ * vY;
	float sY = uZ * vX - uX * vZ;
	float sZ = uX * vY - uY * vX;

	return Vector( sX, sY, sZ );
}

Vector& Vector::crossProduct(const Vector& v) {
	float cross_x = y * v.z - z * v.y;
	float cross_y = z * v.x - x * v.z;
	float cross_z = x * v.y - y * v.x;
	return Vector(cross_x, cross_y, cross_z);
}

float Vector::dotProduct(const Vector& v) {
	float dot_x = x * v.x;
	float dot_y = y * v.y;
	float dot_z = z * v.z;

	return dot_x + dot_y + dot_z;
}