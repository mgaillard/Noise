#include "math3d.h"

Point3D& Point3D::operator+=(const Vec3D& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

Point3D& Point3D::operator-=(const Vec3D& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}
