#ifndef MATH3D_H
#define MATH3D_H

#include <cmath>

#include "math2d.h"

struct Point3D
{
	double x;
	double y;
	double z;

	Point3D() : x(0.0), y(0.0), z(0.0) { }

	Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
};

struct Segment3D
{
	Point3D a;
	Point3D b;

	Segment3D() = default;

	Segment3D(const Point3D& _a, const Point3D& _b) : a(_a), b(_b) { }
};

inline Point2D ProjectionZ(const Point3D& p)
{
	return Point2D(p.x, p.y);
}

inline Segment2D ProjectionZ(const Segment3D& s)
{
	return Segment2D(ProjectionZ(s.a), ProjectionZ(s.b));
}

#endif // MATH3D_H