#ifndef MATH3D_H
#define MATH3D_H

#include <cmath>

#include "utils.h"
#include "math2d.h"

struct Point3D
{
	double x;
	double y;
	double z;

	Point3D() : x(0.0), y(0.0), z(0.0) { }

	Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }

	bool operator==(const Point3D& other) const
	{
		return ((fabs(x - other.x) < EPS) && (fabs(y - other.y) < EPS) && (fabs(z - other.z) < EPS));
	}

	bool operator!=(const Point3D& other) const
	{
		return !(operator==(other));
	}

	Point3D& operator+=(const Point3D& p)
	{
		x += p.x;
		y += p.y;
		z += p.z;
		return *this;
	}

	Point3D& operator-=(const Point3D& p)
	{
		x -= p.x;
		y -= p.y;
		z -= p.z;
		return *this;
	}

	Point3D& operator*=(double s)
	{
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	Point3D& operator/=(double s)
	{
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}

	static const double EPS;
};

struct Vec3D
{
	double x;
	double y;
	double z;

	Vec3D() : x(0.0), y(0.0), z(0.0) { }

	Vec3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }

	Vec3D(const Point3D& a, const Point3D& b) : x(b.x - a.x), y(b.y - a.y), z(b.z - a.z) { }
};

struct Segment3D
{
	Point3D a;
	Point3D b;

	Segment3D() = default;

	Segment3D(const Point3D& _a, const Point3D& _b) : a(_a), b(_b) { }
};

inline double dist_sq(const Point3D& lhs, const Point3D& rhs)
{
	return (lhs.x - rhs.x) * (lhs.x - rhs.x)
		 + (lhs.y - rhs.y) * (lhs.y - rhs.y)
		 + (lhs.z - rhs.z) * (lhs.z - rhs.z);
}

inline double dist(const Point3D& lhs, const Point3D& rhs)
{
	return sqrt(dist_sq(lhs, rhs));
}

inline double norm_sq(const Vec3D& a)
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

inline Point3D lerp(const Point3D& a, const Point3D& b, double t)
{
	return Point3D(
		lerp(a.x, b.x, t),
		lerp(a.y, b.y, t),
		lerp(a.z, b.z, t)
	);
}

inline Point3D lerp(const Segment3D& s, double t)
{
	return lerp(s.a, s.b, t);
}

inline Point2D ProjectionZ(const Point3D& p)
{
	return Point2D(p.x, p.y);
}

inline Vec2D ProjectionZ(const Vec3D& v)
{
	return Vec2D(v.x, v.y);
}

inline Segment2D ProjectionZ(const Segment3D& s)
{
	return Segment2D(ProjectionZ(s.a), ProjectionZ(s.b));
}

inline Point3D MidPoint(const Segment3D& s)
{
	return Point3D(
		(s.a.x + s.b.x) / 2.0,
		(s.a.y + s.b.y) / 2.0,
		(s.a.z + s.b.z) / 2.0
	);
}

inline Point3D translate(const Point3D& p, const Vec3D& v)
{
	return Point3D(p.x + v.x, p.y + v.y, p.z + v.z);
}

inline Point3D operator*(const Point3D& a, double s)
{
	return Point3D(a) *= s;
}

inline Point3D operator*(double s, const Point3D& a)
{
	return Point3D(a) *= s;
}

inline Point3D operator+(const Point3D& a, const Point3D& b)
{
	return Point3D(a) += b;
}

inline Point3D operator-(const Point3D& a, const Point3D& b)
{
	return Point3D(a) -= b;
}

#endif // MATH3D_H