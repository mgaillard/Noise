#ifndef MATH2D_H
#define MATH2D_H

#include <cmath>

struct Point
{
	double x;
	double y;

	Point() = default;

	Point(double _x, double _y) : x(_x), y(_y) { }
};

struct Vec
{
	double x;
	double y;

	Vec() : x(0.0), y(0.0) { }

	Vec(double _x, double _y) : x(_x), y(_y) { }

	Vec(const Point& a, const Point& b) : x(b.x - a.x), y(b.y - a.y) { }
};

struct Segment
{
	Point a;
	Point b;

	Segment() = default;

	Segment(const Point& _a, const Point& _b) : a(_a), b(_b) { }
};

inline double dist(const Point& lhs, const Point& rhs)
{
	return hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline double norm_sq(const Vec& a)
{
	return a.x * a.x + a.y * a.y;
}

inline Vec scale(const Vec& v, double s)
{
	return Vec(v.x * s, v.y * s);
}

inline double dot(const Vec& a, const Vec& b)
{
	return a.x * b.x + a.y * b.y;
}

inline Point translate(const Point& p, const Vec& v)
{
	return Point(p.x + v.x, p.y + v.y);
}

double distToLine(const Point& p, const Point& a, const Point& b, Point& c);

double distToLineSegment(const Point& p, const Point& a, const Point& b, Point& c);

inline double distToLineSegment(const Point& p, const Segment& s, Point& c)
{
	return distToLineSegment(p, s.a, s.b, c);
}

#endif // MATH2D_H