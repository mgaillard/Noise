#ifndef MATH2D_H
#define MATH2D_H

#include <cmath>

struct Point2D
{
	double x;
	double y;

	Point2D() : x(0.0), y(0.0) { }

	Point2D(double _x, double _y) : x(_x), y(_y) { }
};

struct Vec2D
{
	double x;
	double y;

	Vec2D() : x(0.0), y(0.0) { }

	Vec2D(double _x, double _y) : x(_x), y(_y) { }

	Vec2D(const Point2D& a, const Point2D& b) : x(b.x - a.x), y(b.y - a.y) { }
};

struct Segment2D
{
	Point2D a;
	Point2D b;

	Segment2D() = default;

	Segment2D(const Point2D& _a, const Point2D& _b) : a(_a), b(_b) { }
};

inline double dist(const Point2D& lhs, const Point2D& rhs)
{
	return hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline double norm_sq(const Vec2D& a)
{
	return a.x * a.x + a.y * a.y;
}

inline Vec2D scale(const Vec2D& v, double s)
{
	return Vec2D(v.x * s, v.y * s);
}

inline double dot(const Vec2D& a, const Vec2D& b)
{
	return a.x * b.x + a.y * b.y;
}

inline Point2D translate(const Point2D& p, const Vec2D& v)
{
	return Point2D(p.x + v.x, p.y + v.y);
}

double pointLineProjection(const Point2D& p, const Point2D& a, const Point2D& b);

inline double pointSegmentProjection(const Point2D& p, const Segment2D& s)
{
	return pointLineProjection(p, s.a, s.b);
}

double distToLine(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c);

double distToLineSegment(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c);

inline double distToLineSegment(const Point2D& p, const Segment2D& s, Point2D& c)
{
	return distToLineSegment(p, s.a, s.b, c);
}

#endif // MATH2D_H