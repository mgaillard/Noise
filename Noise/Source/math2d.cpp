#include "math2d.h"

Point2D& Point2D::operator+=(const Vec2D& v)
{
	x += v.x;
	y += v.y;
	return *this;
}

Point2D& Point2D::operator-=(const Vec2D& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

double angle(const Point2D& a, const Point2D& o, const Point2D& b)
{
	const Vec2D oa(o, a);
	const Vec2D ob(o, b);
	return angle(oa, ob);
}

double pointLineProjection(const Point2D& p, const Point2D& a, const Point2D& b)
{
	const Vec2D ap(a, p);
	const Vec2D ab(a, b);
	// Project vector AP on AB
	return dot(ap, ab) / norm_sq(ab);
}

double pointLineProjection(const Point2D& p, const Segment2D& s)
{
	return pointLineProjection(p, s.a, s.b);
}

double pointLineSegmentProjection(const Point2D& p, const Point2D& a, const Point2D& b)
{
	const double u = pointLineProjection(p, a, b);
	return clamp(u, 0.0, 1.0);
}

double pointLineSegmentProjection(const Point2D& p, const Segment2D& s)
{
	return pointLineSegmentProjection(p, s.a, s.b);
}

double distToLine(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c)
{
	const Vec2D ap(a, p);
	const Vec2D ab(a, b);
	const double u = dot(ap, ab) / norm_sq(ab);

	c = a + ab * u;

	return dist(p, c);
}

double distToLineSegment(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c)
{
	const Vec2D ap(a, p);
	const Vec2D ab(a, b);
	const double u = dot(ap, ab) / norm_sq(ab);

	if (u < 0.0)
	{
		// Closer to A
		c = a;
		return dist(p, a);
	}
	else if (u > 1.0)
	{
		// Closer to B
		c = b;
		return dist(p, b);
	}
	else
	{
		// Between A and B, equivalent to distToLine(p, a, b, c);
		c = a + ab * u;
		return dist(p, c);
	}
}

double distToLineSegment(const Point2D& p, const Segment2D& s, Point2D& c)
{
	return distToLineSegment(p, s.a, s.b, c);
}
