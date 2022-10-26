#include "math2d.h"

#include <algorithm>

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

	// Segment is only a point and has no length
	if (norm_sq(ab) <= 0.0)
	{
		// The nearest point on the segment is A (or B)
		return 0.0;
	}

	// Segment has a length greater than 0
	// Projection of the point p on the line (AB)
	return dot(ap, ab) / norm_sq(ab);
}

double pointLineProjection(const Point2D& p, const Segment2D& s)
{
	return pointLineProjection(p, s.a, s.b);
}

double pointLineSegmentProjection(const Point2D& p, const Point2D& a, const Point2D& b)
{
	const double u = pointLineProjection(p, a, b);
	return std::clamp(u, 0.0, 1.0);
}

double pointLineSegmentProjection(const Point2D& p, const Segment2D& s)
{
	return pointLineSegmentProjection(p, s.a, s.b);
}

double distToLine(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c)
{
	const Vec2D ab(a, b);
	const double u = pointLineProjection(p, a, b);

	c = a + ab * u;

	return dist(p, c);
}

double distToLineSegment(const Point2D& p, const Point2D& a, const Point2D& b, Point2D& c)
{
	const Vec2D ab(a, b);
	const double u = pointLineProjection(p, a, b);

	if (u < 0.0)
	{
		// P is closer to A
		c = a;
		return dist(p, a);
	}
	
	if (u > 1.0)
	{
		// P is closer to B
		c = b;
		return dist(p, b);
	}

	// Projection of P is between A and B, equivalent to distToLine(p, a, b, c);
	c = a + ab * u;
	return dist(p, c);
}

double distToLineSegment(const Point2D& p, const Segment2D& s, Point2D& c)
{
	return distToLineSegment(p, s.a, s.b, c);
}
