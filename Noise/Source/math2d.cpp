#include "math2d.h"

double distToLine(const Point& p, const Point& a, const Point& b, Point& c)
{
	const Vec ap(a, p);
	const Vec ab(a, b);
	const double u = dot(ap, ab) / norm_sq(ab);

	c = translate(a, scale(ab, u));

	return dist(p, c);
}

double distToLineSegment(const Point& p, const Point& a, const Point& b, Point& c)
{
	const Vec ap(a, p);
	const Vec ab(a, b);
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
		c = translate(a, scale(ab, u));
		return dist(p, c);
	}
}