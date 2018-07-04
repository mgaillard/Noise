#include "height.h"

double ComputeHeightSegmentsProjection(const std::array<Segment3D, 2>& segments, const Point2D& point)
{
	double value = 0.0;

	double u1 = pointLineSegmentProjection(point, ProjectionZ(segments[0]));
	Point3D p1 = lerp(segments[0], u1);
	double d1 = dist(point, ProjectionZ(p1));

	double u2 = pointLineSegmentProjection(point, ProjectionZ(segments[1]));
	Point3D p2 = lerp(segments[1], u2);
	double d2 = dist(point, ProjectionZ(p2));

	// Smoothstep the projections
	u1 = u1 * u1 * u1 * (u1 * (u1 * 6.0 - 15.0) + 10.0);
	u2 = u2 * u2 * u2 * (u2 * (u2 * 6.0 - 15.0) + 10.0);

	if ((1 - u1) + u2 == 0.0) {
		value = (p1.z + d1 + p2.z + d2) / 2.0;
	}
	else {
		value = ((1 - u1) * (p1.z + d1) + u2 * (p2.z + d2)) / ((1 - u1) + u2);
	}

	return value;
}

double ComputeHeightSegmentsDistance(const std::array<Segment3D, 3>& segments, const Point2D& point)
{
	double value = 0.0;

	double f1 = std::numeric_limits<double>::max();
	double f2 = std::numeric_limits<double>::max();
	Segment3D s1, s2;
	Point3D p1, p2;

	// Two nearest segments
	for (const Segment3D& s : segments)
	{
		double u = pointLineSegmentProjection(point, ProjectionZ(s));
		Point3D p = lerp(s, u);
		double d = dist(point, ProjectionZ(p));

		if (d < f1)
		{
			f2 = f1;
			p2 = p1;
			s2 = s1;

			f1 = d;
			p1 = p;
			s1 = s;
		}
		else if (d < f2)
		{
			f2 = d;
			p2 = p;
			s2 = s;
		}
	}

	// Where is the connection between the 2 segments
	Point3D connection;
	if (s1.a == s2.a || s1.a == s2.b)
	{
		connection = s1.a;
	}
	else if (s1.b == s2.a || s1.b == s2.b)
	{
		connection = s1.b;
	}
	else
	{
		// Problem: no connection, that's not supposed to happen.
		assert(false);
	}

	if (f1 == f2)
	{
		// (d1 ; d1) => H_i (connection.z + d1)
		value = connection.z + f1;
	}
	else if (f2 > f1)
	{
		// (0 ; d2) => p1.z
		// (d2 ; d2) => H_i (connection.z + d2)
		value = lerp(p1.z, connection.z + f2, f1 / f2);
	}
	else if (f2 < f1)
	{
		// (d1 ; 0) => p2.z
		// (d1 ; d1) => H_i (connection.z + d1)
		value = lerp(p2.z, connection.z + f1, f2 / f1);
	}

	return value;
}

double TestMerge(double x, double y)
{
	double value = 0.0;

	Point2D point(x, y);

	std::array<Point3D, 6> points = {
		{
			{ 1.0, 1.5, 0.5 },
			{ 1.0, 2.0, 0.5 },
			{ 1.0, 2.5, 0.5 },
			{ 1.0, 3.0, 0.5 },
			{ 2.5, 1.5, 0.5 },
			{ 3.5, 3.5, 0.5 }
		}
	};

	double numerator = 0.0;
	double denominator = 0.0;

	for (int i = 0; i < points.size(); i++)
	{
		const double R = 1.5;

		double d = dist(point, ProjectionZ(points[i]));

		double alpha = 0.0;
		if (d < R) {
			alpha = (1 - (d / R) * (d / R)) * (1 - (d / R) * (d / R)) * (1 - (d / R) * (d / R));
		}

		numerator += alpha * (points[i].z + 1.0 * d);
		denominator += alpha;
	}

	if (denominator != 0.0) {
		value = numerator / denominator;
	}

	return value;
}

double TestConvolution(double x, double y)
{
	const double R = 1.5;
	const Point2D point(x, y);

	double value = 0.0;

	const Point3D a(1.0, 1.0, 0.5);
	const Point3D b(2.0, 1.0, 1.0);
	const Point3D c(3.0, 2.0, 2.0);

	std::array<Segment3D, 2> segments = { {
		{ a, b },
		{ b, c }
	} };

	Point2D placeholderPoint;

	double d1 = distToLineSegment(point, ProjectionZ(segments[0]), placeholderPoint);
	double d2 = distToLineSegment(point, ProjectionZ(segments[1]), placeholderPoint);

	// Closed-form
	for (const Segment3D& segment : segments)
	{
		const double R0 = segment.a.z;
		const double R1 = segment.b.z;
		Segment2D segment2d = ProjectionZ(segment);

		double u = pointLineSegmentProjection(point, segment2d);
		Point2D h = ProjectionZ(lerp(segment, u));

		double d = dist(point, h);
		double l = length(segment);
		double a0 = -u * l;
		double a1 = (1.0 - u) * l;

		double A = atan(a1 / d) + atan(-a0 / d);
		double B = log((a1 * a1 + d * d) / (a0 * a0 + d * d));
		double C = (R0 - R1) * (R0 - R1);
		double D = R0 * a1 - R1 * a0;
		double E = a0 - a1;


		double t1 = (d * C - (D * D) / d) * A;
		double t2 = (R0 - R1) * D * B;
		double t3 = E * C;

		value -= (t1 + t2 + t3) / (E * E);
	}

	value = log(value);

	return value;
}
