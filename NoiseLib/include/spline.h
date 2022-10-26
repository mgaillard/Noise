#ifndef SPLINE_H
#define SPLINE_H

#include "math2d.h"
#include "math3d.h"

// Chordal Catmull-Rom spline
// t is an absolute time
Point2D CatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, double t);
Point3D CatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t);

// Subdivide the segment between p1 and p2 using a chordal Catmull-Rom spline
// x is the proportion of time between t1 and t2
Point2D SubdivideCatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, double x);
Point3D SubdivideCatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double x);

// Subdivide the segment between p1 and p2 using a chordal Catmull-Rom spline in N points
template <size_t N>
std::array<Point2D, N> SubdivideCatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
	std::array<Point2D, N> points;

	for (int n = 0; n < points.size(); n++)
	{
		const double t = double(n + 1) / (N + 1);
		points[n] = SubdivideCatmullRomSpline(p0, p1, p2, p3, t);
	}

	return points;
}

// Subdivide the segment between p1 and p2 using a chordal Catmull-Rom spline in N points
template <size_t N>
std::array<Point3D, N> SubdivideCatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
	std::array<Point3D, N> points;

	for (int n = 0; n < points.size(); n++)
	{
		const double t = double(n + 1) / (N + 1);
		points[n] = SubdivideCatmullRomSpline(p0, p1, p2, p3, t);
	}

	return points;
}

#endif // SPLINE_H