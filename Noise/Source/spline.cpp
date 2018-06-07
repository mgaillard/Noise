#include "spline.h"

#include <cassert>

Point2D CatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, double t)
{
	const double t0 = 0.0;
	const double t1 = dist(p0, p1) + t0;
	const double t2 = dist(p1, p2) + t1;
	const double t3 = dist(p2, p3) + t2;

	assert(t0 != t1);
	assert(t0 != t2);
	assert(t1 != t2);
	assert(t1 != t3);
	assert(t2 != t3);

	const Point2D a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
	const Point2D a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
	const Point2D a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));

	const Point2D b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
	const Point2D b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));

	return b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1));
}

Point3D CatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t)
{
	const double t0 = 0.0;
	const double t1 = dist(p0, p1) + t0;
	const double t2 = dist(p1, p2) + t1;
	const double t3 = dist(p2, p3) + t2;

	assert(t0 != t1);
	assert(t0 != t2);
	assert(t1 != t2);
	assert(t1 != t3);
	assert(t2 != t3);

	const Point3D a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
	const Point3D a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
	const Point3D a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));

	const Point3D b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
	const Point3D b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));

	return b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1));
}

Point2D SubdivideCatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
	const double t0 = 0.0;
	const double t1 = dist(p0, p1) + t0;
	const double t2 = dist(p1, p2) + t1;
	const double t3 = dist(p2, p3) + t2;

	assert(t0 != t1);
	assert(t0 != t2);
	assert(t1 != t3);
	assert(t2 != t3);

	// Evaluate the Spline between p1 and p2
	const double t = (t1 + t2) / 2.0;

	const Point2D a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
	const Point2D a2 = p1 * 0.5 + p2 * 0.5;
	const Point2D a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));

	const Point2D b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
	const Point2D b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));

	return b1 * 0.5 + b2 * 0.5;
}

Point3D SubdivideCatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3)
{
	const double t0 = 0.0;
	const double t1 = dist(p0, p1) + t0;
	const double t2 = dist(p1, p2) + t1;
	const double t3 = dist(p2, p3) + t2;

	assert(t0 != t1);
	assert(t0 != t2);
	assert(t1 != t3);
	assert(t2 != t3);

	// Evaluate the Spline between p1 and p2
	const double t = (t1 + t2) / 2.0;

	const Point3D a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
	const Point3D a2 = p1 * 0.5 + p2 * 0.5;
	const Point3D a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));

	const Point3D b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
	const Point3D b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));

	return b1 * 0.5 + b2 * 0.5;
}