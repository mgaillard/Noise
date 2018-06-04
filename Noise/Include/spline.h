#ifndef SPLINE_H
#define SPLINE_H

#include "math2d.h"
#include "math3d.h"

// Chordal Catmull-Rom spline
Point2D CatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3, double t);
Point3D CatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3, double t);

// Subdivide the segment between p1 and p2 using a chordal Catmull-Rom spline
Point2D SubdivideCatmullRomSpline(const Point2D& p0, const Point2D& p1, const Point2D& p2, const Point2D& p3);
Point3D SubdivideCatmullRomSpline(const Point3D& p0, const Point3D& p1, const Point3D& p2, const Point3D& p3);

#endif // SPLINE_H