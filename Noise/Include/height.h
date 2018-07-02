#ifndef HEIGHT_H
#define HEIGHT_H

#include <array>

#include "utils.h"
#include "math2d.h"
#include "math3d.h"

// Experimental
// Compute the height around an array of connected segments
// const Point3D a(0.5, 1.5, 1.5);
// const Point3D b(2.0, 1.5, 1.5);
// const Point3D c(3.5, 2.5, 1.0);
// 
// std::array<Segment3D, 2> segments = { {
//     { a, b },
//     { b, c }
// } };
//
// double value = ComputeHeightSegmentsProjection(segments, point);
double ComputeHeightSegmentsProjection(const std::array<Segment3D, 2>& segments, const Point2D& point);

// Experimental
// Compute the height around an array of connected segments
// 
// const Point2D point(x, y);
// 
// const Point3D a(0.5, 1.5, 0.5);
// const Point3D b(2.0, 1.5, 1.5);
// const Point3D c(3.5, 2.5, 1.0);
// const Point3D d(4.0, 3.5, 0.5);
// 
// std::array<Segment3D, 3> segments = { {
//     { a, b },
//     { b, c },
//     { c, d }
// } };
//
// double value = ComputeHeightSegmentsDistance(segments, point);
double ComputeHeightSegmentsDistance(const std::array<Segment3D, 3>& segments, const Point2D& point);

#endif // HEIGHT_H