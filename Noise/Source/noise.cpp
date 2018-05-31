#include "noise.h"

#include <iostream>
#include <vector>
#include <cstdint>
#include <limits>
#include <cmath>
#include <tuple>
#include <cassert>

#include "perlin.h"
#include "math2d.h"
#include "math3d.h"
#include "utils.h"

using namespace std;

Noise::Noise(const Point2D& noiseTopLeft, const Point2D& noiseBottomRight, const Point2D& perlinTopLeft, const Point2D& perlinBottomRight, int seed, double eps, bool displayPoints, bool displaySegments, bool displayGrid) :
	m_seed(seed),
	m_displayPoints(displayPoints),
	m_displaySegments(displaySegments),
	m_displayGrid(displayGrid),
	m_noiseTopLeft(noiseTopLeft),
	m_noiseBottomRight(noiseBottomRight),
	m_perlinTopLeft(perlinTopLeft),
	m_perlinBottomRight(perlinBottomRight),
	m_eps(eps)
{

}

int Noise::GenerateSeedNoise(int i, int j) const
{
	// TODO: implement a better permutation method
	return (541 * i + 79 * j + m_seed) % numeric_limits<int>::max();
}

Point2D Noise::GeneratePoint(int x, int y) const
{
	// Fixed seed for internal consistency
	const int seed = GenerateSeedNoise(x, y);
	RandomGenerator generator(seed);

	uniform_real_distribution<double> distribution(m_eps, 1.0 - m_eps);
	const double px = distribution(generator);
	const double py = distribution(generator);

	return Point2D(double(x) + px, double(y) + py);
}

array<array<Point2D, 5>, 5> Noise::GenerateNeighboringPoints5(int cx, int cy) const
{
	array<array<Point2D, 5>, 5> points;

	// Exploring neighboring cells
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			const int x = cx + j - int(points[i].size()) / 2;
			const int y = cy + i - int(points.size()) / 2;

			points[i][j] = GeneratePoint(x, y);
		}
	}

	return points;
}

array<array<Point2D, 7>, 7> Noise::GenerateNeighboringPoints7(int cx, int cy) const
{
	array<array<Point2D, 7>, 7> points;

	// Exploring neighboring cells
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			const int x = cx + j - int(points[i].size()) / 2;
			const int y = cy + i - int(points.size()) / 2;

			points[i][j] = GeneratePoint(x, y);
		}
	}

	return points;
}

array<array<double, 7>, 7> Noise::ComputeElevations(const array<array<Point2D, 7>, 7>& points) const
{
	array<array<double, 7>, 7> elevations;

	for (int i = 0; i < elevations.size(); i++)
	{
		for (int j = 0; j < elevations[i].size(); j++)
		{
			const double x = Remap(points[i][j].x, m_noiseTopLeft.x, m_noiseBottomRight.x, m_perlinTopLeft.x, m_perlinBottomRight.x);
			const double y = Remap(points[i][j].y, m_noiseTopLeft.y, m_noiseBottomRight.y, m_perlinTopLeft.y, m_perlinBottomRight.y);

			elevations[i][j] = (Perlin(x, y) + 1.0) / 2.0;
		}
	}

	return elevations;
}

array<Segment3D, 25> Noise::GenerateSegments(const array<array<Point2D, 7>, 7>& points) const 
{
	const array<array<double, 7>, 7> elevations = ComputeElevations(points);

	array<Segment3D, 25> segments;
	for (int i = 1; i < points.size() - 1; i++)
	{
		for (int j = 1; j < points[i].size() - 1; j++)
		{
			// Lowest neighbor
			double lowestNeighborElevation = numeric_limits<double>::max();
			int lowestNeighborI = i;
			int lowestNeighborJ = j;

			for (int k = i - 1; k <= i + 1; k++)
			{
				for (int l = j - 1; l <= j + 1; l++)
				{
					if (elevations[k][l] < lowestNeighborElevation)
					{
						lowestNeighborElevation = elevations[k][l];
						lowestNeighborI = k;
						lowestNeighborJ = l;
					}
				}
			}

			const Point3D a(points[i][j].x, points[i][j].y, elevations[i][j]);
			const Point3D b(points[lowestNeighborI][lowestNeighborJ].x, points[lowestNeighborI][lowestNeighborJ].y, lowestNeighborElevation);

			segments[5 * (i - 1) + j - 1] = Segment3D(a, b);
		}
	}

	return segments;
}

array<Segment3D, 9> Noise::GenerateSubSegments(const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 25>& segments) const
{
	// Connect each point to the nearest segment
	array<Segment3D, 9> subSegments;
	for (int i = 1; i < points.size() - 1; i++)
	{
		for (int j = 1; j < points[i].size() - 1; j++)
		{
			double nearestSegmentDist = numeric_limits<double>::max();
			Segment3D nearestSegment;
			Point2D nearestSegmentIntersection;

			for (const Segment3D& segment : segments)
			{
				Point2D segmentNearestPoint;
				double dist = distToLineSegment(points[i][j], ProjectionZ(segment), segmentNearestPoint);

				if (dist < nearestSegmentDist)
				{
					nearestSegmentDist = dist;
					nearestSegment = segment;
					nearestSegmentIntersection = segmentNearestPoint;
				}
			}

			// Elevation in on the nearest segment
			const double elevationA = nearestSegment.a.z;
			const double elevationB = nearestSegment.b.z;
			const double u = pointSegmentProjection(points[i][j], ProjectionZ(nearestSegment));
			const double elevation = lerp_clamp(elevationA, elevationB, u);
			
			const Point3D a(points[i][j].x, points[i][j].y, elevation);
			const Point3D b(nearestSegmentIntersection.x, nearestSegmentIntersection.y, elevation);

			subSegments[3 * (i - 1) + j - 1] = Segment3D(a, b);
		}
	}

	return subSegments;
}

double Noise::ComputeColor(double x, double y, const array<array<Point2D, 7>, 7>& points, const array<Segment3D, 25>& segments) const
{
	// Find color
	double value = 0.0;

	// White when near to a control point
	if (m_displayPoints)
	{
		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < points[i].size(); j++)
			{
				double dist_center = dist(Point2D(x, y), points[i][j]);

				if (dist_center < 0.0625)
				{
					value = 1.0;
				}
			}
		}
	}

	// White when near to a segment
	if (m_displaySegments)
	{
		for (const Segment3D& segment : segments)
		{
			Point2D c;
			double dist = distToLineSegment(Point2D(x, y), ProjectionZ(segment), c);

			if (dist < 0.015625)
			{
				value = 1.0;
			}
		}
	}

	// When near to the grid
	if (m_displayGrid)
	{
		if (abs(x - floor(x)) < 0.0078125 || abs(y - floor(y)) < 0.0078125)
		{
			value = 1.0;
		}
	}

	return value;
}

double Noise::ComputeColorSub(double x, double y, const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 9>& segments) const
{
	// Find color
	double value = 0.0;

	// White when near to a control point
	if (m_displayPoints)
	{
		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < points[i].size(); j++)
			{
				double dist_center = dist(Point2D(x, y), points[i][j]);

				if (dist_center < 0.03125)
				{
					value = 1.0;
				}
			}
		}
	}

	// White when near to a segment
	if (m_displaySegments)
	{
		for (const Segment3D& segment : segments)
		{
			Point2D c;
			double dist = distToLineSegment(Point2D(x, y), ProjectionZ(segment), c);

			if (dist < 0.0078125)
			{
				value = 1.0;
			}
		}
	}

	// When near to the grid
	if (m_displayGrid)
	{
		if (abs(x - floor(x) - 0.5) < 0.00390625 || abs(y - floor(y) - 0.5) < 0.00390625)
		{
			value = 1.0;
		}
	}

	return value;
}

double Noise::ComputeColorWorley(double x, double y, const array<Segment3D, 25>& segments, const array<Segment3D, 9>& subSegments) const
{
	// Distance to the nearest segment
	double nearestSegmentDistance = numeric_limits<double>::max();
	Segment3D nearestSegment;

	// For each level 1 segment
	for (const Segment3D& segment : segments)
	{
		Point2D c;
		double dist = distToLineSegment(Point2D(x, y), ProjectionZ(segment), c);

		if (dist < nearestSegmentDistance)
		{
			nearestSegmentDistance = dist;
			nearestSegment = segment;
		}
	}

	// For each level 2 segment
	for (const Segment3D& segment : subSegments)
	{
		Point2D c;
		double dist = distToLineSegment(Point2D(x, y), ProjectionZ(segment), c);

		if (dist < nearestSegmentDistance)
		{
			nearestSegmentDistance = dist;
			nearestSegment = segment;
		}
	}

	// Elevation in on the nearest segment
	const double elevationA = nearestSegment.a.z;
	const double elevationB = nearestSegment.b.z;
	const double u = pointSegmentProjection(Point2D(x, y), ProjectionZ(nearestSegment));
	const double elevation = lerp_clamp(elevationA, elevationB, u);

	return nearestSegmentDistance + elevation;
}

tuple<int, int> Noise::GetSubQuadrant(double cx, double cy, double x, double y) const
{
	// Return the coordinates of the quadrant in which (x, y) if we divide the cell (cx, cy) in 4 quadrants
	//      cx    cx+1    cx+2
	//   cy -----------------
	//      |0;0|1;0|2;0|3;0|
	//      -----------------
	//      |0;1|1;1|2;1|3;1|
	// cy+1 -----------------
	//      |0;2|1;2|2;2|3;2|
	//      -----------------
	//      |0;3|1;3|2;3|3;3|
	// cy+2 -----------------
	// 
	// If x is in [cx, cx + 0.5[ and y is in [cy, cy + 0.5[, then the quadrant is (0, 0)
	// In the same way, if (x - cx) is in [0, 0.5[ and (y - cy) is in [0, 0.5[, then the quadrant is (0, 0) as well

	// ----- int(floor(2.0 * a)) -----
	// If a is in [-1.5, -1[ return -3
	// If a is in [-1, -0.5[ return -2
	// If a is in [-0.5, 0[  return -1
	// If a is in [0, 0.5[   return  0
	// If a is in [0.5, 1[   return  1
	// If a is in [1, 1.5[   return  2
	// Etc...

	int quadrantX = int(floor(2.0 *(x - cx)));
	int quadrantY = int(floor(2.0 *(y - cy)));

	return make_tuple(quadrantX, quadrantY);
}

array<array<Point2D, 5>, 5> Noise::GenerateNeighboringSubPoints(double cx, double cy, double x, double y, const array<array<Point2D, 7>, 7>& points) const
{
	// In which cell is the point (x, y)
	const int cxInt = int(cx);
	const int cyInt = int(cy);

	// Detect in which quadrant is the current point (x, y)
	int quadrantX, quadrantY;
	tie(quadrantX, quadrantY) = GetSubQuadrant(cx, cy, x, y);
	array<array<Point2D, 5>, 5> subPoints = GenerateNeighboringPoints5(2 * cxInt + quadrantX, 2 * cyInt + quadrantY);

	// Divide point coordinates by 2
	for (int i = 0; i < subPoints.size(); i++)
	{
		for (int j = 0; j < subPoints[i].size(); j++)
		{
			subPoints[i][j].x /= 2.0;
			subPoints[i][j].y /= 2.0;
		}
	}

	// Replace subpoints by the already existing points
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			int qX, qY;
			tie(qX, qY) = GetSubQuadrant(cx, cy, points[i][j].x, points[i][j].y);

			int k = (int(subPoints.size()) / 2) - quadrantY + qY;
			int l = (int(subPoints.front().size()) / 2) - quadrantX + qX;

			if (k >= 0 && k < subPoints.size() && l >= 0 && l < subPoints.front().size())
			{
				subPoints[k][l] = points[i][j];
			}
		}
	}

	return subPoints;
}

double Noise::evaluate(double x, double y) const
{
	// In which cell is the point (x, y)
	const double cx = floor(x);
	const double cy = floor(y);
	const int cxInt = int(cx);
	const int cyInt = int(cy);

	// Level 1: Points in neighboring cells
	array<array<Point2D, 7>, 7> points = GenerateNeighboringPoints7(cxInt, cyInt);
	// Level 1: List of segments 
	array<Segment3D, 25> segments = GenerateSegments(points);

	// Level 2: Points in neighboring cells
	array<array<Point2D, 5>, 5> subPoints = GenerateNeighboringSubPoints(cx, cy, x, y, points);
	// Level 2: List of segments 
	array<Segment3D, 9> subSegments = GenerateSubSegments(subPoints, segments);
	
	return max(
		ComputeColorWorley(x, y, segments, subSegments),
		max(
			ComputeColor(x, y, points, segments),
			ComputeColorSub(x, y, subPoints, subSegments)			
		)
	);
}
