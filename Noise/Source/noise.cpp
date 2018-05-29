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
#include "utils.h"

using namespace std;

Noise::Noise(const Point& noiseTopLeft, const Point& noiseBottomRight, const Point& perlinTopLeft, const Point& perlinBottomRight, int seed, double eps, bool displayPoints, bool displaySegments, bool displayGrid) :
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

Point Noise::GeneratePoint(int x, int y) const
{
	// Fixed seed for internal consistency
	const int seed = GenerateSeedNoise(x, y);
	RandomGenerator generator(seed);

	uniform_real_distribution<double> distribution(m_eps, 1.0 - m_eps);
	const double px = distribution(generator);
	const double py = distribution(generator);

	return Point(double(x) + px, double(y) + py);
}

array<array<Point, 5>, 5> Noise::GenerateNeighboringPoints(int cx, int cy) const
{
	array<array<Point, 5>, 5> points;

	// Exploring neighboring cells
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			const int x = cx + j - 2;
			const int y = cy + i - 2;

			points[i][j] = GeneratePoint(x, y);
		}
	}

	return points;
}

array<array<double, 5>, 5> Noise::ComputeElevations(const array<array<Point, 5>, 5>& points) const
{
	array<array<double, 5>, 5> elevations;

	for (int i = 0; i < elevations.size(); i++)
	{
		for (int j = 0; j < elevations[i].size(); j++)
		{
			const double x = Remap(points[i][j].x, m_noiseTopLeft.x, m_noiseBottomRight.x, m_perlinTopLeft.x, m_perlinBottomRight.x);
			const double y = Remap(points[i][j].y, m_noiseTopLeft.y, m_noiseBottomRight.y, m_perlinTopLeft.y, m_perlinBottomRight.y);

			elevations[i][j] = Perlin(x, y);
		}
	}

	return elevations;
}

array<Segment, 9> Noise::GenerateSegments(const array<array<Point, 5>, 5>& points) const 
{
	const array<array<double, 5>, 5> elevations = ComputeElevations(points);

	array<Segment, 9> segments;
	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
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

			segments[3 * (i - 1) + j - 1] = Segment(points[i][j], points[lowestNeighborI][lowestNeighborJ]);
		}
	}

	return segments;
}

array<Segment, 9> Noise::GenerateSubSegments(const array<array<Point, 5>, 5>& points, const array<Segment, 9>& segments) const
{
	// Connect each point to the nearest segment
	array<Segment, 9> subSegments;
	for (int i = 1; i <= 3; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			double nearestSegmentDist = numeric_limits<double>::max();
			Point nearestSegmentIntersection;

			for (const Segment& segment : segments)
			{
				Point segmentNearestPoint;
				double dist = distToLineSegment(points[i][j], segment, segmentNearestPoint);

				if (dist < nearestSegmentDist)
				{
					nearestSegmentDist = dist;
					nearestSegmentIntersection = segmentNearestPoint;
				}
			}

			subSegments[3 * (i - 1) + j - 1] = Segment(points[i][j], nearestSegmentIntersection);
		}
	}

	return subSegments;
}

double Noise::ComputeColor(double x, double y, const array<array<Point, 5>, 5>& points, const array<Segment, 9>& segments) const
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
				double dist_center = dist(Point(x, y), points[i][j]);

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
		for (const Segment& segment : segments)
		{
			Point c;
			double dist = distToLineSegment(Point(x, y), segment, c);

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

double Noise::ComputeColorSub(double x, double y, const array<array<Point, 5>, 5>& points, const array<Segment, 9>& segments) const
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
				double dist_center = dist(Point(x, y), points[i][j]);

				if (dist_center < 0.015625)
				{
					value = 1.0;
				}
			}
		}
	}

	// White when near to a segment
	if (m_displaySegments)
	{
		for (const Segment& segment : segments)
		{
			Point c;
			double dist = distToLineSegment(Point(x, y), segment, c);

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

array<array<Point, 5>, 5> Noise::GenerateNeighboringSubPoints(double cx, double cy, double x, double y, const array<array<Point, 5>, 5>& points) const
{
	// In which cell is the point (x, y)
	const int cxInt = int(cx);
	const int cyInt = int(cy);

	// Detect in which quadrant is the current point (x, y)
	int quadrantX, quadrantY;
	tie(quadrantX, quadrantY) = GetSubQuadrant(cx, cy, x, y);
	array<array<Point, 5>, 5> subPoints = GenerateNeighboringPoints(2 * cxInt + quadrantX, 2 * cyInt + quadrantY);

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

			int k = 2 - quadrantY + qY;
			int l = 2 - quadrantX + qX;

			if (k >= 0 && k < 5 && l >= 0 && l < 5)
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
	array<array<Point, 5>, 5> points = GenerateNeighboringPoints(cxInt, cyInt);
	// Level 1: List of segments 
	array<Segment, 9> segments = GenerateSegments(points);

	// Level 2: Points in neighboring cells
	array<array<Point, 5>, 5> subPoints = GenerateNeighboringSubPoints(cx, cy, x, y, points);
	// Level 2: List of segments 
	array<Segment, 9> subSegments = GenerateSubSegments(subPoints, segments);
	
	return max(
		ComputeColor(x, y, points, segments),
		ComputeColorSub(x, y, subPoints, subSegments)
	);
}
