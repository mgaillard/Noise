#include "noise.h"

#include <iostream>
#include <vector>
#include <cstdint>
#include <limits>
#include <cmath>
#include <tuple>

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
	m_distribution(eps, 1.0 - eps)
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
	default_random_engine generator(GenerateSeedNoise(x, y));

	return Point(double(x) + m_distribution(generator), double(y) + m_distribution(generator));
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

double Noise::evaluate(double x, double y) const
{
	// In which cell is the point
	const int cx = int(floor(x));
	const int cy = int(floor(y));

	// Points in neighboring cells
	array<array<Point, 5>, 5> points = GenerateNeighboringPoints(cx, cy);
	// List of segments 
	array<Segment, 9> segments = GenerateSegments(points);

	return ComputeColor(x, y, points, segments);
}
