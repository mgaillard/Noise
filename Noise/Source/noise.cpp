#include "noise.h"

#include <iostream>
#include <vector>
#include <array>
#include <random>
#include <cstdint>
#include <limits>
#include <cmath>
#include <tuple>

#include "perlin.h"
#include "math2d.h"

using namespace std;

int GenerateSeedNoise(int i, int j)
{
	// TODO: implement a better permutation method
	return (541 * i + 79 * j) % numeric_limits<int>::max();
}

Point GeneratePoint(int x, int y)
{
	// Fixed seed for internal consistency
	default_random_engine generator(GenerateSeedNoise(x, y));
	uniform_real_distribution<double> distribution(0.0, 1.0);

	return Point(double(x) + distribution(generator), double(y) + distribution(generator));
}

array<array<Point, 5>, 5> GenerateNeighboringPoints(int cx, int cy)
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

array<array<double, 5>, 5> ComputeElevations(const array<array<Point, 5>, 5>& points)
{
	array<array<double, 5>, 5> elevations;

	for (int i = 0; i < elevations.size(); i++)
	{
		for (int j = 0; j < elevations[i].size(); j++)
		{
			elevations[i][j] = Perlin(points[i][j].x / 16.0, points[i][j].y / 16.0);
		}
	}

	return elevations;
}

array<Segment, 9> GenerateSegments(const array<array<Point, 5>, 5>& points)
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

double ComputeColor(double x, double y, const array<array<Point, 5>, 5>& points, const array<Segment, 9>& segments)
{
	// Find color
	double value = 0.0;

	// White when near to a control point
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

	// White when near to a segment
	for (const Segment& segment : segments)
	{
		Point c;
		double dist = distToLineSegment(Point(x, y), segment, c);

		if (dist < 0.015625)
		{
			value = 1.0;
		}
	}

	// When near to the grid
	if (abs(x - floor(x)) < 0.015625 || abs(y - floor(y)) < 0.015625)
	{
		value = 1.0;
	}

	return value;
}

double Noise(double x, double y)
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