#ifndef NOISE_H
#define NOISE_H

#include <array>
#include <random>

#include "math2d.h"

using namespace std;

class Noise
{
public:
	Noise(const Point& noiseTopLeft, const Point& noiseBottomRight, const Point& perlinTopLeft, const Point& perlinBottomRight, int seed = 0, double eps = 0.0, bool displayPoints = true, bool displaySegments = true, bool displayGrid = true);

	double evaluate(double x, double y) const;

private:
	int GenerateSeedNoise(int i, int j) const;
	Point GeneratePoint(int x, int y) const;
	array<array<Point, 5>, 5> GenerateNeighboringPoints(int cx, int cy) const;
	array<array<double, 5>, 5> ComputeElevations(const array<array<Point, 5>, 5>& points) const;
	array<Segment, 9> GenerateSegments(const array<array<Point, 5>, 5>& points) const;
	double ComputeColor(double x, double y, const array<array<Point, 5>, 5>& points, const array<Segment, 9>& segments) const;

	const int m_seed;

	const bool m_displayPoints;
	const bool m_displaySegments;
	const bool m_displayGrid;

	const Point m_noiseTopLeft;
	const Point m_noiseBottomRight;
	const Point m_perlinTopLeft;
	const Point m_perlinBottomRight;

	uniform_real_distribution<double> m_distribution;
};

#endif // NOISE_H