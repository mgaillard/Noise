#ifndef NOISE_H
#define NOISE_H

#include <array>
#include <random>

#include "math2d.h"
#include "math3d.h"

using namespace std;

class Noise
{
public:
	Noise(const Point2D& noiseTopLeft, const Point2D& noiseBottomRight, const Point2D& perlinTopLeft, const Point2D& perlinBottomRight, int seed = 0, double eps = 0.0, bool displayPoints = true, bool displaySegments = true, bool displayGrid = true);

	double evaluate(double x, double y) const;

private:
	int GenerateSeedNoise(int i, int j) const;

	Point2D GeneratePoint(int x, int y) const;

	tuple<int, int> GetSubQuadrant(double cx, double cy, double x, double y) const;

	array<array<Point2D, 5>, 5> GenerateNeighboringPoints(int cx, int cy) const;

	array<array<Point2D, 5>, 5> GenerateNeighboringSubPoints(double cx, double cy, double x, double y, const array<array<Point2D, 5>, 5>& points) const;

	array<array<double, 5>, 5> ComputeElevations(const array<array<Point2D, 5>, 5>& points) const;

	array<Segment3D, 9> GenerateSegments(const array<array<Point2D, 5>, 5>& points) const;

	array<Segment3D, 9> GenerateSubSegments(const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 9>& segments) const;

	double ComputeColor(double x, double y, const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 9>& segments) const;

	double ComputeColorSub(double x, double y, const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 9>& segments) const;

	double ComputeColorWorley(double x, double y, const array<Segment3D, 9>& segments, const array<Segment3D, 9>& subSegments) const;
	
	// Random generator used by the class
	typedef minstd_rand RandomGenerator;

	// Seed of the noise
	const int m_seed;

	const bool m_displayPoints;
	const bool m_displaySegments;
	const bool m_displayGrid;

	const Point2D m_noiseTopLeft;
	const Point2D m_noiseBottomRight;
	const Point2D m_perlinTopLeft;
	const Point2D m_perlinBottomRight;

	// Epsilon used to biais the area where points are generated in cells
	const double m_eps;
};

#endif // NOISE_H