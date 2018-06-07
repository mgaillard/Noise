#ifndef NOISE_H
#define NOISE_H

#include <array>
#include <vector>
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
	void InitPointCache();

	int GenerateSeedNoise(int i, int j) const;

	Point2D GeneratePoint(int x, int y) const;
	Point2D GeneratePointCached(int x, int y) const;

	tuple<int, int> GetSubQuadrant(double cx, double cy, double x, double y) const;

	array<array<Point2D, 5>, 5> GenerateNeighboringPoints5(int cx, int cy) const;
	array<array<Point2D, 7>, 7> GenerateNeighboringPoints7(int cx, int cy) const;

	array<array<Point2D, 5>, 5> GenerateNeighboringSubPoints(double cx, double cy, double x, double y, const array<array<Point2D, 7>, 7>& points) const;

	array<array<double, 7>, 7> ComputeElevations(const array<array<Point2D, 7>, 7>& points) const;

	array<Segment3D, 25> GenerateSegments(const array<array<Point2D, 7>, 7>& points) const;

	void SubdivideSegments(const array<Segment3D, 25>& segments, array<Segment3D, 25>& segmentsBegin, array<array<Point2D, 5>, 5>& midPoints, array<Segment3D, 25>& segmentsEnd) const;

	array<Segment3D, 25> GenerateSubSegments(const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 25>& segmentsBegin, const array<Segment3D, 25>& segmentsEnd) const;

	double ComputeColorPoint(double x, double y, const Point2D& point, double radius) const;

	double ComputeColorPoints5(double x, double y, const array<array<Point2D, 5>, 5>& points, double radius) const;
	double ComputeColorPoints7(double x, double y, const array<array<Point2D, 7>, 7>& points, double radius) const;

	double ComputeColorSegment9(double x, double y, const array<Segment3D, 9>& segments, double radius) const;
	double ComputeColorSegment25(double x, double y, const array<Segment3D, 25>& segments, double radius) const;

	double ComputeColorGrid(double x, double y, double deltaX, double deltaY, double radius) const;

	double ComputeColor(double x, double y, const array<array<Point2D, 7>, 7>& points, const array<array<Point2D, 5>, 5>& midPoints, const array<Segment3D, 25>& segmentsBegin, const array<Segment3D, 25>& segmentsEnd) const;

	double ComputeColorSub(double x, double y, const array<array<Point2D, 5>, 5>& points, const array<Segment3D, 25>& segments) const;

	double ComputeColorWorley(double x, double y, const array<Segment3D, 25>& segmentsBegin, const array<Segment3D, 25>& segmentsEnd, const array<Segment3D, 25>& subSegments) const;
	
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

	const int CACHE_X = 32;
	const int CACHE_Y = 32;
	vector<vector<Point2D> > m_pointCache;
};

#endif // NOISE_H