#ifndef NOISE_H
#define NOISE_H

#include <array>
#include <vector>
#include <random>
#include <tuple>
#include <limits>
#include <cmath>
#include <cassert>
#include <memory>

#include "math2d.h"
#include "math3d.h"
#include "spline.h"
#include "utils.h"
#include "controlfunction.h"

template <typename I>
class Noise
{
public:
	Noise(std::unique_ptr<ControlFunction<I> > controlFunction, const Point2D& noiseTopLeft, const Point2D& noiseBottomRight, const Point2D & controlFunctionTopLeft, const Point2D & controlFunctionBottomRight, int seed = 0, double eps = 0.0, bool displayPoints = true, bool displaySegments = true, bool displayGrid = true);

	double evaluate(double x, double y) const;

private:
	// ----- Types -----
	template <typename T, size_t N>
	using Array2d = std::array<std::array<T, N>, N>;

	template <size_t N>
	using Segment3DChain = std::array<Segment3D, N>;

	template <size_t N>
	using DoubleArray = Array2d<double, N>;

	template <size_t N>
	using Point2DArray = Array2d<Point2D, N>;

	template <size_t N>
	using Segment3DArray = Array2d<Segment3D, N>;

	template <size_t N, size_t M>
	using Segment3DChainArray = Array2d<Segment3DChain<M>, N>;

	/// <summary>
	/// Represents a cell at a specific resolution
	/// </summary>
	struct Cell
	{
		int x;
		int y;
		int resolution;

		Cell() = default;

		Cell(int x, int y, int resolution) : x(x), y(y), resolution(resolution) {}
	};

	// ----- Points -----

	void InitPointCache();

	int GenerateSeedNoise(int i, int j) const;

	Point2D GeneratePoint(int x, int y) const;
	Point2D GeneratePointCached(int x, int y) const;

	// ----- Utils -----

	Cell GetCell(double x, double y, int resolution) const;

	double EvaluateControlFunction(const Point2D& point) const;

	Segment3D ConnectPointToSegmentAngle(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	Segment3D ConnectPointToSegmentAngleMid(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	Segment3D ConnectPointToSegmentNearestPoint(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	template <typename T, size_t N>
	std::tuple<int, int> GetArrayCell(const Cell& arrCell, const Array2d<T, N>& arr, const Cell& cell) const;

	template <size_t N>
	double NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DArray<N>& segments) const;

	template <size_t N, size_t D>
	double NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DChainArray<N, D>& segments) const;

	template <size_t N, typename ...Tail>
	double NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DArray<N>& segments, Tail&&... tail) const;

	template <size_t N, size_t D, typename ...Tail>
	double NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DChainArray<N, D>& segments, Tail&&... tail) const;

	template <size_t N>
	int SegmentsEndingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentEndingInP) const;

	template <size_t N>
	int SegmentsStartingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentStartingInP) const;

	// ----- Generate -----

	template <size_t N>
	Point2DArray<N> GenerateNeighboringPoints(const Cell& cell) const;

	template <size_t N, size_t M>
	void ReplaceNeighboringPoints(const Cell& cell, const Point2DArray<M>& points, const Cell& subCell, Point2DArray<N>& subPoints) const;

	template <size_t N>
	DoubleArray<N> ComputeElevations(const Point2DArray<N>& points) const;
	
	template <size_t N>
	Segment3DArray<N - 2> GenerateSegments(const Point2DArray<N>& points) const;

	template <size_t N, size_t D>
	void SubdivideSegments(const Cell& cell, const Segment3DArray<N>& segments, Segment3DChainArray<N - 2, D>& subdividedSegments) const;

	template <size_t N, size_t M, size_t D>
	Segment3DArray<N> GenerateSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Point2DArray<N>& points) const;

	template <size_t N, size_t M, size_t D, size_t K>
	Segment3DArray<N> GenerateSubSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<K>& subSegments, const Point2DArray<N>& points) const;

	// ----- Compute Color -----

	double ComputeColorPoint(double x, double y, const Point2D& point, double radius) const;

	template <size_t N>
	double ComputeColorPoints(double x, double y, const Point2DArray<N>& points, double radius) const;

	template <size_t N>
	double ComputeColorPoints(double x, double y, const Segment3DArray<N>& segments, double radius) const;

	template <size_t N, size_t D>
	double ComputeColorPoints(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const;

	double ComputeColorSegment(double x, double y, const Segment2D& segment, double radius) const;

	template <size_t N>
	double ComputeColorSegments(const Cell& cell, const Segment3DArray<N>& segments, int neighborhood, double x, double y, double radius) const;

	template <size_t N, size_t D>
	double ComputeColorSegments(const Cell& cell, const Segment3DChainArray<N, D>& segments, int neighborhood, double x, double y, double radius) const;

	double ComputeColorGrid(double x, double y, double deltaX, double deltaY, double radius) const;

	template <size_t N, size_t D>
	double ComputeColor(const Cell& cell, const Segment3DChainArray<N- 4, D>& subdividedSegments, const Point2DArray<N>& points, double x, double y) const;

	template <size_t N>
	double ComputeColorSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const;

	template <size_t N>
	double ComputeColorSubSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const;

	template <size_t N, size_t D, size_t M, size_t K>
	double ComputeColorPrimitives(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, const Point2DArray<K>& subSubPoints, double x, double y) const;

	template <size_t N, size_t D, size_t M, size_t K>
	double ComputeColorControlFunction(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, double x, double y) const;
	
	// Random generator used by the class
	typedef std::minstd_rand RandomGenerator;

	// Seed of the noise
	const int m_seed;

	// A control function
	const std::unique_ptr<ControlFunction<I> > m_controlFunction;

	const bool m_displayPoints;
	const bool m_displaySegments;
	const bool m_displayGrid;

	const Point2D m_noiseTopLeft;
	const Point2D m_noiseBottomRight;
	const Point2D m_controlFunctionTopLeft;
	const Point2D m_controlFunctionBottomRight;

	// Epsilon used to biais the area where points are generated in cells
	const double m_eps;

	const int CACHE_X = 32;
	const int CACHE_Y = 32;
	std::vector<std::vector<Point2D> > m_pointCache;
};

template <typename I>
Noise<I>::Noise(std::unique_ptr<ControlFunction<I> > controlFunction, const Point2D& noiseTopLeft, const Point2D& noiseBottomRight, const Point2D & controlFunctionTopLeft, const Point2D & controlFunctionBottomRight, int seed, double eps, bool displayPoints, bool displaySegments, bool displayGrid) :
	m_seed(seed),
	m_controlFunction(std::move(controlFunction)),
	m_displayPoints(displayPoints),
	m_displaySegments(displaySegments),
	m_displayGrid(displayGrid),
	m_noiseTopLeft(noiseTopLeft),
	m_noiseBottomRight(noiseBottomRight),
	m_controlFunctionTopLeft(controlFunctionTopLeft),
	m_controlFunctionBottomRight(controlFunctionBottomRight),
	m_eps(eps)
{
	InitPointCache();
}

template <typename I>
void Noise<I>::InitPointCache()
{
	m_pointCache.resize(CACHE_X);

	for (int x = -CACHE_X / 2; x < CACHE_X / 2; x++)
	{
		m_pointCache[x + CACHE_X / 2].resize(CACHE_Y);

		for (int y = -CACHE_Y / 2; y < CACHE_Y / 2; y++)
		{
			m_pointCache[x + CACHE_X / 2][y + CACHE_Y / 2] = GeneratePoint(x, y);
		}
	}
}

template <typename I>
int Noise<I>::GenerateSeedNoise(int i, int j) const
{
	// TODO: implement a better permutation method
	return (541 * i + 79 * j + m_seed) % std::numeric_limits<int>::max();
}

/// <summary>
/// Generate a point in a cell.
/// This function is reproducible.
/// </summary>
/// <param name="x">x coordinate of the cell</param>
/// <param name="y">y coordinate of the cell</param>
/// <returns>A Point2D in this cell</returns>
template <typename I>
Point2D Noise<I>::GeneratePoint(int x, int y) const
{
	// Fixed seed for internal consistency
	const int seed = GenerateSeedNoise(x, y);
	RandomGenerator generator(seed);

	std::uniform_real_distribution<double> distribution(m_eps, 1.0 - m_eps);
	const double px = distribution(generator);
	const double py = distribution(generator);

	return Point2D(double(x) + px, double(y) + py);
}

/// <summary>
/// Generate a point in a cell.
/// This function is reproducible.
/// Use the point cache if possible.
/// </summary>
/// <param name="x">x coordinate of the cell</param>
/// <param name="y">y coordinate of the cell</param>
/// <returns>A Point2D in this cell</returns>
template <typename I>
Point2D Noise<I>::GeneratePointCached(int x, int y) const
{
	if (x >= -CACHE_X / 2 && x < CACHE_X / 2 && y >= -CACHE_Y / 2 && y < CACHE_Y / 2)
	{
		return m_pointCache[x + CACHE_X / 2][y + CACHE_Y / 2];
	}
	else
	{
		return GeneratePoint(x, y);
	}
}

template <typename I>
typename Noise<I>::Cell Noise<I>::GetCell(double x, double y, int resolution) const
{
	// Return the coordinates of the cell in which (x, y)
	// For example, for resolution 1:
	// 
	//      0   1   2   3   
	//    0 -----------------
	//      |0;0|1;0|2;0|3;0|
	//    1 -----------------
	//      |0;1|1;1|2;1|3;1|
	//    2 -----------------
	//      |0;2|1;2|2;2|3;2|
	//    3 -----------------
	//      |0;3|1;3|2;3|3;3|
	//      -----------------
	//
	// If x is in [0, 1[ and y is in [0, 1[, then the cell is (0, 0)
	//
	// For example, for resolution 2:
	// 
	//      0       1       2  
	//    0 -----------------
	//      |0;0|1;0|2;0|3;0|
	//      -----------------
	//      |0;1|1;1|2;1|3;1|
	//    1 -----------------
	//      |0;2|1;2|2;2|3;2|
	//      -----------------
	//      |0;3|1;3|2;3|3;3|
	//    2 -----------------
	//
	// If x is in [0, 0.5[ and y is in [0, 0.5[, then the cell is (0, 0)

	// Resolution is strictly positive
	assert(resolution > 0);

	Cell c;

	c.x = int(floor(x * resolution));
	c.y = int(floor(y * resolution));
	c.resolution = resolution;

	return c;
}

/// <summary>
/// Evaluate the control function at a point (x, y)
/// </summary>
/// <param name="point">Coordinates of the point</param>
/// <returns>The value of the function at the point</returns>
template <typename I>
double Noise<I>::EvaluateControlFunction(const Point2D& point) const
{
	const double x = Remap(point.x, m_noiseTopLeft.x, m_noiseBottomRight.x, m_controlFunctionTopLeft.x, m_controlFunctionBottomRight.x);
	const double y = Remap(point.y, m_noiseTopLeft.y, m_noiseBottomRight.y, m_controlFunctionTopLeft.y, m_controlFunctionBottomRight.y);

	double value = 0.0;

	if (m_controlFunction)
	{
		value = m_controlFunction->evaluate(x, y);
	}

	return value;
}

template <typename I>
Segment3D Noise<I>::ConnectPointToSegmentAngle(const Point3D & point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineSegmentProjection(ProjectionZ(point), ProjectionZ(segment));

	// If, on the segment, the nearest point is between A and B, we shift it so that the angle constraint is respected
	if (u > 0.0 && u < 1.0)
	{
		// Find the intersection so that the angle between the two segments is 45°
		// v designates the ratio of the segment on which the intersection is located
		// v = 0 is point A of the segment ; v = 1 is point B of the segment
		double v = u + segmentDist / length(ProjectionZ(segment));

		if (v > 1.0)
		{
			// If the intersection is farther than B, simply take B as intersection
			u = 1.0;
		}
		else
		{
			// Otherwise take a point on the segment
			u = v;
		}
	}

	const Point3D segmentEnd(lerp(segment, u));

	return Segment3D(point, segmentEnd);
}

template <typename I>
Segment3D Noise<I>::ConnectPointToSegmentAngleMid(const Point3D& point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineProjection(ProjectionZ(point), ProjectionZ(segment));

	// Find the intersection so that the angle between the two segments is 45°
	// v designates the ratio of the segment on which the intersection is located
	// v = 0 is point A of the segment ; v = 1 is point B of the segment
	double v = u + segmentDist / length(ProjectionZ(segment));
	// The intersection must lie on the segment
	v = clamp(v, 0.0, 1.0);

	const Point3D segmentEnd(lerp(segment, u));

	return Segment3D(point, segmentEnd);
}

template <typename I>
Segment3D Noise<I>::ConnectPointToSegmentNearestPoint(const Point3D& point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineSegmentProjection(ProjectionZ(point), ProjectionZ(segment));

	const Point3D segmentEnd(lerp(segment, u));

	return Segment3D(point, segmentEnd);
}

template <typename I>
double Noise<I>::ComputeColorPoint(double x, double y, const Point2D& point, double radius) const
{
	double value = 0.0;

	if (dist(Point2D(x, y), point) < radius)
	{
		value = 1.0;
	}

	return value;
}

template <typename I>
double Noise<I>::ComputeColorSegment(double x, double y, const Segment2D& segment, double radius) const
{
	double value = 0.0;

	Point2D c;
	if (distToLineSegment(Point2D(x, y), segment, c) < radius)
	{
		value = 1.0;
	}

	return value;
}

template <typename I>
double Noise<I>::ComputeColorGrid(double x, double y, double deltaX, double deltaY, double radius) const
{
	double value = 0.0;

	// When near to the grid
	if (abs(x - floor(x) - deltaX) < radius || abs(y - floor(y) - deltaY) < radius)
	{
		value = 1.0;
	}

	return value;
}

template <typename I>
double Noise<I>::evaluate(double x, double y) const
{
	// In which level 1 cell is the point (x, y)
	Cell cell = GetCell(x, y, 1);
	// Level 1: Points in neighboring cells
	Point2DArray<9> points = GenerateNeighboringPoints<9>(cell);
	// Level 1: List of segments
	Segment3DArray<7> segments = GenerateSegments(points);
	// Subdivide segments of level 1
	Segment3DChainArray<5, 4> subdividedSegments;
	Point2DArray<5> midPoints;
	SubdivideSegments(cell, segments, subdividedSegments);


	// In which level 2 cell is the point (x, y)
	Cell subCell = GetCell(x, y, 2);
	// Level 2: Points in neighboring cells
	Point2DArray<5> subPoints = GenerateNeighboringPoints<5>(subCell);
	ReplaceNeighboringPoints(cell, points, subCell, subPoints);
	// Level 2: List of segments
	Segment3DArray<5> subSegments = GenerateSubSegments(cell, subdividedSegments, subPoints);


	// In which level 3 cell is the point (x, y)
	Cell subSubCell = GetCell(x, y, 4);
	// Level 3: Points in neighboring cells
	Point2DArray<5> subSubPoints = GenerateNeighboringPoints<5>(subSubCell);
	ReplaceNeighboringPoints(subCell, subPoints, subSubCell, subSubPoints);
	// Level 3: List of segments
	Segment3DArray<5> subSubSegments = GenerateSubSubSegments(cell, subdividedSegments, subCell, subSegments, subSubPoints);

	double value = 0.0;

	value = std::max(value, ComputeColorPrimitives(cell, subdividedSegments, subCell, subSegments, subSubCell, subSubSegments, subSubPoints, x, y));
	value = std::max(value, ComputeColor(cell, subdividedSegments, points, x, y));
	value = std::max(value, ComputeColorSub(subCell, subSegments, subPoints, x, y));
	value = std::max(value, ComputeColorSubSub(subSubCell, subSubSegments, subSubPoints, x, y));

	return value;
}

template <typename I>
template <typename T, size_t N>
std::tuple<int, int> Noise<I>::GetArrayCell(const Cell& arrCell, const Array2d<T, N>& arr, const Cell& cell) const
{
	int i = (int(arr.size()) / 2) - arrCell.y + cell.y;
	int j = (int(arr.front().size()) / 2) - arrCell.x + cell.x;

	return std::make_tuple(i, j);
}

template <typename I>
template <size_t N>
double Noise<I>::NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DArray<N>& segments) const
{
	assert(neighborhood >= 0);

	// Distance to the nearest segment
	double nearestSegmentDistance = std::numeric_limits<double>::max();

	int ci, cj;
	std::tie(ci, cj) = GetArrayCell(cell, segments, GetCell(point.x, point.y, cell.resolution));
	for (int i = ci - neighborhood; i <= ci + neighborhood; i++)
	{
		for (int j = cj - neighborhood; j <= cj + neighborhood; j++)
		{
			assert(i >= 0 && i < segments.size());
			assert(j >= 0 && j < segments.front().size());

			Point2D c;
			double dist = distToLineSegment(point, ProjectionZ(segments[i][j]), c);

			if (dist < nearestSegmentDistance)
			{
				nearestSegmentDistance = dist;
				nearestSegmentOut = segments[i][j];
			}
		}
	}

	return nearestSegmentDistance;
}

template <typename I>
template <size_t N, size_t D>
double Noise<I>::NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DChainArray<N, D>& segments) const
{
	assert(neighborhood >= 0);

	// Distance to the nearest segment
	double nearestSegmentDistance = std::numeric_limits<double>::max();

	int ci, cj;
	std::tie(ci, cj) = GetArrayCell(cell, segments, GetCell(point.x, point.y, cell.resolution));
	for (int i = ci - neighborhood; i <= ci + neighborhood; i++)
	{
		for (int j = cj - neighborhood; j <= cj + neighborhood; j++)
		{
			assert(i >= 0 && i < segments.size());
			assert(j >= 0 && j < segments.front().size());

			for (int k = 0; k < segments[i][j].size(); k++)
			{
				Point2D c;
				double dist = distToLineSegment(point, ProjectionZ(segments[i][j][k]), c);

				if (dist < nearestSegmentDistance)
				{
					nearestSegmentDistance = dist;
					nearestSegmentOut = segments[i][j][k];
				}
			}
		}
	}

	return nearestSegmentDistance;
}

template <typename I>
template <size_t N, typename ...Tail>
double Noise<I>::NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DArray<N>& segments, Tail&&... tail) const
{
	assert(neighborhood >= 0);

	// Nearest segment in the sub resolutions
	Segment3D nearestSubSegment;
	double nearestSubSegmentDistance = NearestSegmentProjectionZ(neighborhood, point, nearestSubSegment, std::forward<Tail>(tail)...);

	// Nearest segment in the current resolution
	double nearestSegmentDistance = NearestSegmentProjectionZ(neighborhood, point, nearestSegmentOut, cell, segments);

	if (nearestSubSegmentDistance < nearestSegmentDistance)
	{
		nearestSegmentDistance = nearestSubSegmentDistance;
		nearestSegmentOut = nearestSubSegment;
	}

	return nearestSegmentDistance;
}

template <typename I>
template <size_t N, size_t D, typename ...Tail>
double Noise<I>::NearestSegmentProjectionZ(int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut, const Cell& cell, const Segment3DChainArray<N, D>& segments, Tail&&... tail) const
{
	assert(neighborhood >= 0);

	// Nearest segment in the sub resolutions
	Segment3D nearestSubSegment;
	double nearestSubSegmentDistance = NearestSegmentProjectionZ(neighborhood, point, nearestSubSegment, std::forward<Tail>(tail)...);

	// Nearest segment in the current resolution
	double nearestSegmentDistance = NearestSegmentProjectionZ(neighborhood, point, nearestSegmentOut, cell, segments);

	if (nearestSubSegmentDistance < nearestSegmentDistance)
	{
		nearestSegmentDistance = nearestSubSegmentDistance;
		nearestSegmentOut = nearestSubSegment;
	}

	return nearestSegmentDistance;
}

template <typename I>
template <size_t N>
int Noise<I>::SegmentsEndingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentEndingInP) const
{
	int numberSegmentEndingInP = 0;

	// In which cell of segments is point
	int ck, cl;
	std::tie(ck, cl) = GetArrayCell(cell, segments, GetCell(point.x, point.y, cell.resolution));
	for (int k = ck - 1; k <= ck + 1; k++)
	{
		for (int l = cl - 1; l <= cl + 1; l++)
		{
			assert(k >= 0 && k < segments.size());
			assert(l >= 0 && l < segments.front().size());

			// If the segment's length is more than 0
			if (segments[k][l].a != segments[k][l].b)
			{
				if (segments[k][l].b == point)
				{
					numberSegmentEndingInP++;
					lastSegmentEndingInP = segments[k][l];
				}
			}
		}
	}

	return numberSegmentEndingInP;
}

template <typename I>
template <size_t N>
int Noise<I>::SegmentsStartingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentStartingInP) const
{
	int numberStartingInP = 0;

	// In which cell of segments is B
	int m, n;
	std::tie(m, n) = GetArrayCell(cell, segments, GetCell(point.x, point.y, cell.resolution));

	assert(m >= 0 && m < segments.size());
	assert(n >= 0 && n < segments.front().size());

	// If the segment's length is more than 0
	if (segments[m][n].a != segments[m][n].b)
	{
		if (segments[m][n].a == point)
		{
			numberStartingInP++;
			lastSegmentStartingInP = segments[m][n];
		}
	}

	return numberStartingInP;
}

template <typename I>
template <size_t N>
typename Noise<I>::Point2DArray<N> Noise<I>::GenerateNeighboringPoints(const Cell& cell) const
{
	Point2DArray<N> points;

	// Exploring neighboring cells
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			const int x = cell.x + j - int(points[i].size()) / 2;
			const int y = cell.y + i - int(points.size()) / 2;

			points[i][j] = GeneratePointCached(x, y) / cell.resolution;
		}
	}

	return points;
}

template <typename I>
template <size_t N, size_t M>
void Noise<I>::ReplaceNeighboringPoints(const Cell& cell, const Point2DArray<M>& points, const Cell& subCell, Point2DArray<N>& subPoints) const
{
	// Ensure that there is enough points around to replace subpoints
	static_assert(M >= (2 * ((N + 1) / 4) + 1), "Not enough points in the vicinity to replace the sub points.");
	
	// Number of cells (or points) to consider in the upper resolution
	int pointsUpRes = 2 * ((N + 1) / 4) + 1;
	// Offset to iterate over points only using the pointsUpRes cells in the center
	int offset = (M - pointsUpRes) / 2;
	// Replace subpoints by the already existing points
	for (int i = offset; i < points.size() - offset; i++)
	{
		for (int j = offset; j < points[i].size() - offset; j++)
		{
			int k, l;
			std::tie(k, l) = GetArrayCell(subCell, subPoints, GetCell(points[i][j].x, points[i][j].y, subCell.resolution));

			if (k >= 0 && k < subPoints.size() && l >= 0 && l < subPoints.front().size())
			{
				subPoints[k][l] = points[i][j];
			}
		}
	}
}

template <typename I>
template <size_t N>
typename Noise<I>::DoubleArray<N> Noise<I>::ComputeElevations(const Point2DArray<N>& points) const
{
	DoubleArray<N> elevations;

	for (int i = 0; i < elevations.size(); i++)
	{
		for (int j = 0; j < elevations[i].size(); j++)
		{
			elevations[i][j] = EvaluateControlFunction(points[i][j]);
		}
	}

	return elevations;
}

template <typename I>
template <size_t N>
typename Noise<I>::Segment3DArray<N - 2> Noise<I>::GenerateSegments(const Point2DArray<N>& points) const
{
	const DoubleArray<N> elevations = ComputeElevations<N>(points);

	Segment3DArray<N - 2> segments;
	for (int i = 1; i < points.size() - 1; i++)
	{
		for (int j = 1; j < points[i].size() - 1; j++)
		{
			// Lowest neighbor
			double lowestNeighborElevation = std::numeric_limits<double>::max();
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

			segments[i - 1][j - 1] = Segment3D(a, b);
		}
	}

	return segments;
}

/// <summary>
/// Subdivide all segments in a Segment3DArray&lt;N&gt; in D smaller segments using an interpolant spline.
/// </summary>
/// Require a Segment3DArray&lt;N&gt; to generate a Segment3DChainArray&lt;N - 2, D&gt; because to subdivide a segment we need its predecessors and successors.
template <typename I>
template <size_t N, size_t D>
void Noise<I>::SubdivideSegments(const Cell& cell, const Segment3DArray<N>& segments, Segment3DChainArray<N - 2, D>& subdividedSegments) const
{
	// Ensure that segments are subdivided.
	static_assert(D > 1, "Segments should be subdivided in more than 1 part.");

	// Subdivide segments
	for (int i = 1; i < segments.size() - 1; i++)
	{
		for (int j = 1; j < segments[i].size() - 1; j++)
		{
			Segment3D currSegment = segments[i][j];

			std::array<Point3D, D - 1> midPoints = Subdivide<D - 1>(currSegment);

			// If the current segment's length is more than 0, we can subdivide and smooth it
			if (currSegment.a != currSegment.b)
			{
				// Segments ending in A
				Segment3D lastEndingInA;
				int numberSegmentEndingInA = SegmentsEndingInP(cell, segments, currSegment.a, lastEndingInA);

				// Segments starting in B
				Segment3D lastStartingInB;
				int numberStartingInB = SegmentsStartingInP(cell, segments, currSegment.b, lastStartingInB);

				if (numberSegmentEndingInA == 1 && numberStartingInB == 1)
				{
					midPoints = SubdivideCatmullRomSpline<D - 1>(lastEndingInA.a, currSegment.a, currSegment.b, lastStartingInB.b);
				}
				else if (numberSegmentEndingInA != 1 && numberStartingInB == 1)
				{
					Point3D fakeStartingPoint = 2.0 * currSegment.a - currSegment.b;
					midPoints = SubdivideCatmullRomSpline<D - 1>(fakeStartingPoint, currSegment.a, currSegment.b, lastStartingInB.b);
				}
				else if (numberSegmentEndingInA == 1 && numberStartingInB != 1)
				{
					Point3D fakeEndingPoint = 2.0 * currSegment.b - currSegment.a;
					midPoints = SubdivideCatmullRomSpline<D - 1>(lastEndingInA.a, currSegment.a, currSegment.b, fakeEndingPoint);
				}
			}
			
			// First subdivided segment
			subdividedSegments[i - 1][j - 1].front().a = currSegment.a;
			for (int d = 0; d < midPoints.size(); d++)
			{
				subdividedSegments[i - 1][j - 1][d].b = midPoints[d];
				subdividedSegments[i - 1][j - 1][d + 1].a = midPoints[d];
			}
			// Last subdivided segment
			subdividedSegments[i - 1][j - 1].back().b = currSegment.b;
		}
	}
}

template <typename I>
template <size_t N, size_t M, size_t D>
typename Noise<I>::Segment3DArray<N> Noise<I>::GenerateSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Point2DArray<N>& points) const
{
	// Ensure that there is enough segments around to connect sub points
	static_assert(M >= (2 * ((N + 1) / 4) + 3), "Not enough segments in the vicinity to connect sub points.");

	// Connect each point to the nearest segment
	Segment3DArray<N> subSegments;
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			// Find the nearest segment
			Segment3D nearestSegment;
			double nearestSegmentDist = NearestSegmentProjectionZ(1, points[i][j], nearestSegment, cell, segments);
			
			double u = pointLineSegmentProjection(points[i][j], ProjectionZ(nearestSegment));

			// Compute elevation of the point on the control function
			double elevationControlFunction = EvaluateControlFunction(points[i][j]);
			// Compute elevation with a constraint on slope
			// Warning: the distance taken to compute the slope is the distance to the nearest segment
			// The final segment will have a flatter slope
			// Minimum slope 0.5 deg, tan(0.5 deg) = 0.01
			double elevationWithMinSlope = lerp(nearestSegment.a.z, nearestSegment.b.z, u) + 0.01 * nearestSegmentDist;

			double elevation = std::max(elevationWithMinSlope, elevationControlFunction);

			Point3D p(points[i][j].x, points[i][j].y, elevation);

			subSegments[i][j] = ConnectPointToSegmentAngle(p, nearestSegmentDist, nearestSegment);
		}
	}

	return subSegments;
}

template <typename I>
template <size_t N, size_t M, size_t D, size_t K>
typename Noise<I>::Segment3DArray<N> Noise<I>::GenerateSubSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<K>& subSegments, const Point2DArray<N>& points) const
{
	// Ensure that there is enough segments around to connect sub points
	static_assert(M >= (2 * ((N + 1) / 4) + 3), "Not enough level 1 segments in the vicinity to connect sub points.");
	static_assert(K >= (2 * ((N + 1) / 4) + 3), "Not enough level 2 segments in the vicinity to connect sub points.");

	// Connect each point to the nearest segment
	Segment3DArray<N> subSubSegments;
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			// Find the nearest segment
			Segment3D nearestSegment;
			double nearestSegmentDist = NearestSegmentProjectionZ(1, points[i][j], nearestSegment, cell, segments, subCell, subSegments);

			double u = pointLineSegmentProjection(points[i][j], ProjectionZ(nearestSegment));

			// Compute elevation of the point on the control function
			double elevationControlFunction = EvaluateControlFunction(points[i][j]);
			// Compute elevation with a constraint on slope
			// Warning: the distance taken to compute the slope is the distance to the nearest segment
			// The final segment will have a flatter slope
			// Minimum slope 0.5 deg, tan(0.5 deg) = 0.01
			double elevationWithMinSlope = lerp(nearestSegment.a.z, nearestSegment.b.z, u) + 0.01 * nearestSegmentDist;

			double elevation = std::max(elevationWithMinSlope, elevationControlFunction);

			Point3D p(points[i][j].x, points[i][j].y, elevation);

			subSubSegments[i][j] = ConnectPointToSegmentAngle(p, nearestSegmentDist, nearestSegment);
		}
	}

	return subSubSegments;
}

template <typename I>
template <size_t N>
double Noise<I>::ComputeColorPoints(double x, double y, const Point2DArray<N>& points, double radius) const
{
	double value = 0.0;

	const int center = int(points.size()) / 2;

	// White when near to a control point
	for (int i = center - 1; i <= center + 1; i++)
	{
		for (int j = center - 1; j <= center + 1; j++)
		{
			value = std::max(value, ComputeColorPoint(x, y, points[i][j], radius));
		}
	}

	return value;
}

template <typename I>
template <size_t N>
double Noise<I>::ComputeColorPoints(double x, double y, const Segment3DArray<N>& segments, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
			value = std::max(value, ComputeColorPoint(x, y, ProjectionZ(segments[i][j].a), radius));
			value = std::max(value, ComputeColorPoint(x, y, ProjectionZ(segments[i][j].b), radius));
		}
	}

	return value;
}

template <typename I>
template <size_t N, size_t D>
double Noise<I>::ComputeColorPoints(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
			for (int k = 0; k < segments[i][j].size(); k++)
			{
				value = std::max(value, ComputeColorPoint(x, y, ProjectionZ(segments[i][j][k].a), radius));
				value = std::max(value, ComputeColorPoint(x, y, ProjectionZ(segments[i][j][k].b), radius));
			}
		}
	}

	return value;
}

template <typename I>
template <size_t N>
double Noise<I>::ComputeColorSegments(const Cell& cell, const Segment3DArray<N>& segments, int neighborhood, double x, double y, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	Segment3D nearestSegment;
	double nearestSegmentDistance = NearestSegmentProjectionZ(neighborhood, Point2D(x, y), nearestSegment, cell, segments);

	if (nearestSegmentDistance < radius)
	{
		value = 1.0;
	}

	return value;
}

template <typename I>
template <size_t N, size_t D>
double Noise<I>::ComputeColorSegments(const Cell& cell, const Segment3DChainArray<N, D>& segments, int neighborhood, double x, double y, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	Segment3D nearestSegment;
	double nearestSegmentDistance = NearestSegmentProjectionZ(neighborhood, Point2D(x, y), nearestSegment, cell, segments);

	if (nearestSegmentDistance < radius)
	{
		value = 1.0;
	}

	return value;
}

template <typename I>
template <size_t N, size_t D>
double Noise<I>::ComputeColor(const Cell& cell, const Segment3DChainArray<N - 4, D>& subdividedSegments, const Point2DArray<N>& points, double x, double y) const
{
	// Find color
	double value = 0.0;

	if (m_displayPoints)
	{
		value = std::max(value, ComputeColorPoints<N>(x, y, points, 0.0625));
		value = std::max(value, ComputeColorPoints<N - 4, D>(x, y, subdividedSegments, 0.03125));
	}

	if (m_displaySegments)
	{
		value = std::max(value, ComputeColorSegments<N - 4>(cell, subdividedSegments, 1, x, y, 0.015625));
	}

	if (m_displayGrid)
	{
		value = std::max(value, ComputeColorGrid(x, y, 0.0, 0.0, 0.0078125));
	}

	return value;
}

template <typename I>
template <size_t N>
double Noise<I>::ComputeColorSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const
{
	// Find color
	double value = 0.0;

	if (m_displayPoints)
	{
		value = std::max(value, ComputeColorPoints<N>(x, y, points, 0.03125));
	}

	if (m_displaySegments)
	{
		value = std::max(value, ComputeColorSegments<N>(cell, segments, 2, x, y, 0.0078125));
	}

	if (m_displayGrid)
	{
		value = std::max(value, ComputeColorGrid(x, y, 0.0, 0.0, 0.00390625));
		value = std::max(value, ComputeColorGrid(x, y, 0.5, 0.5, 0.00390625));
	}

	return value;
}

template <typename I>
template <size_t N>
double Noise<I>::ComputeColorSubSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const
{
	// Find color
	double value = 0.0;

	if (m_displayPoints)
	{
		value = std::max(value, ComputeColorPoints<N>(x, y, points, 0.015625));
	}

	if (m_displaySegments)
	{
		value = std::max(value, ComputeColorSegments<N>(cell, segments, 2, x, y, 0.0078125));
	}

	if (m_displayGrid)
	{
		value = std::max(value, ComputeColorGrid(x, y, 0.0, 0.0, 0.001953125));
		value = std::max(value, ComputeColorGrid(x, y, 0.25, 0.25, 0.001953125));
		value = std::max(value, ComputeColorGrid(x, y, 0.50, 0.50, 0.001953125));
		value = std::max(value, ComputeColorGrid(x, y, 0.75, 0.75, 0.001953125));
	}

	return value;
}

template <typename I>
template <size_t N, size_t D, size_t M, size_t K>
double Noise<I>::ComputeColorPrimitives(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, const Point2DArray<K>& subSubPoints, double x, double y) const
{
	const Point2D point(x, y);

	const int resolutionSteps = 3;

	// Generate higher resolution points, which are going to be the centers of primitives
	Cell higherResCell = subSubCell;
	Point2DArray<5> points = subSubPoints;
	for (int i = 0; i < resolutionSteps; i++)
	{
		Cell newCell = GetCell(x, y, 2 * higherResCell.resolution);
		Point2DArray<5> newPoints = GenerateNeighboringPoints<5>(newCell);
		ReplaceNeighboringPoints(higherResCell, subSubPoints, newCell, newPoints);

		higherResCell = newCell;
		points = newPoints;
	}

	// Radius of primitives
	const double R = 2.0 / higherResCell.resolution;
	// Power to the Wyvill-Galin function
	const double P = 3.0;
	// tan(45 deg) = 1.00
	const double tanSlope = 1.0;

	// Numerator and denominator used to compute the blend of primitives
	double numerator = 0.0;
	double denominator = 0.0;

	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			// Nearest segment to points[i][j] and nearest point on this segment
			Segment3D primitiveNearestSegment;
			double distancePrimitiveCenter = NearestSegmentProjectionZ(1, points[i][j], primitiveNearestSegment, cell, subdividedSegments, subCell, subSegments, subSubCell, subSubSegments);
			double uPrimitive = pointLineSegmentProjection(points[i][j], ProjectionZ(primitiveNearestSegment));

			double distancePrimitive = dist(point, points[i][j]);

			double alphaPrimitive = WyvillGalinFunction(distancePrimitive, R, P);
			// Slope of approximately arctan(tanSlope) deg
			double elevation = lerp(primitiveNearestSegment.a.z, primitiveNearestSegment.b.z, uPrimitive) + tanSlope * distancePrimitiveCenter;
			numerator += alphaPrimitive * elevation;
			denominator += alphaPrimitive;
		}
	}

	// denominator shouldn't be equal to zero if there is enough primitives around the point.
	assert(denominator != 0.0);

	return numerator / denominator;
}

template <typename I>
template <size_t N, size_t D, size_t M, size_t K>
double Noise<I>::ComputeColorControlFunction(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, double x, double y) const
{
	const Point2D point(x, y);

	// nearest segment
	Segment3D nearestSegment;
	double d = NearestSegmentProjectionZ(1, point, nearestSegment, cell, subdividedSegments, subCell, subSegments, subSubCell, subSubSegments);

	double value = 0.0;

	if (d < (1.0 / 32.0))
	{
		double u = pointLineSegmentProjection(point, ProjectionZ(nearestSegment));
		// Elevation of the nearest point
		value = lerp(nearestSegment.a.z, nearestSegment.b.z, u);
	}
	else
	{
		value = EvaluateControlFunction(point);
	}

	return value;
}

#endif // NOISE_H