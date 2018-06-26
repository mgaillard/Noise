#ifndef NOISE_H
#define NOISE_H

#include <array>
#include <vector>
#include <random>
#include <tuple>
#include <limits>
#include <cmath>
#include <cassert>

#include "perlin.h"
#include "math2d.h"
#include "math3d.h"
#include "spline.h"
#include "utils.h"

class Noise
{
public:
	Noise(const Point2D& noiseTopLeft, const Point2D& noiseBottomRight, const Point2D& perlinTopLeft, const Point2D& perlinBottomRight, int seed = 0, double eps = 0.0, bool displayPoints = true, bool displaySegments = true, bool displayGrid = true);

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

	Segment3D ConnectPointToSegmentAngle(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	Segment3D ConnectPointToSegmentAngleMid(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	Segment3D ConnectPointToSegmentNearestPoint(const Point3D& point, double segmentDist, const Segment3D& segment) const;

	template<typename T, size_t N>
	std::tuple<int, int> GetArrayCell(const Cell& arrCell, const Array2d<T, N>& arr, const Cell& cell) const;

	template<size_t N>
	double NearestSegmentProjectionZ(const Cell& cell, const Segment3DArray<N>& segments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const;

	template<size_t N, size_t M>
	double NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<N, M>& segments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const;

	template<size_t M, size_t D, size_t N>
	double NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<N>& subSegments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const;

	template<size_t M, size_t D, size_t N, size_t K>
	double NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<N>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const;

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
	double ComputeColorWorley(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, double x, double y) const;
	
	// Random generator used by the class
	typedef std::minstd_rand RandomGenerator;

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
	std::vector<std::vector<Point2D> > m_pointCache;
};

template<typename T, size_t N>
std::tuple<int, int> Noise::GetArrayCell(const Cell& arrCell, const Array2d<T, N>& arr, const Cell& cell) const
{
	int i = (int(arr.size()) / 2) - arrCell.y + cell.y;
	int j = (int(arr.front().size()) / 2) - arrCell.x + cell.x;

	return std::make_tuple(i, j);
}

template<size_t N>
double Noise::NearestSegmentProjectionZ(const Cell& cell, const Segment3DArray<N>& segments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const
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

template<size_t N, size_t M>
double Noise::NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<N, M>& segments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const
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

template<size_t M, size_t D, size_t N>
double Noise::NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<N>& subSegments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const
{
	assert(neighborhood >= 0);

	// Distance to segments
	double nearestSegmentDistance = NearestSegmentProjectionZ(cell, segments, neighborhood, point, nearestSegmentOut);

	// Distance to sub segments
	Segment3D nearestSubSegment;
	double nearestSubSegmentDistance = NearestSegmentProjectionZ(subCell, subSegments, neighborhood, point, nearestSubSegment);
	if (nearestSubSegmentDistance < nearestSegmentDistance)
	{
		nearestSegmentDistance = nearestSubSegmentDistance;
		nearestSegmentOut = nearestSubSegment;
	}

	return nearestSegmentDistance;
}

template<size_t M, size_t D, size_t N, size_t K>
double Noise::NearestSegmentProjectionZ(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<N>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, int neighborhood, const Point2D& point, Segment3D& nearestSegmentOut) const
{
	assert(neighborhood >= 0);

	// Distance to segments
	double nearestSegmentDistance = NearestSegmentProjectionZ(cell, segments, subCell, subSegments, neighborhood, point, nearestSegmentOut);

	// Distance to sub segments
	Segment3D nearestSubSegment;
	double nearestSubSegmentDistance = NearestSegmentProjectionZ(subSubCell, subSubSegments, neighborhood, point, nearestSubSegment);
	if (nearestSubSegmentDistance < nearestSegmentDistance)
	{
		nearestSegmentDistance = nearestSubSegmentDistance;
		nearestSegmentOut = nearestSubSegment;
	}

	return nearestSegmentDistance;
}

template <size_t N>
int Noise::SegmentsEndingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentEndingInP) const
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

template <size_t N>
int Noise::SegmentsStartingInP(const Cell& cell, const Segment3DArray<N>& segments, const Point3D& point, Segment3D& lastSegmentStartingInP) const
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

template <size_t N>
Noise::Point2DArray<N> Noise::GenerateNeighboringPoints(const Cell& cell) const
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

template <size_t N, size_t M>
void Noise::ReplaceNeighboringPoints(const Cell& cell, const Point2DArray<M>& points, const Cell& subCell, Point2DArray<N>& subPoints) const
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

template <size_t N>
Noise::DoubleArray<N> Noise::ComputeElevations(const Point2DArray<N>& points) const
{
	DoubleArray<N> elevations;

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

template <size_t N>
Noise::Segment3DArray<N - 2> Noise::GenerateSegments(const Point2DArray<N>& points) const
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
template <size_t N, size_t D>
void Noise::SubdivideSegments(const Cell& cell, const Segment3DArray<N>& segments, Segment3DChainArray<N - 2, D>& subdividedSegments) const
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

template <size_t N, size_t M, size_t D>
Noise::Segment3DArray<N> Noise::GenerateSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Point2DArray<N>& points) const
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
			double nearestSegmentDist = NearestSegmentProjectionZ(cell, segments, 1, points[i][j], nearestSegment);
			
			double u = pointLineSegmentProjection(points[i][j], ProjectionZ(nearestSegment));

			// Compute elevation of the point
			double x = Remap(points[i][j].x, m_noiseTopLeft.x, m_noiseBottomRight.x, m_perlinTopLeft.x, m_perlinBottomRight.x);
			double y = Remap(points[i][j].y, m_noiseTopLeft.y, m_noiseBottomRight.y, m_perlinTopLeft.y, m_perlinBottomRight.y);
			double perlin = (Perlin(cell.resolution * x, cell.resolution * y) + 1.0) / 4.0;

			// minimum slope 5 degrees, tan(5 degrees) = 0.09 
			double elevation = lerp(nearestSegment.a.z, nearestSegment.b.z, u) + std::max(0.09 * nearestSegmentDist, perlin);

			Point3D p(points[i][j].x, points[i][j].y, elevation);

			subSegments[i][j] = ConnectPointToSegmentAngle(p, nearestSegmentDist, nearestSegment);
		}
	}

	return subSegments;
}

template <size_t N, size_t M, size_t D, size_t K>
Noise::Segment3DArray<N> Noise::GenerateSubSubSegments(const Cell& cell, const Segment3DChainArray<M, D>& segments, const Cell& subCell, const Segment3DArray<K>& subSegments, const Point2DArray<N>& points) const
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
			double nearestSegmentDist = NearestSegmentProjectionZ(cell, segments, subCell, subSegments, 1, points[i][j], nearestSegment);

			double u = pointLineSegmentProjection(points[i][j], ProjectionZ(nearestSegment));
			double elevation = lerp(nearestSegment.a.z, nearestSegment.b.z, u);
			Point3D p(points[i][j].x, points[i][j].y, elevation);

			subSubSegments[i][j] = ConnectPointToSegmentAngle(p, nearestSegmentDist, nearestSegment);
		}
	}

	return subSubSegments;
}

template <size_t N>
double Noise::ComputeColorPoints(double x, double y, const Point2DArray<N>& points, double radius) const
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

template <size_t N>
double Noise::ComputeColorPoints(double x, double y, const Segment3DArray<N>& segments, double radius) const
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

template <size_t N, size_t D>
double Noise::ComputeColorPoints(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const
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

template <size_t N>
double Noise::ComputeColorSegments(const Cell& cell, const Segment3DArray<N>& segments, int neighborhood, double x, double y, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	Segment3D nearestSegment;
	double nearestSegmentDistance = NearestSegmentProjectionZ(cell, segments, neighborhood, Point2D(x, y), nearestSegment);

	if (nearestSegmentDistance < radius)
	{
		value = 1.0;
	}

	return value;
}

template <size_t N, size_t D>
double Noise::ComputeColorSegments(const Cell& cell, const Segment3DChainArray<N, D>& segments, int neighborhood, double x, double y, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	Segment3D nearestSegment;
	double nearestSegmentDistance = NearestSegmentProjectionZ(cell, segments, neighborhood, Point2D(x, y), nearestSegment);

	if (nearestSegmentDistance < radius)
	{
		value = 1.0;
	}

	return value;
}

template <size_t N, size_t D>
double Noise::ComputeColor(const Cell& cell, const Segment3DChainArray<N - 4, D>& subdividedSegments, const Point2DArray<N>& points, double x, double y) const
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

template <size_t N>
double Noise::ComputeColorSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const
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

template <size_t N>
double Noise::ComputeColorSubSub(const Cell& cell, const Segment3DArray<N>& segments, const Point2DArray<N>& points, double x, double y) const
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

template <size_t N, size_t D, size_t M, size_t K>
double Noise::ComputeColorWorley(const Cell& cell, const Segment3DChainArray<N, D>& subdividedSegments, const Cell& subCell, const Segment3DArray<M>& subSegments, const Cell& subSubCell, const Segment3DArray<K>& subSubSegments, double x, double y) const
{
	const Point2D point(x, y);
	
	Segment3D nearestSegment;
	double distance = NearestSegmentProjectionZ(cell, subdividedSegments, subCell, subSegments, subSubCell, subSubSegments, 1, point, nearestSegment);

	// Elevation in on the nearest segment
	const double u = pointLineProjection(point, ProjectionZ(nearestSegment));
	const double elevation = lerp(nearestSegment, u).z;

	return distance; // Otherwise: distance + elevation;
}

#endif // NOISE_H