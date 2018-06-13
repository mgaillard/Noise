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

	void InitPointCache();

	int GenerateSeedNoise(int i, int j) const;

	Point2D GeneratePoint(int x, int y) const;
	Point2D GeneratePointCached(int x, int y) const;

	std::tuple<int, int> GetCell(double x, double y, int resolution) const;

	template<typename T, size_t N>
	std::tuple<int, int> GetArrayCell(int arrCellX, int arrCellY, const Array2d<T, N>& arr, int cellX, int cellY) const;

	template<typename T, size_t N>
	std::tuple<int, int> GetArrayCell(int arrCellX, int arrCellY, const Array2d<T, N>& arr, const std::tuple<int, int>& cell) const;

	template<size_t N>
	double NearestSegmentProjectionZ(int cellX, int cellY, const Segment3DArray<N>& segments, const Point2D& point, Segment3D& nearestSegmentOut) const;

	template<size_t N, size_t M>
	double NearestSegmentProjectionZ(int cellX, int cellY, const Segment3DChainArray<N, M>& segments, const Point2D& point, Segment3D& nearestSegmentOut) const;

	template <size_t N>
	Point2DArray<N> GenerateNeighboringPoints(int cx, int cy, int resolution) const;

	template <size_t N, size_t M>
	Point2DArray<N> GenerateNeighboringSubPoints(int cellX, int cellY, const Point2DArray<M>& points, int subCellX, int subCellY) const;

	template <size_t N>
	DoubleArray<N> ComputeElevations(const Point2DArray<N>& points) const;
	
	template <size_t N>
	Segment3DArray<N - 2> GenerateSegments(const Point2DArray<N>& points) const;

	// Subdivide all segments in a Segment3DArray<N> in D smaller segments
	// Require a Segment3DArray<N> to generate a Segment3DChainArray<N - 2, D> because to subdivide a segment we need its predecessors and successors.
	template <size_t N, size_t D>
	void SubdivideSegments(int cellX, int cellY, const Segment3DArray<N>& segments, Segment3DChainArray<N - 2, D>& subdividedSegments) const;

	template <size_t N, size_t M, size_t D>
	Segment3DArray<N> GenerateSubSegments(int cellX, int cellY, const Point2DArray<N>& points, const Segment3DChainArray<M, D>& subdividedSegments) const;

	double ComputeColorPoint(double x, double y, const Point2D& point, double radius) const;

	template <size_t N>
	double ComputeColorPoints(double x, double y, const Point2DArray<N>& points, double radius) const;

	template <size_t N>
	double ComputeColorPoints(double x, double y, const Segment3DArray<N>& segments, double radius) const;

	template <size_t N, size_t D>
	double ComputeColorPoints(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const;

	double ComputeColorSegment(double x, double y, const Segment2D& segment, double radius) const;

	template <size_t N>
	double ComputeColorSegments(double x, double y, const Segment3DArray<N>& segments, double radius) const;

	template <size_t N, size_t D>
	double ComputeColorSegments(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const;

	double ComputeColorGrid(double x, double y, double deltaX, double deltaY, double radius) const;

	template <size_t N, size_t D>
	double ComputeColor(double x, double y, const Point2DArray<N>& points, const Segment3DChainArray<N- 4, D>& subdividedSegments) const;

	template <size_t N>
	double ComputeColorSub(double x, double y, const Point2DArray<N>& points, const Segment3DArray<N>& segments) const;

	template <size_t N, size_t D, size_t M>
	double ComputeColorWorley(double x, double y, int cellX, int cellY, const Segment3DChainArray<N, D>& subdividedSegments, int subCellX, int subCellY, const Segment3DArray<M>& subSegments) const;
	
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
std::tuple<int, int> Noise::GetArrayCell(int arrCellX, int arrCellY, const Array2d<T, N>& arr, int cellX, int cellY) const
{
	int i = (int(arr.size()) / 2) - arrCellY + cellY;
	int j = (int(arr.front().size()) / 2) - arrCellX + cellX;

	return std::make_tuple(i, j);
}

template<typename T, size_t N>
std::tuple<int, int> Noise::GetArrayCell(int arrCellX, int arrCellY, const Array2d<T, N>& arr, const std::tuple<int, int>& cell) const
{
	return GetArrayCell(arrCellX, arrCellY, arr, std::get<0>(cell), std::get<1>(cell));
}

template<size_t N>
double Noise::NearestSegmentProjectionZ(int cellX, int cellY, const Segment3DArray<N>& segments, const Point2D& point, Segment3D& nearestSegmentOut) const
{
	// Distance to the nearest segment
	double nearestSegmentDistance = std::numeric_limits<double>::max();

	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
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
double Noise::NearestSegmentProjectionZ(int cellX, int cellY, const Segment3DChainArray<N, M>& segments, const Point2D& point, Segment3D& nearestSegmentOut) const
{
	// Distance to the nearest segment
	double nearestSegmentDistance = std::numeric_limits<double>::max();

	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
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

template <size_t N>
Noise::Point2DArray<N> Noise::GenerateNeighboringPoints(int cx, int cy, int resolution) const
{
	Point2DArray<N> points;

	// Exploring neighboring cells
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < points[i].size(); j++)
		{
			const int x = cx + j - int(points[i].size()) / 2;
			const int y = cy + i - int(points.size()) / 2;

			points[i][j] = GeneratePointCached(x, y) / resolution;
		}
	}

	return points;
}

template <size_t N, size_t M>
Noise::Point2DArray<N> Noise::GenerateNeighboringSubPoints(int cellX, int cellY, const Point2DArray<M>& points, int subCellX, int subCellY) const
{
	// Ensure that there is enough points around to replace subpoints
	static_assert(M >= (2 * ((N + 1) / 4) + 1), "Not enough points in the vicinity to replace the sub points.");

	Point2DArray<N> subPoints = GenerateNeighboringPoints<N>(subCellX, subCellY, 2);
	
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
			std::tie(k, l) = GetArrayCell(subCellX, subCellY, subPoints, GetCell(points[i][j].x, points[i][j].y, 2));

			if (k >= 0 && k < subPoints.size() && l >= 0 && l < subPoints.front().size())
			{
				subPoints[k][l] = points[i][j];
			}
		}
	}

	return subPoints;
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

template <size_t N, size_t D>
void Noise::SubdivideSegments(int cellX, int cellY, const Segment3DArray<N>& segments, Segment3DChainArray<N - 2, D>& subdividedSegments) const
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
				int numberSegmentEndingInA = 0;
				Segment3D lastEndingInA;

				// In which cell of segments is A
				int ck, cl;
				std::tie(ck, cl) = GetArrayCell(cellX, cellY, segments, GetCell(currSegment.a.x, currSegment.a.y, 1));
				for (int k = ck - 1; k <= ck + 1; k++)
				{
					for (int l = cl - 1; l <= cl + 1; l++)
					{
						assert(k >= 0 && k < segments.size());
						assert(l >= 0 && l < segments.front().size());

						// If the segment's length is more than 0
						if (segments[k][l].a != segments[k][l].b)
						{
							if (segments[k][l].b == currSegment.a)
							{
								numberSegmentEndingInA++;
								lastEndingInA = segments[k][l];
							}
						}
					}
				}

				// Segments starting in B
				int numberStartingInB = 0;
				Segment3D lastStartingInB;

				// In which cell of segments is B
				int m, n;
				std::tie(m, n) = GetArrayCell(cellX, cellY, segments, GetCell(currSegment.b.x, currSegment.b.y, 1));

				assert(m >= 0 && m < segments.size());
				assert(n >= 0 && n < segments.front().size());

				// If the segment's length is more than 0
				if (segments[m][n].a != segments[m][n].b)
				{
					if (segments[m][n].a == currSegment.b)
					{
						numberStartingInB++;
						lastStartingInB = segments[m][n];
					}
				}

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
Noise::Segment3DArray<N> Noise::GenerateSubSegments(int cellX, int cellY, const Point2DArray<N>& points, const Segment3DChainArray<M, D>& subdividedSegments) const
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
			double nearestSegmentDist = std::numeric_limits<double>::max();
			Segment3D nearestSegment;

			// In which cell of subdividedSegments is the point
			int ck, cl;
			std::tie(ck, cl) = GetArrayCell(cellX, cellY, subdividedSegments, GetCell(points[i][j].x, points[i][j].y, 1));
			for (int k = ck - 1; k <= ck + 1; k++)
			{
				for (int l = cl - 1; l <= cl + 1; l++)
				{
					assert(k >= 0 && k < subdividedSegments.size());
					assert(l >= 0 && l < subdividedSegments.front().size());

					for (int m = 0; m < subdividedSegments[k][l].size(); m++)
					{
						Point2D c;
						double dist = distToLineSegment(points[i][j], ProjectionZ(subdividedSegments[k][l][m]), c);

						if (dist < nearestSegmentDist)
						{
							nearestSegmentDist = dist;
							nearestSegment = subdividedSegments[k][l][m];
						}
					}
				}
			}

			// Find an intersection on the segment with respect to constraints
			// u = 0 is point A of the segment ; u = 1 is point B of the segment
			double u = pointLineProjection(points[i][j], ProjectionZ(nearestSegment));
			// The intersection must lie on the segment
			u = clamp(u, 0.0, 1.0);

			// If, on the segment, the nearest point is between A and B, we shift it so that the angle constraint is respected
			if (u > 0.0 && u < 1.0)
			{
				// Find the intersection so that the angle between the two segments is 45°
				// v designates the ratio of the segment on which the intersection is located
				// v = 0 is point A of the segment ; v = 1 is point B of the segment
				double v = u + nearestSegmentDist / length(ProjectionZ(nearestSegment));

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

			// TODO compute the elevation of segmentStart in a better way
			const Point3D segmentEnd(lerp(nearestSegment, u));
			const Point3D segmentStart(points[i][j].x, points[i][j].y, segmentEnd.z);

			subSegments[i][j] = Segment3D(segmentStart, segmentEnd);
		}
	}

	return subSegments;
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
double Noise::ComputeColorSegments(double x, double y, const Segment3DArray<N>& segments, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
			value = std::max(value, ComputeColorSegment(x, y, ProjectionZ(segments[i][j]), radius));
		}
	}

	return value;
}

template <size_t N, size_t D>
double Noise::ComputeColorSegments(double x, double y, const Segment3DChainArray<N, D>& segments, double radius) const
{
	double value = 0.0;

	// White when near to a segment
	for (int i = 0; i < segments.size(); i++)
	{
		for (int j = 0; j < segments[i].size(); j++)
		{
			for (int k = 0; k < segments[i][j].size(); k++)
			{
				value = std::max(value, ComputeColorSegment(x, y, ProjectionZ(segments[i][j][k]), radius));
			}
		}
	}

	return value;
}

template <size_t N, size_t D>
double Noise::ComputeColor(double x, double y, const Point2DArray<N>& points, const Segment3DChainArray<N - 4, D>& subdividedSegments) const
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
		value = std::max(value, ComputeColorSegments<N - 4>(x, y, subdividedSegments, 0.015625));
	}

	if (m_displayGrid)
	{
		value = std::max(value, ComputeColorGrid(x, y, 0.0, 0.0, 0.0078125));
	}

	return value;
}

template <size_t N>
double Noise::ComputeColorSub(double x, double y, const Point2DArray<N>& points, const Segment3DArray<N>& segments) const
{
	// Find color
	double value = 0.0;

	if (m_displayPoints)
	{
		value = std::max(value, ComputeColorPoints<N>(x, y, points, 0.03125));
	}

	if (m_displaySegments)
	{
		value = std::max(value, ComputeColorSegments<N>(x, y, segments, 0.0078125));
	}

	if (m_displayGrid)
	{
		value = std::max(value, ComputeColorGrid(x, y, 0.5, 0.5, 0.00390625));
	}

	return value;
}

template <size_t N, size_t D, size_t M>
double Noise::ComputeColorWorley(double x, double y, int cellX, int cellY, const Segment3DChainArray<N, D>& subdividedSegments, int subCellX, int subCellY, const Segment3DArray<M>& subSegments) const
{
	// Find the nearest segment
	
	// Distance to level 1 segments
	Segment3D nearestSegment;
	double nearestSegmentDistance = NearestSegmentProjectionZ(cellX, cellY, subdividedSegments, Point2D(x, y), nearestSegment);

	// Distance to level 2 segments
	Segment3D nearestSubSegment;
	double nearestSubSegmentDistance = NearestSegmentProjectionZ(subCellX, subCellY, subSegments, Point2D(x, y), nearestSubSegment);
	if (nearestSubSegmentDistance < nearestSegmentDistance)
	{
		nearestSegmentDistance = nearestSubSegmentDistance;
		nearestSegment = nearestSubSegment;
	}

	// Elevation in on the nearest segment
	const double elevationA = nearestSegment.a.z;
	const double elevationB = nearestSegment.b.z;
	const double u = pointLineProjection(Point2D(x, y), ProjectionZ(nearestSegment));
	const double elevation = lerp_clamp(elevationA, elevationB, u);

	return nearestSegmentDistance + elevation;
}

#endif // NOISE_H