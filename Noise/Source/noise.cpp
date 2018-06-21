#include "noise.h"

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
	InitPointCache();
}

void Noise::InitPointCache()
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

int Noise::GenerateSeedNoise(int i, int j) const
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
Point2D Noise::GeneratePoint(int x, int y) const
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
Point2D Noise::GeneratePointCached(int x, int y) const
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

Noise::Cell Noise::GetCell(double x, double y, int resolution) const
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

Segment3D Noise::ConnectPointToSegmentAngle(const Point2D& point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineProjection(point, ProjectionZ(segment));
	// The intersection must lie on the segment
	u = clamp(u, 0.0, 1.0);

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
	const double length = dist(point, ProjectionZ(segmentEnd));
	const Point3D segmentStart(point.x, point.y, segmentEnd.z + 0.1 * length);

	return Segment3D(segmentStart, segmentEnd);
}

Segment3D Noise::ConnectPointToSegmentAngleMid(const Point2D& point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineProjection(point, ProjectionZ(segment));

	// Find the intersection so that the angle between the two segments is 45°
	// v designates the ratio of the segment on which the intersection is located
	// v = 0 is point A of the segment ; v = 1 is point B of the segment
	double v = u + segmentDist / length(ProjectionZ(segment));
	// The intersection must lie on the segment
	v = clamp(v, 0.0, 1.0);

	const Point3D segmentEnd(lerp(segment, u));
	const double length = dist(point, ProjectionZ(segmentEnd));
	const Point3D segmentStart(point.x, point.y, segmentEnd.z + 0.1 * length);

	return Segment3D(segmentStart, segmentEnd);
}

Segment3D Noise::ConnectPointToSegmentNearestPoint(const Point2D& point, double segmentDist, const Segment3D& segment) const
{
	// Find an intersection on the segment with respect to constraints
	// u = 0 is point A of the segment ; u = 1 is point B of the segment
	double u = pointLineProjection(point, ProjectionZ(segment));

	// The intersection must lie on the segment
	u = clamp(u, 0.0, 1.0);

	const Point3D segmentEnd(lerp(segment, u));
	const double length = dist(point, ProjectionZ(segmentEnd));
	const Point3D segmentStart(point.x, point.y, segmentEnd.z + 0.1 * length);

	return Segment3D(segmentStart, segmentEnd);
}

double Noise::ComputeColorPoint(double x, double y, const Point2D& point, double radius) const
{
	double value = 0.0;

	if (dist(Point2D(x, y), point) < radius)
	{
		value = 1.0;
	}

	return value;
}

double Noise::ComputeColorSegment(double x, double y, const Segment2D& segment, double radius) const
{
	double value = 0.0;

	Point2D c;
	if (distToLineSegment(Point2D(x, y), segment, c) < radius)
	{
		value = 1.0;
	}

	return value;
}

double Noise::ComputeColorGrid(double x, double y, double deltaX, double deltaY, double radius) const
{
	double value = 0.0;

	// When near to the grid
	if (abs(x - floor(x) - deltaX) < radius || abs(y - floor(y) - deltaY) < radius)
	{
		value = 1.0;
	}

	return value;
}

double Noise::evaluate(double x, double y) const
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
	// Level 2: List of segments
	Segment3DArray<5> subSubSegments = GenerateSubSubSegments(cell, subdividedSegments, subCell, subSegments, subSubPoints);

	double value = 0.0;

	value = std::max(value, ComputeColorWorley(cell, subdividedSegments, subCell, subSegments, subSubCell, subSubSegments, x, y));
	value = std::max(value, ComputeColor(cell, subdividedSegments, points, x, y));
	value = std::max(value, ComputeColorSub(subCell, subSegments, subPoints, x, y));
	value = std::max(value, ComputeColorSubSub(subSubCell, subSubSegments, subSubPoints, x, y));

	return value;
}
