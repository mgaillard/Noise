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

std::tuple<int, int> Noise::GetSubQuadrant(double cx, double cy, double x, double y) const
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

	return std::make_tuple(quadrantX, quadrantY);
}

double Noise::evaluate(double x, double y) const
{
	// In which cell is the point (x, y)
	const double cx = floor(x);
	const double cy = floor(y);
	const int cxInt = int(cx);
	const int cyInt = int(cy);

	// Level 1: Points in neighboring cells
	Point2DArray<9> points = GenerateNeighboringPoints<9>(cxInt, cyInt);
	// Level 1: List of segments 
	Segment3DArray<7> segments = GenerateSegments(points);

	// Subdivide segments of level 1
	Segment3DChainArray<5, 2> subdividedSegments;
	Point2DArray<5> midPoints;
	SubdivideSegments(cx, cy, segments, subdividedSegments);

	// Level 2: Points in neighboring cells
	Point2DArray<5> subPoints = GenerateNeighboringSubPoints<5, 9>(cx, cy, x, y, points);
	// Level 2: List of segments
	Segment3DArray<5> subSegments = GenerateSubSegments<5, 5>(cx, cy, subPoints, subdividedSegments);

	double value = 0.0;

	value = std::max(value, ComputeColorWorley(x, y, subdividedSegments, subSegments));
	value = std::max(value, ComputeColor(x, y, points, subdividedSegments));
	value = std::max(value, ComputeColorSub(x, y, subPoints, subSegments));

	return value;
}
