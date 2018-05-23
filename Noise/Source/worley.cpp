#include "worley.h"

#include <random>
#include <cstdint>
#include <limits>
#include <cmath>
#include <tuple>

using namespace std;

struct WorleyResult
{
	double x;
	double y;
	double dist_sq;

	WorleyResult() : x(0.0), y(0.0), dist_sq(numeric_limits<double>::max()) {}

	WorleyResult(double _x, double _y, double _dist_sq) : x(_x), y(_y), dist_sq(_dist_sq) {}
};

int GenerateSeed(int i, int j)
{
	return (541 * i + 79 * j) % numeric_limits<int>::max();
}

tuple<WorleyResult, WorleyResult> WorleyUtils(double x, double y)
{
	default_random_engine generator;
	uniform_real_distribution<double> distribution(0.0, 1.0);

	// In which cell is the point
	const double int_x = floor(x);
	const double int_y = floor(y);

	const double u = x - int_x;
	const double v = y - int_y;

	const int cx = int(int_x);
	const int cy = int(int_y);

	// Distance to the first and second nearest neighbors
	WorleyResult f1, f2;

	// Exploring neighboring cells
	for (int i = cy - 1; i <= cy + 1; i++)
	{
		for (int j = cx - 1; j <= cx + 1; j++)
		{
			// Fixed seed for internal consistency
			generator.seed(GenerateSeed(i, j));

			// TODO: How many points in this cell
			int n = 1;
			for (int p = 0; p < n; p++)
			{
				// Generate each points
				double px = distribution(generator);
				double py = distribution(generator);

				double dist_sq = hypot(j + px - x, i + py - y);

				if (dist_sq < f1.dist_sq)
				{
					f2 = f1;
					f1 = WorleyResult(j + px, i + py, dist_sq);
				}
				else if (dist_sq < f2.dist_sq)
				{
					f2 = WorleyResult(j + px, i + py, dist_sq);
				}
			}
		}
	}

	return make_tuple(f1, f2);
}

double WorleyF1(double x, double y)
{
	WorleyResult f1, f2;
	tie(f1, f2) = WorleyUtils(x, y);

	return sqrt(f1.dist_sq / 2.0);
}

double WorleyF1Minus(double x, double y)
{
	WorleyResult f1, f2;
	tie(f1, f2) = WorleyUtils(x, y);

	return 1.0 - sqrt(f1.dist_sq / 2.0);
}

double WorleyF2(double x, double y)
{
	WorleyResult f1, f2;
	tie(f1, f2) = WorleyUtils(x, y);

	return (sqrt(f2.dist_sq) - sqrt(f1.dist_sq)) / (2.0 * sqrt(2.0));
}

double WorleyF2Minus(double x, double y)
{
	WorleyResult f1, f2;
	tie(f1, f2) = WorleyUtils(x, y);

	return 1.0 - (sqrt(f2.dist_sq) - sqrt(f1.dist_sq)) / (2.0 * sqrt(2.0));
}
