#include "imagecontrolfunction.h"

double ImageControlFunction::sample(double ri, double rj) const
{
	assert(0.0 <= ri && ri <= 1.0);
	assert(0.0 <= rj && rj <= 1.0);

	ri *= m_image.rows - 1;
	rj *= m_image.cols - 1;

	// If the coordinates are integer, return directly the value
	if (nearbyint(ri) == ri && nearbyint(rj) == rj)
	{
		return get(int(ri), int(rj));
	}

	const auto i1 = int(floor(ri));
	const auto j1 = int(floor(rj));

	// i = {i1 - 1, i1, i1 + 1, i1 + 2}
	const std::array<int, 4> i = {
		std::max(i1 - 1, 0),
		i1,
		std::min(i1 + 1, m_image.rows - 1),
		std::min(i1 + 2, m_image.rows - 1)
	};

	// j = {j1 - 1, j1, j1 + 1, j1 + 2}
	const std::array<int, 4> j = {
		std::max(j1 - 1, 0),
		j1,
		std::min(j1 + 1, m_image.cols - 1),
		std::min(j1 + 2, m_image.cols - 1)
	};

	std::array<std::array<double, 4>, 4> p{};
	for (int k = 0; k < 4; k++)
	{
		for (int l = 0; l < 4; l++)
		{
			p[k][l] = get(i[k], j[l]);
		}
	}

	const double interpolation = bi_cubic_interpolate(p, ri - floor(ri), rj - floor(rj));

	return clamp(interpolation, 0.0, 1.0);
}
