#include "utils.h"

double cubic_interpolate(double p0, double p1, double p2, double p3, double t)
{
	assert(0.0 <= t && t <= 1.0);

	return p1 + 0.5 * t * (p2 - p0 + t * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3 + t * (3.0 * (p1 - p2) + p3 - p0)));
}

double cubic_interpolate(const std::array<double, 4>& p, double t)
{
	assert(0.0 <= t && t <= 1.0);

	return cubic_interpolate(p[0], p[1], p[2], p[3], t);
}

double bi_cubic_interpolate(const std::array<std::array<double, 4>, 4>& p, double u, double v)
{
	assert(0.0 <= u && u <= 1.0);
	assert(0.0 <= v && v <= 1.0);

	std::array<double, 4> temp{};
	for (unsigned int i = 0; i < 4; i++)
	{
		temp[i] = cubic_interpolate(p[i], v);
	}
	return cubic_interpolate(temp, u);
}

double matlab_jet_base(double val)
{
	if (val <= 0.125) {
		return 0.0;
	}
	else if (val <= 0.375) {
		return remap_clamp(val, 0.125, 0.375, 0.0, 1.0);
	}
	else if (val <= 0.625) {
		return 1.0;
	}
	else if (val <= 0.875) {
		return remap_clamp(val, 0.625, 0.875, 1.0, 0.0);
	}
	else {
		return 0.0;
	}
}

std::array<double, 3> matlab_jet(double u)
{
	const double r = matlab_jet_base(u - 0.25);
	const double g = matlab_jet_base(u);
	const double b = matlab_jet_base(u + 0.25);

	return { { r, g, b } };
}
