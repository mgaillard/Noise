#ifndef UTILS_H
#define UTILS_H

#include <cassert>
#include <array>

template<typename T>
T remap(const T& x, const T& in_start, const T& in_end, const T& out_start, const T& out_end)
{
	assert(in_start != in_end);
	
	return out_start + (out_end - out_start) * (x - in_start) / (in_end - in_start);
}

template<typename T>
T remap_clamp(const T& x, const T& in_start, const T& in_end, const T& out_start, const T& out_end)
{
	assert(in_start != in_end);

	if (x < in_start)
	{
		return out_start;
	}
	else if (x > in_end)
	{
		return out_end;
	}
	else
	{
		return out_start + (out_end - out_start) * (x - in_start) / (in_end - in_start);
	}
}

template<typename T>
const T& clamp(const T& v, const T& lo, const T& hi)
{
	assert(lo <= hi);

	if (v < lo)
	{
		return lo;
	}
	else if (v > hi)
	{
		return hi;
	}
	else
	{
		return v;
	}
}

template<typename T>
T lerp(const T& a, const T& b, const T& x)
{
	// FMA friendly
	return x * b + (a - a * x);
}

template<typename T>
T lerp_clamp(const T& a, const T& b, const T& x)
{
	if (x < 0.0)
	{
		return a;
	}
	else if (x > 1.0)
	{
		return b;
	}
	else
	{
		return lerp(a, b, x);
	}
}

template<typename T>
T smootherstep(const T& edge0, const T& edge1, const T& x)
{
	// Scale, and clamp x to 0..1 range
	const T t = remap_clamp(x, edge0, edge1, 0.0, 1.0);
	// Evaluate polynomial
	return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

template<typename T>
T WyvillGalinFunction(const T& distance, const T& R, const T& N)
{
	T alpha = 0.0;

	if (distance < R)
	{
		alpha = pow(1 - (distance / R) * (distance / R), N);
	}

	return alpha;
}

double cubic_interpolate(double p0, double p1, double p2, double p3, double t);

double cubic_interpolate(const std::array<double, 4>& p, double t);

double bi_cubic_interpolate(const std::array<std::array<double, 4>, 4>& p, double u, double v);

#endif // UTILS_H