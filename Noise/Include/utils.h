#ifndef UTILS_H
#define UTILS_H

#include <cassert>

template<typename T>
inline T Remap(const T& x, const T& in_start, const T& in_end, const T& out_start, const T& out_end)
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
inline const T& clamp(const T& v, const T& lo, const T& hi)
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
inline T lerp(const T& a, const T& b, const T& x)
{
	// Equivalent to (1 - x) * a + x * b;
	return fma(x, b, fma(-x, a, a));
}

template<typename T>
inline T lerp_clamp(const T& a, const T& b, const T& x)
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

#endif // UTILS_H