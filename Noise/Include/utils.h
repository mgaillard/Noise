#ifndef UTILS_H
#define UTILS_H

template<typename T>
inline T Remap(T x, T in_start, T in_end, T out_start, T out_end)
{
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
inline T lerp(T a, T b, T x)
{
	return (1 - x) * a + x * b;
}

template<typename T>
inline T lerp_clamp(T a, T b, T x)
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