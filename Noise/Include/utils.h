#ifndef UTILS_H
#define UTILS_H

template<typename T>
inline double Remap(T x, T in_start, T in_end, T out_start, T out_end)
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

#endif // UTILS_H