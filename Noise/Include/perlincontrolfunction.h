#ifndef PERLINCONTROLFUNCTION_H
#define PERLINCONTROLFUNCTION_H

#include "controlfunction.h"

#include "perlin.h"

class PerlinControlFunction : public ControlFunction<PerlinControlFunction>
{
	friend class ControlFunction<PerlinControlFunction>;

protected:
	double EvaluateImpl(double x, double y) const
	{
		return (Perlin(x, y) + 1.0) / 2.0;
	}

	bool InsideDomainImpl(double x, double y) const
	{
		return true;
	}

	double DistToDomainImpl(double x, double y) const
	{
		return 0.0;
	}
};

#endif // PERLINCONTROLFUNCTION_H