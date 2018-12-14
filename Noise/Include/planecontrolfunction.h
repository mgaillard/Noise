#ifndef PLANECONTROLFUNCTION_H
#define PLANECONTROLFUNCTION_H

#include "controlfunction.h"

class PlaneControlFunction : public ControlFunction<PlaneControlFunction>
{
	friend class ControlFunction<PlaneControlFunction>;

protected:
	double EvaluateImpl(double x, double y) const
	{
		return x / 8.0 + (Perlin(4.0 * x, 4.0 * y) + 1.0) / 8.0;
	}

	bool InsideDomainImpl(double x, double y) const
	{
		return true;
	}

	double DistToDomainImpl(double x, double y) const
	{
		return 0.0;
	}

	double MinimumImpl() const
	{
		return 0.0;
	}

	double MaximumImpl() const
	{
		// The maximum depends on where the function is evaluated
		// When x is big, evaluate(x, y) is big as well
		return 1.0;
	}
};

#endif // PLANECONTROLFUNCTION_H