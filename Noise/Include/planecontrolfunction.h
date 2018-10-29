#ifndef PLANECONTROLFUNCTION_H
#define PLANECONTROLFUNCTION_H

#include "controlfunction.h"

class PlaneControlFunction : public ControlFunction<PlaneControlFunction>
{
	friend class ControlFunction<PlaneControlFunction>;

protected:
	double EvaluateImpl(double x, double y) const
	{
		return x / 8.0 + (Perlin(x, y) + 1.0) / 8.0;
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

#endif // PLANECONTROLFUNCTION_H