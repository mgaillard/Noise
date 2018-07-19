#ifndef PLANECONTROLFUNCTION_H
#define PLANECONTROLFUNCTION_H

#include "controlfunction.h"

class PlaneControlFunction : public ControlFunction<PlaneControlFunction>
{
	friend class ControlFunction<PlaneControlFunction>;

protected:
	double evaluate_impl(double x, double y) const
	{
		return x / 8.0 + (Perlin(x, y) + 1.0) / 8.0;
	}
};

#endif // PLANECONTROLFUNCTION_H