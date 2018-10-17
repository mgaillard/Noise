#ifndef LICHTENBERGCONTROLFUNCTION_H
#define LICHTENBERGCONTROLFUNCTION_H

#include "controlfunction.h"

class LichtenbergControlFunction : public ControlFunction<LichtenbergControlFunction>
{
	friend class ControlFunction<LichtenbergControlFunction>;

protected:
	double EvaluateImpl(double x, double y) const
	{
		return x * x + y * y;
	}

	bool InsideDomainImpl(double x, double y) const
	{
		return std::max(abs(x), abs(y)) <= 1.0;
	}
};

#endif // LICHTENBERGCONTROLFUNCTION_H
