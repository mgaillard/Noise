#ifndef PERLINCONTROLFUNCTION_H
#define PERLINCONTROLFUNCTION_H

#include "controlfunction.h"

#include "perlin.h"

class PerlinControlFunction : public ControlFunction<PerlinControlFunction>
{
	friend class ControlFunction<PerlinControlFunction>;

public:
	PerlinControlFunction(double scale = 1.0) : m_scale(scale) {}

protected:
	double EvaluateImpl(double x, double y) const
	{
		return m_scale * (Perlin(x, y) + 1.0) / 2.0;
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
		return m_scale;
	}

private:
	const double m_scale;
};

#endif // PERLINCONTROLFUNCTION_H