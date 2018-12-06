#ifndef LICHTENBERGCONTROLFUNCTION_H
#define LICHTENBERGCONTROLFUNCTION_H

#include "controlfunction.h"
#include "math2d.h"

class LichtenbergControlFunction : public ControlFunction<LichtenbergControlFunction>
{
	friend class ControlFunction<LichtenbergControlFunction>;

protected:
	double EvaluateImpl(double x, double y) const
	{
		if (InsideDomainImpl(x, y))
		{
			return x * x + (y + 1.0) * (y + 1.0);
		}

		// An arbitrary big value (more than 5.0)
		return 16.0;
	}

	bool InsideDomainImpl(double x, double y) const
	{
		return x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0;
	}

	double DistToDomainImpl(double x, double y) const
	{
		if (InsideDomainImpl(x, y))
		{
			return 0.0;
		}

		const Point2D p(x, y);

		const Point2D topLeft(-1.0, -1.0);
		const Point2D topRight(1.0, -1.0);
		const Point2D bottomLeft(-1.0, 1.0);
		const Point2D bottomRight(1.0, 1.0);

		Point2D c; // Useless point for distToLineSegment

		auto dist = std::numeric_limits<double>::max();

		dist = std::min(dist, distToLineSegment(p, topLeft, topRight, c));
		dist = std::min(dist, distToLineSegment(p, topRight, bottomRight, c));
		dist = std::min(dist, distToLineSegment(p, bottomRight, bottomLeft, c));
		dist = std::min(dist, distToLineSegment(p, bottomLeft, topLeft, c));

		return dist;
	}

	double MinimumImpl() const
	{
		return 0.0;
	}

	double MaximumImpl() const
	{
		return 16.0;
	}
};

#endif // LICHTENBERGCONTROLFUNCTION_H
