#ifndef CONTROLFUNCTION_H
#define CONTROLFUNCTION_H

/// <summary>
/// A function to control the shape of the noise.
/// Use the curiously recurring template pattern
/// </summary>
template<typename Implementation>
class ControlFunction
{
public:
	/// <summary>
	/// Evaluate the function in (x, y)
	/// </summary>
	double evaluate(double x, double y) const
	{
		return static_cast<const Implementation*>(this)->EvaluateImpl(x, y);
	}

	/// <summary>
	/// Check whether a point is inside the domain of the function
	/// </summary>
	/// <returns>True if the point is inside the domain</return>
	bool insideDomain(double x, double y) const
	{
		return static_cast<const Implementation*>(this)->InsideDomainImpl(x, y);
	}
};

#endif // CONTROLFUNCTION_H