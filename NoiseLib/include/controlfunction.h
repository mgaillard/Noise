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

	/// <summary>
	/// Return the distance between the point and the inside of the domain
	/// </summary>
	/// <returns>The distance between the point and the inside of the domain</return>
	double distToDomain(double x, double y) const
	{
		return static_cast<const Implementation*>(this)->DistToDomainImpl(x, y);
	}

	/// <summary>
	/// Return the minimum value of the control function
	/// </summary>
	/// <returns>The minimum value of the control function</return>
	double minimum() const
	{
		return static_cast<const Implementation*>(this)->MinimumImpl();
	}

	/// <summary>
	/// Return the maximum value of the control function
	/// </summary>
	/// <returns>The maximum value of the control function</return>
	double maximum() const
	{
		return static_cast<const Implementation*>(this)->MaximumImpl();
	}
};

#endif // CONTROLFUNCTION_H