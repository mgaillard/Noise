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
	double evaluate(double x, double y) const
	{
		return static_cast<const Implementation*>(this)->evaluate_impl(x, y);
	}
};

#endif // CONTROLFUNCTION_H