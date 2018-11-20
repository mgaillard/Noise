#ifndef NOISEPARAMETERS_H
#define NOISEPARAMETERS_H

enum class NoiseType
{
	terrain = 0,
	lichtenberg = 1
};

struct NoiseParameters
{
	NoiseType type;
	int seed;
	int widthResolution;
	int heightResolution;
	int levels;
	double epsilon;
	double displacement;
	double noiseTop;
	double noiseBottom;
	double noiseLeft;
	double noiseRight;
	double controlFunctionTop;
	double controlFunctionBottom;
	double controlFunctionLeft;
	double controlFunctionRight;
};

#endif // NOISEPARAMETERS_H