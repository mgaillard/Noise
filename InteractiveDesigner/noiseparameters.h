#ifndef NOISEPARAMETERS_H
#define NOISEPARAMETERS_H

struct NoiseParameters
{
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