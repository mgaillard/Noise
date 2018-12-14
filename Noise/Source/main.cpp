#include <iostream>
#include <iomanip>

#include "examples.h"

using namespace std;

int main(int argc, char* argv[])
{
	std::cout << "Performance Test" << std::endl;
	const int PERFORMANCE_WIDTH = 1024;
	const int PERFORMANCE_HEIGHT = 1024;
	const string PERFORMANCE_OUTPUT = "performance_test.png";
	std::cout << std::fixed << std::setprecision(2) << PerformanceTest(PERFORMANCE_WIDTH, PERFORMANCE_HEIGHT, PERFORMANCE_OUTPUT) << std::endl;

	const int CONTROL_FUNCTION_WIDTH = 512;
	const int CONTROL_FUNCTION_HEIGHT = 512;
	
	std::cout << "Perlin control function" << std::endl;
	const string PERLIN_CONTROL_OUTPUT = "perlin_function.png";
	PerlinControlFunctionImage(CONTROL_FUNCTION_WIDTH, CONTROL_FUNCTION_HEIGHT, PERLIN_CONTROL_OUTPUT);

	std::cout << "Lichtenberg control function" << std::endl;
	const string LICHTENBERG_CONTROL_OUTPUT = "lichtenberg_function.png";
	LichtenbergControlFunctionImage(CONTROL_FUNCTION_WIDTH, CONTROL_FUNCTION_HEIGHT, LICHTENBERG_CONTROL_OUTPUT);

	std::cout << "Perlin plane control function" << std::endl;
	const string PERLIN_PLANE_CONTROL_OUTPUT = "perlin_plane_function.png";
	PerlinPlaneControlFunctionImage(CONTROL_FUNCTION_WIDTH, CONTROL_FUNCTION_HEIGHT, PERLIN_PLANE_CONTROL_OUTPUT);
	
	std::cout << "Amplification of a small terrain" << std::endl;
	const int SMALL_AMP_WIDTH = 512;
	const int SMALL_AMP_HEIGHT = 512;
	const int SMALL_AMP_SEED = 1;
	const string SMALL_AMP_INPUT = "../Images/amplification_small.png";
	const string SMALL_AMP_OUTPUT = "amplification_small_result.png";
	SmallAmplificationImage(SMALL_AMP_WIDTH, SMALL_AMP_HEIGHT, SMALL_AMP_SEED, SMALL_AMP_INPUT, SMALL_AMP_OUTPUT);

	std::cout << "Amplification of a big terrain" << std::endl;
	const int BIG_AMP_WIDTH = 1024;
	const int BIG_AMP_HEIGHT = 1024;
	const int BIG_AMP_SEED = 0;
	const string BIG_AMP_INPUT = "../Images/amplification_big.png";
	const string BIG_AMP_OUTPUT = "amplification_big_result.png";
	BigAmplificationImage(BIG_AMP_WIDTH, BIG_AMP_HEIGHT, BIG_AMP_SEED, BIG_AMP_INPUT, BIG_AMP_OUTPUT);

	std::cout << "Procedural generation of a small terrain" << std::endl;
	const int SMALL_TERRAIN_WIDTH = 512;
	const int SMALL_TERRAIN_HEIGHT = 512;
	const int SMALL_TERRAIN_SEED = 1;
	const string SMALL_TERRAIN_OUTPUT = "small_terrain.png";
	SmallTerrainImage(SMALL_TERRAIN_WIDTH, SMALL_TERRAIN_HEIGHT, SMALL_TERRAIN_SEED, SMALL_TERRAIN_OUTPUT);

	std::cout << "Procedural generation of the teaser 1 terrain" << std::endl;
	const int TEASER_1_TERRAIN_WIDTH = 512;
	const int TEASER_1_TERRAIN_HEIGHT = 512;
	const int TEASER_1_TERRAIN_SEED = 0;
	const string TEASER_1_DISTANCE_OUTPUT = "teaser_1_distance.png";
	const string TEASER_1_TERRAIN_OUTPUT = "teaser_1_terrain.png";
	TeaserFirstDistanceImage(TEASER_1_TERRAIN_WIDTH, TEASER_1_TERRAIN_HEIGHT, TEASER_1_TERRAIN_SEED, TEASER_1_DISTANCE_OUTPUT);
	TeaserFirstTerrainImage(TEASER_1_TERRAIN_WIDTH, TEASER_1_TERRAIN_HEIGHT, TEASER_1_TERRAIN_SEED, TEASER_1_TERRAIN_OUTPUT);

	std::cout << "Procedural generation of the teaser 2 terrain" << std::endl;
	const int TEASER_2_TERRAIN_WIDTH = 768;
	const int TEASER_2_TERRAIN_HEIGHT = 768;
	const int TEASER_2_TERRAIN_SEED = 0;
	const string TEASER_2_DISTANCE_OUTPUT = "teaser_2_distance.png";
	const string TEASER_2_TERRAIN_OUTPUT = "teaser_2_terrain.png";
	TeaserSecondDistanceImage(TEASER_2_TERRAIN_WIDTH, TEASER_2_TERRAIN_HEIGHT, TEASER_2_TERRAIN_SEED, TEASER_2_DISTANCE_OUTPUT);
	TeaserSecondTerrainImage(TEASER_2_TERRAIN_WIDTH, TEASER_2_TERRAIN_HEIGHT, TEASER_2_TERRAIN_SEED, TEASER_2_TERRAIN_OUTPUT);

	std::cout << "Procedural generation of the teaser 3 terrain" << std::endl;
	const int TEASER_3_TERRAIN_WIDTH = 1024;
	const int TEASER_3_TERRAIN_HEIGHT = 1024;
	const int TEASER_3_TERRAIN_SEED = 0;
	const string TEASER_3_DISTANCE_OUTPUT = "teaser_3_distance.png";
	const string TEASER_3_TERRAIN_OUTPUT = "teaser_3_terrain.png";
	TeaserThirdDistanceImage(TEASER_3_TERRAIN_WIDTH, TEASER_3_TERRAIN_HEIGHT, TEASER_3_TERRAIN_SEED, TEASER_3_DISTANCE_OUTPUT);
	TeaserThirdTerrainImage(TEASER_3_TERRAIN_WIDTH, TEASER_3_TERRAIN_HEIGHT, TEASER_3_TERRAIN_SEED, TEASER_3_TERRAIN_OUTPUT);

	std::cout << "Procedural generation of a set of medium terrains for evaluation" << std::endl;
	const int EVALUATION_TERRAIN_WIDTH = 512;
	const int EVALUATION_TERRAIN_HEIGHT = 512;
	const int EVALUATION_TERRAIN_SEED_START = 0;
	const int EVALUATION_TERRAIN_SEED_END = 9;
	const string EVALUATION_TERRAIN_OUTPUT = "evaluation_terrain_";
	const string EVALUATION_TERRAIN_EXTENSION = ".png";
	for (int s = EVALUATION_TERRAIN_SEED_START; s <= EVALUATION_TERRAIN_SEED_END; s++)
	{
		string filename = EVALUATION_TERRAIN_OUTPUT;
		filename += std::to_string(s);
		filename += EVALUATION_TERRAIN_EXTENSION;

		std::cout << "Terrain: " << filename << std::endl;

		EvaluationTerrainImage(EVALUATION_TERRAIN_WIDTH, EVALUATION_TERRAIN_HEIGHT, s, filename);
	}

	// TODO: fine tune the slopePower and the amplitude of the control function
	std::cout << "Segments and terrain with the perlin plane control function" << std::endl;
	const int PERLIN_PLANE_WIDTH = 512;
	const int PERLIN_PLANE_HEIGHT = 512;
	const int PERLIN_PLANE_SEED = 0;
	const string PERLIN_PLANE_SEGMENTS_OUTPUT = "perlin_plane_segments.png";
	const string PERLIN_PLANE_OUTPUT = "perlin_plane_terrain.png";
	PerlinPlaneSegmentsImage(PERLIN_PLANE_WIDTH, PERLIN_PLANE_HEIGHT, PERLIN_PLANE_SEED, PERLIN_PLANE_SEGMENTS_OUTPUT);
	PerlinPlaneTerrainImage(PERLIN_PLANE_WIDTH, PERLIN_PLANE_HEIGHT, PERLIN_PLANE_SEED, PERLIN_PLANE_OUTPUT);

	std::cout << "Procedural generation of a Lichtenberg figure" << std::endl;
	const int LICHTENBERG_WIDTH = 2048;
	const int LICHTENBERG_HEIGHT = 2048;
	const int LICHTENBERG_SEED = 33058;
	const string LICHTENBERG_OUTPUT = "lichtenberg.png";
	LichtenbergFigureImage(LICHTENBERG_WIDTH, LICHTENBERG_HEIGHT, LICHTENBERG_SEED, LICHTENBERG_OUTPUT);

	std::cout << "Procedural generation of figures showing the effect of parameters" << std::endl;
	const int EFFECT_WIDTH = 512;
	const int EFFECT_HEIGHT = 512;
	const int EFFECT_DEFAULT_SEED = 5;
	const int EFFECT_DEFAULT_RESOLUTION = 3;
	const double EFFECT_DEFAULT_EPSILON = 0.25;
	// Special value of delta when varying epsilon
	const double EFFECT_EPSILON_DELTA = 0.0;
	const double EFFECT_DEFAULT_DELTA = 0.05;
	const string EFFECT_OUTPUT = "effect_";
	const string EFFECT_EXTENTION = ".png";
	// Vary the seed
	for (int seed = EFFECT_DEFAULT_SEED; seed < EFFECT_DEFAULT_SEED + 5; seed++)
	{
		const string filename = EFFECT_OUTPUT + "seed_" + std::to_string(seed) + EFFECT_EXTENTION;

		std::cout << "File: " << filename << std::endl;

		EffectParametersImage(EFFECT_WIDTH, EFFECT_HEIGHT, seed, EFFECT_DEFAULT_RESOLUTION, EFFECT_DEFAULT_EPSILON, EFFECT_DEFAULT_DELTA, filename);
	}
	// Vary the resolution
	for (int resolution = 1; resolution <= 5; resolution++)
	{
		const string filename = EFFECT_OUTPUT + "resolution_" + std::to_string(resolution) + EFFECT_EXTENTION;

		std::cout << "File: " << filename << std::endl;

		EffectParametersImage(EFFECT_WIDTH, EFFECT_HEIGHT, EFFECT_DEFAULT_SEED, resolution, EFFECT_DEFAULT_EPSILON, EFFECT_DEFAULT_DELTA, filename);
	}
	// Vary the epsilon
	for (int e = 0; e <= 4; e++)
	{
		const double eps = e * 0.125;
		const string filename = EFFECT_OUTPUT + "epsilon_" + std::to_string(e) + EFFECT_EXTENTION;

		std::cout << "File: " << filename << std::endl;

		EffectParametersImage(EFFECT_WIDTH, EFFECT_HEIGHT, EFFECT_DEFAULT_SEED, EFFECT_DEFAULT_RESOLUTION, eps, EFFECT_EPSILON_DELTA, filename);
	}
	// Vary the delta
	for (int d = 0; d <= 4; d++)
	{
		const double delta = d * 0.025;
		const string filename = EFFECT_OUTPUT + "delta_" + std::to_string(d) + EFFECT_EXTENTION;

		std::cout << "File: " << filename << std::endl;

		EffectParametersImage(EFFECT_WIDTH, EFFECT_HEIGHT, EFFECT_DEFAULT_SEED, EFFECT_DEFAULT_RESOLUTION, EFFECT_DEFAULT_EPSILON, delta, filename);
	}

	return 0;
}
