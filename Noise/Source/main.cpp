#include <iostream>

#include "examples.h"

using namespace std;

int main(int argc, char* argv[])
{
	std::cout << "Amplification of a small terrain" << std::endl;
	const int SMALL_AMP_WIDTH = 512;
	const int SMALL_AMP_HEIGHT = 512;
	const int SMALL_AMP_SEED = 1;
	const string SMALL_AMP_INPUT = "../Images/amplification_small.png";
	const string SMALL_AMP_OUTPUT = "amplification_small_result.png";
	// SmallAmplificationImage(SMALL_AMP_WIDTH, SMALL_AMP_HEIGHT, SMALL_AMP_SEED, SMALL_AMP_INPUT, SMALL_AMP_OUTPUT);

	std::cout << "Amplification of a big terrain" << std::endl;
	const int BIG_AMP_WIDTH = 1024;
	const int BIG_AMP_HEIGHT = 1024;
	const int BIG_AMP_SEED = 0;
	const string BIG_AMP_INPUT = "../Images/amplification_big.png";
	const string BIG_AMP_OUTPUT = "amplification_big_result.png";
	// BigAmplificationImage(BIG_AMP_WIDTH, BIG_AMP_HEIGHT, BIG_AMP_SEED, BIG_AMP_INPUT, BIG_AMP_OUTPUT);

	std::cout << "Procedural generation of a small terrain" << std::endl;
	const int SMALL_TERRAIN_WIDTH = 512;
	const int SMALL_TERRAIN_HEIGHT = 512;
	const int SMALL_TERRAIN_SEED = 1;
	const string SMALL_TERRAIN_OUTPUT = "small_terrain.png";
	// SmallTerrainImage(SMALL_TERRAIN_WIDTH, SMALL_TERRAIN_HEIGHT, SMALL_TERRAIN_SEED, SMALL_TERRAIN_OUTPUT);

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

		// EvaluationTerrainImage(EVALUATION_TERRAIN_WIDTH, EVALUATION_TERRAIN_HEIGHT, s, filename);
	}
	
	std::cout << "Procedural generation of a Lichtenberg figure" << std::endl;
	const int LICHTENBERG_WIDTH = 2048;
	const int LICHTENBERG_HEIGHT = 2048;
	const int LICHTENBERG_SEED = 33058;
	const string LICHTENBERG_OUTPUT = "lichtenberg.png";
	// LichtenbergFigureImage(LICHTENBERG_WIDTH, LICHTENBERG_HEIGHT, LICHTENBERG_SEED, LICHTENBERG_OUTPUT);
	
	return 0;
}
