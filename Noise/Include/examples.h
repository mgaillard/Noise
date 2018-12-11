#ifndef EXAMPLES_H
#define EXAMPLES_H

#include <string>

void SmallAmplificationImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void BigAmplificationImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void SmallTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserFirstDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserFirstTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserSecondDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserSecondTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserThirdDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserThirdTerrainImage(int width, int height, int seed, const std::string& filename);

void EvaluationTerrainImage(int width, int height, int seed, const std::string& filename);

void LichtenbergFigureImage(int width, int height, int seed, const std::string& filename);

#endif // EXAMPLES_H