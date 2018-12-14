#ifndef EXAMPLES_H
#define EXAMPLES_H

#include <string>

void PerlinControlFunctionImage(int width, int height, const std::string& filename);

void LichtenbergControlFunctionImage(int width, int height, const std::string& filename);

void PerlinPlaneControlFunctionImage(int width, int height, const std::string& filename);

void SmallAmplificationImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void BigAmplificationImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void SmallTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserFirstDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserFirstTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserSecondDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserSecondTerrainImage(int width, int height, int seed, const std::string& filename);

void TeaserThirdDistanceImage(int width, int height, int seed, const std::string& filename);

void TeaserThirdTerrainImage(int width, int height, int seed, const std::string& filename);

void SketchSegmentsImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void SketchTerrainImage(int width, int height, int seed, const std::string& input, const std::string& filename);

void EvaluationTerrainImage(int width, int height, int seed, const std::string& filename);

void PerlinSegmentsImage(int width, int height, int seed, const std::string& filename);

void PerlinPlaneSegmentsImage(int width, int height, int seed, const std::string& filename);

void PerlinPlaneTerrainImage(int width, int height, int seed, const std::string& filename);

void LichtenbergFigureImage(int width, int height, int seed, const std::string& filename);

void EffectParametersImage(int width, int height, int seed, int resolution, double eps, double displacement, const std::string& filename);

/**
 * \brief Measure the time in ms taken to generate Lichtenberg figure.
 * #!/bin/bash
 * for i in {256..4..4}
 *   do
 *     echo "Num Threads $i"
 *     export OMP_NUM_THREADS=$i
 *     for j in {1..3}
 *       do
 *         ./Noise
 *       done
 *   done
 * \param width Resolution in the width axis
 * \param height Resolution in the height axis
 * \param filename File in which the result is saved, ideal case: check it with a checksum.
 * \return The time taken to generate the image in ms.
 */
double PerformanceTest(int width, int height, const std::string& filename);

#endif // EXAMPLES_H