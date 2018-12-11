#include "examples.h"

#include <iostream>
#include <iomanip>
#include <memory>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "noise.h"
#include "math2d.h"
#include "utils.h"
#include "perlincontrolfunction.h"
#include "planecontrolfunction.h"
#include "lichtenbergcontrolfunction.h"
#include "imagecontrolfunction.h"

using namespace std;

struct Progress
{
	const int totalSteps;
	const int moduloSteps;
	int completedSteps;

	/// <summary>
	/// Construct a Progress object to monitor the progress in an OpenMP loop.
	/// </summary>
	/// <param name="totalSteps">The total number of steps in the loop</param>
	/// <param name="numberDisplay">The number of times the progress is going to be displayed</param>
	Progress(int totalSteps, int numberDisplay = 100) :
		totalSteps(totalSteps),
		moduloSteps(totalSteps / numberDisplay),
		completedSteps(0)
	{
		assert(numberDisplay > 0);
	}

	/// <summary>
	/// Update the progress.
	/// This function should be called each time a step is completed.
	/// </summary>
	void Update()
	{
#pragma omp atomic
		++completedSteps;
	}

	/// <summary>
	/// Display the progress.
	/// This function display the progress every "moduloSteps" steps have been completed.
	/// </summary>
	void Display() const
	{
		// stepsCompleted may have changed, however it is not a big problem if the progress is not very precise.
		if ((completedSteps % moduloSteps) == 0)
		{
#pragma omp critical
			cout << "Progress: " << (100LLU * completedSteps / totalSteps) << " %\n";
		}
	}
};

template<typename I>
cv::Mat SegmentImage(const Noise<I>& noise, const Point2D& a, const Point2D&b, int width, int height)
{
	const Point3D point(5.69, -1.34, 4.0);
	const std::array<Segment3D, 2> segments = { {
		Segment3D(Point3D(1.0, 1.0, 2.0), Point3D(2.0, 3.0, 1.0)),
		Segment3D(Point3D(2.0, 3.0, 1.0), Point3D(2.0, 5.0, 0.0))
	} };

	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = remap(double(j), 0.0, double(width), a.x, b.x);
			const double y = remap(double(i), 0.0, double(height), a.y, b.y);

			const double value = noise.displaySegment(x, y, segments, point);

			image.at<uint16_t>(i, j) = uint16_t(value * numeric_limits<uint16_t>::max());
		}
	}

	return image;
}

template<typename I>
vector<vector<double> > EvaluateTerrain(const Noise<I>& noise, const Point2D& a, const Point2D&b, int width, int height)
{
	vector<vector<double> > values(height, vector<double>(width));

	// Display progress 25 times.
	Progress progress(width * height, 25);

#pragma omp parallel for shared(values)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = remap_clamp(double(j), 0.0, double(width), a.x, b.x);
			const double y = remap_clamp(double(i), 0.0, double(height), a.y, b.y);

			values[i][j] = noise.evaluateTerrain(x, y);

			progress.Update();
			progress.Display();
		}
	}

	return values;
}

template<typename I>
vector<vector<double> > EvaluateLichtenbergFigure(const Noise<I>& noise, const Point2D& a, const Point2D& b, int width, int height)
{
	vector<vector<double> > values(height, vector<double>(width));

	// Display progress 25 times.
	Progress progress(width * height, 25);

#pragma omp parallel for shared(values)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = remap_clamp(double(j), 0.0, double(width), a.x, b.x);
			const double y = remap_clamp(double(i), 0.0, double(height), a.y, b.y);

			values[i][j] = noise.evaluateLichtenberg(x, y);

			progress.Update();
			progress.Display();
		}
	}

	return values;
}

cv::Mat GenerateImage(const vector<vector<double> > &values)
{
	const int width = int(values.size());
	const int height = int(values.front().size());

	// Find min and max to remap to 16 bits
	double minimum = numeric_limits<double>::max();
	double maximum = numeric_limits<double>::lowest();
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			minimum = min(minimum, values[i][j]);
			maximum = max(maximum, values[i][j]);
		}
	}

	// Convert to 16 bits image
	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double value = remap_clamp(values[i][j], minimum, maximum, 0.0, 65535.0);

			image.at<uint16_t>(i, j) = uint16_t(value);
		}
	}

	return image;
}

cv::Mat GenerateImageNegative(const vector<vector<double> > &values)
{
	cv::Mat image = GenerateImage(values);
	cv::bitwise_not(image, image);
	return image;
}

cv::Mat GenerateImageMatlab(const vector<vector<double> > &values)
{
	const int width = int(values.size());
	const int height = int(values.front().size());

	// Find min and max to remap to 16 bits
	double minimum = numeric_limits<double>::max();
	double maximum = numeric_limits<double>::lowest();
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			minimum = min(minimum, values[i][j]);
			maximum = max(maximum, values[i][j]);
		}
	}

	// Convert to 16 bits image
	cv::Mat image(height, width, CV_8UC3);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const std::array<double, 3> color = matlab_jet(1.0 - remap_clamp(values[i][j], minimum, maximum, 0.0, 1.0));

			auto& pixel = image.at<cv::Vec3b>(i, j);

			// Blue
			pixel.val[0] = uint8_t(color[2] * std::numeric_limits<uint8_t>::max());
			// Green
			pixel.val[1] = uint8_t(color[1] * std::numeric_limits<uint8_t>::max());
			// Red
			pixel.val[2] = uint8_t(color[0] * std::numeric_limits<uint8_t>::max());
		}
	}

	return image;
}

void SmallAmplificationImage(int width, int height, int seed, const string& input, const string& filename)
{
	const auto inputImage = cv::imread(input, cv::ImreadModes::IMREAD_ANYDEPTH);

	typedef ImageControlFunction ControlFunctionType;
	std::unique_ptr<ControlFunctionType> controlFunction(std::make_unique<ControlFunctionType>(inputImage));

	const double eps = 0.10;
	const int resolution = 1;
	const double displacement = 0.05;
	const int primitivesResolutionSteps = 2;
	const double slopePower = 1.0;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(12.0, 12.0);
	const Point2D controlFunctionTopLeft(0.0, 0.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::minstd_rand
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void BigAmplificationImage(int width, int height, int seed, const string& input, const string& filename)
{
	const auto inputImage = cv::imread(input, cv::ImreadModes::IMREAD_ANYDEPTH);

	typedef ImageControlFunction ControlFunctionType;
	std::unique_ptr<ControlFunctionType> controlFunction(std::make_unique<ControlFunctionType>(inputImage));

	const double eps = 0.10;
	const int resolution = 1;
	const double displacement = 0.05;
	const int primitivesResolutionSteps = 2;
	const double slopePower = 1.0;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(48.0, 48.0);
	const Point2D controlFunctionTopLeft(0.0, 0.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::minstd_rand
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void SmallTerrainImage(int width, int height, int seed, const string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 2;
	const double displacement = 0.08;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 1.5;
	const double noiseAmplitudeProportion = 0.025;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(0.0, 0.0);
	const Point2D controlFunctionBottomRight(0.5, 0.5);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserFirstDistanceImage(int width, int height, int seed, const std::string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 1;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.5;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, false, false, false, false, true);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImageMatlab(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserFirstTerrainImage(int width, int height, int seed, const std::string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 1;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.5;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserSecondDistanceImage(int width, int height, int seed, const std::string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 2;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.5;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, false, false, false, false, true);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImageMatlab(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserSecondTerrainImage(int width, int height, int seed, const string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 2;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.5;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserThirdDistanceImage(int width, int height, int seed, const std::string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>(0.250));

	const double eps = 0.25;
	const int resolution = 3;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.1;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, false, false, false, false, true);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImageMatlab(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void TeaserThirdTerrainImage(int width, int height, int seed, const string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>(0.250));

	const double eps = 0.25;
	const int resolution = 3;
	const double displacement = 0.075;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.1;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.5);
	const Point2D controlFunctionBottomRight(1.40, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void EvaluationTerrainImage(int width, int height, int seed, const string& filename)
{
	typedef PerlinControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.25;
	const int resolution = 2;
	const double displacement = 0.1;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 0.5;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(4.0, 4.0);
	const Point2D controlFunctionTopLeft(-0.2, -0.4);
	const Point2D controlFunctionBottomRight(1.4, 0.7);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, false, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateTerrain(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}

void LichtenbergFigureImage(int width, int height, int seed, const string& filename)
{
	const int antiAliasingLevel = 4;

	typedef LichtenbergControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const double eps = 0.1;
	const int resolution = 6;
	const double displacement = 0.05;
	const int primitivesResolutionSteps = 3;
	const double slopePower = 1.0;
	const double noiseAmplitudeProportion = 0.05;
	const Point2D noiseTopLeft(-2.0, -2.0);
	const Point2D noiseBottomRight(1.0, 1.0);
	const Point2D controlFunctionTopLeft(-1.0, -1.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, true, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImage(EvaluateLichtenbergFigure(noise, noiseTopLeft, noiseBottomRight, width, height));

	// Resize image (anti aliasing)
	cv::Mat resized_image(height / antiAliasingLevel, width / antiAliasingLevel, CV_16U);
	resize(image, resized_image, resized_image.size(), 0, 0, CV_INTER_AREA);

	cv::imwrite(filename, resized_image);
}

void EffectParametersImage(int width, int height, int seed, int resolution, double eps, double displacement, const std::string& filename)
{
	typedef LichtenbergControlFunction ControlFunctionType;
	unique_ptr<ControlFunctionType> controlFunction(make_unique<ControlFunctionType>());

	const int primitivesResolutionSteps = 3;
	const double slopePower = 1.0;
	const double noiseAmplitudeProportion = 0.0;
	const Point2D noiseTopLeft(-2.0, -2.0);
	const Point2D noiseBottomRight(1.0, 1.0);
	const Point2D controlFunctionTopLeft(-1.0, -1.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	const Noise<ControlFunctionType> noise(move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, resolution, displacement, primitivesResolutionSteps, slopePower, noiseAmplitudeProportion, true, false, true, false, false);
	// TODO: Random generator std::mt19937_64
	const cv::Mat image = GenerateImageNegative(EvaluateLichtenbergFigure(noise, noiseTopLeft, noiseBottomRight, width, height));

	cv::imwrite(filename, image);
}
