#include <iostream>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "worley.h"
#include "perlin.h"
#include "noise.h"
#include "math2d.h"
#include "utils.h"
#include "perlincontrolfunction.h"
#include "planecontrolfunction.h"

using namespace std;

cv::Mat PerlinImage(const Point2D& a, const Point2D&b, int width, int height)
{
	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = Remap(double(j), 0.0, double(width), a.x, b.x);
			const double y = Remap(double(i), 0.0, double(height), a.y, b.y);

			double value = (1.0 + Perlin(x, y)) / 2.0;

			image.at<uint16_t>(i, j) = uint16_t(value * numeric_limits<uint16_t>::max());
		}
	}

	return image;
}

template<typename I>
cv::Mat NoiseImage(const Noise<I>& noise, const Point2D& a, const Point2D&b, int width, int height)
{
	vector<vector<double> > temp(height, vector<double>(width));

#pragma omp parallel for shared(temp)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = Remap(double(j), 0.0, double(width), a.x, b.x);
			const double y = Remap(double(i), 0.0, double(height), a.y, b.y);

			temp[i][j] = noise.evaluate(x, y);
		}
	}

	// Find min and max to remap to 16 bits
	double minimum = numeric_limits<double>::max();
	double maximum = numeric_limits<double>::lowest();
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			minimum = min(minimum, temp[i][j]);
			maximum = max(maximum, temp[i][j]);
		}
	}

	// Convert to 16 bits image
	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double value = Remap(temp[i][j], minimum, maximum, 0.0, 65535.0);

			image.at<uint16_t>(i, j) = uint16_t(value);
		}
	}

	return image;
}

void RemapImage(cv::Mat& image)
{
	double minimum, maximum;
	cv::minMaxLoc(image, &minimum, &maximum);

#pragma omp parallel for shared(image)
	for (int i = 0; i < image.rows; i++) {
		for (int j = 0; j < image.cols; j++) {
			const double value = Remap(double(image.at<uint16_t>(i, j)), minimum, maximum, 0.0, 65535.0);

			image.at<uint16_t>(i, j) = uint16_t(value);
		}
	}
}

cv::Mat MergePerlinAndNoise(const cv::Mat& imagePerlin, const cv::Mat& imageNoise)
{
	assert(imagePerlin.rows == imageNoise.rows);
	assert(imagePerlin.cols == imageNoise.cols);

	cv::Mat image(imageNoise.rows, imageNoise.cols, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < imageNoise.rows; i++) {
		for (int j = 0; j < imageNoise.cols; j++) {
			image.at<uint16_t>(i, j) = imagePerlin.at<uint16_t>(i, j) / 2 + imageNoise.at<uint16_t>(i, j) / 2;
		}
	}

	return image;
}

int main(int argc, char* argv[])
{
	const int WIDTH = 512;
	const int HEIGHT = 512;
	const string FILENAME = "output.png";

	PerlinControlFunction perlinControlFunction;
	
	const int seed = 0;
	const double eps = 0.0;
	const Point2D noiseTopLeft(0.0, 0.0);
	const Point2D noiseBottomRight(8.0, 8.0);
	const Point2D controlFunctionTopLeft(0.0, 0.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	Noise<decltype(perlinControlFunction)> noise(&perlinControlFunction, noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, seed, eps, false, false, false);

	// cv::Mat imagePerlin = PerlinImage(perlinTopLeft, perlinBottomRight, WIDTH, HEIGHT);
	cv::Mat imageNoise = NoiseImage(noise, noiseTopLeft, noiseBottomRight, WIDTH, HEIGHT);

	// cv::Mat image = MergePerlinAndNoise(imagePerlin, imageNoise);

	cv::imwrite(FILENAME, imageNoise);

	return 0;
}
