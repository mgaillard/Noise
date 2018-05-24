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

using namespace std;

cv::Mat PerlinImage(const Point& a, const Point&b, int width, int height)
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

cv::Mat NoiseImage(const Noise& noise, const Point& a, const Point&b, int width, int height)
{
	cv::Mat image(height, width, CV_16U);	

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = Remap(double(j), 0.0, double(width), a.x, b.x);
			const double y = Remap(double(i), 0.0, double(height), a.y, b.y);

			image.at<uint16_t>(i, j) = uint16_t(noise.evaluate(x, y) * numeric_limits<uint16_t>::max());
		}
	}

	return image;
}

cv::Mat MergePerlinAndNoise(const cv::Mat& imagePerlin, const cv::Mat& imageNoise)
{
	assert(imagePerlin.rows == imageNoise.rows);
	assert(imagePerlin.cols == imageNoise.cols);

	cv::Mat image(imageNoise.rows, imagePerlin.cols, CV_16U);

	for (int i = 0; i < imageNoise.rows; i++) {
		for (int j = 0; j < imageNoise.cols; j++) {
			uint16_t value = imagePerlin.at<uint16_t>(i, j);

			if (imageNoise.at<uint16_t>(i, j) == numeric_limits<uint16_t>::max()) {
				value = imageNoise.at<uint16_t>(i, j);
			}

			image.at<uint16_t>(i, j) = value;
		}
	}

	return image;
}

int main(int argc, char* argv[])
{
	const int WIDTH = 512;
	const int HEIGHT = 512;
	const string FILENAME = "output.png";
	
	const int seed = 0;
	const Point noiseTopLeft(0.0, 0.0);
	const Point noiseBottomRight(8.0, 8.0);
	const Point perlinTopLeft(0.0, 0.0);
	const Point perlinBottomRight(1.0, 1.0);

	Noise noise(noiseTopLeft, noiseBottomRight, perlinTopLeft, perlinBottomRight, seed);

	cv::Mat imagePerlin;
	cv::Mat imageNoise;
	cv::Mat image;

	imagePerlin = PerlinImage(perlinTopLeft, perlinBottomRight, WIDTH, HEIGHT);
	imageNoise = NoiseImage(noise, noiseTopLeft, noiseBottomRight, WIDTH, HEIGHT);

	image = MergePerlinAndNoise(imagePerlin, imageNoise);

	cv::imwrite(FILENAME, image);

	return 0;
}
