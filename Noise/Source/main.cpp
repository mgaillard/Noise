#include <iostream>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "worley.h"
#include "perlin.h"
#include "noise.h"
#include "math2d.h"

using namespace std;

cv::Mat PerlinImage(const Point& a, const Point&b, int width, int height)
{
	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = (double(j) / width) * (b.x - a.x) + a.x;
			const double y = (double(i) / height) * (b.y - a.y) + a.y;

			double value = (1.0 + Perlin(x, y)) / 2.0;

			image.at<uint16_t>(i, j) = uint16_t(value * numeric_limits<uint16_t>::max());
		}
	}

	return image;
}

void DisplayNoiseControlPoints(int startX, int endX, int startY, int endY)
{
	for (int x = startX; x <= endX; x++)
	{
		for (int y = startY; y <= endY; y++)
		{
			Point p = GeneratePoint(x, y);
			double elevation = Perlin(p.x, p.y);
			cout << x << '\t' << y << '\t' << p.x << '\t' << -p.y << '\t' << elevation << endl;
		}
	}
}

cv::Mat NoiseImage(const Point& a, const Point&b, int width, int height)
{
	cv::Mat image(height, width, CV_16U);

#pragma omp parallel for shared(image)
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			const double x = (double(j) / width) * (b.x - a.x) + a.x;
			const double y = (double(i) / height) * (b.y - a.y) + a.y;

			image.at<uint16_t>(i, j) = uint16_t(Noise(x, y) * numeric_limits<uint16_t>::max());
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
	const int WIDTH = 1024;
	const int HEIGHT = 1024;
	const string FILENAME = "output.png";
	
	cv::Mat imagePerlin;
	cv::Mat imageNoise;
	cv::Mat image;

	imagePerlin = PerlinImage(Point(0.0, 0.0), Point(1.0, 1.0), WIDTH, HEIGHT);
	imageNoise = NoiseImage(Point(0.0, 0.0), Point(16.0, 16.0), WIDTH, HEIGHT);

	image = MergePerlinAndNoise(imagePerlin, imageNoise);

	cv::imwrite(FILENAME, image);

	return 0;
}
