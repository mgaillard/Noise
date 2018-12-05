#ifndef IMAGECONTROLFUNCTION_H
#define IMAGECONTROLFUNCTION_H

#include <utility>
#include <cassert>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "controlfunction.h"
#include "math2d.h"
#include "utils.h"

class ImageControlFunction : public ControlFunction<ImageControlFunction>
{
	friend class ControlFunction<ImageControlFunction>;

public:
	explicit ImageControlFunction(cv::Mat image) : m_image(std::move(image))
	{
		assert(image.data != nullptr);
		assert(image.type() == CV_8U || image.type() == CV_16U);
		assert(image.rows > 1);
		assert(image.cols > 1);
	}

protected:
	double EvaluateImpl(double x, double y) const
	{
		x = clamp(x, 0.0, 1.0);
		y = clamp(y, 0.0, 1.0);

		return sample(y, x);
	}

	bool InsideDomainImpl(double x, double y) const
	{
		return x >= 0.0 && x <= 1.0 && y >= 0.0 && y <= 1.0;
	}

	double DistToDomainImpl(double x, double y) const
	{
		if (InsideDomainImpl(x, y))
		{
			return 0.0;
		}

		const Point2D p(x, y);

		const Point2D topLeft(0.0, 0.0);
		const Point2D topRight(1.0, 0.0);
		const Point2D bottomLeft(0.0, 1.0);
		const Point2D bottomRight(1.0, 1.0);

		Point2D c; // Useless point for distToLineSegment

		auto dist = std::numeric_limits<double>::max();

		dist = std::min(dist, distToLineSegment(p, topLeft, topRight, c));
		dist = std::min(dist, distToLineSegment(p, topRight, bottomRight, c));
		dist = std::min(dist, distToLineSegment(p, bottomRight, bottomLeft, c));
		dist = std::min(dist, distToLineSegment(p, bottomLeft, topLeft, c));

		return dist;
	}

private:
	double get(int i, int j) const
	{
		double value = 0.0;

		switch (m_image.type())
		{
		case CV_8U:
			// Remap the value between min_value and min_value
			value = double(m_image.at<uint8_t>(i, j)) / std::numeric_limits<uint8_t>::max();
			break;
		case CV_16U:
			// Remap the value between min_value and min_value
			value = double(m_image.at<uint16_t>(i, j)) / std::numeric_limits<uint16_t>::max();
			break;
		}

		return value;
	}

	double sample(double ri, double rj) const;

	const cv::Mat m_image;
};

#endif // IMAGECONTROLFUNCTION_H
