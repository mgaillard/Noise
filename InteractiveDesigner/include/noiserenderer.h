#ifndef NOISERENDERER_H
#define NOISERENDERER_H

#include <vector>

#include <QObject>
#include <QImage>
#include <QtConcurrent>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "noiseparameters.h"

class NoiseRenderer : public QObject
{
	Q_OBJECT

public:
	explicit NoiseRenderer(QObject *parent, const NoiseParameters& parameters);

	/**
	 * \brief Set the noise parameters
	 * \param parameters The noise parameters
	 */
	void setParameters(const NoiseParameters& parameters);

	/**
	 * \brief Return the rendered image as a QImage
	 * \return The rendered image
	 */
	QImage resultQImage() const;

	/**
	 * \brief Return the rendered image as a cv::Mat
	 * \return The rendered image
	 */
	cv::Mat resultCvMat() const;

	/**
	 * \brief Start the rendering of the image
	 * \return True if the rendering successfully started, false otherwise
	 */
	bool start();

signals:
	/**
	 * \brief Emitted when the computation is finished
	 */
	void finished();

private slots:
	/**
	 * \brief Called when rendering is finished
	 */
	void OnRenderingFinished();

private:

	/**
	 * \brief A 2D std::vector<double> to temporarily store a result
	 */
	struct VectorDouble2D
	{
		std::size_t height;
		std::size_t width;
		std::vector<double> data;

		VectorDouble2D() :
			height(0),
			width(0)
		{
		}

		VectorDouble2D(std::size_t h, std::size_t w) :
			height(h),
			width(w),
			data(h * w)
		{
		}

		const double& at(std::size_t i, std::size_t j) const
		{
			return data.at(i * width + j);
		}

		double& at(std::size_t i, std::size_t j)
		{
			return data.at(i * width + j);
		}
	};

	void ConfigureFutureWatcher();

	/**
	 * \brief Render the terrain noise in a QImage.
	 * \return An image of the noise.
	 */
	VectorDouble2D RenderTerrain() const;

	/**
	 * \brief Render the Lichtenberg noise in a QImage.
	 * \return An image of the noise.
	 */
	VectorDouble2D RenderLichtenberg() const;

	QFutureWatcher<VectorDouble2D>* m_futureImageWatcher;

	NoiseParameters m_parameters;

	VectorDouble2D m_result;
};

#endif // NOISERENDERER_H