#include "noiserenderer.h"

#include "lichtenbergcontrolfunction.h"
#include "perlincontrolfunction.h"
#include "imagecontrolfunction.h"
#include "noise.h"

NoiseRenderer::NoiseRenderer(QObject *parent, const NoiseParameters& parameters)
	: QObject(parent),
	m_futureImageWatcher(new QFutureWatcher<VectorDouble2D>(this)),
	m_parameters(parameters)
{
	ConfigureFutureWatcher();
}

void NoiseRenderer::setParameters(const NoiseParameters& parameters)
{
	m_parameters = parameters;
}

QImage NoiseRenderer::resultQImage() const
{
	QImage image(m_result.width, m_result.height, QImage::Format::Format_Grayscale8);

	// Find min and max to remap to 16 bits
	double minimum = std::numeric_limits<double>::max();
	double maximum = std::numeric_limits<double>::lowest();
	for (auto value : m_result.data) {
		minimum = std::min(minimum, value);
		maximum = std::max(maximum, value);
	}

	for (std::size_t i = 0; i < m_result.height; i++) {
		for (std::size_t j = 0; j < m_result.width; j++) {
			const auto grayValue = remap_clamp(m_result.at(i, j), minimum, maximum, 0.0, double(std::numeric_limits<uint8_t>::max()));
			image.setPixel(j, i, qRgb(grayValue, grayValue, grayValue));
		}
	}

	return image;
}

cv::Mat NoiseRenderer::resultCvMat() const
{
	cv::Mat image(m_result.height, m_result.width, CV_16U);

	// Find min and max to remap to 16 bits
	double minimum = std::numeric_limits<double>::max();
	double maximum = std::numeric_limits<double>::lowest();
	for (auto value : m_result.data) {
		minimum = std::min(minimum, value);
		maximum = std::max(maximum, value);
	}

	for (std::size_t i = 0; i < m_result.height; i++) {
		for (std::size_t j = 0; j < m_result.width; j++) {
			const auto grayValue = remap_clamp(m_result.at(i, j), minimum, maximum, 0.0, double(std::numeric_limits<uint16_t>::max()));
			image.at<uint16_t>(i, j) = static_cast<uint16_t>(grayValue);
		}
	}

	return image;
}

bool NoiseRenderer::start()
{
	// Check that the renderer is not currently running before starting a new computation
	if (!m_futureImageWatcher->isRunning())
	{
		QFuture<VectorDouble2D> futureImage;

		switch (m_parameters.type)
		{
		case NoiseType::terrain:
			futureImage = QtConcurrent::run(this, &NoiseRenderer::RenderTerrain);
			break;

		case NoiseType::lichtenberg:
			futureImage = QtConcurrent::run(this, &NoiseRenderer::RenderLichtenberg);
			break;
		};

		m_futureImageWatcher->setFuture(futureImage);

		return true;
	}

	return false;
}

void NoiseRenderer::OnRenderingFinished()
{
	m_result = m_futureImageWatcher->future().result();
	emit finished();
}

void NoiseRenderer::ConfigureFutureWatcher()
{
	connect(m_futureImageWatcher, &QFutureWatcher<VectorDouble2D>::finished, this, &NoiseRenderer::OnRenderingFinished);
}

NoiseRenderer::VectorDouble2D NoiseRenderer::RenderTerrain() const
{
	typedef PerlinControlFunction ControlFunctionType;
	std::unique_ptr<ControlFunctionType> controlFunction(std::make_unique<ControlFunctionType>());

	const Point2D noiseTopLeft(m_parameters.noiseLeft, m_parameters.noiseTop);
	const Point2D noiseBottomRight(m_parameters.noiseRight, m_parameters.noiseBottom);
	const Point2D controlFunctionTopLeft(m_parameters.controlFunctionLeft, m_parameters.controlFunctionTop);
	const Point2D controlFunctionBottomRight(m_parameters.controlFunctionRight, m_parameters.controlFunctionBottom);

	const Noise<ControlFunctionType> noise(std::move(controlFunction),
		noiseTopLeft,
		noiseBottomRight,
		controlFunctionTopLeft,
		controlFunctionBottomRight,
		m_parameters.seed,
		m_parameters.epsilon,
		m_parameters.levels,
		m_parameters.displacement,
		m_parameters.primitivesResolutionSteps,
		m_parameters.slopePower,
		false,
		false,
		false);

	VectorDouble2D result(m_parameters.heightResolution, m_parameters.widthResolution);

#pragma omp parallel for
	for (int i = 0; i < m_parameters.heightResolution; i++) {
		for (int j = 0; j < m_parameters.widthResolution; j++) {
			const double x = remap_clamp(double(j), 0.0, double(m_parameters.widthResolution - 1), noiseTopLeft.x, noiseBottomRight.x);
			const double y = remap_clamp(double(i), 0.0, double(m_parameters.heightResolution - 1), noiseTopLeft.y, noiseBottomRight.y);

			result.at(i, j) = noise.evaluateTerrain(x, y);
		}
	}

	return result;
}

NoiseRenderer::VectorDouble2D NoiseRenderer::RenderLichtenberg() const
{
	typedef LichtenbergControlFunction ControlFunctionType;
	std::unique_ptr<ControlFunctionType> controlFunction(std::make_unique<ControlFunctionType>());

	const Point2D noiseTopLeft(m_parameters.noiseLeft, m_parameters.noiseTop);
	const Point2D noiseBottomRight(m_parameters.noiseRight, m_parameters.noiseBottom);
	const Point2D controlFunctionTopLeft(m_parameters.controlFunctionLeft, m_parameters.controlFunctionTop);
	const Point2D controlFunctionBottomRight(m_parameters.controlFunctionRight, m_parameters.controlFunctionBottom);

	const Noise<ControlFunctionType> noise(std::move(controlFunction),
										   noiseTopLeft,
										   noiseBottomRight,
										   controlFunctionTopLeft,
										   controlFunctionBottomRight,
										   m_parameters.seed,
										   m_parameters.epsilon,
										   m_parameters.levels,
										   m_parameters.displacement,
										   m_parameters.primitivesResolutionSteps,
										   m_parameters.slopePower,
										   false,
										   true,
										   false);

	VectorDouble2D result(m_parameters.heightResolution, m_parameters.widthResolution);

#pragma omp parallel for
	for (int i = 0; i < m_parameters.heightResolution; i++) {
		for (int j = 0; j < m_parameters.widthResolution; j++) {
			const double x = remap_clamp(double(j), 0.0, double(m_parameters.widthResolution - 1), noiseTopLeft.x, noiseBottomRight.x);
			const double y = remap_clamp(double(i), 0.0, double(m_parameters.heightResolution - 1), noiseTopLeft.y, noiseBottomRight.y);

			result.at(i, j) = noise.evaluateLichtenberg(x, y);
		}
	}

	return result;
}
