#include "noiserenderer.h"

#include "lichtenbergcontrolfunction.h"
#include "noise.h"

NoiseRenderer::NoiseRenderer(QObject *parent, const NoiseParameters& parameters)
	: QObject(parent),
	m_futureImageWatcher(new QFutureWatcher<QImage>(this)),
	m_parameters(parameters)
{
	ConfigureFutureWatcher();
}

void NoiseRenderer::setParameters(const NoiseParameters& parameters)
{
	m_parameters = parameters;
}

const QImage& NoiseRenderer::result() const
{
	return m_result;
}

bool NoiseRenderer::start()
{
	// Check that the renderer is not currently running before starting a new computation
	if (!m_futureImageWatcher->isRunning())
	{
		const QFuture<QImage> futureImage = QtConcurrent::run(this, &NoiseRenderer::Render);
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
	connect(m_futureImageWatcher, &QFutureWatcher<QImage>::finished, this, &NoiseRenderer::OnRenderingFinished);
}

QImage NoiseRenderer::Render() const
{
	typedef LichtenbergControlFunction ControlFunctionType;
	std::unique_ptr<ControlFunctionType> controlFunction(std::make_unique<ControlFunctionType>());

	const Point2D noiseTopLeft(-3.0, -3.0);
	const Point2D noiseBottomRight(2.0, 2.0);
	const Point2D controlFunctionTopLeft(-1.0, -1.0);
	const Point2D controlFunctionBottomRight(1.0, 1.0);

	const Noise<ControlFunctionType> noise(std::move(controlFunction), noiseTopLeft, noiseBottomRight, controlFunctionTopLeft, controlFunctionBottomRight, m_parameters.seed, m_parameters.epsilon, m_parameters.levels, false, true, false);

	QImage image(m_parameters.widthResolution, m_parameters.heightResolution, QImage::Format::Format_Grayscale8);

#pragma omp parallel for shared(image, noise)
	for (int i = 0; i < image.height(); i++) {
		for (int j = 0; j < image.width(); j++) {
			const double x = remap_clamp(double(j), 0.0, double(image.width() - 1), noiseTopLeft.x, noiseBottomRight.x);
			const double y = remap_clamp(double(i), 0.0, double(image.height() - 1), noiseTopLeft.y, noiseBottomRight.y);

			const double value = noise.evaluateLichtenberg(x, y);
			const auto grayValue = uint8_t(value * std::numeric_limits<uint8_t>::max());

			image.setPixel(j, i, qRgb(grayValue, grayValue, grayValue));
		}
	}

	return image;
}
