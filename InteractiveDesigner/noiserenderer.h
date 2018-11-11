#ifndef NOISERENDERER_H
#define NOISERENDERER_H

#include <QObject>
#include <QImage>
#include <QtConcurrent>

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
	 * \brief Return the rendered image
	 * \return The rendered image
	 */
	const QImage& result() const;

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
	void ConfigureFutureWatcher();

	/**
	 * \brief Render the noise in a QImage.
	 * \return An image of the noise.
	 */
	QImage Render() const;

	QFutureWatcher<QImage>* m_futureImageWatcher;

	NoiseParameters m_parameters;

	QImage m_result;
};

#endif // NOISERENDERER_H