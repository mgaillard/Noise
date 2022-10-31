#include "displaywidget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QVBoxLayout>
#include <QScrollBar>

DisplayWidget::DisplayWidget(QWidget *parent)
	: QWidget(parent),
	m_imageLabel(new QLabel),
	m_scrollArea(new QScrollArea),
	m_scaleFactor(1.0)
{
	m_imageLabel->setBackgroundRole(QPalette::Base);
	m_imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
	m_imageLabel->setScaledContents(true);

	m_scrollArea->setBackgroundRole(QPalette::Dark);
	m_scrollArea->setWidget(m_imageLabel);
	m_scrollArea->setVisible(false);

	auto layout = new QVBoxLayout(this);
	layout->addWidget(m_scrollArea);
	setLayout(layout);
}

void DisplayWidget::setImage(const QImage& newImage)
{
	m_image = newImage;
	m_imageLabel->setPixmap(QPixmap::fromImage(m_image));
	m_scaleFactor = 1.0;

	m_scrollArea->setVisible(true);
	m_imageLabel->adjustSize();
}

void DisplayWidget::normalSize()
{
	m_imageLabel->adjustSize();
	m_scaleFactor = 1.0;
}

void DisplayWidget::fitToWindow(bool fitToWindow)
{
	m_scrollArea->setWidgetResizable(fitToWindow);

	if (!fitToWindow)
	{
		normalSize();
	}
}

void DisplayWidget::zoomIn()
{
	scaleImage(1.25);
}

void DisplayWidget::zoomOut()
{
	scaleImage(0.8);
}

void DisplayWidget::scaleImage(double factor)
{
	m_scaleFactor *= factor;
	m_imageLabel->resize(m_scaleFactor * m_imageLabel->pixmap().size());

	adjustScrollBar(m_scrollArea->horizontalScrollBar(), factor);
	adjustScrollBar(m_scrollArea->verticalScrollBar(), factor);
}

void DisplayWidget::adjustScrollBar(QScrollBar* scrollBar, double factor) const
{
	scrollBar->setValue(int(factor * scrollBar->value() + ((factor - 1) * scrollBar->pageStep() / 2)));
}
