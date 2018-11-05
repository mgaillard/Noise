#ifndef DISPLAYWIDGET_H
#define DISPLAYWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QScrollArea>

class DisplayWidget : public QWidget
{
	Q_OBJECT

public:
	explicit DisplayWidget(QWidget *parent = nullptr);

public slots:
	void setImage(const QImage &newImage);
	void normalSize();
	void fitToWindow(bool fitToWindow);
	void zoomIn();
	void zoomOut();

private:
	void scaleImage(double factor);
	void adjustScrollBar(QScrollBar *scrollBar, double factor) const;

	QImage m_image;
	QLabel* m_imageLabel;
	QScrollArea* m_scrollArea;
	double m_scaleFactor;
};

#endif // DISPLAYWIDGET_H