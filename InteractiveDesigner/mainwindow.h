#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QProgressDialog>

#include "parameterdock.h"
#include "noiserenderer.h"

namespace Ui {
	class MainWindowClass;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = Q_NULLPTR);
	virtual ~MainWindow();

private slots:
	void StartRendering();
	void RenderingFinished();
	void Save();

private:
	void SetupUi();
	void CreateActions();

	static const NoiseParameters default_noise_parameters;

	Ui::MainWindowClass* ui;

	ParameterDock* m_parameterDock;

	QProgressDialog* m_progressDialog;

	NoiseRenderer* m_noiseRenderer;
};

#endif // MAINWINDOW_H