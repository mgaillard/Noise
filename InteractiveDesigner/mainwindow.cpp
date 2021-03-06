#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>

// Default parameters for the noise function
const NoiseParameters MainWindow::default_noise_parameters = {
	NoiseType::terrain,
	0,    // seed
	512,  // width
	512,  // height
	1,    // levels
	0.25, // epsilon
	0.075,// displacement
	0.0,  // noiseTop
	4.0,  // noiseBottom
	0.0,  // noiseLeft
	4.0,  // noiseRight
	-0.5, // controlFunctionTop
	0.7,  // controlFunctionBottom
	-0.2, // controlFunctionLeft
	1.4,  // controlFunctionRight
	3,    // primitivesResolutionSteps
	0.5,  // slopePower
	0.05, // noiseAmplitudeProportion
	1.0   // controlScale
};

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent),
	ui(new Ui::MainWindowClass),
	m_progressDialog(nullptr),
	m_noiseRenderer(new NoiseRenderer(this, default_noise_parameters))
{
	SetupUi();
	CreateActions();
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::StartRendering()
{
	m_noiseRenderer->setParameters(m_parameterDock->parameters());
	const bool isStarted = m_noiseRenderer->start();

	if (isStarted)
	{
		// Display a progress dialog during the computation.
		m_progressDialog = new QProgressDialog("Rendering", "Cancel", 0, 0, this);
		m_progressDialog->setAttribute(Qt::WA_DeleteOnClose);
		m_progressDialog->setRange(0, 0);
		m_progressDialog->setValue(0);

		m_progressDialog->exec();
	}
	else
	{
		// The renderer is already running
		QMessageBox::warning(this, "Cannot start rendering", "Renderer is already running.");
	}
}

void MainWindow::RenderingFinished()
{
	ui->display_widget->setImage(m_noiseRenderer->resultQImage());

	// Close the progress dialog
	if (m_progressDialog != nullptr)
	{
		m_progressDialog->reset();
	}
}

void MainWindow::Save()
{
	cv::Mat image = m_noiseRenderer->resultCvMat();

	if (!image.empty())
	{
		QString filename = QFileDialog::getSaveFileName(this, tr("Save the image"), "", tr("Images (*.png *.jpg)"));

		if (!filename.isEmpty())
		{
			const bool saved = cv::imwrite(filename.toStdString(), image);

			if (!saved)
			{
				QMessageBox::critical(this, tr("Error while saving"), tr("Impossible to save the image"));
			}
		}
	}
	else
	{
		QMessageBox::critical(this, tr("Error while saving"), tr("No result to save"));
	}
}

void MainWindow::SetupUi()
{
	ui->setupUi(this);

	m_parameterDock = new ParameterDock(this);
	m_parameterDock->setParameters(default_noise_parameters);
	addDockWidget(Qt::RightDockWidgetArea, m_parameterDock);
	ui->menuWindow->addAction(m_parameterDock->toggleViewAction());
}

void MainWindow::CreateActions()
{
	ui->actionSave->setShortcut(QKeySequence::Save);
	ui->actionRender->setShortcut(QKeySequence::Refresh);
	ui->actionZoom_In_25->setShortcut(QKeySequence::ZoomIn);
	ui->actionZoom_Out_25->setShortcut(QKeySequence::ZoomOut);

	connect(ui->actionSave, &QAction::triggered, this, &MainWindow::Save);
	connect(ui->actionNormal_Size, &QAction::triggered, ui->display_widget, &DisplayWidget::normalSize);
	connect(ui->actionFit_to_Window, &QAction::triggered, ui->display_widget, &DisplayWidget::fitToWindow);
	connect(ui->actionZoom_In_25, &QAction::triggered, ui->display_widget, &DisplayWidget::zoomIn);
	connect(ui->actionZoom_Out_25, &QAction::triggered, ui->display_widget, &DisplayWidget::zoomOut);
	
	connect(ui->actionRender, &QAction::triggered, this, &MainWindow::StartRendering);
	connect(m_noiseRenderer, &NoiseRenderer::finished, this, &MainWindow::RenderingFinished);
}
