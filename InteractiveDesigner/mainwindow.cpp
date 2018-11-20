#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>

// Default parameters for the noise function
const NoiseParameters MainWindow::default_noise_parameters = {
	NoiseType::lichtenberg,
	0,    // seed
	512,  // width
	512,  // height
	1,    // levels
	0.1,  // epsilon
	0.0,  // displacement
	-2.0, // noiseTop
	1.0,  // noiseBottom
	-2.0, // noiseLeft
	1.0,  // noiseRight
	-1.0, // controlFunctionTop
	1.0,  // controlFunctionBottom
	-1.0, // controlFunctionLeft
	1.0   // controlFunctionRight
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
	ui->display_widget->setImage(m_noiseRenderer->result());

	// Close the progress dialog
	if (m_progressDialog != nullptr)
	{
		m_progressDialog->reset();
	}
}

void MainWindow::Save()
{
	if (!m_noiseRenderer->result().isNull())
	{
		QString filename = QFileDialog::getSaveFileName(this, tr("Save the image"), "", tr("Images (*.png *.jpg)"));

		if (!filename.isEmpty())
		{
			const bool saved = m_noiseRenderer->result().save(filename);

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
