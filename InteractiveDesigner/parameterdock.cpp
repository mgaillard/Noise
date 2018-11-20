#include "parameterdock.h"

#include "ui_parameterdock.h"

ParameterDock::ParameterDock(QWidget *parent)
	: QDockWidget(parent),
	ui(new Ui::ParameterDock)
{
	ui->setupUi(this);
}

ParameterDock::~ParameterDock()
{
	delete ui;
}

void ParameterDock::setParameters(const NoiseParameters& parameters)
{
	ui->typeComboBox->setCurrentIndex(static_cast<int>(parameters.type));
	ui->seedSpinBox->setValue(parameters.seed);
	ui->widthResolutionSpinBox->setValue(parameters.widthResolution);
	ui->heightResolutionSpinBox->setValue(parameters.heightResolution);
	ui->levelSpinBox->setValue(parameters.levels);
	ui->epsilonDoubleSpinBox->setValue(parameters.epsilon);
	ui->displacementDoubleSpinBox->setValue(parameters.displacement),
	ui->noiseTopDoubleSpinBox->setValue(parameters.noiseTop);
	ui->noiseBottomDoubleSpinBox->setValue(parameters.noiseBottom);
	ui->noiseLeftDoubleSpinBox->setValue(parameters.noiseLeft);
	ui->noiseRightDoubleSpinBox->setValue(parameters.noiseRight);
	ui->controlFunctionTopDoubleSpinBox->setValue(parameters.controlFunctionTop);
	ui->controlFunctionBottomDoubleSpinBox->setValue(parameters.controlFunctionBottom);
	ui->controlFunctionLeftDoubleSpinBox->setValue(parameters.controlFunctionLeft);
	ui->controlFunctionRightDoubleSpinBox->setValue(parameters.controlFunctionRight);
}

NoiseParameters ParameterDock::parameters() const
{
	return {
		static_cast<NoiseType>(ui->typeComboBox->currentIndex()),
		ui->seedSpinBox->value(),
		ui->widthResolutionSpinBox->value(),
		ui->heightResolutionSpinBox->value(),
		ui->levelSpinBox->value(),
		ui->epsilonDoubleSpinBox->value(),
		ui->displacementDoubleSpinBox->value(),
		ui->noiseTopDoubleSpinBox->value(),
		ui->noiseBottomDoubleSpinBox->value(),
		ui->noiseLeftDoubleSpinBox->value(),
		ui->noiseRightDoubleSpinBox->value(),
		ui->controlFunctionTopDoubleSpinBox->value(),
		ui->controlFunctionBottomDoubleSpinBox->value(),
		ui->controlFunctionLeftDoubleSpinBox->value(),
		ui->controlFunctionRightDoubleSpinBox->value()
	};
}
