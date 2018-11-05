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
	ui->seedSpinBox->setValue(parameters.seed);
	ui->widthResolutionSpinBox->setValue(parameters.widthResolution);
	ui->heightResolutionSpinBox->setValue(parameters.heightResolution);
	ui->levelSpinBox->setValue(parameters.levels);
	ui->epsilonDoubleSpinBox->setValue(parameters.epsilon);
}

NoiseParameters ParameterDock::parameters() const
{
	return { 
		ui->seedSpinBox->value(),
		ui->widthResolutionSpinBox->value(),
		ui->heightResolutionSpinBox->value(),
		ui->levelSpinBox->value(),
		ui->epsilonDoubleSpinBox->value()
	};
}
