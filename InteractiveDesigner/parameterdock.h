#ifndef PARAMETERDOCK_H
#define PARAMETERDOCK_H

#include <QDockWidget>

#include "noiseparameters.h"

namespace Ui {
	class ParameterDock;
}

class ParameterDock : public QDockWidget
{
	Q_OBJECT

public:
	explicit ParameterDock(QWidget *parent = Q_NULLPTR);
	virtual ~ParameterDock();
	
	/**
	 * \brief Set the parameters
	 * \return The parameters
	 */
	void setParameters(const NoiseParameters& parameters);

	/**
	 * \brief Return the parameters specified by the user
	 * \return The parameters
	 */
	NoiseParameters parameters() const;

private:
	Ui::ParameterDock* ui;
};

#endif // PARAMETERDOCK_H