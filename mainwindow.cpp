#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "simple_xml.hpp"

const float maximum_force = 150.f;
const float d_force = maximum_force/1000.f;

const QString config_main("config.main.xml");

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
  , m_force(0)
  , m_forceGoal(0)
  , m_pressed_force(false)
  , m_max_incline_range(30)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(50);

	connect(ui->widgetView, SIGNAL(push_logs(QString)), this, SLOT(onPushLog(QString)));

	connect(ui->widget_joystick, SIGNAL(valueChanged(float,float)), this, SLOT(onChangeValue(float,float)));

	load_xml();

	ui->hs_yaw->setValue(ui->widgetView->yaw() / 180. * ui->hs_yaw->maximum());
	ui->hs_roll->setValue(ui->widgetView->roll() / m_max_incline_range * ui->hs_roll->maximum());
	ui->hs_tangage->setValue(ui->widgetView->tangage() / m_max_incline_range * ui->hs_tangage->maximum());

	ui->hs_yaw_goal->setValue(ui->widgetView->model().yawGoal() / 180. * ui->hs_yaw_goal->maximum());
}

MainWindow::~MainWindow()
{
	save_xml();

	delete ui;
}

void MainWindow::on_hs_yaw_valueChanged(int value)
{
	ui->widgetView->set_yaw(180. * value/ui->hs_yaw->maximum());
}

void MainWindow::on_hs_tangage_valueChanged(int value)
{
	ui->widgetView->set_tangage(m_max_incline_range * value/ui->hs_tangage->maximum());
}

void MainWindow::on_hs_roll_valueChanged(int value)
{
	ui->widgetView->set_roll(m_max_incline_range * value/ui->hs_roll->maximum());
}

void MainWindow::on_hs_yaw_goal_valueChanged(int value)
{
	ui->widgetView->model().setYawGoal(180. * value/ui->hs_yaw->maximum());
}

void MainWindow::on_hs_tangage_goal_valueChanged(int value)
{
	ui->widgetView->model().setTangageGoal(m_max_incline_range * value/ui->hs_yaw->maximum());
}

void MainWindow::on_hs_roll_goal_valueChanged(int value)
{
	ui->widgetView->model().setRollGoal(m_max_incline_range * value/ui->hs_yaw->maximum());
}

void MainWindow::load_xml()
{
	QMap< QString, QVariant > params;

	if(!SimpleXML::load_param(config_main, params))
		return;

	QString state = params["state"].toString();
	QByteArray bstate = QByteArray::fromBase64(state.toLatin1());
	restoreState(bstate);

	ui->dsb_common_force->setValue(params["f_common"].toFloat());
	ui->dsb_f1->setValue(params["f1"].toFloat());
	ui->dsb_f2->setValue(params["f2"].toFloat());
	ui->dsb_f3->setValue(params["f3"].toFloat());
	ui->dsb_f4->setValue(params["f4"].toFloat());
	ui->dsb_height->setValue(params["height"].toFloat());
	ui->chb_tracking->setChecked(params["tracking"].toBool());

	if(params.contains("incline_range"))
		m_max_incline_range = params["incline_range"].toDouble();

	ui->widgetView->set_tracking(ui->chb_tracking->isChecked());
}

void MainWindow::save_xml()
{
	QMap< QString, QVariant > params;

	params["state"] = QString(saveState().toBase64());

	params["f_common"] = ui->dsb_common_force->value();
	params["f1"] = ui->dsb_f1->value();
	params["f2"] = ui->dsb_f2->value();
	params["f3"] = ui->dsb_f3->value();
	params["f4"] = ui->dsb_f4->value();
	params["height"] = ui->dsb_height->value();
	params["tracking"] = ui->chb_tracking->isChecked();
	params["incline_range"] = m_max_incline_range;

	SimpleXML::save_param(config_main, params);
}

void MainWindow::on_pushButton_clicked()
{
	ui->hs_roll->setValue(0);
	ui->hs_tangage->setValue(0);
	ui->hs_yaw->setValue(0);
}

void MainWindow::on_vs_force_valueChanged(int value)
{
	m_force = maximum_force * value / ui->vs_force->maximum();
	ui->widgetView->set_force(m_force);
}

void MainWindow::on_vs_force_sliderPressed()
{
	m_pressed_force = true;
}

void MainWindow::on_vs_force_sliderReleased()
{
	m_pressed_force = false;
}

void MainWindow::onTimeout()
{
	if(ui->pb_setGoal->isChecked()){
		if(!m_pressed_force && std::abs(m_force * m_forceGoal) > 1.5 * d_force){
			float delta = m_force - m_forceGoal;
			float signDelta = delta > 0 ? -1 : 1;
			m_force += d_force * signDelta;

			ui->widgetView->set_force(m_force);

			ui->vs_force->setValue(m_force / maximum_force * ui->vs_force->maximum());
		}
	}
	if(m_force < 0)
		m_force = 0;

	ui->lb_yaw->setText(QString::number(ui->widgetView->model().yaw(), 'f', 2));
	ui->lb_roll->setText(QString::number(ui->widgetView->model().roll(), 'f', 2));
	ui->lb_tangage->setText(QString::number(ui->widgetView->model().tangage(), 'f', 2));

	ui->lb_f1->setText(QString::number(ui->widgetView->model().force(1), 'f', 3));
	ui->lb_f2->setText(QString::number(ui->widgetView->model().force(2), 'f', 3));
	ui->lb_f3->setText(QString::number(ui->widgetView->model().force(3), 'f', 3));
	ui->lb_f4->setText(QString::number(ui->widgetView->model().force(4), 'f', 3));
}

void MainWindow::onPushLog(const QString &val)
{
	ui->log_output->appendPlainText(val);
}

void MainWindow::on_pb_setGoal_clicked(bool checked)
{
	if(checked){
		m_forceGoal = m_force;
	}
}

void MainWindow::on_pb_height_goal_clicked(bool checked)
{
	ui->widgetView->setUseHeightGoal(checked);
}

void MainWindow::on_doubleSpinBox_valueChanged(double arg1)
{
}

void MainWindow::on_pb_use_forces_clicked(bool checked)
{
	ui->widgetView->model().setUseMultipleForces(checked);
	ui->widgetView->model().setForces(ui->dsb_f1->value(),
									  ui->dsb_f2->value(),
									  ui->dsb_f3->value(),
									  ui->dsb_f4->value());
}

void MainWindow::on_dsb_f1_valueChanged(double arg1)
{
	ui->widgetView->model().setForce(1, arg1);
}

void MainWindow::on_dsb_f2_valueChanged(double arg1)
{
	ui->widgetView->model().setForce(2, arg1);
}

void MainWindow::on_dsb_f3_valueChanged(double arg1)
{
	ui->widgetView->model().setForce(3, arg1);
}

void MainWindow::on_dsb_f4_valueChanged(double arg1)
{
	ui->widgetView->model().setForce(4, arg1);
}

void MainWindow::on_pb_rese_angles_clicked()
{
	ui->widgetView->model().reset_angles();
	ui->pb_use_forces->setChecked(false);
}

void MainWindow::on_dsb_common_force_valueChanged(double arg1)
{
	ui->dsb_f1->setValue(arg1);
	ui->dsb_f2->setValue(arg1);
	ui->dsb_f3->setValue(arg1);
	ui->dsb_f4->setValue(arg1);
}

void MainWindow::on_dsb_height_valueChanged(double arg1)
{
	ui->widgetView->setHeightGoal(arg1);
}

void MainWindow::on_chb_tracking_clicked(bool checked)
{
	ui->widgetView->set_tracking(checked);
}

void MainWindow::on_actionControl_triggered()
{
	ui->dw_control->show();
}

void MainWindow::on_actionLog_triggered()
{
	ui->dw_log->show();
}

void MainWindow::onChangeValue(float x, float y)
{
	x = x / ui->widget_joystick->radius();
	y = y / ui->widget_joystick->radius();

	ui->widgetView->model().setTangageGoal(-y * 20);
	ui->widgetView->model().setRollGoal(-x * 20);
	//ui->hs_tangage_goal->setValue(y * ui->hs_tangage_goal->maximum());
	//ui->hs_roll_goal->setValue(x * ui->hs_roll_goal->maximum());
}

void MainWindow::on_chb_route_clicked(bool checked)
{
	ui->widgetView->setShowRoute(checked);
}

void MainWindow::on_pb_generate_route_clicked()
{
	ui->widgetView->generate_route(ui->sb_number_point->value());
}

void MainWindow::on_chb_useIntegalError_clicked(bool checked)
{
	ui->widgetView->model().setUseIntegralError(checked);
}

void MainWindow::on_pb_power_clicked(bool checked)
{
	ui->widgetView->model().setPower(checked);
}

void MainWindow::on_pb_goToGoal_clicked(bool checked)
{
	ui->widgetView->model().setTrackToGoalPoint(checked);
}
