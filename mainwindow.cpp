#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "simple_xml.hpp"

const float maximum_force = 150.f;
const float d_force = maximum_force/1000.f;
const double maximum_vert_vel = 3;

const QString config_main("config.main.xml");

QString red_lamp(int radius)
{
	return QString("background-color:qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.29, fy:0.284, stop:0 rgba(255, 137, 133, 255), stop:1 rgba(150, 0, 0, 255));"
					"border-color: rgb(60, 60, 60);"
					"border-radius: %1px;").arg(radius);
}

QString green_lamp(int radius)
{
	return QString("background-color:qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.29, fy:0.284, stop:0 rgba(137, 255, 133, 255), stop:1 rgba(0, 150, 0, 255));"
				   "border-radius: %1px;"
				   "border-color: rgb(60, 60, 60);").arg(radius);
}

//const QString green_lamp("background-color:qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.29, fy:0.284, stop:0 rgba(137, 255, 133, 255), stop:1 rgba(0, 150, 0, 255));"
//						 "border-radius: 10px;"
//						 "border-color: rgb(60, 60, 60);");

//const QString red_lamp("background-color:qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.29, fy:0.284, stop:0 rgba(255, 137, 133, 255), stop:1 rgba(150, 0, 0, 255));"
//					   "border-radius: 10px;"
//					   "border-color: rgb(60, 60, 60);");

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
  , m_force(0)
  , m_forceGoal(0)
  , m_pressed_force(false)
  , m_max_incline_range(40)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(50);

	connect(ui->widgetView, SIGNAL(push_logs(QString)), this, SLOT(onPushLog(QString)));

	connect(ui->widget_joystick, SIGNAL(valueChanged(float,float)), this, SLOT(onChangeValue(float,float)));

	ui->widgetView->model().setHeightControl(Model::EGoToToHeight);

	load_xml();

	ui->hs_yaw->setValue(ui->widgetView->yaw() / 180. * ui->hs_yaw->maximum());
	ui->hs_roll->setValue(ui->widgetView->roll() / m_max_incline_range * ui->hs_roll->maximum());
	ui->hs_tangage->setValue(ui->widgetView->tangage() / m_max_incline_range * ui->hs_tangage->maximum());

	ui->hs_yaw_goal->setValue(ui->widgetView->model().yawGoal() / 180. * ui->hs_yaw_goal->maximum());

	ui->pb_use_forces->setChecked(ui->widgetView->model().isUseEngines());
	ui->chb_drawTrack->setChecked(ui->widgetView->isDrawTrack());
	ui->chb_route->setChecked(ui->widgetView->isShowRoute());

	ui->dsb_accuracy->setValue(ui->widgetView->model().accuracy_goal());
	ui->dsb_radius_goal->setValue(ui->widgetView->model().radiusOfInfluence_goal());
	ui->dsb_accuracy_vel->setValue(ui->widgetView->model().accuracyVelocity());

	if(!ui->rb_use_hover->isChecked())
		ui->hs_vert_vel->setEnabled(false);
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
	ui->widgetView->model().setTangageGoal(m_max_incline_range * ui->hs_tangage_goal->valueQuadratic());
}

void MainWindow::on_hs_roll_goal_valueChanged(int value)
{
	ui->widgetView->model().setRollGoal(m_max_incline_range * ui->hs_roll_goal->valueQuadratic());
}

void MainWindow::load_xml()
{
	QMap< QString, QVariant > params;

	if(!SimpleXML::load_param(config_main, params))
		return;

	QString state = params["state"].toString();
	QByteArray bstate = QByteArray::fromBase64(state.toLatin1());
	restoreState(bstate);

	ui->dsb_height->setValue(params["height"].toFloat());
	ui->chb_tracking->setChecked(params["tracking"].toBool());

	int id = params["height_control"].toInt();
	if(id == 0){
		ui->rb_none_height->setChecked(true);
		ui->widgetView->model().setHeightControl(Model::ENone);
	}else if(id == 1){
		ui->rb_use_go_to_height->setChecked(true);
		ui->widgetView->model().setHeightControl(Model::EGoToToHeight);
	}else{
		ui->rb_use_hover->setChecked(true);
		ui->widgetView->model().setHeightControl(Model::EHover);
	}

	if(params.contains("incline_range"))
		m_max_incline_range = params["incline_range"].toDouble();

	ui->widgetView->set_tracking(ui->chb_tracking->isChecked());
}

void MainWindow::save_xml()
{
	QMap< QString, QVariant > params;

	params["state"] = QString(saveState().toBase64());

	params["height"] = ui->dsb_height->value();
	params["tracking"] = ui->chb_tracking->isChecked();
	params["incline_range"] = m_max_incline_range;

	params["height_control"] = ui->rb_none_height->isChecked()? 0 : ui->rb_use_go_to_height->isChecked()? 1 : 2;

	SimpleXML::save_param(config_main, params);
}

void MainWindow::on_pushButton_clicked()
{
	ui->hs_roll->setValue(0);
	ui->hs_tangage->setValue(0);
	ui->hs_yaw->setValue(0);
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
	ui->lb_yaw->setText(QString::number(ui->widgetView->model().yaw(), 'f', 2));
	ui->lb_roll->setText(QString::number(ui->widgetView->model().roll(), 'f', 2));
	ui->lb_tangage->setText(QString::number(ui->widgetView->model().tangage(), 'f', 2));
	ui->lb_vertVel->setText(QString::number(ui->widgetView->model().vertVel(), 'f', 2));

	ui->lb_f1->setText(QString::number(ui->widgetView->model().force(1), 'f', 3));
	ui->lb_f2->setText(QString::number(ui->widgetView->model().force(2), 'f', 3));
	ui->lb_f3->setText(QString::number(ui->widgetView->model().force(3), 'f', 3));
	ui->lb_f4->setText(QString::number(ui->widgetView->model().force(4), 'f', 3));

	ui->lb_posx->setText(QString::number(ui->widgetView->model().pos()[0], 'f', 3));
	ui->lb_posy->setText(QString::number(ui->widgetView->model().pos()[1], 'f', 3));
	ui->lb_posz->setText(QString::number(ui->widgetView->model().pos()[2], 'f', 3));

	ui->lb_velx->setText(QString::number(ui->widgetView->model().velocity()[0], 'f', 3));
	ui->lb_vely->setText(QString::number(ui->widgetView->model().velocity()[1], 'f', 3));
	ui->lb_velz->setText(QString::number(ui->widgetView->model().velocity()[2], 'f', 3));

	ui->lb_vel->setText("V(norm)=" + QString::number(ui->widgetView->model().velocity().norm(), 'f', 5));
	ui->lb_goal_a->setText("goals: φ=" + QString::number(ui->widgetView->model().goal_yaw(), 'f', 2) +
			" θ=" + QString::number(ui->widgetView->model().goal_tangage(), 'f', 2) +
			" α=" + QString::number(ui->widgetView->model().goal_roll(), 'f', 2));

	if(ui->widgetView->model().is_goal_reached()){
		ui->lb_goal_reached->setStyleSheet(green_lamp(10));
	}else{
		ui->lb_goal_reached->setStyleSheet(red_lamp(10));
	}

	if(ui->chb_autoNext->isChecked()){
		if(ui->widgetView->model().is_goal_reached() && ui->widgetView->model().isTrackToGoalPoint()){
			if(!ui->widgetView->modelRoute().isEnd()){
				ui->widgetView->model().setGoalPoint(ui->widgetView->modelRoute().current_point());
				ui->widgetView->modelRoute().next();
			}else{
				ui->chb_autoNext->setChecked(false);
			}
		}
	}
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

void MainWindow::on_doubleSpinBox_valueChanged(double arg1)
{
}

void MainWindow::on_pb_use_forces_clicked(bool checked)
{
	ui->widgetView->model().setUseEngines(checked);
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
	ui->widgetView->model().setUseIntegralErrorAngles(checked);
}

void MainWindow::on_pb_power_clicked(bool checked)
{
	ui->widgetView->model().setPower(checked);
}

void MainWindow::on_pb_goToGoal_clicked(bool checked)
{
	ui->widgetView->model().setTrackToGoalPoint(checked);
}

void MainWindow::on_hs_vert_vel_valueChanged(int value)
{
	double v = ui->hs_vert_vel->valueQuadratic();
	v *= maximum_vert_vel;
	ui->widgetView->model().setGoalVerticalVelocity(v);
}

void MainWindow::on_rb_none_height_clicked(bool checked)
{
	ui->widgetView->model().setHeightControl(Model::ENone);
	ui->hs_vert_vel->setEnabled(false);
}

void MainWindow::on_rb_use_go_to_height_clicked(bool checked)
{
	ui->widgetView->model().setHeightControl(Model::EGoToToHeight);
	ui->hs_vert_vel->setEnabled(false);
}

void MainWindow::on_rb_use_hover_clicked(bool checked)
{
	ui->widgetView->model().setHeightControl(Model::EHover);
	ui->hs_vert_vel->setEnabled(true);
}

void MainWindow::on_chb_useIntegalErrorHeight_clicked(bool checked)
{
	ui->widgetView->model().setUseIntegralErrorHeight(checked);
}

void MainWindow::on_pb_toBegin_clicked()
{
	ui->widgetView->modelRoute().first();
}

void MainWindow::on_pb_toNext_clicked()
{
	if(!ui->widgetView->modelRoute().isEnd()){
		ct::Vec3d pt = ui->widgetView->modelRoute().current_point();
		ui->widgetView->model().setGoalPoint(pt);
		ui->widgetView->modelRoute().next();
	}
}

void MainWindow::on_chb_drawTrack_clicked(bool checked)
{
	ui->widgetView->setDrawTrack(checked);
}

void MainWindow::on_chb_show_graphics_2_clicked(bool checked)
{
	ui->widgetView->setShowGraphics(checked);
}

void MainWindow::on_dsb_accuracy_valueChanged(double arg1)
{
	ui->widgetView->model().setAccuracyGoal(arg1);
}

void MainWindow::on_dsb_radius_goal_valueChanged(double arg1)
{
	ui->widgetView->model().setRadiusGoal(arg1);
}

void MainWindow::on_dsb_accuracy_vel_valueChanged(double arg1)
{
	ui->widgetView->model().setAccuracyVelocity(arg1);
}

void MainWindow::on_chb_bind_rotation_clicked(bool checked)
{
	ui->widgetView->setBindToRotation(checked);
}
