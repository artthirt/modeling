#include "mainwindow.h"
#include "ui_mainwindow.h"

const float maximum_force = 150.;
const float d_force = maximum_force/1000.;

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
  , m_force(0)
  , m_forceGoal(0)
  , m_pressed_force(false)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(50);

	connect(ui->widgetView, SIGNAL(push_logs(QString)), this, SLOT(onPushLog(QString)));
}

MainWindow::~MainWindow()
{
	delete ui;
}

void MainWindow::on_hs_yaw_valueChanged(int value)
{
	ui->widgetView->set_yaw(ct::angle2rad(180. * value/ui->hs_yaw->maximum()));
}

void MainWindow::on_hs_tangage_valueChanged(int value)
{
	ui->widgetView->set_tangage(ct::angle2rad(180. * value/ui->hs_tangage->maximum()));
}

void MainWindow::on_hs_roll_valueChanged(int value)
{
	ui->widgetView->set_roll(ct::angle2rad(180. * value/ui->hs_roll->maximum()));
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
	ui->widgetView->setHeightGoal(arg1);
}
