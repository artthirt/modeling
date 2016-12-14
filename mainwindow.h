#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget *parent = 0);
	~MainWindow();

private slots:
	void on_hs_yaw_valueChanged(int value);

	void on_hs_tangage_valueChanged(int value);

	void on_hs_roll_valueChanged(int value);

	void on_pushButton_clicked();

	void on_vs_force_sliderPressed();

	void on_vs_force_sliderReleased();

	void onTimeout();

	void onPushLog(const QString& val);

	void on_pb_setGoal_clicked(bool checked);

	void on_doubleSpinBox_valueChanged(double arg1);

	void on_pb_use_forces_clicked(bool checked);

	void on_dsb_f1_valueChanged(double arg1);

	void on_dsb_f2_valueChanged(double arg1);

	void on_dsb_f3_valueChanged(double arg1);

	void on_dsb_f4_valueChanged(double arg1);

	void on_pb_rese_angles_clicked();

	void on_hs_yaw_goal_valueChanged(int value);

	void on_hs_tangage_goal_valueChanged(int value);

	void on_hs_roll_goal_valueChanged(int value);

	void on_dsb_height_valueChanged(double arg1);

	void on_chb_tracking_clicked(bool checked);

	void on_actionControl_triggered();

	void on_actionLog_triggered();

	void onChangeValue(float x, float y);

	void on_chb_route_clicked(bool checked);

	void on_pb_generate_route_clicked();

	void on_chb_useIntegalError_clicked(bool checked);

	void on_pb_power_clicked(bool checked);

	void on_pb_goToGoal_clicked(bool checked);

	void on_chb_searchHover_clicked(bool checked);

	void on_hs_vert_vel_valueChanged(int value);

	void on_rb_none_height_clicked(bool checked);

	void on_rb_use_go_to_height_clicked(bool checked);

	void on_rb_use_hover_clicked(bool checked);

	void on_chb_useIntegalErrorHeight_clicked(bool checked);

	void on_pb_toBegin_clicked();

	void on_pb_toNext_clicked();

	void on_chb_drawTrack_clicked(bool checked);

	void on_chb_show_graphics_2_clicked(bool checked);

	void on_dsb_accuracy_valueChanged(double arg1);

	void on_dsb_radius_goal_valueChanged(double arg1);

private:
	Ui::MainWindow *ui;

	QTimer m_timer;
	bool m_pressed_force;
	float m_force;
	float m_forceGoal;

	double m_max_incline_range;

	void load_xml();
	void save_xml();
};

#endif // MAINWINDOW_H
