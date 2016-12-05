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

	void on_vs_force_valueChanged(int value);

	void on_vs_force_sliderPressed();

	void on_vs_force_sliderReleased();

	void onTimeout();

	void onPushLog(const QString& val);

	void on_pb_setGoal_clicked(bool checked);

	void on_pb_height_goal_clicked(bool checked);

	void on_doubleSpinBox_valueChanged(double arg1);

private:
	Ui::MainWindow *ui;

	QTimer m_timer;
	bool m_pressed_force;
	float m_force;
	float m_forceGoal;
};

#endif // MAINWINDOW_H
