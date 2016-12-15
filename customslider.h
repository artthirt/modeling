#ifndef CUSTOMSLIDER_H
#define CUSTOMSLIDER_H

#include <QWidget>
#include <QSlider>
#include <QTimer>

class CustomSlider : public QSlider
{
	Q_OBJECT
public:
	explicit CustomSlider(QWidget *parent = 0);

	void setGoalValue(float value);

	double valueQuadratic() const;

signals:

public slots:
	void onTimeout();
	void onSliderPressed();
	void onSliderReleased();
	void onChangeValue(int value);

private:
	QTimer m_timer;
	float m_goal_value;
	float m_value;
	float m_prev_value;
	bool m_slider_pressed;
	float m_kp;
	float m_kd;
};

#endif // CUSTOMSLIDER_H
