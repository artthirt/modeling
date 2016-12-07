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
	void setDTick(float val);

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
	float m_dtick;
	bool m_slider_pressed;
};

#endif // CUSTOMSLIDER_H
