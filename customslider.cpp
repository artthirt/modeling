#include "customslider.h"

CustomSlider::CustomSlider(QWidget *parent)
	: QSlider(parent)
	, m_goal_value(0)
	, m_kp(8)
	, m_kd(10)
	, m_prev_value(0)
	, m_slider_pressed(false)
{
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(30);

	m_value = value();

	connect(this, SIGNAL(sliderPressed()), SLOT(onSliderPressed()));
	connect(this, SIGNAL(sliderReleased()), SLOT(onSliderReleased()));
	connect(this, SIGNAL(valueChanged(int)), SLOT(onChangeValue(int)));
}

void CustomSlider::setGoalValue(float value)
{
	m_goal_value = value;
	if(m_goal_value < minimum())
		m_goal_value = minimum();
	if(m_goal_value > maximum())
		m_goal_value = maximum();
}

double CustomSlider::valueQuadratic() const
{
	double v1 = maximum();
	double res = m_value / v1;
	if(res >= 0)
		return res * res;
	else
		return -res * res;
}

void CustomSlider::onTimeout()
{
	if(!m_slider_pressed && m_value != m_goal_value){
		if(m_goal_value < minimum())
			m_goal_value = minimum();
		if(m_goal_value > maximum())
			m_goal_value = maximum();
		if(qAbs(m_value - m_goal_value) > 1e-6){

			double e = m_goal_value - m_value;
			double de = m_value - m_prev_value;
			m_prev_value = m_value;

			double u = m_kp * e + m_kd * de;
			m_value += u * m_timer.interval() / 1000.;

			if(m_value > maximum())
				m_value = maximum();
			if(m_value < minimum())
				m_value = minimum();

			setValue(m_value);
		}else{
			m_value = m_goal_value;
			setValue(m_value);
		}
	}
}

void CustomSlider::onSliderPressed()
{
	m_slider_pressed = true;
}

void CustomSlider::onSliderReleased()
{
	m_slider_pressed = false;
	m_prev_value = m_value;
}

void CustomSlider::onChangeValue(int value)
{
	m_value = value;
}
