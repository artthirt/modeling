#include "customslider.h"

CustomSlider::CustomSlider(QWidget *parent)
	: QSlider(parent)
	, m_goal_value(0)
	, m_dtick(2)
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

void CustomSlider::setDTick(float val)
{
	if(val <=0 )
		val = 1;
	m_dtick = val;
}

void CustomSlider::onTimeout()
{
	if(!m_slider_pressed && m_value != m_goal_value){
		if(m_goal_value < minimum())
			m_goal_value = minimum();
		if(m_goal_value > maximum())
			m_goal_value = maximum();
		if(qAbs(m_value - m_goal_value) > 1.5 * m_dtick){
			float sign = m_goal_value - m_value > 0? 1 : -1;
			m_value += sign * m_dtick;
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
}

void CustomSlider::onChangeValue(int value)
{
	m_value = value;
}
