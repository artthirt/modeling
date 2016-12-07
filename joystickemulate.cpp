#include "joystickemulate.h"
#include <QMouseEvent>
#include <QPainter>

const float r_cyrc = 10.;

double norm(const QPointF& pt)
{
	double ret = pt.x() * pt.x() + pt.y() * pt.y();
	return sqrt(ret);
}

JoystickEmulate::JoystickEmulate(QWidget *parent)
	: QWidget(parent)
	, m_R(100)
	, m_alpha(0.1)
	, m_update(false)
	, m_mouse_pressed(false)
{
	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(30);

	setMouseTracking(true);
}

void JoystickEmulate::setValue(float X, float Y)
{
	float r = norm(QPointF(X, Y));
	float x = X, y = Y;
	if(r > m_R){
		x = x / r * m_R;
		y = y / r * m_R;
	}

	m_value = QPointF(x, y);

	emit valueChanged(m_value.x(), m_value.y());

	set_update();
}

float JoystickEmulate::valX() const
{
	return m_value.y();
}

float JoystickEmulate::valY() const
{
	return m_value.x();
}

float JoystickEmulate::radius() const
{
	return m_R;
}

void JoystickEmulate::setRadius(float v)
{
	if(v > 0)
		m_R = v;
}

void JoystickEmulate::setAlpha(float a)
{
	if(a > 0)
		m_alpha = a;
	set_update();
}

void JoystickEmulate::set_update()
{
	m_update = true;
}

QPointF JoystickEmulate::crop_value(const QPointF val)
{
	float r = norm(val);
	float x = val.x(), y = val.y();
	if(r > m_R){
		x = x / r * m_R;
		y = y / r * m_R;
	}

	return QPointF(x, y);
}

void JoystickEmulate::onTimeout()
{
	if(m_update){
		m_update = false;

		if(!m_mouse_pressed && qAbs(m_value.x()) > m_alpha && qAbs(m_value.y()) > m_alpha){
			float kp = 10;
			float kd = 5;

			QPointF e = QPointF(0, 0) - m_value;

			QPointF de = e - m_prev_e;
			m_prev_e = e;

			QPointF u = kp * e + kd * de;

			float dt = 1. * m_timer.interval() / 1000.;

			m_value += u * dt;

			m_value = crop_value(m_value);

			emit valueChanged(m_value.x(), m_value.y());

			set_update();
		}

		update();
	}
}


void JoystickEmulate::mousePressEvent(QMouseEvent *event)
{
	m_mouse_pressed = true;
	m_mouse_pt = event->pos();
	changed_pos(event->pos());
}

void JoystickEmulate::mouseReleaseEvent(QMouseEvent *event)
{
	m_mouse_pressed = false;
	m_mouse_pt = event->pos();
	changed_pos(event->pos());
}

void JoystickEmulate::mouseMoveEvent(QMouseEvent *event)
{
	if(event->buttons().testFlag(Qt::LeftButton)){
		changed_pos(event->pos());
	}
}

void JoystickEmulate::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);

	painter.fillRect(rect(), Qt::white);

	int w = width(), h = height();

	QPointF p1(w/2, 0), p2(w/2, h), p3(0, h/2), p4(w, h/2);

	painter.setBrush(QColor(200, 200, 200));
	painter.drawEllipse(QPointF(w/2, h/2), w/2, h/2);

	QPen pen;
	pen.setWidth(2);
	pen.setColor(Qt::red);

	painter.setPen(pen);
	painter.drawLine(p1, p2);
	painter.drawLine(p3, p4);

	painter.setPen(Qt::black);
	painter.setBrush(Qt::black);

	float x = m_value.x(), y = m_value.y();

	x = x / m_R * (w - 2 * r_cyrc)/2.;
	y = y / m_R * (h - 2 * r_cyrc)/2.;

	painter.drawEllipse(QPointF(w/2 + x, h/2 + y), r_cyrc, r_cyrc);
}

void JoystickEmulate::changed_pos(const QPointF &pos)
{
	float w = width();
	float h = height();
	QPointF cp(w/2, h/2);

	QPointF dp = pos - cp;

	float x = dp.x() / ((w - 2 * r_cyrc)/2.);
	float y = dp.y() / ((h - 2 * r_cyrc)/2.);

	float r = norm(QPointF(x, y));
	if( r > 1)
		x = x/r, y = y/r;
	x = x * m_R;
	y = y * m_R;

	m_value = QPointF(x, y);

	emit valueChanged(x, y);

	set_update();
}
