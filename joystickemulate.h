#ifndef JOYSTICKEMULATE_H
#define JOYSTICKEMULATE_H

#include <QWidget>
#include <QPointF>
#include <QTimer>

class JoystickEmulate : public QWidget
{
	Q_OBJECT
public:
	explicit JoystickEmulate(QWidget *parent = 0);

	void setValue(float X, float Y);
	float valX() const;
	float valY() const;
	float radius() const;
	void setRadius(float v);
	void setAlpha(float a);

	void set_update();

	QPointF crop_value(const QPointF val);

signals:
	void valueChanged(float x, float y);

public slots:
	void onTimeout();

	// QWidget interface
protected:
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void paintEvent(QPaintEvent *event);

private:
	QTimer m_timer;
	QPointF m_value;
	float m_alpha;
	QPointF m_mouse_pt;
	bool m_update;
	bool m_mouse_pressed;

	QPointF m_prev_e;

	float m_R;

	void changed_pos(const QPointF& pos);
};

#endif // JOYSTICKEMULATE_H
