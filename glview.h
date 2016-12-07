#ifndef GLVIEW_H
#define GLVIEW_H

#include <QWidget>
#include <QGLWidget>
#include <QPointF>
#include <QTimer>

#include <model.h>

namespace Ui {
class GLView;
}

class GLView : public QGLWidget
{
	Q_OBJECT

public:
	explicit GLView(QWidget *parent = 0);
	~GLView();

	Model &model();

	void set_yaw(float v);
	void set_tangage(float v);
	void set_roll(float v);

	float yaw() const;
	float tangage() const;
	float roll() const;

	void set_force(float f);

	void setUseHeightGoal(bool val);
	void setHeightGoal(float h);

	void set_tracking(bool v);

	void set_update();

private:
	Ui::GLView *ui;
	bool m_init;
	bool m_update;

	bool m_tracking;

	QTimer m_timer;
	QTimer m_timer_model;

	QPointF m_mouse_pt;
	QPointF m_delta_pt;
	QPointF m_wheel_pt;
	float m_delta_z;
	float m_current_z;

	Model m_model;

	bool m_left_down;
	bool m_wheel_down;

	ct::Vec3f m_angles;

	ct::Vec3f m_color_space;

	void init();
	void draw_net();
	void draw_model();

	void load_xml();
	void save_xml();

	// QGLWidget interface
public slots:
	virtual void updateGL();
	void onTimeout();
	void onTimeoutModel();

signals:
	void push_logs(const QString& val);

protected:
	virtual void resizeGL(int w, int h);
	virtual void paintGL();
	virtual void glDraw();

	// QWidget interface
protected:
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
};

#endif // GLVIEW_H
