#ifndef GLVIEW_H
#define GLVIEW_H

#include <QWidget>
#include <QGLWidget>
#include <QPointF>
#include <QTimer>

#include <model.h>
#include "modelroute.h"

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
	ModelRoute &modelRoute();

	void set_yaw(float v);
	void set_tangage(float v);
	void set_roll(float v);

	float yaw() const;
	float tangage() const;
	float roll() const;

	void set_force(float f);

	void setHeightGoal(float h);

	void set_tracking(bool v);

	void set_update();

	void generate_route(size_t count);

	void setShowRoute(bool v);
	bool isShowRoute() const;

	void setDrawTrack(bool v);
	bool isDrawTrack() const;

	void setShowGraphics(bool v);
	bool isGhowGraphics() const;

private:
	Ui::GLView *ui;
	bool m_init;
	bool m_update;

	bool m_show_graphics;

	double m_prev_e_track;
	double m_prev_u;

	bool m_tracking;
	double m_tracking_angle;

	bool m_show_route;

	QTimer m_timer;
	QTimer m_timer_model;

	QPointF m_mouse_pt;
	QPointF m_delta_pt;
	QPointF m_wheel_pt;
	double m_delta_z;
	double m_current_z;

	Model m_model;

	bool m_left_down;
	bool m_wheel_down;

	ct::Vec3d m_angles;

	ct::Vec3d m_color_space;

	ModelRoute m_modelRoute;

	double m_timer_goal;

	bool m_is_draw_track;

	void init();
	void draw_net();
	void draw_model();
	void draw_route();
	void draw_goal();
	void draw_track();
	////// graphic ////////
	void draw_plane();
	void draw_scaled_graph();
	void draw_graphics();
	void draw_curve_height();

	void load_xml();
	void save_xml();

	void calculate_track();
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
