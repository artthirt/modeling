#include "glview.h"
#include "ui_glview.h"

#ifdef _MSC_VER
#include <Windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <QMouseEvent>
#include <QDebug>

#include <simple_xml.hpp>

const QString xml_config("config.glview.xml");

typedef float GLVertex3f[3];

////////////////////////////////////

void draw_line(const ct::Vec3d& v, const ct::Vec3d & col = ct::Vec3d::ones(), double scale = 1.)
{
	glLineWidth(3);

	glPushMatrix();

	glScaled(scale, scale, scale);

	glColor3dv(col.ptr());
	glBegin(GL_LINES);
	glVertex3dv(ct::Vec3d::zeros().ptr());
	glVertex3dv(v.ptr());
	glEnd();

	glPopMatrix();

	glLineWidth(1);
}

void draw_cylinder(double R, double H, int cnt = 10, const ct::Vec4d& col = ct::Vec4d::ones())
{
	double z0 = 0;
	double z1 = H;

	glColor4dv(col.val);
	//glColor4d(0.8, 0.4, 0.1, 0.5);

	glBegin(GL_TRIANGLE_STRIP);
	for(int i = 0; i <= cnt; i++){
		double xi = sin(2. * i / cnt * M_PI);
		double yi = cos(2. * i / cnt * M_PI);
		double x0 = R * xi;
		double y0 = R * yi;

//		x1 = R * sin(2. * (i + 1) / cnt * M_PI);
//		y1 = R * cos(2. * (i + 1) / cnt * M_PI);

		glNormal3d(yi, xi, 0);
		glVertex3d(x0, y0, z0);
		glVertex3d(x0, y0, z1);
	}
	glEnd();
}

void draw_circle(const ct::Vec3d &pt, double R, const ct::Vec4d &col = ct::Vec4d::zeros())
{
	const int cnt = 32;
	ct::Vec3d z1 = pt, p1;

	glColor4dv(col.val);
	glBegin(GL_TRIANGLE_STRIP);
	for(int i = 0; i <= cnt; i++){
		glVertex3dv(z1.val);

		double id = 1. * i / cnt * 2 * M_PI;
		double x = R * sin(id);
		double y = R * cos(id);

		p1 = pt + ct::Vec3d(x, y, 0);
		glVertex3dv(p1.val);
	}
	glEnd();
}

////////////////////////////////////

GLView::GLView(QWidget *parent) :
	QGLWidget(parent),
	ui(new Ui::GLView)
  , m_init(false)
  , m_update(false)
  , m_delta_z(0)
  , m_current_z(0)
  , m_color_space(ct::Vec3d::ones())
  , m_tracking(false)
  , m_tracking_angle(0)
  , m_show_route(false)
  , m_prev_e_track(0)
  , m_prev_u(0)
  , m_timer_goal(0)
{
	ui->setupUi(this);

	connect(&m_timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
	m_timer.start(30);

	connect(&m_timer_model, SIGNAL(timeout()), this, SLOT(onTimeoutModel()));
	m_timer_model.start(m_model.dt() * 1000.);

	if(m_model.open("data/quad.obj")){
		qDebug() << "obj opened";
	}else{
		qDebug() << "obj not opened";
	}

	load_xml();

	setMouseTracking(true);

#ifdef _MSC_VER
	QGLFormat newFormat = format();
	newFormat.setSampleBuffers(true);
	newFormat.setSamples(4);
	setFormat(newFormat);
#endif
}

GLView::~GLView()
{
	delete ui;

	save_xml();
}

Model &GLView::model()
{
	return m_model;
}

ModelRoute &GLView::modelRoute()
{
	return m_modelRoute;
}

void GLView::set_yaw(float v)
{
	m_angles[2] = ct::angle2rad(v);
	m_model.setYaw(v);
	set_update();
}

void GLView::set_tangage(float v)
{
	m_angles[0] = ct::angle2rad(v);
	m_model.setTangage(v);
	set_update();
}

void GLView::set_roll(float v)
{
	m_angles[1] = ct::angle2rad(v);
	m_model.setRoll(v);
	set_update();
}

float GLView::yaw() const
{
	return ct::rad2angle(m_angles[2]);
}

float GLView::tangage() const
{
	return ct::rad2angle(m_angles[0]);
}

float GLView::roll() const
{
	return ct::rad2angle(m_angles[1]);
}

void GLView::set_force(float f)
{
	m_model.set_force(f);
	set_update();
}

void GLView::setHeightGoal(float h)
{
	m_model.setHeightGoal(h);
	set_update();
}

void GLView::set_tracking(bool v)
{
	m_tracking = v;
	set_update();
}

void GLView::set_update()
{
	m_update = true;
}

void GLView::generate_route(size_t count)
{
	m_modelRoute.generate_route_uniform(count,
										ct::Vec3f(-virtual_xy_edge/1.5, -virtual_xy_edge/1.5, virtual_z_edge/100.f),
										ct::Vec3f(virtual_xy_edge/1.5, virtual_xy_edge/1.5, virtual_z_edge/2));
	set_update();
}

void GLView::setShowRoute(bool v)
{
	m_show_route = v;
	set_update();
}

void GLView::init()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_POINT_SMOOTH);

	glFrontFace(GL_FRONT);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GLView::draw_net()
{
	const int count = 100;

	static GLVertex3f ptLineX[count][2] = {0}, ptLineY[count][2] = {0};
	static bool isInit = false;

	if(!isInit){
		isInit = true;

		const double width = 100;
		for(int i = 0; i < count; i++){
			double it = (double)i/count;
			double x = -width/2 + width * it;

			ptLineX[i][0][0] = x;
			ptLineX[i][0][1] = -width/2;
			ptLineX[i][1][0] = x;
			ptLineX[i][1][1] = width/2;

			ptLineY[i][0][1] = x;
			ptLineY[i][0][0] = -width/2;
			ptLineY[i][1][1] = x;
			ptLineY[i][1][0] = width/2;
		}

	}

	glEnableClientState(GL_VERTEX_ARRAY);

	glColor3f(0, 0, 0);
	glVertexPointer(3, GL_FLOAT, 0, ptLineX);
	glDrawArrays(GL_LINES, 0, count * 2);

	glVertexPointer(3, GL_FLOAT, 0, ptLineY);
	glDrawArrays(GL_LINES, 0, count * 2);

	glDisableClientState(GL_VERTEX_ARRAY);
}

void GLView::draw_model()
{
	if(!m_model.vobjs().size())
		return;

	glPushMatrix();

	ct::Vec3d pos = m_model.pos();
	ct::Matd eiler = m_model.eiler();
	glTranslatef(pos[0], pos[1], pos[2]);

//	draw_line(m_model.direction_force(), ct::Vec3d(1, 0.3, 0.3), 1);

	glMultMatrixd(eiler.ptr());

//	ct::Matf mat = ct::get_eiler_mat4<float>(m_angles);

//	glMultMatrixf(mat.ptr());

	glRotatef(90, 1, 0, 0);

	glScalef(0.1f, 0.1f, 0.1f);

	glColor3f(0.5f, 0.5f, 0.5f);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	const float diffuse_material[] = {0.3f, 0.3f, 0.3f, 1.f};
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_material);

	for(auto it = m_model.vobjs().begin(); it != m_model.vobjs().end(); it++){
		const VObj &obj = *it;

		if(!obj.v.size())
			continue;

		const std::map< std::string, Mtl > &mtls = obj.mtls;

		const std::vector< ct::Vec3d> &_v = obj.v;
		const std::vector< ct::Vec3d> &_vn = obj.vn;

		for(auto itf = obj.faces.begin(); itf != obj.faces.end(); itf++){

			const VObj::Faces& faces = (*itf).second;

			auto mtlIt = mtls.find((std::string)faces.usemtl);
			if(mtlIt != mtls.end()){
				const Mtl mtl = (*mtlIt).second;

				glMaterialfv(GL_FRONT, GL_DIFFUSE, mtl.Kd.val);
				glMaterialfv(GL_FRONT, GL_AMBIENT, mtl.Ka.val);
				glMaterialfv(GL_FRONT, GL_SPECULAR, mtl.Ks.val);
			}

			const std::vector< std::vector< int > > & fv = faces.fv;
			const std::vector< std::vector< int > > & fn = faces.fn;

			for(size_t i = 0; i < fv.size(); ++i){

				const std::vector< int > &fvi = fv[i];
				const std::vector< int > &fni = fn[i];

				for(size_t j = 0; j < fvi.size() - 1; j++){
					glBegin(GL_TRIANGLES);

					int iv[] = {fvi[0] - 1,
						fvi[j] - 1,
						fvi[j + 1] - 1};

					int in[] = {fni[0] - 1,
						fni[j] - 1,
						fni[j + 1] - 1};
					for(size_t l = 0; l < 3; ++l){
						const ct::Vec3d& v = _v[iv[l]];
						const ct::Vec3d& vn = _vn[in[l]];
						glNormal3dv(vn.val);
						glVertex3dv(v.val);
					}

					glEnd();
				}
			}
		}
	}
	glDisable(GL_LIGHTING);

	glPopMatrix();
}

void GLView::draw_route()
{
	if(!m_modelRoute.count())
		return;

	glLineWidth(3);

	const std::vector< ct::Vec3f > &pts = m_modelRoute.points();

	glEnable(GL_BLEND);

	glColor3f(0.8f, 0.5f, 0.7f);

	glBegin(GL_LINE_STRIP);
	for(size_t i = 0; i < pts.size(); i++){
		glVertex3fv(pts[i].val);
	}
	glEnd();

	glPointSize(10);

	glBegin(GL_POINTS);
	for(size_t i = 1; i < pts.size() - 1; i++){
		glVertex3fv(pts[i].val);
	}
	glEnd();

	glBegin(GL_POINTS);
	glColor3f(0.1f, 0.7f, 0.1f);
	glVertex3fv(pts[0].val);
	glColor3f(0.7f, 0.1f, 0.1f);
	glVertex3fv(pts[pts.size() - 1].val);
	glEnd();

	glDisable(GL_BLEND);

	glLineWidth(1);
}

void GLView::draw_goal()
{
	using namespace ct;

	Vec3d p1 = m_model.goal_point();
	Vec3d p2 = p1;
	p2[2] = 0;

	glPointSize(20);

	glColor3f(1.f, 0.3f, 0.1f);

	glEnable(GL_BLEND);

	glBegin(GL_POINTS);
	glVertex3dv(p1.val);
	glVertex3dv(p2.val);
	glEnd();

	glPointSize(1);

	Vec4d col(0.8, 0.5, 0.3, 0.5 + 0.35 * sin(m_timer_goal));

	draw_circle(p2, 2, col);
	glDisable(GL_BLEND);

	m_timer_goal += 0.1;
}

void GLView::load_xml()
{
	QMap< QString, QVariant > params;

	if(!SimpleXML::load_param(xml_config, params))
		return;

	m_delta_pt.setX(params["rotate_x"].toFloat());
	m_delta_pt.setY(params["rotate_y"].toFloat());

	m_current_z = params["offset_z"].toFloat();

	m_color_space[0] = params["color_space_r"].toFloat();
	m_color_space[1] = params["color_space_g"].toFloat();
	m_color_space[2] = params["color_space_b"].toFloat();

//	m_angles[0] = params["tangage"].toFloat();
//	m_angles[1] = params["roll"].toFloat();
	m_angles[2] = ct::angle2rad(params["yaw"].toFloat());

	m_model.setYaw(params["yaw"].toFloat());

	m_model.setYawGoal(params["goal_yaw"].toFloat());
}

void GLView::save_xml()
{
	QMap< QString, QVariant > params;

	params["rotate_x"] = m_delta_pt.x();
	params["rotate_y"] = m_delta_pt.y();

	params["offset_z"] = m_current_z;

	params["color_space_r"] = m_color_space[0];
	params["color_space_g"] = m_color_space[1];
	params["color_space_b"] = m_color_space[2];

//	params["tangage"] = m_angles[0];
//	params["roll"] = m_angles[1];
	params["yaw"] = ct::rad2angle(m_angles[2]);
	params["goal_yaw"] = m_model.yawGoal();

	SimpleXML::save_param(xml_config, params);
}

void GLView::calculate_track()
{
//		ct::Vec3f ps = m_model.pos();
	ct::Vec3d dm = m_model.direct_model();

	const double kp = 0.1;
	const double kd = 0.3;

	dm[2] = 0;
	dm /= dm.norm();
	double angle = M_PI/2 - atan2(dm[1], dm[0]);
	double e = angle - ct::angle2rad(m_tracking_angle);
	e = atan2(sin(e), cos(e));

	double de = e - m_prev_e_track;
	m_prev_e_track = e;
	de = atan2(sin(de), cos(de));

	double u = kp * e + kd * de;
	u = ct::rad2angle(u);

	double avg_u = 0.5 * (m_prev_u + u);
	m_prev_u = avg_u;

	m_tracking_angle += avg_u;

//		gluLookAt(ps[0], ps[1], ps[2], ps[0] + dm[0], ps[1] + dm[1], ps[2] + dm[2], 0, 0, 1);
}


void GLView::updateGL()
{
}

void GLView::onTimeout()
{
	if(m_update || m_model.is_dynamic()){
		m_update = false;
		update();
	}

	if(m_model.is_log_exists()){
		while(m_model.is_log_exists()){
			emit push_logs(QString::fromStdString(m_model.pop_log()));
		}
	}
}

void GLView::onTimeoutModel()
{
	if(m_model.is_dynamic()){
		m_model.calulcate();
	}
}

void GLView::resizeGL(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60., (double)w/h, 0.1, 500);
	glMatrixMode(GL_MODELVIEW);

	update();
}

void GLView::paintGL()
{
}

void GLView::glDraw()
{
	if(!m_init){
		m_init = true;
		init();
	}

	makeCurrent();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(m_color_space[0], m_color_space[1], m_color_space[2], 1);
	glLoadIdentity();

	const float poslight[] = {10, -100, 100, 1};
	const float alight[] = {0.0f, 0.0f, 0.0f, 1};
	const float slight[] = {0.0f, 0.0f, 0.0f, 1};
	const float dlight[] = {0.3f, 0.3f, 0.3f, 1};

	glTranslatef(0, 0, -5);

	if(!m_tracking){
		glTranslatef(0, 0, -(m_current_z + m_delta_z));

		glRotatef(m_delta_pt.x(), 0, 1, 0);
		glRotatef(m_delta_pt.y(), 1, 0, 0);
	}else{
	}


	if(m_tracking){
		calculate_track();

		glTranslatef(0, 0, - (m_current_z + m_delta_z));
		glRotatef(m_delta_pt.y(), 1, 0, 0);
		glRotatef(m_delta_pt.x(), 0, 0, 1);
		glRotatef(m_tracking_angle, 0, 0, 1);
		glTranslatef(-m_model.pos()[0], -m_model.pos()[1], -m_model.pos()[2]);
	}

	glLightfv(GL_LIGHT0, GL_POSITION, poslight);
	glLightfv(GL_LIGHT0, GL_AMBIENT, alight);
	glLightfv(GL_LIGHT0, GL_SPECULAR, slight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, dlight);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 10);

	draw_net();

	draw_model();

	if(m_show_route){
		draw_route();
	}

	if(m_model.isTrackToGoalPoint()){
		draw_goal();
	}

	glEnable(GL_BLEND);
	draw_cylinder(virtual_xy_edge, 1, 64, ct::Vec4d(0.8, 0.4, 0.1, 0.5));
	glDisable(GL_BLEND);

	swapBuffers();
}


void GLView::mousePressEvent(QMouseEvent *event)
{
	m_mouse_pt = event->pos();

	m_left_down = event->buttons().testFlag(Qt::LeftButton);
	m_wheel_down = event->buttons().testFlag(Qt::MiddleButton);

	m_wheel_pt = event->pos();

	m_delta_z = 0;
}

void GLView::mouseReleaseEvent(QMouseEvent *event)
{
	m_mouse_pt = event->pos();

	m_left_down = false;
	m_wheel_down = false;

	m_current_z += m_delta_z;
	m_delta_z = 0;
}

void GLView::mouseMoveEvent(QMouseEvent *event)
{
	if(event->buttons().testFlag(Qt::LeftButton)){
		QPointF pt = event->pos() - m_mouse_pt;
		m_delta_pt += pt;
		m_mouse_pt = event->pos();

		set_update();
	}
	if(event->buttons().testFlag(Qt::MiddleButton)){
		QPointF pt = event->pos() - m_wheel_pt;

		m_delta_z = pt.y()/5.;

		set_update();
	}
}
