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

void draw_line(const ct::Vec3f& v, const ct::Vec3f & col = ct::Vec3f::ones(), float scale = 1.)
{
	glLineWidth(3);

	glPushMatrix();

	glScalef(scale, scale, scale);

	glColor3fv(col.ptr());
	glBegin(GL_LINES);
	glVertex3fv(ct::Vec3f::zeros().ptr());
	glVertex3fv(v.ptr());
	glEnd();

	glPopMatrix();

	glLineWidth(1);
}

void draw_cylinder(float R, float H, int cnt = 10, const ct::Vec3f& col = ct::Vec4f::ones())
{
	float z0 = 0;
	float z1 = H;

	glColor4fv(col.ptr());

	glBegin(GL_TRIANGLE_STRIP);
	for(int i = 0; i <= cnt; i++){
		float xi = sin(2. * i / cnt * M_PI);
		float yi = cos(2. * i / cnt * M_PI);
		float x0 = R * xi;
		float y0 = R * yi;

//		x1 = R * sin(2. * (i + 1) / cnt * M_PI);
//		y1 = R * cos(2. * (i + 1) / cnt * M_PI);

		glNormal3f(yi, xi, 0);
		glVertex3f(x0, y0, z0);
		glVertex3f(x0, y0, z1);
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
  , m_color_space(ct::Vec3f::ones())
  , m_tracking(false)
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

void GLView::setUseHeightGoal(bool val)
{
	m_model.setSimpleHeightControl(val);
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

void GLView::init()
{
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

	ct::Vec3f pos = m_model.pos();
	ct::Matf eiler = m_model.eiler();
	glTranslatef(pos[0], pos[1], pos[2]);

	draw_line(m_model.direction_force(), ct::Vec3f(1, 0.3, 0.3), 1);

	glMultMatrixf(eiler.ptr());

//	ct::Matf mat = ct::get_eiler_mat4<float>(m_angles);

//	glMultMatrixf(mat.ptr());

	glRotatef(90, 1, 0, 0);

	glScalef(0.3, 0.3, 0.3);

	glColor3f(0.5, 0.5, 0.5);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	const float diffuse_material[] = {0.3, 0.3, 0.3, 1};
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_material);

	for(auto it = m_model.vobjs().begin(); it != m_model.vobjs().end(); it++){
		const VObj &obj = *it;

		if(!obj.v.size())
			continue;

		const std::vector< ct::Vec3f> &_v = obj.v;
		const std::vector< ct::Vec3f> &_vn = obj.vn;

		for(auto itf = obj.faces.begin(); itf != obj.faces.end(); itf++){

			const VObj::Faces& faces = *itf;

			const std::vector< std::vector< int > > & fv = faces.fv;
			const std::vector< std::vector< int > > & fn = faces.fn;

			for(size_t i = 0; i < fv.size(); ++i){

				const std::vector< int > &fvi = fv[i];
				const std::vector< int > &fni = fn[i];

				glBegin(GL_POLYGON);
				for(size_t j = 0; j < fvi.size(); ++j){
					const ct::Vec3f& v = _v[fvi[j] - 1];
					const ct::Vec3f& vn = _vn[fni[j] - 1];

					glNormal3fv(vn.val);
					glVertex3fv(v.val);
				}
				glEnd();
			}
		}
	}
	glDisable(GL_LIGHTING);

	glPopMatrix();
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
	gluPerspective(45., (double)w/h, 1, 100);
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

	const float poslight[] = {0, 0, 10, 1};

	glLightfv(GL_LIGHT0, GL_POSITION, poslight);

	glTranslatef(0, 0, -5);

	if(!m_tracking){
		glTranslatef(0, 0, -(m_current_z + m_delta_z));

		glRotatef(m_delta_pt.x(), 0, 1, 0);
		glRotatef(m_delta_pt.y(), 1, 0, 0);
	}else{
	}


	if(m_tracking){
//		ct::Vec3f ps = m_model.pos();
//		ct::Vec3f dm = m_model.direct_model();
//		gluLookAt(ps[0], ps[1], ps[2], ps[0] + dm[0], ps[1] + dm[1], ps[2] + dm[2], 0, 0, 1);
		glTranslatef(0, 0, - (m_current_z + m_delta_z));
		glRotatef(m_delta_pt.x(), 0, 1, 0);
		glRotatef(m_delta_pt.y(), 1, 0, 0);
		glTranslatef(-m_model.pos()[0], -m_model.pos()[1], -m_model.pos()[2]);
	}

	draw_net();

	draw_model();

	glEnable(GL_BLEND);
	draw_cylinder(virtual_xy_edge, 0.3, 16, ct::Vec4f(0.8, 0.4, 0.1, 0.5));
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

		m_delta_z = pt.y();

		set_update();
	}
}
