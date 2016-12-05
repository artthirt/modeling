#include "glview.h"
#include "ui_glview.h"

#ifdef _MSC_VER
#include <Windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>

#include <QMouseEvent>
#include <QDebug>

typedef float GLVertex3f[3];

GLView::GLView(QWidget *parent) :
	QGLWidget(parent),
	ui(new Ui::GLView)
  , m_init(false)
  , m_update(false)
  , m_delta_z(0)
  , m_current_z(10)
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

	setMouseTracking(true);
}

GLView::~GLView()
{
	delete ui;
}

void GLView::set_yaw(float v)
{
	m_angles[2] = v;
	m_update = true;
}

void GLView::set_tangage(float v)
{
	m_angles[0] = v;
	m_update = true;
}

void GLView::set_roll(float v)
{
	m_angles[1] = v;
	m_update = true;
}

void GLView::set_force(float f)
{
	m_model.set_force(f);
	m_update = true;
}

void GLView::setUseHeightGoal(bool val)
{
	m_model.setSimpleHeightControl(val);
}

void GLView::setHeightGoal(float h)
{
	m_model.setHeightGoal(h);
	m_update = true;
}

void GLView::init()
{
	glEnable(GL_DEPTH_TEST);
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

	glMultMatrixf(eiler.ptr());

	ct::Matf mat = ct::get_eiler_mat4<float>(m_angles);

	glMultMatrixf(mat.ptr());

	glRotatef(90, 1, 0, 0);

	glColor3f(0.5, 0.5, 0.5);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

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
	glClearColor(1, 1, 1, 1);
	glLoadIdentity();
	glTranslatef(0, 0, -10);

	glTranslatef(0, 0, -(m_current_z + m_delta_z));

	glRotatef(m_delta_pt.x(), 0, 1, 0);
	glRotatef(m_delta_pt.y(), 1, 0, 0);

	draw_net();
	draw_model();

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

		m_update = true;
	}
	if(event->buttons().testFlag(Qt::MiddleButton)){
		QPointF pt = event->pos() - m_wheel_pt;

		m_delta_z = pt.y();

		m_update = true;
	}
}
