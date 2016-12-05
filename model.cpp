#include "model.h"

#include <iostream>
#include <ctime>

using namespace ct;
using namespace std;

const float gravity = 9.8f;
const int maximum_logs = 1000;

const float virtual_z_edge = 50.;
const float virtual_xy_edge = 15.;

Model::Model()
	: m_useSimpleHeightControl(0)
	, m_heightGoal(8)

{
	m_mass = 1;
	m_kp_vel = 1;
	m_kd_vel = 1;
	m_kp_angle = 1;
	m_kd_angle = 1;
	m_dt = 1.f/100;
	m_force = 0;
	m_max_force = 40.;

	prev_e = 0;
	e_I = 0;
}

void Model::calulcate()
{
	Vec3f vel(0, 0, 1);

	Matf m = get_eiler_mat(m_angles);

	//push_log("eiler:\n" + m.operator std::string());

	Matf mv = m * vel;

	//push_log("mult to vec:\n" + mv.operator std::string());

	vel = mv.toVec<3>();

	simpleHeightControl(vel);
	//push_log("from mat: " + vel.operator std::string());
	vel *= (m_force/m_mass * m_dt);

	m_vel += vel;

	m_vel += Vec3f(0.f, 0.f, -m_mass * gravity * m_dt);

	push_log("after force: " + m_vel.operator std::string());

	m_pos += m_vel;

	m_angles += m_angles_vel;


	if(m_pos[2] < 0){
		m_pos[2] = 0;
		m_vel = Vec3f::zeros();
		cout << "break\n";
	}

	if(m_pos[2] > virtual_z_edge){
		m_pos[2] = virtual_z_edge;
	}
	Vec3f f = m_pos;
	f[2] = 0;
	if(f.norm() > virtual_xy_edge){
		f /= f.norm();
		f *= virtual_xy_edge;
		m_pos[0] = f[0];
		m_pos[1] = f[1];
	}

}

void Model::initialize()
{
	m_force = 0;
	m_pos = Vec3f();
	m_angles = Vec3f();
	m_angles_vel = Vec3f();
	prev_e = 0;
	e_I = 0;
}

bool Model::open(const string &fn)
{
	return m_container.open(fn);
}

VObjContainer &Model::container()
{
	return m_container;
}

const std::deque<VObj> &Model::vobjs() const
{
	return m_container.vobjs();
}

const Vec3f &Model::pos() const
{
	return m_pos;
}

Matf Model::eiler() const
{
	return get_eiler_mat4(m_angles);
}

void Model::set_force(float force)
{
	m_force = force;
}

float Model::force() const
{
	return m_force;
}

bool Model::is_dynamic() const
{
	return m_force != 0 || m_pos.val[2] != 0;
}

float Model::dt() const
{
	return m_dt;
}

bool Model::is_log_exists() const
{
	return m_logs.size();
}

string Model::pop_log()
{
	std::string ret = m_logs.front();
	m_logs.pop_front();
	return ret;
}

void Model::push_log(const string &str)
{
	while(m_logs.size() > maximum_logs){
		m_logs.pop_front();
	}
	time_t t;
	time(&t);
	tm *tt = localtime(&t);
	const char* c = asctime(tt);
	string res = c;
	res.erase(res.end() - 1);
	res += ": " + str;
	m_logs.push_back(res);
}

void Model::setSimpleHeightControl(bool val)
{
	m_useSimpleHeightControl = val;
	m_force = 0.0001;
}

void Model::setHeightGoal(float h)
{
	m_heightGoal = h;
}

void Model::simpleHeightControl(const Vec3f &normal)
{
	if(!m_useSimpleHeightControl)
		return;

	if(normal[2] < 0){
		push_log("very bad. quad headfirst");
		return;
	}

	float e = m_heightGoal - m_pos[2];
//	Vec3f vec_force = normal * m_force;
//	float force = vec_force[2];

	const float kp = 1.0;
	const float kd = 20.0;
	const float ki = 0.005;

	e_I += e;

	float de = e - prev_e;
	prev_e = e;

	float u = kp * e + gravity + kd * de;

	u = std::max(0.f, std::min(m_max_force, u));

	m_force = m_mass * u;
}
