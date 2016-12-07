#include "model.h"

#include <iostream>
#include <ctime>

using namespace ct;
using namespace std;

const float gravity = 9.8f;
const int maximum_logs = 1000;

const float attenuation = 0.9;

Model::Model()
	: m_useSimpleHeightControl(0)
	, m_heightGoal(8)
	, m_useMultipleForces(false)
	, m_direct_model(1, 0, 0)
{
	m_mass = 1;
	m_kp_vel = 1;
	m_kd_vel = 1;
	m_kp_angle = 1;
	m_kd_angle = 1;
	m_dt = 1.f/100;
	m_force = 0;
	m_max_force = 15.;

	prev_e = 0;
	e_I = 0;

	m_force_1 = 0;
	m_force_2 = 0;
	m_force_3 = 0;
	m_force_4 = 0;
	m_arm = 0.2;
}

void Model::calulcate()
{
	Vec3f force_direction;

	state_model_angles();
	state_model_position(force_direction);

	calculate_angles();
	simpleHeightControl(force_direction);

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

float Model::tangageFloat() const
{
	return rad2angle(m_angles_goal[0]);
}

void Model::setTangageGoal(float v)
{
	m_angles_goal[0] = angle2rad(v);
}

float Model::rollGoal() const
{
	return rad2angle(m_angles_goal[1]);
}

void Model::setRollGoal(float v)
{
	m_angles_goal[1] = angle2rad(v);
}

float Model::yawGoal() const
{
	return rad2angle(m_angles_goal[2]);
}

void Model::setYawGoal(float v)
{
	m_angles_goal[2] = angle2rad(v);
}

void Model::setForces(float f1, float f2, float f3, float f4)
{
	if(f1 < 0) f1 = 0;
	if(f2 < 0) f2 = 0;
	if(f3 < 0) f3 = 0;
	if(f4 < 0) f4 = 0;
	m_force_1 = f1;
	m_force_2 = f2;
	m_force_3 = f3;
	m_force_4 = f4;
}

void Model::setForce(int index, float v)
{
	if(v < 0)
		v = 0;
	switch (index) {
		case 1:
			m_force_1 = v;
			break;
		case 2:
			m_force_2 = v;
			break;
		case 3:
			m_force_3 = v;
			break;
		case 4:
			m_force_4 = v;
			break;
		default:
			break;
	}
}

void Model::setUseMultipleForces(bool f)
{
	m_useMultipleForces = f;
	m_force = 0.0001;
}

float Model::force(int index)
{
	if(index == 1)
		return m_force_1;
	if(index == 2)
		return m_force_2;
	if(index == 3)
		return m_force_3;
	if(index == 4)
		return m_force_4;
}

void Model::reset_angles()
{
	m_angles = Vec3f::zeros();
	m_force_1 = 0;
	m_force_2 = 0;
	m_force_3 = 0;
	m_force_4 = 0;

	m_pos = Vec3f::zeros();
}

void Model::setYaw(float v)
{
	m_angles[2] = angle2rad(v);
}

void Model::setRoll(float v)
{
	m_angles[1] = angle2rad(v);
}

void Model::setTangage(float v)
{
	m_angles[0] = angle2rad(v);
}

Vec3f Model::direction_force() const
{
	return m_direction_force;
}

Vec3f Model::direct_model() const
{
	return m_direct_model;
}

void Model::calculate_angles()
{
	if(!m_useMultipleForces)
		return;

	/// [0] - tangage
	/// [1] - roll
	/// [2] - yaw
	ct::Vec3f e = m_angles_goal - m_angles;
	e = crop_angles(e);

	const float kp = 30;
	const float kd = 50;

	Vec3f de = e - prev_angles_e;
	de = crop_angles(de);
	prev_angles_e = e;

	Vec3f u = e * kp + de * kd;

	//u = sign(u) * (u * u);
	u *= m_mass * m_dt;

	float avg_f = m_force;
	/// f1 + f3 -> -tangage
	///	f2 + f4 -> +tangage
	///	f1 + f4 -> -roll
	/// f3 + f2 -> +roll
	/// f1 + f2 > f2 + f4 -> +yaw
	/// f1 + f2 < f2 + f4 -> -yaw

	u /= 4;

	m_force_1 = avg_f/4 + u[0] - u[1] + u[2];
	m_force_3 = avg_f/4 + u[0] + u[1] - u[2];
	m_force_2 = avg_f/4 - u[0] + u[1] + u[2];
	m_force_4 = avg_f/4 - u[0] - u[1] - u[2];

	float part_f = m_force_1 + m_force_2 + m_force_3 + m_force_4;
	float pf1 = m_force_1/part_f;
	float pf2 = m_force_2/part_f;
	float pf3 = m_force_3/part_f;
	float pf4 = m_force_4/part_f;

	m_force_1 = pf1 * m_force;
	m_force_2 = pf2 * m_force;
	m_force_3 = pf3 * m_force;
	m_force_4 = pf4 * m_force;

//	m_force_1 = /*avg_f*/ - u[1]/4;
//	m_force_4 = /*avg_f*/ - u[1]/4;
//	m_force_3 = /*avg_f*/ + u[1]/4;
//	m_force_2 = /*avg_f*/ + u[1]/4;

//	m_force_1 = /*avg_f*/ - u[2]/4;
//	m_force_2 = /*avg_f*/ - u[2]/4;
//	m_force_4 = /*avg_f*/ + u[2]/4;
//	m_force_2 = /*avg_f*/ + u[2]/4;

	m_force_1 = std::max(0.f, m_force_1);
	m_force_2 = std::max(0.f, m_force_2);
	m_force_3 = std::max(0.f, m_force_3);
	m_force_4 = std::max(0.f, m_force_4);

	m_force_1 = std::min(m_max_force, m_force_1);
	m_force_2 = std::min(m_max_force, m_force_2);
	m_force_3 = std::min(m_max_force, m_force_3);
	m_force_4 = std::min(m_max_force, m_force_4);
//	u /= (m_mass * m_arm);
//	Vec3f w = sign(u) * sqrt(u);

//	Vec3f arm_1(m_arm, m_arm, 0);
//	Vec3f arm_2(-m_arm, -m_arm, 0);
//	Vec3f arm_3(m_arm, -m_arm, 0);
//	Vec3f arm_4(-m_arm, m_arm, 0);


}

void Model::state_model_angles()
{
	float df_12 = m_force_1 - m_force_2;
	float df_34 = m_force_3 - m_force_4;

	if(df_12 != 0 || df_34 != 0){

		float a_12 = df_12 / m_mass;
		float a_34 = df_34 / m_mass;

		/// a = F / m; a = w^2 * R
		float w_12 = sqrt(std::abs(a_12) / m_arm) * m_dt;
		if(a_12 < 0) w_12 = -w_12;
		float w_34 = sqrt(std::abs(a_34) / m_arm) * m_dt;
		if(a_34 < 0) w_34 = -w_34;
		float w_tan_12	= w_12 * sin(M_PI / 4);
		float w_roll_12 = w_12 * cos(M_PI / 4);
		float w_tan_34	= w_34 * sin(M_PI / 4);
		float w_roll_34 = w_34 * cos(M_PI / 4);

		m_angles[0] += w_tan_12 + w_tan_34;
		m_angles[1] += w_roll_34 - w_roll_12;

	}
	/// yaw = F1 + F2 - (F3 + F4)
	float df_1234 = m_force_1 + m_force_2 - m_force_3 - m_force_4;
	if(df_1234){
		float a1234 = df_1234 / m_mass;
		float w_1234 = sqrt(std::abs(a1234) / m_arm) * m_dt;
		if(a1234 < 0) w_1234 = -w_1234;
		m_angles[2] += w_1234;
	}

	m_force = m_force_1 + m_force_2 + m_force_3 + m_force_4;
}

void Model::state_model_position(Vec3f &force_direction)
{
	Vec3f vel(0, 0, 1);
	Vec3f direct_model(1, 0, 0);

	Matf m = get_eiler_mat(m_angles);
	m = m.t();

	//push_log("eiler:\n" + m.operator std::string());

	Matf mv = m * vel;

	//push_log("mult to vec:\n" + mv.operator std::string());

	vel = mv.toVec<3>();
	m_direction_force = vel;

	force_direction = vel;

	mv = m * direct_model;

	m_direct_model = mv.toVec<3>();
	//push_log("from mat: " + vel.operator std::string());
	vel *= (m_force/m_mass * m_dt);

	m_vel += vel;

	m_vel += Vec3f(0.f, 0.f, -m_mass * gravity * m_dt);

	m_vel *= attenuation;

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

	const float kp = 1.1;
	const float kd = 20.0;
	const float ki = 0.005;

	e_I += e;

	float de = e - prev_e;
	prev_e = e;

	float u = kp * e + gravity + kd * de;

	u = std::max(0.f, std::min(m_max_force, u));

	float force = m_mass * u;

	float part_f = m_force_1 + m_force_2 + m_force_3 + m_force_4;

	if(part_f > 0){
		float pf1 = m_force_1 / part_f;
		float pf2 = m_force_2 / part_f;
		float pf3 = m_force_3 / part_f;
		float pf4 = m_force_4 / part_f;

		m_force_1 = pf1 * force;
		m_force_2 = pf2 * force;
		m_force_3 = pf3 * force;
		m_force_4 = pf4 * force;
	}else{
		m_force_1 = force / 4;
		m_force_2 = force / 4;
		m_force_3 = force / 4;
		m_force_4 = force / 4;
	}
}
