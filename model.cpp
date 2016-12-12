#include "model.h"

#include <iostream>
#include <ctime>
#include <iostream>
#include <fstream>

#include <simple_xml.hpp>

using namespace ct;
using namespace std;

const double gravity = 9.8f;
const int maximum_logs = 1000;

const double attenuation = 0.85f;

/// coefficient friction of oac-tree (for example)
const double coeff_friction = 0.62f;
/// for weak epsilon weak
const double eps = 1e-3;
/// for hard epsilon weak
const double eps_hard = 1e-6;
/// minumum angle for choose state
const double angle_for_chose = angle2rad(5);


const QString config_file("config.model.xml");

////////////////////////////////////////////////

string fromFloat(double v)
{
	stringstream ss;
	ss << v;
	return ss.str();
}

template< typename T >
T sigmoid(const T &val)
{
	T res = exp(-val);
	return 1. / (1. + res);
}

////////////////////////////////////////////////

enum WORK_MODE{
	NORMAL,					/// normal mode
	ROTATE_TO_HOVER			/// if got to ground
};

Model::Model()
	: m_Eheight_control(ENone)
	, m_heightGoal(8)
	, m_useMultipleForces(false)
	, m_direct_model(1, 0, 0)
	, m_roll_model(0, 1, 0)
	, m_power(false)
	, m_track_to_goal_point(false)
	, m_goal_point(10, 10, 10)
	, m_radius_goal(0.1)
	, m_state(NORMAL)
	, m_use_eI_height(false)
	, m_search_hover(false)
	, m_found_hover(false)
	, m_goal_vert_vel(0)
{
	m_mass = 1;
	m_kp_vel = 1;
	m_kd_vel = 1;
	m_kp_angle = 1;
	m_kd_angle = 1;
	m_dt = 1.f/100;
	m_force = 0;
	m_max_force = 15.;

//	m_prev_e = 0;
//	m_e_I = 0;

//	m_eI_height = 0;
//	m_prev_e_height = 0;

	m_force_1 = 0;
	m_force_2 = 0;
	m_force_3 = 0;
	m_force_4 = 0;
	m_arm = 0.2f;

	m_force_hover = 0;

	load_params();

	if(m_control_angles.eI().empty())
		m_use_integral_angles = true;
	else
		m_use_integral_angles = false;
}

Model::~Model()
{
	save_params();
}

void Model::calulcate()
{
	state_model_angles();
	state_model_position();

	calculate_angles();


	switch (m_Eheight_control) {
		case EGoToToHeight:
			simpleHeightControl();
			break;
		case EHover:
			search_hover();
			calculate_hovering();
			break;
		default:
			break;
	}

	calculate_track_to_goal();
}

void Model::setHeightControl(Model::EHeightControl hc)
{
	m_Eheight_control = hc;
}

void Model::initialize()
{
	m_force = 0;
	m_pos = Vec3d();
	m_angles = Vec3d();
	m_angles_vel = Vec3d();
	m_control_angles.reset();
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

const Vec3d &Model::pos() const
{
	return m_pos;
}

Vec3d Model::velocity() const
{
	ct::Vec3d v = m_pos - m_prev_pos;
	v /= m_dt;
	return v;
}

Matd Model::eiler() const
{
	return get_eiler_mat4(m_angles);
}

void Model::set_force(double force)
{
	m_force = force;
}

double Model::force() const
{
	return m_force;
}

bool Model::is_dynamic() const
{
	return m_pos.val[2] != 0 || m_power;
}

double Model::dt() const
{
	return m_dt;
}

bool Model::is_log_exists() const
{
	return m_logs.size() > 0;
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

void Model::setHeightGoal(double h)
{
	m_heightGoal = h;
}

double Model::tangageGoal() const
{
	return rad2angle(m_angles_goal[0]);
}

void Model::setTangageGoal(double v)
{
	m_angles_goal[0] = angle2rad(v);
}

double Model::rollGoal() const
{
	return rad2angle(m_angles_goal[1]);
}

void Model::setRollGoal(double v)
{
	m_angles_goal[1] = angle2rad(v);
}

double Model::yawGoal() const
{
	return rad2angle(m_angles_goal[2]);
}

void Model::setYawGoal(double v)
{
	m_angles_goal[2] = angle2rad(v);
}

void Model::setForces(double f1, double f2, double f3, double f4)
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

void Model::setForce(int index, double v)
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
}

double Model::force(int index)
{
	if(index == 1)
		return m_force_1;
	if(index == 2)
		return m_force_2;
	if(index == 3)
		return m_force_3;
	if(index == 4)
		return m_force_4;
	return 0;
}

void Model::reset_angles()
{
	m_angles = Vec3d::zeros();
	m_force_1 = 0;
	m_force_2 = 0;
	m_force_3 = 0;
	m_force_4 = 0;

	m_pos = Vec3d::zeros();
}

void Model::setYaw(double v)
{
	m_angles[2] = angle2rad(v);
}

void Model::setRoll(double v)
{
	m_angles[1] = angle2rad(v);
}

void Model::setTangage(double v)
{
	m_angles[0] = angle2rad(v);
}

double Model::roll() const
{
	Vec3d v = m_roll_model;
	v[2] = 0;
	double x = v.norm();
	double y = m_roll_model[2];
	return rad2angle(atan2(y, x));
}

double Model::tangage() const
{
	Vec3d v = m_direct_model;
	v[2] = 0;
	double x = v.norm();
	double y = m_direct_model[2];
	return rad2angle(atan2(y, x));
}

double Model::yaw() const
{
	Vec3d v = m_direct_model;
	v[2] = 0;
	return rad2angle(atan2(v[1], v[0]));
}

double Model::goal_roll() const
{
	return rad2angle(m_angles_goal[1]);
}

double Model::goal_tangage() const
{
	return rad2angle(m_angles_goal[0]);
}

double Model::goal_yaw() const
{
	return rad2angle(m_angles_goal[2]);
}


Vec3d Model::direction_force() const
{
	return m_direction_force;
}

Vec3d Model::direct_model() const
{
	return m_direct_model;
}

bool Model::isUseInegralErrorAngles() const
{
	return m_use_integral_angles;
}

void Model::setUseIntegralErrorAngles(bool v)
{
	m_use_integral_angles = v;
}

void Model::setUseIntegralErrorHeight(bool v)
{
	m_use_eI_height = v;
}

void Model::setPower(bool v)
{
	m_power = v;
}

bool Model::isPower() const
{
	return m_power;
}

void Model::calculate_angles()
{
	if(!m_useMultipleForces)
		return;

	/// [0] - tangage
	/// [1] - roll
	/// [2] - yaw

	const double kp = 30;
	const double ki = 0.7;
	const double kd = 70;

	m_control_angles.setKpid(kp, kd, ki);
	m_control_angles.use_eI = m_use_integral_angles;
	m_control_angles.setGoal(m_angles_goal);
	m_control_angles.crop_value = &crop_angles;

	Vec3d u = m_control_angles.get(m_angles);
	//u = sign(u) * (u * u);
	u *= m_mass * m_dt;

	double avg_f = m_force;
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

	double part_f = m_force_1 + m_force_2 + m_force_3 + m_force_4;
	double pf1 = m_force_1/part_f;
	double pf2 = m_force_2/part_f;
	double pf3 = m_force_3/part_f;
	double pf4 = m_force_4/part_f;

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

	m_force_1 = std::max(0., m_force_1);
	m_force_2 = std::max(0., m_force_2);
	m_force_3 = std::max(0., m_force_3);
	m_force_4 = std::max(0., m_force_4);

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

void Model::calculate_hovering()
{
	if(m_state == NORMAL){

		if(m_direction_force[2] < 0){
			m_state = ROTATE_TO_HOVER;
			setTangageGoal(0);
			return;
		}
		normal_work();
	}else{
		if(abs(tangage()) < angle_for_chose){
			m_state = NORMAL;
		}
	}
}

void Model::normal_work()
{
	const double kp = 1;
	const double ki = 0.1;
	const double kd = 1;

	double force = m_force_1 + m_force_2 + m_force_3 + m_force_4;

	if(force < eps && !m_found_hover)
		return;

	if(m_found_hover && force < eps){
		force = m_force_hover;
	}

	double pf1 = m_force_1 / force;
	double pf2 = m_force_2 / force;
	double pf3 = m_force_3 / force;
	double pf4 = m_force_4 / force;

	Vec3d v = velocity();

	//double e = m_pos[2] - m_heightGoal;
	m_control_vert_vel2.setGoal(m_goal_vert_vel);
	m_control_vert_vel2.setKpid(kp, kd, ki);
	m_control_vert_vel2.set_use_eI(m_use_eI_height);

	double u = m_control_vert_vel2.get(v[2]);//kp * e + ki * eI + kd * de;

	u = value2range(u, m_max_force);

	force += u;

	force = value2range(force, m_max_force);

	m_force_1 = pf1 * force;
	m_force_2 = pf2 * force;
	m_force_3 = pf3 * force;
	m_force_4 = pf4 * force;
}

void Model::search_hover()
{
	if(!m_search_hover)
		return;

	m_goal_vert_vel = 0;

	Vec3d v = velocity();

	const double default_height = 1;
	static double add_force = 0.1;

	double e = default_height - m_pos[2] + 0 - v[2];

	push_log("error=" + fromFloat(e));

	if(abs(e) < eps_hard){
		m_found_hover = true;

		m_force_hover = m_force_1 + m_force_2 + m_force_3 + m_force_4;
	}

	if(!m_found_hover){

		const double kp = 1;
		const double kd = 1;

		m_control_vert_vel.setKpid(kp, kd, 0);
		m_control_vert_vel.setGoal(default_height);
		double u = m_control_vert_vel.get(m_pos[2]);

		m_goal_vert_vel = u;
		m_goal_vert_vel = value2range(m_goal_vert_vel, add_force);

		if(m_pos[2] < default_height && v[2] <=0){
			m_force_1 += add_force;
			m_force_2 += add_force;
			m_force_3 += add_force;
			m_force_4 += add_force;
			add_force += 0.001;
		}
		if(m_pos[2] > default_height && v[2] >= 0){
			m_force_1 -= add_force;
			m_force_2 -= add_force;
			m_force_3 -= add_force;
			m_force_4 -= add_force;
			add_force /= 2;
		}
	}
}

void Model::state_model_angles()
{
	/// ************** if not power then not change angles
	if(!m_power)
		return;

	/// for some uncertainty that influence the each force
	const double coeff_f1 = 0.985;
	const double coeff_f2 = 0.983;
	const double coeff_f3 = 0.988;
	const double coeff_f4 = 0.982;

	double df_12 = coeff_f1 * m_force_1 - coeff_f2 * m_force_2;
	double df_34 = coeff_f3 * m_force_3 - coeff_f4 * m_force_4;

	if(df_12 != 0 || df_34 != 0){

		double a_12 = df_12 / m_mass;
		double a_34 = df_34 / m_mass;

		/// a = F / m; a = w^2 * R
		double w_12 = sqrt(std::abs(a_12) / m_arm) * m_dt;
		if(a_12 < 0) w_12 = -w_12;
		double w_34 = sqrt(std::abs(a_34) / m_arm) * m_dt;
		if(a_34 < 0) w_34 = -w_34;
		double w_tan_12	= w_12 * sin(M_PI / 4.);
		double w_roll_12 = w_12 * cos(M_PI / 4.);
		double w_tan_34	= w_34 * sin(M_PI / 4.);
		double w_roll_34 = w_34 * cos(M_PI / 4.);

		m_angles[0] += w_tan_12 + w_tan_34;
		m_angles[1] += w_roll_34 - w_roll_12;

	}
	/// yaw = F1 + F2 - (F3 + F4)
	double df_1234 = m_force_1 + m_force_2 - m_force_3 - m_force_4;
	if(df_1234){
		double a1234 = df_1234 / m_mass;
		double w_1234 = sqrt(std::abs(a1234) / m_arm) * m_dt;
		if(a1234 < 0) w_1234 = -w_1234;
		m_angles[2] += w_1234;
	}

	m_force = coeff_f1 * m_force_1 + coeff_f2 * m_force_2 + coeff_f3 * m_force_3 + coeff_f4 * m_force_4;
}

void Model::state_model_position()
{
	m_prev_pos = m_pos;

	Vec3d vel(0, 0, 1);
	Vec3d direct_model(1, 0, 0);
	Vec3d roll_model(0, 1, 0);

	Matd m = get_eiler_mat(m_angles);
	m = m.t();

	//push_log("eiler:\n" + m.operator std::string());

	Matd mv = m * vel;

	//push_log("mult to vec:\n" + mv.operator std::string());

	vel = mv.toVec<3>();
	m_direction_force = vel;

	/// get direct model
	mv = m * direct_model;
	m_direct_model = mv.toVec<3>();

	/// get roll model
	mv = m * roll_model;
	m_roll_model = mv.toVec<3>();
	//push_log("from mat: " + vel.operator std::string());

	///************* engine power on/off
	double force = m_power? m_force : 0;
	///*************

	vel *= (force/m_mass * m_dt);

	m_vel += vel;

	m_vel += Vec3d(0.f, 0.f, -m_mass * gravity * m_dt);

	m_vel *= attenuation;

	push_log("after force: " + m_vel.operator std::string());

	if(m_pos[2] + m_vel[2] <= 0){
		Vec3d v = m_vel;
		m_vel[2] = 0;
		double v_ground = v.norm();
		/// if the model on ground
		if(v_ground / m_dt < coeff_friction * gravity){
			m_vel[0] = 0;
			m_vel[1] = 0;
			m_angles[0] = 0;
			m_angles[1] = 0;
		}
	}

	m_pos += m_vel;

	m_angles += m_angles_vel;

	if(m_pos[2] < 0){
		m_pos[2] = 0;
		m_vel = Vec3d::zeros();
		cout << "break\n";
	}

	if(m_pos[2] > virtual_z_edge){
		m_pos[2] = virtual_z_edge;
	}
	Vec3d f = m_pos;
	f[2] = 0;
	if(f.norm() > virtual_xy_edge){
		f /= f.norm();
		f *= virtual_xy_edge;
		m_pos[0] = f[0];
		m_pos[1] = f[1];
	}
}

void Model::simpleHeightControl()
{
	if(m_state == ROTATE_TO_HOVER){
		/// rotate to vertical
		if(abs(tangage() < angle_for_chose)){
			m_state = NORMAL;
		}
	}else if(m_state == NORMAL){
		if(m_direction_force[2] < 0){
			push_log("very bad. quad headfirst");
			m_state = ROTATE_TO_HOVER;
			setTangageGoal(0);
			return;
		}

		normal_work_simple();
	}

}

void Model::normal_work_simple()
{
//	double e = m_heightGoal - m_pos[2];
//	Vec3f vec_force = normal * m_force;
//	float force = vec_force[2];

	const double kp = 1.1;
	const double kd = 20.0;
	const double ki = 0.1;

	m_control_height.setGoal(m_heightGoal);
	m_control_height.setKpid(kp, kd, ki);
	m_control_height.set_use_eI(m_use_eI_height);

//	m_e_I += e;

//	double de = e - m_prev_e;
//	m_prev_e = e;

	double u = m_control_height.get(m_pos[2]);

	u = std::max(0., std::min(m_max_force, u));

	double part_f = m_force_1 + m_force_2 + m_force_3 + m_force_4;
	double force = m_mass * u;

	if(part_f > 0){
		double pf1 = m_force_1 / part_f;
		double pf2 = m_force_2 / part_f;
		double pf3 = m_force_3 / part_f;
		double pf4 = m_force_4 / part_f;

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

void Model::load_params()
{
	QMap< QString, QVariant > params;
	if(!SimpleXML::load_param(config_file, params))
		return;

	m_control_angles.eI()[0] = params["angles"].toMap()["integral"].toMap()["tangage"].toDouble();
	m_control_angles.eI()[1] = params["angles"].toMap()["integral"].toMap()["roll"].toDouble();
	m_control_angles.eI()[2] = params["angles"].toMap()["integral"].toMap()["yaw"].toDouble();

	m_control_vert_vel2.eI() = params["height"].toMap()["integral_vel"].toDouble();
	m_control_height.eI() = params["height"].toMap()["integral_goal"].toDouble();
}

void Model::save_params()
{
	QMap< QString, QVariant > params, pang, pint;


	pint["tangage"]		= m_control_angles.eI()[0];
	pint["roll"]		= m_control_angles.eI()[1];
	pint["yaw"]			= m_control_angles.eI()[2];

	pang["integral"] = pint;

	params["angles"] = pang;

	pint.clear();
	pint["integral_vel"] = m_control_vert_vel2.eI();
	pint["integral_goal"] = m_control_height.eI();
	params["height"] = pint;

	SimpleXML::save_param(config_file, params);
}

void Model::setGoalPoint(const ct::Vec3d &pt)
{
	m_goal_point = pt;
}

ct::Vec3d Model::goal_point() const
{
	return m_goal_point;
}

void Model::setTrackToGoalPoint(bool v)
{
	m_track_to_goal_point = v;
}

bool Model::isTrackToGoalPoint() const
{
	return m_track_to_goal_point;
}

void Model::calculate_track_to_goal()
{
	if(!m_track_to_goal_point)
		return;

	Vec3d e = m_goal_point - m_pos;
	if(e.norm() < m_radius_goal && velocity().norm() * m_dt < eps)
		return;

	/// set goal height
	setHeightGoal(m_goal_point[2]);

	/// get angle between goal and direction of model

	Vec3d direct = e / e.norm();			/// direction to goal

	Vec3d p1 = m_direct_model, p2 = direct;
	p1[2] = 0, p2[2] = 0;					/// only Oxy plane

	if(p1.norm() > 0 && p2.norm() > 0){
		p1 /= p1.norm(), p2 /= p2.norm();	/// normalize
		Vec3d p3 = p1.cross(p2);			/// cross product for get left or right a side of direction

		double cosa = p1.dot(p2);			/// cos angle
		double sina = p1.cross(p2).norm();	/// sin angle

		double a = atan2(sina, cosa);		/// angle
		a = p3[2] > 0 ? a : -a;				/// direction of angle
		double sign = p3[2] > 0 ? 1 : -1;

		const double max_yaw_change = angle2rad(30.);
		const double max_other_change = angle2rad(10.);

		const double kp_y = 1;
		const double kd_y = 10;

		const double kp_tr = 0.5;
		const double kd_tr = 10;

		double e_yaw = a;
		double de_yaw = e_yaw - m_prev_goal_e[2];
		de_yaw = atan2(sin(de_yaw), cos(de_yaw));
		m_prev_goal_e[2] = e_yaw;

		double u = kp_y * e_yaw + kd_y * de_yaw;
		u *= m_dt;
		u = max(-max_yaw_change, min(max_yaw_change, u));
		u = atan2(sin(u), cos(u));
		m_angles_goal[2] -= u;

		double ta = cos(a);
		double ra = sin(a);

		Vec3d exy = e;
		exy[2] = 0;
		Vec3d velxy = velocity();
		velxy[2] = 0;

		if(exy.norm() > m_radius_goal){
			double n = (exy.norm() / (50 * m_radius_goal));

			ta /= M_PI, ra /= M_PI;

			double dta = ta - m_prev_goal_e[0];
			double dra = ra - m_prev_goal_e[1];
			m_prev_goal_e[0] = ta;
			m_prev_goal_e[1] = ra;

			double uta = kp_tr * ta + kd_tr * dta;
			double ura = kp_tr * ra + kd_tr * dra;

			m_angles_goal[0] = uta * n;
			m_angles_goal[0] = value2range(m_angles_goal[0], max_other_change);
			m_angles_goal[1] = ura * n;
			m_angles_goal[1] = value2range(m_angles_goal[1], max_other_change);

		}else{
			m_angles_goal[0] = 0;
			m_angles_goal[1] = 0;
		}

		push_log("goals: φ=" + fromFloat(m_angles_goal[2]) +
				" θ=" + fromFloat(m_angles_goal[0]) +
				" α=" + fromFloat(m_angles_goal[1]) +
				" gtg=" + fromFloat(rad2angle(a)));

		m_angles_goal = crop_angles(m_angles_goal);
	}
}

double Model::radius_goal() const
{
	return m_radius_goal;
}

void Model::setRadiusGoal(double v)
{
	m_radius_goal = v;
}

void Model::setSearchHover(bool v)
{
	m_use_eI_height = v;
	m_search_hover = v;
}

void Model::setGoalVerticalVelocity(double v)
{
	m_goal_vert_vel = v;
}

double Model::vertVel() const
{
	return velocity()[2];
}

bool Model::found_hover() const
{
	return m_found_hover;
}
