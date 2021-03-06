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

const double attenuation = 0.97f;

/// coefficient friction of oac-tree (for example)
const double coeff_friction = 0.62f;
/// for weak epsilon weak
const double eps_weak = 1e-3;
/// for hard epsilon weak
const double eps_hard = 1e-6;
/// minumum angle for choose state
const double angle_for_chose = angle2rad(5);

const QString config_file("./config.model.xml");

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
	, m_useEngines(true)
	, m_direct_model(1, 0, 0)
	, m_roll_model(0, 1, 0)
	, m_power(false)
	, m_track_to_goal_point(false)
	, m_goal_point(32, 32, 10)
	, m_radius_goal(3.0)
	, m_accuracy_goal(0.05)
	, m_state(NORMAL)
	, m_use_eI_height(false)
	, m_goal_vert_vel(0)
	, m_is_goal_reached(false)
	, m_accuracy_velocity(3 * eps_weak)
{
	m_mass = 1;
	m_kp_vel = 1;
	m_kd_vel = 1;
	m_kp_angle = 1;
	m_kd_angle = 1;
	m_dt = 1.f/100;
	m_max_force = 30.;

//	m_prev_e = 0;
//	m_e_I = 0;

//	m_eI_height = 0;
//	m_prev_e_height = 0;

	m_arm = 0.2f;

	load_params();

	if(m_control_angles_TR.eI().empty())
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

	switch (m_Eheight_control) {
		case EGoToToHeight:
			simpleHeightControl();
			break;
		case EHover:
			calculate_hovering();
			break;
		default:
			break;
	}
	calculate_angles();

	calculate_track_to_goal();

	push_track_point(m_pos);
}

void Model::setHeightControl(Model::EHeightControl hc)
{
	m_Eheight_control = hc;
}

void Model::initialize()
{
	m_pos = Vec3d();
	m_angles = Vec3d();
	m_angles_vel = Vec3d();
	m_control_angles_TR.reset();
	m_track_points.clear();
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
//	ct::Vec3d v = m_pos - m_prev_pos;
//	v /= m_dt;
	return m_vel;
}

Matd Model::eiler() const
{
	return get_eiler_mat4(m_angles);
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
	m_forces = Vec4d(f1, f2, f3, f4);
}

void Model::setForce(int index, double v)
{
	if(v < 0)
		v = 0;
	m_forces[index - 1] = v;
}

void Model::setUseEngines(bool f)
{
	m_useEngines = f;
}

bool Model::isUseEngines() const
{
	return m_useEngines;
}

double Model::force(int index)
{
	return m_forces[index - 1];
}

void Model::reset_angles()
{
	m_angles = Vec3d::zeros();
	m_angles_vel = Vec3d::zeros();
	m_forces = Vec4d::zeros();

	m_control_angles_TR.eI() = Vec3d::zeros();
	m_control_height.eI() = 0;

	m_pos = Vec3d::zeros();

	m_track_points.clear();

	m_state = NORMAL;
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
	if(!m_useEngines)
		return;

	/// [0] - tangage
	/// [1] - roll
	/// [2] - yaw

	const double kpTR = 0.90;
	const double kiTR = 0.005;
	const double kdTR = 5.0;

	const double kpY = 0.70;
	const double kiY = 0.005;
	const double kdY = 5.00;

	m_control_angles_TR.setKpid(kpTR, kdTR, kiTR);
	m_control_angles_TR.use_eI = m_use_integral_angles;
	m_control_angles_TR.setGoal(m_angles_goal);
	m_control_angles_TR.crop_value = &crop_angles;

	m_control_angles_Y.setKpid(kpY, kdY, kiY);
	m_control_angles_Y.use_eI = m_use_integral_angles;
	m_control_angles_Y.setGoal(m_angles_goal[2]);
	m_control_angles_Y.crop_value = &crop_angle;

	Vec2d uTR = m_control_angles_TR.get(m_angles);
	double uY = m_control_angles_Y.get(m_angles[2]);
	//u = sign(u) * (u * u);

	//double avg_f = m_force;
	/// f1 + f3 -> -tangage
	///	f2 + f4 -> +tangage
	///	f1 + f4 -> -roll
	/// f3 + f2 -> +roll
	/// f1 + f2 > f2 + f4 -> +yaw
	/// f1 + f2 < f2 + f4 -> -yaw

//	u = values2range(u, -m_max_force, m_max_force);

	uTR /= 4;
	uY	/= 4;
	double force0 = m_forces.sum();

	m_prev_forces = m_forces;

	Vec4d vu;

	vu[0] = force0/4 +	uTR[0]		- uTR[1] + uY;
	vu[2] = force0/4 +	uTR[0]		+ uTR[1] - uY;
	vu[1] = force0/4 + (-uTR[0])	+ uTR[1] + uY;
	vu[3] = force0/4 + (-uTR[0])	- uTR[1] - uY;

	double force1 = vu.sum();

	//m_forces = vu;
	if(force1){
//	double part_f = m_force_1 + m_force_2 + m_force_3 + m_force_4;
		Vec4d pfs = vu / force1;
		m_forces = pfs * force0;
	}
	m_forces = values2range(m_forces, 0., m_max_force);
}

void Model::normal_work_simple()
{
//	double e = m_heightGoal - m_pos[2];
//	Vec3f vec_force = normal * m_force;
//	float force = vec_force[2];

	const double kp = 3.5;
	const double kd = 30.0;
	const double ki = 0.2;

	m_control_height.setGoal(m_heightGoal);
	m_control_height.setKpid(kp, kd, ki);
	m_control_height.set_use_eI(m_use_eI_height);

//	m_e_I += e;

//	double de = e - m_prev_e;
//	m_prev_e = e;
	double f = m_forces.sum();
	double prevf = f;

	double u = m_control_height.get(m_pos[2]);

	u = value2range(u, -m_max_force, m_max_force);

	f = u;
	f = value2range(f, 0., m_max_force);

	if(prevf){

		Vec4d pfs = m_forces / prevf;
		m_forces = pfs * f;
	}else{
		m_forces = f / 4.;
	}
}

void Model::calculate_hovering()
{
	if(m_state == NORMAL){

		if(m_direction_force[2] < 0){
			m_state = ROTATE_TO_HOVER;
			setTangageGoal(0);
			setRollGoal(0);
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
	const double kp = 1.5;
	const double ki = 0.1;
	const double kd = 50.;

	double force = m_forces.sum();

	if(force < eps_weak){
		force = 1;
		m_forces = 0.25;
	}
//	if(force < eps_weak && !m_found_hover)
//		return;

//	if(m_found_hover && force < eps_weak){
//		force = m_force_hover;
//	}

	Vec4d pfs = m_forces / force;

	Vec3d v = velocity();

	//double e = m_pos[2] - m_heightGoal;
	m_control_vert_vel2.setGoal(m_goal_vert_vel);
	m_control_vert_vel2.setKpid(kp, kd, ki);
	m_control_vert_vel2.set_use_eI(m_use_eI_height);

	double u = m_control_vert_vel2.get(v[2]);//kp * e + ki * eI + kd * de;

	u = value2range(u, -m_max_force, m_max_force);

	force += u;

	force = value2range(force, -m_max_force, m_max_force);

	m_forces = pfs * force;
}

void Model::state_model_angles()
{
	/// ************** if not power then not change angles
	if(!m_power)
		return;

	/// for some uncertainty that influence the each force
	const Vec4d coeffs_f = Vec4d(0.985, 0.983, 0.988, 0.982);

	Vec4d forces = m_forces * coeffs_f;

	double df_12 = forces[0] - forces[1];
	double df_34 = forces[2] - forces[3];

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

		m_angles_vel[0] += w_tan_12 + w_tan_34;
		m_angles_vel[1] += w_roll_34 - w_roll_12;

	}
	/// yaw = F1 + F2 - (F3 + F4)
	double df_1234 = forces[0] + forces[1] - forces[2] - forces[3];
	if(df_1234){
		double a1234 = df_1234 / m_mass;
		double w_1234 = sqrt(std::abs(a1234) / m_arm) * m_dt;
		if(a1234 < 0) w_1234 = -w_1234;
		m_angles_vel[2] += w_1234;
	}
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
	double force_general = m_forces.sum();
	double force = m_power? force_general : 0;
	///*************

	vel *= (force/m_mass * m_dt);

	m_vel += vel;

	m_vel += Vec3d(0.f, 0.f, -m_mass * gravity * m_dt);

	m_vel *= attenuation;

//	push_log("after force: " + m_vel.operator std::string());

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
			m_angles_vel = Vec3d::zeros();
		}
	}

	m_pos += m_vel;

	m_angles += m_angles_vel;

	if(m_pos[2] < 0){
		m_pos[2] = 0;
		m_vel = Vec3d::zeros();
		m_angles_vel = Vec3d::zeros();
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
			setRollGoal(0);
			return;
		}

		normal_work_simple();
	}

}



void Model::load_params()
{
	QMap< QString, QVariant > params;
	if(!SimpleXML::load_param(config_file, params))
		return;

	m_control_angles_TR.eI()[0] = params["angles"].toMap()["integral"].toMap()["tangage"].toDouble();
	m_control_angles_TR.eI()[1] = params["angles"].toMap()["integral"].toMap()["roll"].toDouble();
	m_control_angles_TR.eI()[2] = params["angles"].toMap()["integral"].toMap()["yaw"].toDouble();

	m_control_vert_vel2.eI() = params["height"].toMap()["integral_vel"].toDouble();
	m_control_height.eI() = params["height"].toMap()["integral_goal"].toDouble();

	m_useEngines = params["use_engines"].toBool();

	m_accuracy_goal = params["accuracy_goal"].toDouble();
	m_radius_goal = params["radius_goal"].toDouble();
	m_accuracy_velocity = params["accuracy_vel"].toDouble();
}

void Model::save_params()
{
	QMap< QString, QVariant > params, pang, pint;

	pint["tangage"]		= m_control_angles_TR.eI()[0];
	pint["roll"]		= m_control_angles_TR.eI()[1];
	pint["yaw"]			= m_control_angles_TR.eI()[2];

	pang["integral"] = pint;

	params["angles"] = pang;

	pint.clear();
	pint["integral_vel"] = m_control_vert_vel2.eI();
	pint["integral_goal"] = m_control_height.eI();
	params["height"] = pint;

	params["use_engines"] = m_useEngines;
	params["accuracy_goal"] = m_accuracy_goal;
	params["radius_goal"] = m_radius_goal;
	params["accuracy_vel"] = m_accuracy_velocity;

	SimpleXML::save_param(config_file, params);
}

void Model::push_track_point(const Vec3d pt)
{
	const size_t max_track_points = 5000;

	while(m_track_points.size() > max_track_points)
		m_track_points.pop_back();

	time_t tm = std::time(nullptr);

	m_track_points.push_front(TrackPoint(pt, tm));
}

void Model::setGoalPoint(const ct::Vec3d &pt)
{
	m_goal_point = pt;
	m_is_goal_reached = false;
}

ct::Vec3d Model::goal_point() const
{
	return m_goal_point;
}

void Model::setTrackToGoalPoint(bool v)
{
	m_track_to_goal_point = v;
	if(v){
		m_is_goal_reached = false;
	}
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
	if(e.norm() < m_accuracy_goal && velocity().norm() < m_accuracy_velocity){
		m_angles_goal[0] = 0;
		m_angles_goal[1] = 0;
		m_is_goal_reached = true;
		return;
	}

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

		const double max_yaw_change = angle2rad(30.);
		const double max_other_change = angle2rad(15.);

		const double kp_y = 5.0;
		const double kd_y = 0.5;

		const double kp_tr = 1;
		const double kd_tr = 10;

		const double kp_vel = 1;
		const double kd_vel = 170;

		const double speed_max = 1;

		double e_yaw = a;
		double de_yaw = e_yaw - m_prev_goal_e[2];
		de_yaw = atan2(sin(de_yaw), cos(de_yaw));
		m_prev_goal_e[2] = e_yaw;

		if(e.norm() > m_radius_goal){
			double u = kp_y * e_yaw + kd_y * de_yaw;
			u *= m_dt;
			u = max(-max_yaw_change, min(max_yaw_change, u));
			u = atan2(sin(u), cos(u));
			m_angles_goal[2] -= u;
		}

		double ta = cos(a);
		double ra = sin(a);

		Vec3d exy = e;
		exy[2] = 0;
		Vec3d velxy = velocity();
		//velxy[2] = 0;

		if(exy.norm() > m_accuracy_goal){
			double n = e.norm();
			n = log(1 + 2 * n);
			m_control_normxy.setKpid(kp_tr, kd_tr, 0);
			m_control_normxy.setGoal(0);
			double u = m_control_normxy.get(n) * m_dt;

			double v = velxy.norm(), v_max = speed_max;
			if(exy.norm() < m_radius_goal){
				v_max = e.norm() / m_radius_goal * speed_max;
			}
			m_control_velxy.setKpid(kp_vel, kd_vel, 0);
			m_control_velxy.setGoal(v_max);
			double vu = m_control_velxy.get(v) * m_dt;

			double uta = value2range((vu - u) * ta, -max_other_change, max_other_change);
			double ura = value2range((vu - u) * ra, -max_other_change, max_other_change);

			m_angles_goal[0] = uta;
			m_angles_goal[1] = ura;
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

void Model::setGoalVerticalVelocity(double v)
{
	m_goal_vert_vel = v;
}

double Model::vertVel() const
{
	return velocity()[2];
}

bool Model::is_goal_reached() const
{
	return m_is_goal_reached;
}

double Model::accuracy_goal() const
{
	return m_accuracy_goal;
}

void Model::setAccuracyGoal(double v)
{
	m_accuracy_goal = v;
}

double Model::radiusOfInfluence_goal() const
{
	return m_radius_goal;
}

void Model::setRadiusOfInfluenceGoal(double v)
{
	m_radius_goal = v;
}

void Model::setAccuracyVelocity(double val)
{
	m_accuracy_velocity = val;
}

double Model::accuracyVelocity() const
{
	return m_accuracy_velocity;
}

std::deque<TrackPoint> &Model::track_points()
{
	return m_track_points;
}
