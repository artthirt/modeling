#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <queue>
#include <deque>

#include "custom_types.h"

#include "vobjcontainer.h"

class Model
{
public:
	Model();
	~Model();
	/**
	 * @brief calulcate
	 */
	void calulcate();
	/**
	 * @brief initialize
	 */
	void initialize();
	/**
	 * @brief open
	 * @param fn
	 * @return
	 */
	bool open(const std::string& fn);
	/**
	 * @brief container
	 * @return
	 */
	VObjContainer &container();
	/**
	 * @brief vobjs
	 * @return
	 */
	const std::deque<VObj> &vobjs() const;

	const ct::Vec3d &pos() const;
	ct::Matd eiler() const;

	void set_force(double force);
	double force() const;

	bool is_dynamic() const;

	double dt() const;

	bool is_log_exists() const;
	std::string pop_log();
	void push_log(const std::string &str);

	void setSimpleHeightControl(bool val);
	void setHeightGoal(double h);

	double tangageGoal() const;
	void setTangageGoal(double v);
	double rollGoal() const;
	void setRollGoal(double v);
	double yawGoal() const;
	void setYawGoal(double v);

	void setForces(double f1, double f2, double f3, double f4);
	void setForce(int index, double v);
	void setUseMultipleForces(bool f);

	double force(int index);

	void reset_angles();

	void setYaw(double v);
	void setRoll(double v);
	void setTangage(double v);

	double roll() const;
	double tangage() const;
	double yaw() const;

	ct::Vec3d direction_force() const;
	ct::Vec3d direct_model() const;

	bool isUseInegralError() const;
	void setUseIntegralError(bool v);

	void setPower(bool v);
	bool isPower() const;

	void setGoalPoint(const ct::Vec3d &pt);
	ct::Vec3d goal_point() const;
	void setTrackToGoalPoint(bool v);
	bool isTrackToGoalPoint() const;

	void calculate_track_to_goal();

	double radius_goal() const;
	void setRadiusGoal(double v);

private:
	ct::Vec3d m_pos;
	ct::Vec3d m_vel;
	ct::Vec3d m_angles;
	ct::Vec3d m_angles_vel;
	double m_max_force;

	bool m_power;

	std::deque< std::string > m_logs;

	bool m_useSimpleHeightControl;
	double m_heightGoal;

	bool m_useMultipleForces;

	double m_mass;
	double m_force;

	double m_dt;
	double m_kp_vel;
	double m_kd_vel;
	double m_kp_angle;
	double m_kd_angle;

	/**
	 * @brief m_angles_goal
	 * [0] = tangage
	 * [1] = roll
	 * [2] = yaw
	 */
	ct::Vec3d m_angles_goal;

	void calculate_angles();
	void state_model_angles();
	void state_model_position(ct::Vec3d &force_direction);

	void simpleHeightControl(const ct::Vec3d& normal);

private:
	double m_prev_e;			/// for calculate differential error
	double m_e_I;				/// for integral error to height. now not use

	ct::Vec3d prev_angles_e;	/// for calculate differential error of angles
	ct::Vec3d angles_eI;		/// integral error of angles
	bool m_use_integral;		/// falg for use or not integral error with angles

	double m_force_1;
	double m_force_2;
	double m_force_3;
	double m_force_4;
	double m_arm;

	ct::Vec3d m_goal_point;
	bool m_track_to_goal_point;
	double m_radius_goal;

	ct::Vec3d m_direction_force;
	ct::Vec3d m_direct_model;
	ct::Vec3d m_roll_model;

	void load_params();
	void save_params();

private:
	VObjContainer m_container;
};

const double virtual_z_edge = 50.;
const double virtual_xy_edge = 50.;

#endif // MODEL_H
