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
	 * main calculation state and control of the model
	 */
	void calulcate();
	/**
	 * @brief initialize
	 * reset state to zero
	 */
	void initialize();
	/**
	 * @brief open
	 * open file with 3d model
	 * @param fn - filename
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
	 * 3d object
	 * @return
	 */
	const std::deque<VObj> &vobjs() const;
	/**
	 * @brief pos
	 * current position of the model
	 * @return
	 */
	const ct::Vec3d &pos() const;
	/**
	 * @brief velocity
	 * @return
	 */
	ct::Vec3d velocity() const;
	/**
	 * @brief eiler
	 * eiler matrix of the model rotation
	 * @return
	 */
	ct::Matd eiler() const;
	/**
	 * @brief set_force
	 * for old force
	 * @param force
	 */
	void set_force(double force);
	double force() const;
	/**
	 * @brief is_dynamic
	 * if some changes or power on
	 * @return
	 */
	bool is_dynamic() const;
	/**
	 * @brief dt
	 * @return
	 */
	double dt() const;
	/// @brief work with log
	bool is_log_exists() const;
	std::string pop_log();
	void push_log(const std::string &str);
	/**
	 * @brief setSimpleHeightControl
	 * @param val
	 */
	void setSimpleHeightControl(bool val);
	/**
	 * @brief setHeightGoal
	 * @param h
	 */
	void setHeightGoal(double h);
	/// @brief the goal angles
	double tangageGoal() const;
	void setTangageGoal(double v);
	double rollGoal() const;
	void setRollGoal(double v);
	double yawGoal() const;
	void setYawGoal(double v);
	/**
	 * @brief setForces
	 * set each force of 4 engines
	 * @param f1
	 * @param f2
	 * @param f3
	 * @param f4
	 */
	void setForces(double f1, double f2, double f3, double f4);
	/**
	 * @brief setForce
	 * set each force with index
	 * @param index
	 * @param v
	 */
	void setForce(int index, double v);
	/**
	 * @brief setUseMultipleForces
	 * set use forces for 4 engines
	 * @param f
	 */
	void setUseMultipleForces(bool f);
	/**
	 * @brief force
	 * get force of engine[index]
	 * @param index
	 * @return
	 */
	double force(int index);

	void reset_angles();
	/**
	 * @brief setYaw
	 * set begin yaw of the model
	 * @param v
	 */
	void setYaw(double v);
	/**
	 * @brief setRoll
	 * set begin roll of the model
	 * @param v
	 */
	void setRoll(double v);
	/**
	 * @brief setTangage
	 * set begin tangage of the model
	 * @param v
	 */
	void setTangage(double v);

	/// @brief the angles of model
	double roll() const;
	double tangage() const;
	double yaw() const;

	/**
	 * @brief direction_force
	 * vector of direction of current force
	 * @return
	 */
	ct::Vec3d direction_force() const;
	/**
	 * @brief direct_model
	 * vector of direction model
	 * @return
	 */
	ct::Vec3d direct_model() const;
	/**
	 * @brief isUseInegralError
	 * @return
	 */
	bool isUseInegralError() const;
	/**
	 * @brief setUseIntegralError
	 * use integral error for equalize angles of the model
	 * @param v
	 */
	void setUseIntegralError(bool v);
	/**
	 * @brief setPower
	 * power on/off model calculation
	 * @param v
	 */
	void setPower(bool v);
	/**
	 * @brief isPower
	 * current state of power
	 * @return
	 */
	bool isPower() const;
	/**
	 * @brief setGoalPoint
	 * set the goal point for track model
	 * @param pt
	 */
	void setGoalPoint(const ct::Vec3d &pt);
	/**
	 * @brief goal_point
	 * current the goal point of track
	 * @return
	 */
	ct::Vec3d goal_point() const;
	/**
	 * @brief setTrackToGoalPoint
	 * set calculation tracking to the goal point
	 * @param v
	 */
	void setTrackToGoalPoint(bool v);
	/**
	 * @brief isTrackToGoalPoint
	 * @return
	 */
	bool isTrackToGoalPoint() const;
	/**
	 * @brief calculate_track_to_goal
	 */
	void calculate_track_to_goal();
	/**
	 * @brief radius_goal
	 * @return
	 */
	double radius_goal() const;
	/**
	 * @brief setRadiusGoal
	 * set radius when model thinks that it's the goal
	 * @param v
	 */
	void setRadiusGoal(double v);

private:
	ct::Vec3d m_pos;
	ct::Vec3d m_prev_pos;
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

	ct::Vec3d m_prev_angles_e;	/// for calculate differential error of angles
	ct::Vec3d m_angles_eI;		/// integral error of angles
	bool m_use_integral;		/// flag for use or not integral error with angles

	ct::Vec3d m_prev_goal_e;	/// previous error for go to goal;

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
