#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <queue>
#include <deque>

#include "custom_types.h"

#include "vobjcontainer.h"
#include "pid_control.h"

struct TrackPoint{
	TrackPoint(){ time = 0; }
	TrackPoint(const ct::Vec3d pt, int64_t tm){ v = pt, time = tm;}
	ct::Vec3d v;
	int64_t time;
};

class Model
{
public:
	enum EHeightControl{
		ENone,
		EGoToToHeight,
		EHover
	};

	Model();
	~Model();
	/**
	 * @brief calulcate
	 * main calculation state and control of the model
	 */
	void calulcate();
	void setHeightControl(EHeightControl hc);
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
	void setUseEngines(bool f);
	bool isUseEngines() const;
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

	/// @brief the angles of model
	double goal_roll() const;
	double goal_tangage() const;
	double goal_yaw() const;

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
	bool isUseInegralErrorAngles() const;
	/**
	 * @brief setUseIntegralError
	 * use integral error for equalize angles of the model
	 * @param v
	 */
	void setUseIntegralErrorAngles(bool v);
	void setUseIntegralErrorHeight(bool v);
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

	void setGoalVerticalVelocity(double v);
	double vertVel() const;

	bool is_goal_reached() const;

	double accuracy_goal() const;
	void setAccuracyGoal(double v);
	double radiusOfInfluence_goal() const;
	void setRadiusOfInfluenceGoal(double v);
	void setAccuracyVelocity(double val);
	double accuracyVelocity() const;

	std::deque< TrackPoint >& track_points();

private:
	ct::Vec3d m_pos;
	ct::Vec3d m_prev_pos;
	ct::Vec3d m_vel;
	ct::Vec3d m_angles;
	ct::Vec3d m_angles_vel;
	double m_max_force;
	int m_state;

	bool m_power;

	std::deque< std::string > m_logs;

	EHeightControl m_Eheight_control;
	double m_heightGoal;

	bool m_useEngines;

	double m_mass;

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

	double m_goal_vert_vel;

	void calculate_angles();
	void state_model_angles();
	void state_model_position();

	void calculate_hovering();
	void normal_work();

	void simpleHeightControl();
	void normal_work_simple();

private:
	pid_control< double, double > m_control_height;
	pid_control< ct::Vec3d, double > m_control_angles;
	pid_control< double, double > m_control_vert_vel;
	pid_control< double, double > m_control_vert_vel2;
	pid_control< double, double > m_control_normxy;
	pid_control< double, double > m_control_velxy;
	pid_control< ct::Vec2d, double > m_control_vert_horiz;

	std::deque< TrackPoint > m_track_points;

	bool m_use_integral_angles;		/// flag for use or not integral error with angles
	bool m_use_eI_height;

	ct::Vec3d m_prev_goal_e;	/// previous error for go to goal;

	ct::Vec4d m_forces;
	ct::Vec4d m_prev_forces;
	double m_arm;

	ct::Vec3d m_goal_point;
	bool m_track_to_goal_point;
	double m_radius_goal;
	double m_accuracy_goal;
	double m_accuracy_velocity;
	bool m_is_goal_reached;

	ct::Vec3d m_direction_force;
	ct::Vec3d m_direct_model;
	ct::Vec3d m_roll_model;

	void load_params();
	void save_params();

	void push_track_point(const ct::Vec3d pt);

private:
	VObjContainer m_container;
};

const double virtual_z_edge = 50.;
const double virtual_xy_edge = 50.;

#endif // MODEL_H
