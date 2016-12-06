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

	const ct::Vec3f& pos() const;
	ct::Matf eiler() const;

	void set_force(float force);
	float force() const;

	bool is_dynamic() const;

	float dt() const;

	bool is_log_exists() const;
	std::string pop_log();
	void push_log(const std::string &str);

	void setSimpleHeightControl(bool val);
	void setHeightGoal(float h);

	void setTangageGoal(float v);
	void setRollGoal(float v);
	void setYawGoal(float v);

	void setForces(float f1, float f2, float f3, float f4);
	void setForce(int index, float v);
	void setUseMultipleForces(bool f);

	void reset_angles();

	void setYaw(float v);
	void setRoll(float v);
	void setTangage(float v);

	ct::Vec3f direction_force() const;

private:
	ct::Vec3f m_pos;
	ct::Vec3f m_vel;
	ct::Vec3f m_angles;
	ct::Vec3f m_angles_vel;
	float m_max_force;

	std::deque< std::string > m_logs;

	bool m_useSimpleHeightControl;
	float m_heightGoal;

	bool m_useMultipleForces;

	float m_mass;
	float m_force;

	float m_dt;
	float m_kp_vel;
	float m_kd_vel;
	float m_kp_angle;
	float m_kd_angle;

	/**
	 * @brief m_angles_goal
	 * [0] = tangage
	 * [1] = roll
	 * [2] = yaw
	 */
	ct::Vec3f m_angles_goal;

	void calculate_angles();
	void state_model_angles();
	void state_model_position(ct::Vec3f &force_direction);


	void simpleHeightControl(const ct::Vec3f& normal);

private:
	float prev_e;
	float e_I;

	ct::Vec3f prev_angles_e;

	float m_force_1;
	float m_force_2;
	float m_force_3;
	float m_force_4;
	float m_arm;

	ct::Vec3f m_direction_force;

private:
	VObjContainer m_container;
};

const float virtual_z_edge = 50.;
const float virtual_xy_edge = 15.;

#endif // MODEL_H
