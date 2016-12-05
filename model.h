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

private:
	ct::Vec3f m_pos;
	ct::Vec3f m_vel;
	ct::Vec3f m_angles;
	ct::Vec3f m_angles_vel;
	float m_max_force;

	std::deque< std::string > m_logs;

	bool m_useSimpleHeightControl;
	float m_heightGoal;

	float m_mass;
	float m_force;

	float m_dt;
	float m_kp_vel;
	float m_kd_vel;
	float m_kp_angle;
	float m_kd_angle;

	void simpleHeightControl(const ct::Vec3f& normal);

private:
	float prev_e;
	float e_I;

private:
	VObjContainer m_container;
};

#endif // MODEL_H
