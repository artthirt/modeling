#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <vector>
#include <algorithm>

template< typename T, typename C >
class pid_control{
public:

	typedef T (tcrop_value)(const T& val);

	pid_control(){
		m_eI = T(0);
		m_prev_e = T(0);
		goal = T(0);

		kp = C(1);
		kd = C(1);
		ki = C(0.1);
		use_eI = false;

		crop_value = 0;
	}
	pid_control(C kp, C kd, C ki, bool use_eI = false){
		m_eI = T(0);
		m_prev_e = T(0);
		goal = T(0);

		this->kp = kp;
		this->kd = kd;
		this->ki = ki;
		this->use_eI = use_eI;

		crop_value = 0;
	}

	void setKpid(C kp, C kd, C ki){
		this->kp = kp;
		this->kd = kd;
		this->ki = ki;
	}
	void set_use_eI(bool v){
		use_eI = v;
	}

	void setGoal(T value){
		goal = value;
	}

	void reset(){
		m_prev_e = T(0);
		m_eI = T(0);
	}

	T get(T current){

		T e = goal - current;

		if(crop_value)
			e = (*crop_value)(e);

		T eI = m_eI;

		T de = e - m_prev_e;
		if(crop_value)
			de = (*crop_value)(de);
		m_prev_e = e;

		if(use_eI){
			eI = m_eI;
			eI += e;
			m_eI = eI;
		}

		T u = e * kp + de * kd + eI * ki;

		return u;
	}

	T& eI(){
		return m_eI;
	}

	C kp;
	C kd;
	C ki;
	T goal;

	tcrop_value *crop_value;

	bool use_eI;
private:
	T m_prev_e;
	T m_eI;
};

#endif // PID_CONTROL_H
