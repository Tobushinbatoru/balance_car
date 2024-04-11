// PID.h
#ifndef PID_H
#define PID_H

class PID
{
private:
	double Kp;
	double Ki;
	double Kd;
	double e;
	double e_prev;
	double e_sum;
	double u;
	double u_min;
	double u_max;
	double dt;
public:
	PID(double Kp, double Ki, double Kd, double u_min, double u_max, double dt);
	~PID();
	void update(double r, double y);
	double getOutput();
	void setGains(double Kp, double Ki, double Kd);
	void getGains(double& Kp, double& Ki, double& Kd);
	void getLimits(double& u_min, double& u_max);
	void setDt(double dt);
	double getDt();
};

#endif
