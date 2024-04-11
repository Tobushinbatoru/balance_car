// PID.cpp
#include "PID.h"

PID::PID(double Kp, double Ki, double Kd, double u_min, double u_max, double dt)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->u_min = u_min;
	this->u_max = u_max;
	this->dt = dt;
	this->e = 0.0;
	this->e_prev = 0.0;
	this->e_sum = 0.0;
	this->u = 0.0;
}

PID::~PID()
{
	// pass
}

void PID::update(double r, double y)
{
	e = r - y;
	double p = Kp * e;
	e_sum += (e + e_prev) / 2.0 * dt;
	double i = Ki * e_sum;
	double d = Kd * (e - e_prev) / dt;
	u = p + i + d;
	if(u < u_min)
	{
		u = u_min;
	}
	if(u > u_max)
	{
		u = u_max;
	}
	e_prev = e;
}

double PID::getOutput()
{
	return u;
}

void PID::setGains(double Kp, double Ki, double Kd)
{
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::getGains(double& Kp, double& Ki, double& Kd)
{
	Kp = this->Kp;
	Ki = this->Ki;
	Kd = this->Kd;
}

void PID::getLimits(double& u_min, double& u_max)
{
	u_min = this->u_min;
	u_max = this->u_max;
}

void PID::setDt(double dt)
{
	this->dt = dt;
}

double PID::getDt()
{
	return dt;
}

