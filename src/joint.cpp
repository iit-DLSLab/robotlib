/**
 * @file joint.cpp
 *
 * @brief Joint class and functions implementation
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "joint.hpp"
#include <iostream>

namespace robotlib
{
	Joint::Joint(const std::string &name)
		: Frame(name), q_min_(0), q_max_(0), qd_max_(0), tau_max_(0), sub_id(-1), id(-1)
	{
	}

	Joint::Joint() 
		: Frame(""), q_min_(0), q_max_(0), qd_max_(0), tau_max_(0) 
	{}

	bool Joint::isAttached() const
	{
		if (sub_id == -1 || id == -1)
			return false;
		else
			return true;
	}

	double Joint::getMinAngle() const {return q_min_;}
	double Joint::getMaxAngle() const {return q_max_;}
	double Joint::getMaxVelocity() const {return qd_max_;}
	double Joint::getMaxEffort() const {return tau_max_;}

	void Joint::setMinAngle(const double q_min) {q_min_ = q_min;}
	void Joint::setMaxAngle(const double q_max) {q_max_ = q_max;}
	void Joint::setMaxVelocity(const double qd_max) {qd_max_ = qd_max;}
	void Joint::setMaxEffort(const double tau_max)  {tau_max_ = tau_max;}

	void Joint::setJointLimits(const double& q_min, const double& q_max, const double& qd_max, const double& tau_max)
	{
		this->setMinAngle(q_min);
		this->setMaxAngle(q_max);
		this->setMaxVelocity(qd_max);
		this->setMaxEffort(tau_max);
	}

	bool Joint::operator==(const Joint& rhs) const
	{
		if(this->getName() != rhs.getName() || 
		   this->getMinAngle() != rhs.getMinAngle() ||
		   this->getMaxAngle() != rhs.getMaxAngle() ||
		   this->getMaxVelocity() != rhs.getMaxVelocity() ||
		   this->getMaxEffort() != rhs.getMaxEffort())
			return false;

		return true;
	}

	Joint &Joint::operator=(const Joint &rhs)
	{
		if (this != &rhs) // self-assignment check
		{
			this->name_ = rhs.name_;
			this->q_min_ = rhs.q_min_;
			this->q_max_ = rhs.q_max_;
			this->qd_max_ = rhs.qd_max_;
			this->tau_max_ = rhs.tau_max_;
		}
		return *this;
	}

} // namespace robotlib