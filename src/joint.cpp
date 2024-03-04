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

namespace robotlib
{
	Joint::Joint(const std::string &name) 
		: Frame(name), q_min_(0), q_max_(0), qd_max_(0), tau_max_(0) 
	{}

	Joint::Joint(const std::string &name, Link& parent) 
		: Frame(name), q_min_(0), q_max_(0), qd_max_(0), tau_max_(0) 
	{
		this->setParent(parent);
	}

	const Link& Joint::getParent() const { return *parent_;}
	const Link& Joint::getChild() const { return *child_;}

	void Joint::setParent(Link& parent) {
		parent_ = std::shared_ptr<Link>(&parent); 
		parent_->addChild(*this);
	}

	void Joint::setChild(Link& child) 
	{ 
		child_ = std::shared_ptr<Link>(&child); 
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

} // namespace robotlib