/**
 * @file joint.cpp
 *
 * @brief Joint class and functions implementation
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "joint.hpp"

namespace robotlib
{
	Joint::Joint(const std::string &name) : Frame(name), q_min_(0), q_max_(0), qd_max_(0), tau_max_(0) {}

	Joint::~Joint(){}

	const std::string Joint::getName() const { return name_;}
	const std::shared_ptr<Link> Joint::getParent() const { return parent_;}
	const std::shared_ptr<Link> Joint::getChild() const { return child_;}

	void Joint::setParent(const std::shared_ptr<Link> parent) { parent_ = parent; }
	void Joint::setChild(const std::shared_ptr<Link> child) { child_ = child; }

	double Joint::getMinAngle() const {return q_min_;}
	double Joint::getMaxAngle() const {return q_max_;}
	double Joint::getMaxVelocity() const {return qd_max_;}
	double Joint::getMaxEffort() const {return tau_max_;}

	void Joint::setMinAngle(const double q_min) {q_min_ = q_min;}
	void Joint::setMaxAngle(const double q_max) {q_max_ = q_max;}
	void Joint::setMaxVelocity(const double qd_max) {qd_max_ = qd_max;}
	void Joint::setMaxEffort(const double tau_max)  {tau_max_ = tau_max;}

} // namespace robotlib