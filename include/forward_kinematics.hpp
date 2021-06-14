#ifndef _ROBOTLIB_FORWARD_KINEMATICS_HPP_
#define _ROBOTLIB_FORWARD_KINEMATICS_HPP_

#include <Eigen/Dense>

namespace dls {
namespace robot {

/**
 * A forward kinematics interface for robots.
 */
class ForwardKinematicsBase
{
public:
	virtual ~ForwardKinematicsBase() = default;

	// pure virtual
	//virtual Vector3d getFootPosLF(const JointState& q) = 0;

	virtual Vector3d getFootPos(const JointState& q, const int foot) = 0;
	

	//virtual Vector3d getFootPosRF(const JointState& q) = 0;
	//virtual Vector3d getFootPosLH(const JointState& q) = 0;
	//virtual Vector3d getFootPosRH(const JointState& q) = 0;
	//virtual Vector3d getFootPos(const JointState& q, const LegID& leg) = 0;
	//virtual Matrix3d getFootOrientation(const JointState& q, const LegID& leg) = 0;
	//virtual Vector3d getShinPos(const JointState& q, const double& contact_pos, const LegID& leg) = 0;

	// virtual
	//virtual dog::LegDataMap<Vector3d> getFeetPos  (const JointState& q);
};


} // namespace robot
} // namespace dls


#endif
