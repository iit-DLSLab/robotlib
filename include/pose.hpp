#ifndef _ROBOTLIB_POSE_HPP_
#define _ROBOTLIB_POSE_HPP_

// =============================================================================
// Includes
// =============================================================================
#include <Eigen/Dense>

namespace dls{
namespace robot {
/**
 * A pose class for robots.
 */
class Pose
{
public:

	Pose(const Eigen::Vector3d& p, const Eigen::Vector3d& ori) : p_(p), ori_(ori){};

	~Pose(){};

	// Get Function
	const Eigen::Vector3d& getPos(){return p_;}
	const Eigen::Vector3d& getOri(){return ori_;}

private: 
	const Eigen::Vector3d p_;						//! 3D position
	const Eigen::Vector3d ori_;					//! 3D orientation
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_POSE_HPP_
