/*******************************************************************************
*                                                       ,----,                 *
*                                                     .'   .' \                *
*                                                   ,----,'    |               *
*               ________  ___       ________        |    :  .  ;               *
*              |\   ___ \|\  \     |\   ____\       ;    |.'  /                *
*              \ \  \_|\ \ \  \    \ \  \___|_      `----'/  ;                 *
*               \ \  \ \\ \ \  \    \ \_____  \       /  ;  /                  *
*                \ \  \_\\ \ \  \____\|____|\  \     ;  /  /-,                 *
*                 \ \_______\ \_______\____\_\  \   /  /  /.`|                 *
*                  \|_______|\|_______|\_________\./__;      :                 *
*                                     \|_________||   :    .'                  *
*                                                 ;   | .'                     *
*                                                 `---'                        *
********************************************************************************
* Author:            Legacy Code                                               *
* Maintainer:        Marco Marchitto                                           *
* Maintainer email:  marco.marchitto@iit.it                                    *
*******************************************************************************/
#ifndef ROBOTLIB_POSE_HPP
#define ROBOTLIB_POSE_HPP

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

} // namespace dog
} // namespace robot

#endif
