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
#ifndef ROBOTLIB_DYN_PARAMS_HPP
#define ROBOTLIB_DYN_PARAMS_HPP

// =============================================================================
// Includes
// =============================================================================
#include <Eigen/Dense>


namespace dls{
namespace robot {
/**
 * A class for robot dynamic parameters (com, mass, inertia).
 */
class DynParams
{
public: 
    DynParams (const Eigen::Vector3d& com, const double m, const Eigen::Matrix3d& I) : com_(com), m_(m), I_(I) {};

	~DynParams(){};
	
	// Get functions
	inline const Eigen::Vector3d& getCom(){return com_;};
	inline const double getM(){return m_;};
	inline const Eigen::Matrix3d& getI(){I_;};
	
private:
	const Eigen::Vector3d com_;					//! Center of mass
	const double m_;							//! Mass
	const Eigen::Matrix3d I_;					//! Inertia
};

} // namespace dog
} // namespace robot

#endif
