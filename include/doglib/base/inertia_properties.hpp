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
* Maintainer:        Hendrik de Bruin                                          *
* Maintainer email:  hendrik.debruin@iit.it                                    *
*******************************************************************************/
#ifndef DOGLIB_BASE_INERTIA_PROPERTIES_HPP
#define DOGLIB_BASE_INERTIA_PROPERTIES_HPP

#include "doglib/rbd/rbd.h"
#include "doglib/base/declarations.hpp"
#include "doglib/base/leg_data_map.hpp"


namespace dls {
namespace dog {

class InertiaPropertiesBase {
public:
	virtual ~InertiaPropertiesBase() = default;

	virtual const dls::rbd::Vector3d& getTrunkCOM() = 0;
	virtual double getTrunkMass()  = 0;
	virtual double getHipAssemblyMass(const LegID& leg) const = 0;
	virtual double getUpperLegMass(const LegID& leg) const = 0;
	virtual double getLowerLegMass(const LegID& leg) const = 0;
	virtual Vector3d getHipAssemblyCOM(const LegID& leg) const = 0;
	virtual Vector3d getUpperLegCOM(const LegID& leg) const = 0;
	virtual Vector3d getLowerLegCOM(const LegID& leg) const = 0;
	virtual double getTotalMass() const = 0;

	/**
	 * @brief getWholeBodyCOM  computes the Center Of Mass (COM) position of
	 * the whole robot, in  base coordinates.
	 * @param q the joint status vector describing the configuration of the robot
	 * @return the position of the Center Of Mass of the whole robot, expressed
			   in base coordinates
	 */
	virtual Vector3d getWholeBodyCOM(const JointState& q) = 0;

};
} // namespace dog
} // namespace dls

#endif
