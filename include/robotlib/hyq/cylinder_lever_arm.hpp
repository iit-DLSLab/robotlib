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
/*
 *  Created on: July 7, 2014
 *	  Author: Marco Frigerio
 */
#ifndef _ROBOTLIB_HYQ_CYLINDER_LEVER_ARM_HPP_
#define _ROBOTLIB_HYQ_CYLINDER_LEVER_ARM_HPP_

#include "robotlib/base/declarations.hpp"
#include "robotlib/base/joint_data_map.hpp"
#include "robotlib/hyq/constants.hpp"

#include <math.h>


namespace dls {
namespace dog {
namespace HyQ {
namespace internal {

class CylinderLeverArm
{
public:
	static const CylinderLeverArm& getInstance();

	double getLeverArm(dog::JointIdentifiers joint, double cyl) const;
	double getCylinderLength(dog::JointIdentifiers joint, double theta) const;
	double getJointStatus(dog::JointIdentifiers joint, double cyl) const;

private:
	CylinderLeverArm();
	static CylinderLeverArm instance;

	typedef  double(CylinderLeverArm::*FuncPtr) (double)const;
	typedef  double(CylinderLeverArm::*FuncPtr2)(dog::JointIdentifiers,double)const;
	dog::JointDataMap<FuncPtr > lengthCalculators;
	dog::JointDataMap<FuncPtr > leverCalculators;
	dog::JointDataMap<FuncPtr2> jointStatusCalculators;

	dog::JointDataMap<int> length_sign_flip;


	double haa_placeholder(double foo) const { return 0; }
	double haa_placeholder(dog::JointIdentifiers joint, double foo) const { return 0; }
	double hfe_cyl_length(double th) const;
	double kfe_cyl_length(double th) const;
	double hfe_lever(double cyl) const;
	double kfe_lever(double cyl) const;
	double hfe_joint_status(dog::JointIdentifiers joint, double cyl) const;
	double kfe_joint_status(dog::JointIdentifiers joint, double cyl) const;

};


} // namespace internal

/**
 * Computes the length of the cylinder lever arm for the given joint and joint
 * angle.
 * \param joint any flexion-extension joint of HyQ; any of the HAA joints
 *		will cause 0 to be returned
 * \param cylinderLength the current total length of the cylinder
 */
double getCylinderLeverArm(dog::JointIdentifiers joint, double cylinderLength);

/**
 * Computes the total length of the cylinder (in meters) for the given joint
 * and joint angle.
 * \param joint any flexion-extension joint of HyQ; any of the HAA joints
 *		will cause 0 to be returned
 * \param theta the joint status, according to the robot convention
 */
double getCylinderLength(dog::JointIdentifiers joint, double theta);

/**
 * Computes the position status of the given joint, corresponding to the given
 * cylinder length.
 *
 * \param joint any flexion-extension joint of HyQ; any of the HAA joints
 *		will cause 0 to be returned
 * \param cylinderLength the current total length of the cylinder
 */
double getJointStatus(dog::JointIdentifiers joint, double cylinderLength);

} // HyQ
} // namespace dog
} // namespace dls
#endif // _ROBOTLIB_HYQ_CYLINDER_LEVER_ARM_HPP_
