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
#ifndef _ROBOTLIB_BASE_DECLARATIONS_HPP_
#define _ROBOTLIB_BASE_DECLARATIONS_HPP_

#include "robotlib/rbd/rbd.h"

namespace dls {
namespace dog {
// =============================================================================
// Enums
// =============================================================================
enum JointIdentifiers
{
	LF_HAA = 0,
	LF_HFE,
	LF_KFE,
	RF_HAA,
	RF_HFE,
	RF_KFE,
	LH_HAA,
	LH_HFE,
	LH_KFE,
	RH_HAA,
	RH_HFE,
	RH_KFE,
};


enum LinkIdentifiers
{
	TRUNK = 0,
	LF_HIPASSEMBLY,
	LF_UPPERLEG,
	LF_LOWERLEG,
	RF_HIPASSEMBLY,
	RF_UPPERLEG,
	RF_LOWERLEG,
	LH_HIPASSEMBLY,
	LH_UPPERLEG,
	LH_LOWERLEG,
	RH_HIPASSEMBLY,
	RH_UPPERLEG,
	RH_LOWERLEG,
};

// =============================================================================
// Constants
// =============================================================================
static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
static const int jointsLegCount = 3;
static const int contactConstrCount = 3;
static const int baseJoints = 0;
static const int activeJoints = 6;
static const int fbjointsCount = 18;
static const int linksCount  = 13;

static const JointIdentifiers orderedJointIDs[jointsCount] =
{
	LF_HAA,
	LF_HFE,
	LF_KFE,
	RF_HAA,
	RF_HFE,
	RF_KFE,
	LH_HAA,
	LH_HFE,
	LH_KFE,
	RH_HAA,
	RH_HFE,
	RH_KFE
};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
{
	TRUNK,
	LF_HIPASSEMBLY,
	LF_UPPERLEG,
	LF_LOWERLEG,
	RF_HIPASSEMBLY,
	RF_UPPERLEG,
	RF_LOWERLEG,
	LH_HIPASSEMBLY,
	LH_UPPERLEG,
	LH_LOWERLEG,
	RH_HIPASSEMBLY,
	RH_UPPERLEG,
	RH_LOWERLEG
};


// =============================================================================
// Typedefs
// =============================================================================
typedef Eigen::Matrix<double, fbjointsCount, 1> Column18d;
typedef Column18d FloatingBaseJointState;
typedef Eigen::Matrix<double, 12, 1> Column12d;
typedef Column12d JointState;
typedef Eigen::Array<bool, 3, 1> LegJointBool;
typedef dls::rbd::Vector3d Vector3d;
typedef dls::rbd::Matrix33d Matrix3d;
typedef dls::rbd::PlainMatrix<double, 3, 3 > FootJac;
typedef Eigen::Vector3d LegJointState;

} // namespace dog
} // namespace dls
#endif // _ROBOTLIB_BASE_DECLARATIONS_HPP_
