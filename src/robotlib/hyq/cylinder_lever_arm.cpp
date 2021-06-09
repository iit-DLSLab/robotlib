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
// =============================================================================
// Includes
// =============================================================================
#include "robotlib/hyq/cylinder_lever_arm.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ::internal;

// =============================================================================
// Implementation
// =============================================================================
namespace dls
{
namespace dog
{
namespace HyQ
{

double getCylinderLeverArm(dog::JointIdentifiers joint, double cylinderLength)
{
	return internal::CylinderLeverArm::getInstance().getLeverArm(joint, cylinderLength);
}

double getCylinderLength(dog::JointIdentifiers joint, double theta)
{
	return internal::CylinderLeverArm::getInstance().getCylinderLength(joint, theta);
}

double getJointStatus(dog::JointIdentifiers joint, double cylinderLength)
{
	return internal::CylinderLeverArm::getInstance().getJointStatus(joint, cylinderLength);
}

// -----------------------------------------------------------------------------
// Namespace Internal
// -----------------------------------------------------------------------------
	namespace internal
	{
		CylinderLeverArm CylinderLeverArm::instance;

		CylinderLeverArm::CylinderLeverArm() :
				length_sign_flip(1)
		{
			length_sign_flip[dog::LF_HFE] = -1;
			length_sign_flip[dog::RF_HFE] = -1;
			length_sign_flip[dog::LF_KFE] = -1;
			length_sign_flip[dog::RF_KFE] = -1;

			lengthCalculators[dog::LF_HAA] =
			lengthCalculators[dog::RF_HAA] =
			lengthCalculators[dog::LH_HAA] =
			lengthCalculators[dog::RH_HAA] = & CylinderLeverArm::haa_placeholder;

			lengthCalculators[dog::LF_HFE] =
			lengthCalculators[dog::RF_HFE] =
			lengthCalculators[dog::LH_HFE] =
			lengthCalculators[dog::RH_HFE] = & CylinderLeverArm::hfe_cyl_length;

			lengthCalculators[dog::LF_KFE] =
			lengthCalculators[dog::RF_KFE] =
			lengthCalculators[dog::LH_KFE] =
			lengthCalculators[dog::RH_KFE] = & CylinderLeverArm::kfe_cyl_length;

			leverCalculators[dog::LF_HAA] =
			leverCalculators[dog::RF_HAA] =
			leverCalculators[dog::LH_HAA] =
			leverCalculators[dog::RH_HAA] = & CylinderLeverArm::haa_placeholder;

			leverCalculators[dog::LF_HFE] =
			leverCalculators[dog::RF_HFE] =
			leverCalculators[dog::LH_HFE] =
			leverCalculators[dog::RH_HFE] = & CylinderLeverArm::hfe_lever;

			leverCalculators[dog::LF_KFE] =
			leverCalculators[dog::RF_KFE] =
			leverCalculators[dog::LH_KFE] =
			leverCalculators[dog::RH_KFE] = & CylinderLeverArm::kfe_lever;

			jointStatusCalculators[dog::LF_HAA] =
			jointStatusCalculators[dog::RF_HAA] =
			jointStatusCalculators[dog::LH_HAA] =
			jointStatusCalculators[dog::RH_HAA] = & CylinderLeverArm::haa_placeholder;

			jointStatusCalculators[dog::LF_HFE] =
			jointStatusCalculators[dog::RF_HFE] =
			jointStatusCalculators[dog::LH_HFE] =
			jointStatusCalculators[dog::RH_HFE] = & CylinderLeverArm::hfe_joint_status;

			jointStatusCalculators[dog::LF_KFE] =
			jointStatusCalculators[dog::RF_KFE] =
			jointStatusCalculators[dog::LH_KFE] =
			jointStatusCalculators[dog::RH_KFE] = & CylinderLeverArm::kfe_joint_status;
		}
		const CylinderLeverArm& CylinderLeverArm::getInstance()
		{
			return instance;
		}

		#define CALL_MEMBER_FN(ptrToMember) ((*this).*(ptrToMember))

		double CylinderLeverArm::getCylinderLength(dog::JointIdentifiers joint, double theta) const
		{
			return CALL_MEMBER_FN(lengthCalculators[joint])( length_sign_flip[joint]*theta );
		}

		double CylinderLeverArm::getLeverArm(dog::JointIdentifiers joint, double cylinderLength) const
		{
			return CALL_MEMBER_FN(leverCalculators[joint])( cylinderLength );
		}

		double CylinderLeverArm::getJointStatus(dog::JointIdentifiers joint, double cylinderLength) const
		{
			return CALL_MEMBER_FN(jointStatusCalculators[joint])(joint, cylinderLength );
		}

		#undef CALL_MEMBER_FN

		double CylinderLeverArm::hfe_cyl_length(double th) const
		{
			static const double _2a1b1 = 2*a1*b1;
			return sqrt(a1_sqr + b1_sqr - _2a1b1*cos( (M_PI/2.0 + th + eps11) ) );
		}

		double CylinderLeverArm::kfe_cyl_length(double th) const
		{
			static const double _2a2b2 = 2*a2*b2;
			return sqrt(a2_sqr + b2_sqr - _2a2b2*cos( (M_PI - th - (eps21 + eps22)) ) );
		}

		double CylinderLeverArm::hfe_lever(double c) const
		{
			return a1*sin( acos( (a1_sqr+c*c-b1_sqr) / (2*a1*c) ) );
		}

		double CylinderLeverArm::kfe_lever(double c) const
		{
			return a2*sin( acos( (a2_sqr+c*c-b2_sqr) / (2*a2*c) ) );
		}

		double CylinderLeverArm::hfe_joint_status(dog::JointIdentifiers joint, double cyl_length) const
		{
			double carg = acos((cyl_length * cyl_length - a1_sqr - b1_sqr) / (- 2 * a1 * b1));
			return + (carg - eps11 - M_PI/2.0) * length_sign_flip[joint];
		}

		double CylinderLeverArm::kfe_joint_status(dog::JointIdentifiers joint, double cyl_length) const
		{
			double carg = acos((cyl_length * cyl_length - a2_sqr - b2_sqr) / (- 2 * a2 * b2));
			return -(carg + eps21 + eps22 - M_PI) * length_sign_flip[joint];
		}
	} // namespace internal
} // namespace HyQ
} // namespace dog
} // namespace dls


