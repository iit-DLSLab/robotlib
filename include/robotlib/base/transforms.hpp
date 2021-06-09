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
#ifndef _ROBOTLIB_BASE_TRANSFORMS_HPP_
#define _ROBOTLIB_BASE_TRANSFORMS_HPP_

#include "robotlib/rbd/TransformsBase.h"
#include "robotlib/rbd/rbd.h"

#include "robotlib/base/declarations.hpp"

namespace dls {
namespace dog {

// =============================================================================
// Typedefs
// =============================================================================
typedef typename dls::rbd::PlainMatrix<double, 6, 6> SpatialTransformPlain ;
typedef typename dls::rbd::PlainMatrix<double, 4, 4> HomogeneousTransformPlain ;

// =============================================================================
// Enumerations
// =============================================================================
enum class DestFrame
{
	TRUNK = 0,
	LF_LOWERLEG,
	RF_LOWERLEG,
	LH_LOWERLEG,
	RH_LOWERLEG,
	LF_FOOT,
	RF_FOOT,
	LH_FOOT,
	RH_FOOT,
	LF_HIPASSEMBLY,
	RF_HIPASSEMBLY,
	LH_HIPASSEMBLY,
	RH_HIPASSEMBLY,
	LF_UPPERLEG,
	RF_UPPERLEG,
	LH_UPPERLEG,
	RH_UPPERLEG
};

enum class OriginFrame
{
	TRUNK = 0,
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
	RH_KFE,
	LF_LOWERLEG,
	RF_LOWERLEG,
	LH_LOWERLEG,
	RH_LOWERLEG,
	LF_HIPASSEMBLY_COM,
	RF_HIPASSEMBLY_COM,
	LH_HIPASSEMBLY_COM,
	RH_HIPASSEMBLY_COM,
	LF_HIPASSEMBLY,
	RF_HIPASSEMBLY,
	LH_HIPASSEMBLY,
	RH_HIPASSEMBLY,
	LF_UPPERLEG_COM,
	RF_UPPERLEG_COM,
	LH_UPPERLEG_COM,
	RH_UPPERLEG_COM,
	LF_UPPERLEG,
	RF_UPPERLEG,
	LH_UPPERLEG,
	RH_UPPERLEG,
	LF_LOWERLEG_COM,
	RF_LOWERLEG_COM,
	LH_LOWERLEG_COM,
	RH_LOWERLEG_COM,
	LF_SHIN,
	RF_SHIN,
	LH_SHIN,
	RH_SHIN,
	LF_FOOT,
	RF_FOOT,
	LH_FOOT,
	RH_FOOT
};

// =============================================================================
// Classes
// =============================================================================
class HomogeneousTransformsBase
{
public:
	virtual ~HomogeneousTransformsBase() = default;
	virtual HomogeneousTransformPlain getTransform
	(
		const JointState& q,
		const OriginFrame& orig,
		const DestFrame& dest
	) = 0;
};

class MotionTransformsBase
{
public:
	virtual ~MotionTransformsBase() = default;

	virtual SpatialTransformPlain getTransform
	(
		const JointState& q,
		const OriginFrame& orig,
		const DestFrame& dest
	) = 0;
};

class ForceTransformsBase
{
public:
	virtual ~ForceTransformsBase() {}
	virtual SpatialTransformPlain getTransform
	(
		const JointState& q,
		const OriginFrame& orig,
		const DestFrame& dest
	) = 0;
};

} // namespace dog
} // namespace dls

#endif // _ROBOTLIB_BASE_TRANSFORMS_HPP_
