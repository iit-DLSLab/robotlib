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
#include "robotlib/base/joint_bool_map.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::robot;

// =============================================================================
// Constructors
// =============================================================================
JointBoolMap::JointBoolMap() :
	JointDataMap<bool>(false)
{ }

JointBoolMap::JointBoolMap(const bool& defaultValue) :
	JointDataMap<bool>(defaultValue)
{ }

JointBoolMap::JointBoolMap(const JointBoolMap& rhs) :
	JointDataMap<bool>(rhs)
{ }

// =============================================================================
// Implementation
// =============================================================================
JointBoolMap JointBoolMap::operator!() const
{
	JointBoolMap res;
	res[LF_HAA] = !(*this)[LF_HAA];
	res[LF_HFE] = !(*this)[LF_HFE];
	res[LF_KFE] = !(*this)[LF_KFE];
	res[RF_HAA] = !(*this)[RF_HAA];
	res[RF_HFE] = !(*this)[RF_HFE];
	res[RF_KFE] = !(*this)[RF_KFE];
	res[LH_HAA] = !(*this)[LH_HAA];
	res[LH_HFE] = !(*this)[LH_HFE];
	res[LH_KFE] = !(*this)[LH_KFE];
	res[RH_HAA] = !(*this)[RH_HAA];
	res[RH_HFE] = !(*this)[RH_HFE];
	res[RH_KFE] = !(*this)[RH_KFE];
	return res;
}

JointBoolMap JointBoolMap::operator ||(const JointBoolMap& rhs) const
{
	JointBoolMap res;
	res[LF_HAA] = (*this)[LF_HAA] || rhs[LF_HAA];
	res[LF_HFE] = (*this)[LF_HFE] || rhs[LF_HFE];
	res[LF_KFE] = (*this)[LF_KFE] || rhs[LF_KFE];
	res[RF_HAA] = (*this)[RF_HAA] || rhs[RF_HAA];
	res[RF_HFE] = (*this)[RF_HFE] || rhs[RF_HFE];
	res[RF_KFE] = (*this)[RF_KFE] || rhs[RF_KFE];
	res[LH_HAA] = (*this)[LH_HAA] || rhs[LH_HAA];
	res[LH_HFE] = (*this)[LH_HFE] || rhs[LH_HFE];
	res[LH_KFE] = (*this)[LH_KFE] || rhs[LH_KFE];
	res[RH_HAA] = (*this)[RH_HAA] || rhs[RH_HAA];
	res[RH_HFE] = (*this)[RH_HFE] || rhs[RH_HFE];
	res[RH_KFE] = (*this)[RH_KFE] || rhs[RH_KFE];
	return res;
}

JointBoolMap JointBoolMap::operator && (const JointBoolMap& rhs) const
{
	JointBoolMap res;
	res[LF_HAA] = (*this)[LF_HAA] && rhs[LF_HAA];
	res[LF_HFE] = (*this)[LF_HFE] && rhs[LF_HFE];
	res[LF_KFE] = (*this)[LF_KFE] && rhs[LF_KFE];
	res[RF_HAA] = (*this)[RF_HAA] && rhs[RF_HAA];
	res[RF_HFE] = (*this)[RF_HFE] && rhs[RF_HFE];
	res[RF_KFE] = (*this)[RF_KFE] && rhs[RF_KFE];
	res[LH_HAA] = (*this)[LH_HAA] && rhs[LH_HAA];
	res[LH_HFE] = (*this)[LH_HFE] && rhs[LH_HFE];
	res[LH_KFE] = (*this)[LH_KFE] && rhs[LH_KFE];
	res[RH_HAA] = (*this)[RH_HAA] && rhs[RH_HAA];
	res[RH_HFE] = (*this)[RH_HFE] && rhs[RH_HFE];
	res[RH_KFE] = (*this)[RH_KFE] && rhs[RH_KFE];
	return res;
}

bool JointBoolMap::OR(const JointDataMap<bool>& rhs)
{
	return
	(
		rhs[LF_HAA] || rhs[LF_HFE] || rhs[LF_KFE] ||
		rhs[RF_HAA] || rhs[RF_HFE] || rhs[RF_KFE] ||
		rhs[LH_HAA] || rhs[LH_HFE] || rhs[LH_KFE] ||
		rhs[RH_HAA] || rhs[RH_HFE] || rhs[RH_KFE]
	);
}

bool JointBoolMap::OR() const
{
	return
	(
		(*this)[LF_HAA] || (*this)[LF_HFE] || (*this)[LF_KFE] ||
		(*this)[RF_HAA] || (*this)[RF_HFE] || (*this)[RF_KFE] ||
		(*this)[LH_HAA] || (*this)[LH_HFE] || (*this)[LH_KFE] ||
		(*this)[RH_HAA] || (*this)[RH_HFE] || (*this)[RH_KFE]
	);
}

bool JointBoolMap::AND(const JointDataMap<bool>& rhs)
{
	return
	(
		rhs[LF_HAA] && rhs[LF_HFE] && rhs[LF_KFE] &&
		rhs[RF_HAA] && rhs[RF_HFE] && rhs[RF_KFE] &&
		rhs[LH_HAA] && rhs[LH_HFE] && rhs[LH_KFE] &&
		rhs[RH_HAA] && rhs[RH_HFE] && rhs[RH_KFE]
	);
}

bool JointBoolMap::AND() const
{
	return
	(
		(*this)[LF_HAA] && (*this)[LF_HFE] && (*this)[LF_KFE] &&
		(*this)[RF_HAA] && (*this)[RF_HFE] && (*this)[RF_KFE] &&
		(*this)[LH_HAA] && (*this)[LH_HFE] && (*this)[LH_KFE] &&
		(*this)[RH_HAA] && (*this)[RH_HFE] && (*this)[RH_KFE]
	);
}
