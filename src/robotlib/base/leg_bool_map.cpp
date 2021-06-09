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
#include "robotlib/base/leg_bool_map.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::robot;

// =============================================================================
// Constructors
// =============================================================================
LegBoolMap::LegBoolMap() :
	LegDataMap<bool>(false)
{ }

LegBoolMap::LegBoolMap(const bool& defaultValue) :
	LegDataMap<bool>(defaultValue)
{ }

LegBoolMap::LegBoolMap(const LegBoolMap& rhs) :
	LegDataMap<bool>(rhs)
{ }

// =============================================================================
// Implementation
// =============================================================================
LegBoolMap LegBoolMap::operator!() const
{
	LegBoolMap res;
	res[LF] = !(*this)[LF];
	res[RF] = !(*this)[RF];
	res[LH] = !(*this)[LH];
	res[RH] = !(*this)[RH];
	return res;
}

LegBoolMap LegBoolMap::operator || (const LegBoolMap& rhs) const
{
	LegBoolMap res;
	res[LF] = (*this)[LF] || rhs[LF];
	res[RF] = (*this)[RF] || rhs[RF];
	res[LH] = (*this)[LH] || rhs[LH];
	res[RH] = (*this)[RH] || rhs[RH];
	return res;
}

LegBoolMap LegBoolMap::operator && (const LegBoolMap& rhs) const
{
	LegBoolMap res;
	res[LF] = (*this)[LF] && rhs[LF];
	res[RF] = (*this)[RF] && rhs[RF];
	res[LH] = (*this)[LH] && rhs[LH];
	res[RH] = (*this)[RH] && rhs[RH];
	return res;
}

bool LegBoolMap::OR(const LegDataMap<bool>& rhs)
{
	return(rhs[LF] || rhs[RF] || rhs[LH] || rhs[RH]);
}

bool LegBoolMap::OR() const
{
	return ((*this)[LF] || (*this)[RF] || (*this)[LH] || (*this)[RH]);
}


bool LegBoolMap::AND(const LegDataMap<bool>& rhs)
{
	return(rhs[LF] && rhs[RF] && rhs[LH] && rhs[RH]);
}

bool LegBoolMap::AND() const
{
	return ((*this)[LF] && (*this)[RF] && (*this)[LH] && (*this)[RH]);
}

