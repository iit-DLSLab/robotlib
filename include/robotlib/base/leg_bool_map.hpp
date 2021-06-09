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
#ifndef _ROBOTLIB_BASE_LEG_BOOL_MAP_HPP_
#define _ROBOTLIB_BASE_LEG_BOOL_MAP_HPP_

#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace dog {

/**
 * @brief The LegBoolMap class specializes the LegDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class LegBoolMap : public LegDataMap<bool>
{
public:
	LegBoolMap();
	LegBoolMap(const bool& defaultValue);
	LegBoolMap(const LegBoolMap& rhs);

	LegBoolMap operator!() const;
	LegBoolMap operator || (const LegBoolMap& rhs) const;
	inline LegBoolMap operator && (const LegBoolMap& rhs) const;

	static bool OR(const LegDataMap<bool>& rhs);
	bool OR() const;
	static bool AND(const LegDataMap<bool>& rhs);
	bool AND() const;
};

} // namespace dog
} // namespace dls

#endif // _ROBOTLIB_BASE_LEG_BOOL_MAP_HPP_
