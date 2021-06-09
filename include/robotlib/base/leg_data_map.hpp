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
#ifndef _ROBOTLIB_BASE_LEG_DATA_MAP_HPP_
#define _ROBOTLIB_BASE_LEG_DATA_MAP_HPP_
#include <stdexcept>

namespace dls {
/**
 * This namespace holds interfaces and data types suitable to be used with
 * quadruped robots.
 *
 * More or less everything is based on the assumption of having four legs and
 * three joints per leg.
 */
namespace dog {

static const int _LEGS_COUNT = 4;
enum LegID{LF=0, RF, LH, RH};
enum LegJoints{HAA=0, HFE=1, KFE=2};


/**
 * A very simple container to associate a generic data item to each leg
 * (or anything related to a leg, e.g. a hip)
 *
 * \tparam T the type of the items that will be stored in the data map.
 */
template<typename T>
class LegDataMap
{
private:
	T data[_LEGS_COUNT];
public:
	LegDataMap() {};
	LegDataMap(const T& defaultValue);
	LegDataMap(const LegDataMap& rhs);
	LegDataMap& operator=(const LegDataMap& rhs);
	LegDataMap& operator=(const T& defaultValue);
	T& operator[](LegID index);
	T& operator[](int index) throw(std::runtime_error);
	const T& operator[](LegID index) const;
	const T& operator[](int index) const throw(std::runtime_error);
private:
	void copydata(const LegDataMap& rhs);
	void assignAll(const T& value);
};
} // namespace dog
} // namespace dls
#include "robotlib/base/leg_data_map.tpp"

#endif // _ROBOTLIB_BASE_LEG_DATA_MAP_HPP_
