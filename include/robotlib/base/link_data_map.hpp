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
#ifndef _ROBOTLIB_BASE_LINK_DATA_MAP_HPP_
#define _ROBOTLIB_BASE_LINK_DATA_MAP_HPP_

#include "robotlib/base/declarations.hpp"

namespace dls {
namespace robot {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T>
class LinkDataMap
{
private:
	T data[linksCount];
public:
	LinkDataMap() {};
	LinkDataMap(const T& defaultValue);
	LinkDataMap(const LinkDataMap& rhs);
	LinkDataMap& operator=(const LinkDataMap& rhs);
	LinkDataMap& operator=(const T& rhs);
		  T& operator[](LinkIdentifiers which);
	const T& operator[](LinkIdentifiers which) const;
private:
	void copydata(const LinkDataMap& rhs);
	void assigndata(const T& commonValue);
};

} // namespace robot
} // namespace dls

#include "robotlib/base/link_data_map.tpp"
#endif // _ROBOTLIB_BASE_LINK_DATA_MAP_HPP_
