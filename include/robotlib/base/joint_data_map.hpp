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
#ifndef _ROBOTLIB_BASE_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_BASE_JOINT_DATA_MAP_HPP_

#include "robotlib/base/joint_id_tricks.hpp"

namespace dls
{
namespace dog
{

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T>
class JointDataMap
{
private:
	T data[jointsCount];
public:
	JointDataMap() {};
	JointDataMap(const T& defaultValue);
	JointDataMap(const JointDataMap& rhs);
	JointDataMap& operator=(const JointDataMap& rhs);
	JointDataMap& operator=(const T& rhs);
	T& operator[](JointIdentifiers which);
	const T& operator[](JointIdentifiers which) const;
private:
	void copydata(const JointDataMap& rhs);
	void assigndata(const T& rhs);
};
} // namespace dog
} // namespace dls
#include "robotlib/base/joint_data_map.tpp"
#endif // _ROBOTLIB_BASE_JOINT_DATA_MAP_HPP_
