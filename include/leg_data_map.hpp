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
// #ifndef ROBOTLIB_BASE_LEG_DATA_MAP_HPP
// #define ROBOTLIB_BASE_LEG_DATA_MAP_HPP
// #include <stdexcept>

// #include "robot.hpp"

// namespace dls {
// /**
//  * This namespace holds interfaces and data types suitable to be used with
//  * robot having any number of legs.
//  */
// namespace robot {

// /**
//  * A very simple container to associate a generic data item to each leg
//  * (or anything related to a leg, e.g. a hip)
//  *
//  * \tparam Data the type of the items that will be stored in the data map.
//  * \tparam Robot<unsigned int NLEGS> the robot type with NLEGS number of legs.
//  */

// template<class Robot<unsigned int NLEGS>, class Data>
// class LegDataMap
// {
// private:
// 	std::array<Data, NLEGS> data_;

// public:
// 	LegDataMap() {};
// 	LegDataMap(const Data& defaultValue);
// 	// LegDataMap(const LegDataMap& rhs);
// 	// LegDataMap& operator=(const LegDataMap& rhs);
// 	// LegDataMap& operator=(const Data& defaultValue);
// 	Data& operator[](int index) throw(std::runtime_error);
// 	// const Data& operator[](int index) const throw(std::runtime_error);
// private:
// 	//void copydata(const LegDataMap& rhs);
// 	void assignAll(const Data& value);
// };
// } // namespace robot
// } // namespace dls
// #include "leg_data_map.tpp"

// #endif

