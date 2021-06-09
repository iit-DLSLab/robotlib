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
#ifndef _ROBOTLIB_BASE_JOINT_BOOL_MAP_HPP_
#define _ROBOTLIB_BASE_JOINT_BOOL_MAP_HPP_

#include "robotlib/base/joint_data_map.hpp"

namespace dls {
namespace robot {

/**
 * @brief The JointBoolMap class specializes the JointDataMap with extended
 * functionalities for logic operations when the type is a bool, including:
 * negation, or, and, cwise or and cwise and.
 */
class JointBoolMap : public JointDataMap<bool> {

public:
    JointBoolMap();
    JointBoolMap(const bool& defaultValue);
    JointBoolMap(const JointBoolMap& rhs);

    JointBoolMap operator!() const;
    JointBoolMap operator || (const JointBoolMap& rhs) const;
    JointBoolMap operator && (const JointBoolMap& rhs) const;

    static bool OR(const JointDataMap<bool>& rhs);
    bool OR() const;
    static inline bool AND(const JointDataMap<bool>& rhs);
    bool AND() const;
};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_BASE_JOINT_BOOL_MAP_HPP_
