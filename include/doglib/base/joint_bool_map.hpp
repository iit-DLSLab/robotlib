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
#ifndef DOGLIB_BASE_JOINT_BOOL_MAP_HPP
#define DOGLIB_BASE_JOINT_BOOL_MAP_HPP

#include "doglib/base/joint_data_map.hpp"

namespace dls {
namespace dog {

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
} // namespace dog
} // namespace dls

#endif
