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
#ifndef _ROBOTLIB_BASE_BODY_LENGTHS_HPP_
#define _ROBOTLIB_BASE_BODY_LENGTHS_HPP_

namespace dls
{
namespace robot
{
// =============================================================================
// class Interface
// =============================================================================
class RobotLengthsBase
{
public:
    virtual ~RobotLengthsBase() = default;

    virtual double getHAA_x() = 0;
    virtual double getHAA_y() = 0;
    virtual double getHAA_z() = 0;

    virtual double getDist_HAA_HFE() = 0;
    virtual double getDist_HFE_KFE() = 0;
    virtual double getFoot_x() = 0;
    virtual double getFoot_y() = 0;
    virtual double getFoot_z() = 0;
};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_BASE_BODY_LENGTHS_HPP_
