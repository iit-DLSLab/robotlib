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
#ifndef DOGLIB_BAE_BODY_LENGTHS_SDFJLSZ_HPP
#define DOGLIB_BAE_BODY_LENGTHS_SDFJLSZ_HPP

namespace dls
{
namespace dog
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
} // namespace dog
} // namespace dls

#endif
