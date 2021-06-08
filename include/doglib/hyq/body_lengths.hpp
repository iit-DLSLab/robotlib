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
#ifndef DOGLIB_HYQ_BODY_LENGTHS_HPP
#define DOGLIB_HYQ_BODY_LENGTHS_HPP
// =============================================================================
// Includes
// =============================================================================
#include "doglib/base/kin_dyn_params.hpp"
#include "doglib/base/body_lengths.hpp"

#include "doglib/hyq/constants.hpp"

namespace dls
{
namespace dog
{
namespace HyQ
{

// =============================================================================
// Declaration
// =============================================================================
class HyQLengths : public dls::dog::RobotLengthsBase
{
public:
	HyQLengths(dls::dog::KinDynParamsBase &getter);
	~HyQLengths() = default;

	double getHAA_x();
	double getHAA_y();
	double getHAA_z();
	double getDist_HAA_HFE();
	double getDist_HFE_KFE();
	double getFoot_x();
	double getFoot_y();
	double getFoot_z();

private:
	dls::dog::KinDynParamsBase &pGetter;
};


} // namespace HyQ
} // namespace dog
} // namespace dls

#endif
