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
// =============================================================================
// Includes
// =============================================================================
#include "robotlib/hyq/body_lengths.hpp"
// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::dog::HyQ;

// =============================================================================
// Constructors
// =============================================================================
HyQLengths::HyQLengths(dls::dog::KinDynParamsBase &getter) :
	pGetter(getter)
{ }

// =============================================================================
// Implementation
// =============================================================================
double  HyQLengths::getHAA_x()         {  return  BASE2HAAX;                  }
double  HyQLengths::getHAA_y()         {  return  BASE2HAAY;                  }
double  HyQLengths::getHAA_z()         {  return  0;                          }
double  HyQLengths::getDist_HAA_HFE()  {  return  HAA2HFE;                    }
double  HyQLengths::getDist_HFE_KFE()  {  return  upleg_length;               }
double  HyQLengths::getFoot_x()        {  return  pGetter.getValue_foot_x();  }
double  HyQLengths::getFoot_y()        {  return  0;                          }
double  HyQLengths::getFoot_z()        {  return  0;                          }
