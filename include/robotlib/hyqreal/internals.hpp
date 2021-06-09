/*
 * internals.h
 *
 *  Created on: Sep 10, 2014
 *      Author: marco
 */

#ifndef _ROBOTLIB_HYQREAL_INTERNALS_HPP_
#define _ROBOTLIB_HYQREAL_INTERNALS_HPP_

#include "robotlib/base/leg_data_map.hpp"

namespace dls {
namespace dog {
namespace HyQReal {
namespace internal {

struct MagicNumbers {
    MagicNumbers();

	dls::dog::LegDataMap<double> HFEJointToMotorOffset;
    dls::dog::LegDataMap<double> HAAJointToMotorOffset;

};

extern const MagicNumbers misc_cfg;

} // internal
} // HyQReal
} // dog
} // dls


#endif
