/*
 * params.h
 *
 *  Created on: Sep 10, 2014
 *      Author: marco
 */

#ifndef DOGLIB_HYQREAL_HYDRAULICS_PARAMS_HPP
#define DOGLIB_HYQREAL_HYDRAULICS_PARAMS_HPP

#include "commons/hydraulics.h"

namespace dls {
namespace dog {
namespace HyQReal {


struct HydraulicsParameters
{
	dls::hydr::PipelineProperties pipeline;
	dls::hydr::MotorProperties    HAA_motor;
	dls::hydr::MotorProperties    HFE_motor;
	dls::hydr::CylinderProperties KFE_cylinder;
	dls::hydr::ValveProperties    valveParams;

	HydraulicsParameters();
};

} // HyQReal
} // dog
} // dls


#endif
