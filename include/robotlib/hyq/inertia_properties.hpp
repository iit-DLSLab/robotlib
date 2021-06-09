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
#ifndef _ROBOTLIB_HYQ_INERTIA_PROPERTIES_HPP_
#define _ROBOTLIB_HYQ_INERTIA_PROPERTIES_HPP_

#include <Eigen/Dense>
#include "robotlib/rbd/rbd.h"
#include "robotlib/rbd/InertiaMatrix.h"
#include "robotlib/rbd/utils.h"

#include "robotlib/base/declarations.hpp"
#include "robotlib/hyq/dynamics_parameters.hpp"
#include "robotlib/base/inertia_properties.hpp"
#include "robotlib/hyq/transforms.hpp"

namespace dls {
namespace dog {
namespace HyQ {
namespace dyn {

typedef dls::rbd::InertiaMatrixDense InertiaMatrix;

class InertiaProperties : public dog::InertiaPropertiesBase {
    public:
        InertiaProperties(const dog::KinDynParamsBase &);
        ~InertiaProperties();
        const InertiaMatrix& getTensor_trunk();
        const InertiaMatrix& getTensor_LF_hipassembly() const;
        const InertiaMatrix& getTensor_LF_upperleg() const;
        const InertiaMatrix& getTensor_LF_lowerleg() const;
        const InertiaMatrix& getTensor_RF_hipassembly() const;
        const InertiaMatrix& getTensor_RF_upperleg() const;
        const InertiaMatrix& getTensor_RF_lowerleg() const;
        const InertiaMatrix& getTensor_LH_hipassembly() const;
        const InertiaMatrix& getTensor_LH_upperleg() const;
        const InertiaMatrix& getTensor_LH_lowerleg() const;
        const InertiaMatrix& getTensor_RH_hipassembly() const;
        const InertiaMatrix& getTensor_RH_upperleg() const;
        const InertiaMatrix& getTensor_RH_lowerleg() const;
        double getMass_trunk() const;
        double getMass_LF_hipassembly() const;
        double getMass_LF_upperleg() const;
        double getMass_LF_lowerleg() const;
        double getMass_RF_hipassembly() const;
        double getMass_RF_upperleg() const;
        double getMass_RF_lowerleg() const;
        double getMass_LH_hipassembly() const;
        double getMass_LH_upperleg() const;
        double getMass_LH_lowerleg() const;
        double getMass_RH_hipassembly() const;
        double getMass_RH_upperleg() const;
        double getMass_RH_lowerleg() const;
        const dls::rbd::Vector3d& getCOM_trunk() const;
        const dls::rbd::Vector3d& getCOM_LF_hipassembly() const;
        const dls::rbd::Vector3d& getCOM_LF_upperleg() const;
        const dls::rbd::Vector3d& getCOM_LF_lowerleg() const;
        const dls::rbd::Vector3d& getCOM_RF_hipassembly() const;
        const dls::rbd::Vector3d& getCOM_RF_upperleg() const;
        const dls::rbd::Vector3d& getCOM_RF_lowerleg() const;
        const dls::rbd::Vector3d& getCOM_LH_hipassembly() const;
        const dls::rbd::Vector3d& getCOM_LH_upperleg() const;
        const dls::rbd::Vector3d& getCOM_LH_lowerleg() const;
        const dls::rbd::Vector3d& getCOM_RH_hipassembly() const;
        const dls::rbd::Vector3d& getCOM_RH_upperleg() const;
        const dls::rbd::Vector3d& getCOM_RH_lowerleg() const;
public:
        double getTotalMass() const;
        const dls::rbd::Vector3d& getTrunkCOM();
        double getTrunkMass();

        double getHipAssemblyMass(const dog::LegID& leg) const;
        double getUpperLegMass(const dog::LegID& leg) const;
        double getLowerLegMass(const dog::LegID& leg) const;

		dls::rbd::Vector3d getHipAssemblyCOM(const dog::LegID& leg) const;
        dls::rbd::Vector3d getUpperLegCOM(const dog::LegID& leg) const;
        dls::rbd::Vector3d getLowerLegCOM(const dog::LegID& leg) const;

        dls::rbd::Vector3d getWholeBodyCOM(const dog::JointState &q);


        /*!
         * Forces the update of the runtime inertia parameters of the robot
         * HyQ.
         *
         * This function uses in turn the getter methods of the RuntimeParamsGetter
         * member of this instance. All the inertia properties contained in
         * this instance that are defined in terms of non-constant parameters,
         * will be updated.
         */
        void updateParameters();
    private:
        const dog::KinDynParamsBase& paramsGetter;

        HomogeneousTransforms homogeneus_transforms;

        InertiaMatrix tensor_trunk;
        InertiaMatrix tensor_LF_hipassembly;
        InertiaMatrix tensor_LF_upperleg;
        InertiaMatrix tensor_LF_lowerleg;
        InertiaMatrix tensor_RF_hipassembly;
        InertiaMatrix tensor_RF_upperleg;
        InertiaMatrix tensor_RF_lowerleg;
        InertiaMatrix tensor_LH_hipassembly;
        InertiaMatrix tensor_LH_upperleg;
        InertiaMatrix tensor_LH_lowerleg;
        InertiaMatrix tensor_RH_hipassembly;
        InertiaMatrix tensor_RH_upperleg;
        InertiaMatrix tensor_RH_lowerleg;
        dls::rbd::Vector3d com_trunk;
        dls::rbd::Vector3d com_LF_hipassembly;
        dls::rbd::Vector3d com_LF_upperleg;
        dls::rbd::Vector3d com_LF_lowerleg;
        dls::rbd::Vector3d com_RF_hipassembly;
        dls::rbd::Vector3d com_RF_upperleg;
        dls::rbd::Vector3d com_RF_lowerleg;
        dls::rbd::Vector3d com_LH_hipassembly;
        dls::rbd::Vector3d com_LH_upperleg;
        dls::rbd::Vector3d com_LH_lowerleg;
        dls::rbd::Vector3d com_RH_hipassembly;
        dls::rbd::Vector3d com_RH_upperleg;
        dls::rbd::Vector3d com_RH_lowerleg;
};


} // dyn
} // HyQ
} // dog
} // dls

#endif // _ROBOTLIB_HYQ_INERTIA_PROPERTIES_HPP_