#ifndef DOGLIB_HYQREAL_JACOBIANS_HPP
#define DOGLIB_HYQREAL_JACOBIANS_HPP

#include "doglib/rbd/TransformsBase.h"
#include "doglib/base/declarations.hpp"
#include "doglib/base/kin_dyn_params.hpp"
#include "doglib/base/leg_data_map.hpp"
#include "doglib/hyqreal/kinematics_parameters.h"

namespace dls {
namespace dog {
namespace HyQReal {
typedef dls::dog::JointState JointState;

template<int COLS, class M>
class JacobianT : public dls::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians {
    public:
        class Type_fr_trunk_J_LF_foot : public JacobianT<3, Type_fr_trunk_J_LF_foot>
        {
        public:
            Type_fr_trunk_J_LF_foot(const Params_lengths& _lengths_values);
            const Type_fr_trunk_J_LF_foot& update(const JointState&);
        protected:
            const Params_lengths* lengths_values;
        };
        
        class Type_fr_trunk_J_RF_foot : public JacobianT<3, Type_fr_trunk_J_RF_foot>
        {
        public:
            Type_fr_trunk_J_RF_foot(const Params_lengths& _lengths_values);
            const Type_fr_trunk_J_RF_foot& update(const JointState&);
        protected:
            const Params_lengths* lengths_values;
        };
        
        class Type_fr_trunk_J_LH_foot : public JacobianT<3, Type_fr_trunk_J_LH_foot>
        {
        public:
            Type_fr_trunk_J_LH_foot(const Params_lengths& _lengths_values);
            const Type_fr_trunk_J_LH_foot& update(const JointState&);
        protected:
            const Params_lengths* lengths_values;
        };
        
        class Type_fr_trunk_J_RH_foot : public JacobianT<3, Type_fr_trunk_J_RH_foot>
        {
        public:
            Type_fr_trunk_J_RH_foot(const Params_lengths& _lengths_values);
            const Type_fr_trunk_J_RH_foot& update(const JointState&);
        protected:
            const Params_lengths* lengths_values;
        };
        
    public:
        Jacobians(const dog::KinDynParamsBase&);
        void updateParameters();
    public:
        Type_fr_trunk_J_LF_foot fr_trunk_J_LF_foot;
        Type_fr_trunk_J_RF_foot fr_trunk_J_RF_foot;
        Type_fr_trunk_J_LH_foot fr_trunk_J_LH_foot;
        Type_fr_trunk_J_RH_foot fr_trunk_J_RH_foot;
public:
        dog::FootJac getFootJacobianXY(const JointState & q,
                                       const dls::dog::LegID& leg,
                                       const double& foot_x,
                                       const double& foot_y);
	/*dog::FootJac getAngularFootJacobianXY(const JointState & q,
                                       const dls::dog::LegID& leg,
                                       const double& foot_x,
                                       const double& foot_y);*/


    protected:
        Params_lengths lengths_values;

        const dog::KinDynParamsBase& valuesGetter_lengths;
        dog::FootJac jacobian_;
};


} // HyQReal
} // dog
} // dls

#endif
