/* CPYHDR { */
/*
 * This file is part of the 'dls-rbd' library.
 * Copyright © 2015 2016 2017, Marco Frigerio (marco.frigerio@dls.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef _ROBOTLIB_RBD_TRANSFORMS_BASE_HPP_
#define _ROBOTLIB_RBD_TRANSFORMS_BASE_HPP_

#include "robotlib/rbd/StateDependentMatrix.h"

namespace dls {
namespace rbd {


/**
 * A 3x3 specialization of StateDependentMatrix, to be used as a base class for
 * rotation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class RotationTransformBase : public StateDependentMatrix<State, 3, 3, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 4x4 specialization of StateDependentMatrix, to be used as a base class for
 * homogeneous transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class HomogeneousTransformBase : public StateDependentMatrix<State, 4, 4, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6x6 specialization of StateDependentMatrix, to be used as a base class for
 * spatial transformation matrices that depend on a state variable.
 */
template<class State, class ActualMatrix>
class SpatialTransformBase : public StateDependentMatrix<State, 6, 6, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * A 6xCols specialization of StateDependentMatrix, to be used as a base class
 * for geometric Jacobians that depend on a state variable.
 */
template<class State, int Cols, class ActualMatrix>
class JacobianBase : public StateDependentMatrix<State, 6, Cols, ActualMatrix>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}
}



#endif // _ROBOTLIB_RBD_TRANSFORMS_BASE_HPP_
