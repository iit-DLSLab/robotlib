/* CPYHDR { */
/*
 * This file is part of the 'dls-rbd' library.
 * Copyright © 2015 2016 2017, Marco Frigerio (marco.frigerio@dls.it)
 *
 * See the LICENSE file for more information.
 */
/* } CPYHDR */

#ifndef DOGLIB_RBD_EIGEN_TRAITS_HPP
#define DOGLIB_RBD_EIGEN_TRAITS_HPP


#include "doglib/rbd/StateDependentMatrix.h"
#include "doglib/rbd/TransformsBase.h"

/**
 * \file
 * This header file contains some instantiations of the \c traits template (in
 * the \c Eigen::internal namespace) for matrix types defined in \c dls::rbd
 */

namespace Eigen {
namespace internal {

/**
 * The Eigen traits for the dls::rbd::HomogeneousTransformBase type
 */
template<typename State, typename M>
struct traits< dls::rbd::HomogeneousTransformBase<State, M> >
{
        typedef typename dls::rbd::HomogeneousTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};

/**
 * The Eigen traits for the dls::rbd::RotationTransformBase type
 */
template<typename State, typename M>
struct traits< dls::rbd::RotationTransformBase<State, M> >
{
        typedef typename dls::rbd::RotationTransformBase<State, M>::MatrixType MxType;
        typedef traits<MxType> Traits;
        typedef typename Traits::Scalar Scalar;
        typedef typename Traits::StorageKind StorageKind;
        typedef typename Traits::Index Index;
        typedef typename Traits::XprKind XprKind;
        enum {
            RowsAtCompileTime    = Traits::RowsAtCompileTime,
            ColsAtCompileTime    = Traits::ColsAtCompileTime,
            MaxRowsAtCompileTime = Traits::MaxRowsAtCompileTime,
            MaxColsAtCompileTime = Traits::MaxColsAtCompileTime,
            Options = Traits::Options,
            Flags   = Traits::Flags,
            CoeffReadCost = Traits::CoeffReadCost,
            InnerStrideAtCompileTime = Traits::InnerStrideAtCompileTime,
            OuterStrideAtCompileTime = Traits::OuterStrideAtCompileTime
        };
};


}
}

#endif
