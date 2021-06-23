#ifndef _ROBOTLIB_DECLARATIONS_HPP_
#define _ROBOTLIB_DECLARATIONS_HPP_

#include <Eigen/Dense>

namespace dls
{
    namespace robotlib
    {
        // TODO: Parametric JointState
        typedef Eigen::Matrix<double, 12, 1> Column12d;
        typedef Column12d JointState;

        template <typename Scalar, int R, int C>
        using PlainMatrix = Eigen::Matrix<Scalar, R, C>;

        typedef PlainMatrix<double, 4, 4> HomogeneousTransformPlain;
    } // namespace hyqlib
} // namespace dls

#endif // _ROBOTLIB_DECLARATIONS_HPP_