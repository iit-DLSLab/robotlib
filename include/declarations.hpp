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
    } // namespace hyqlib
} // namespace dls

#endif // _ROBOTLIB_DECLARATIONS_HPP_