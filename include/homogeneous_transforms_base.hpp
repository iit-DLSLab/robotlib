#ifndef _ROBOTLIB_HOMOGENEOUS_TRANSFORMS_BASE_HPP_
#define _ROBOTLIB_HOMOGENEOUS_TRANSFORMS_BASE_HPP_

#include "declarations.hpp"

namespace dls
{
    namespace robotlib
    {
        class HomogeneousTransformsBase
        {
        public:
            HomogeneousTransformsBase(){};
            virtual ~HomogeneousTransformsBase(){};

        private:
            virtual HomogeneousTransformPlain getTransform(const JointState &q,
                                                           const int &orig,
                                                           const int &dest) = 0;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_HOMOGENEOUS_TRANSFORMS_BASE_HPP_