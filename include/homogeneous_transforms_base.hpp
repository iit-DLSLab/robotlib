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

            virtual HomogeneousTransformPlain getTransform(const JointState &q,
                                                           int origin,
                                                           int destination) = 0;
            // TODO: Review values below
            //const OriginFrame &orig,
            //const DestFrame &dest) = 0;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_HOMOGENEOUS_TRANSFORMS_BASE_HPP_