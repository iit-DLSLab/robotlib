#ifndef _ROBOTLIB_ARM_BASE_HPP_
#define _ROBOTLIB_ARM_BASE_HPP_

#include "limb_base.hpp"

namespace dls
{
    namespace robotlib
    {
        /**
        * An arm interface for robots.
        */

        class ArmBase : public LimbBase
        {
        public:
            ArmBase(const std::string &name) : LimbBase(name){};

            ~ArmBase(){};

            // TODO: Functions for arms (and not for legs)
        };

    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_ARM_BASE_HPP_
