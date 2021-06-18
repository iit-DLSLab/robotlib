#ifndef _ROBOTLIB_LEG_BASE_HPP_
#define _ROBOTLIB_LEG_BASE_HPP_

#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"

namespace dls
{
    namespace robotlib
    {
        /**
        * A leg interface for robots.
        */

        class LegBase : public LimbBase
        {
        public:
            LegBase(const std::string &name) : LimbBase(name){};

            ~LegBase(){};

            // Functions for legs (and not for arms) TO DO
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LEG_BASE_HPP_
