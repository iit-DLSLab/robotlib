#ifndef _ROBOTLIB_ARM_BASE_HPP_
#define _ROBOTLIB_ARM_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================

#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"

namespace dls
{
    namespace robot
    {
        /**
 * An arm interface for robots.
 */

        class ArmBase : public LimbBase
        {
        public:
            ArmBase(const std::string &name) : LimbBase(name){};

            ~ArmBase(){};

            // Functions for arms (and not for legs) TO DO
        };

    } // namespace robot
} // namespace dls

#endif // _ROBOTLIB_ARM_BASE_HPP_
