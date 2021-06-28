#ifndef _ROBOTLIB_ARM_HPP_
#define _ROBOTLIB_ARM_HPP_

#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Arm : public Limb<NJOINTS, NLINKS>
        {
        public:
            Arm(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> &joints, const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Arm();
        };
    } // namespace robotlib
} // namespace dls

#include "../src/arm.tpp"

#endif // _ROBOTLIB_ARM_HPP_
