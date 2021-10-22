#ifndef _ROBOTLIB_LEG_HPP_
#define _ROBOTLIB_LEG_HPP_

#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Leg : public Limb<NJOINTS, NLINKS>
        {
        public:
            Leg(const std::string &name,
                const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Leg();
        };
    } // namespace robotlib
} // namespace dls

#include "leg.tpp"

#endif // _ROBOTLIB_LEG_HPP_
