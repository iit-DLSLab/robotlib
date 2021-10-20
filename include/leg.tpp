#include "leg.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        Leg<NJOINTS, NLINKS>::Leg(const std::string &name,
                                  const std::array<std::shared_ptr<const Joint>, NJOINTS> &joints,
                                  const std::array<std::shared_ptr<const Link>, NLINKS> &links)
            : Limb<NJOINTS, NLINKS>(name, joints, links){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        Leg<NJOINTS, NLINKS>::~Leg(){};
    } // namespace robotlib
} // namespace dls