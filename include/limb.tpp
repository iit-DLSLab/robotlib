#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::Limb(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> &joints, const std::array<std::shared_ptr<Link>, NLINKS> &links)
            : LimbBase(name), joints_(joints), links_(links){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::~Limb(){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        std::shared_ptr<void> Limb<NJOINTS, NLINKS>::getLink(const int linkId) { return links_[linkId]; };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        std::shared_ptr<void> Limb<NJOINTS, NLINKS>::getJoint(const int jointId) { return joints_[jointId]; };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const int Limb<NJOINTS, NLINKS>::getNLinks() const { return links_.size(); };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const int Limb<NJOINTS, NLINKS>::getNJoints() const { return joints_.size(); };
    } // namespace robotlib
} // namespace dls