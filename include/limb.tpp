#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::Limb(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> &joints, const std::array<std::shared_ptr<Link>, NLINKS> &links)
            : LimbBase(name),
              joints_(std::make_shared<Container<Joint, NJOINTS>>(joints)),
              links_(std::make_shared<Container<Link, NLINKS>>(links)){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::~Limb(){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<Link> Limb<NJOINTS, NLINKS>::getLink(const int linkId) const
        {
            return (*links_)[linkId];
        };
        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<Joint> Limb<NJOINTS, NLINKS>::getJoint(const int jointId) const
        {
            return (*joints_)[jointId];
        };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const int Limb<NJOINTS, NLINKS>::getNLinks() const
        {
            return links_->size();
        };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const int Limb<NJOINTS, NLINKS>::getNJoints() const { return joints_->size(); };
        template <unsigned int NJOINTS, unsigned int NLINKS>
        void Limb<NJOINTS, NLINKS>::setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child)
        {
            joint->setChild(child);
        }

    } // namespace robotlib
} // namespace dls