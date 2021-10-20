#include "limb.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::Limb(
            const std::string &name,
            const std::array<const std::shared_ptr<const Joint>, NJOINTS> &joints,
            const std::array<const std::shared_ptr<const Link>, NLINKS> &links)
            : LimbBase(name),
              joints_(std::make_shared<const Container<const std::shared_ptr<const Joint>, NJOINTS>>(joints)),
              links_(std::make_shared<const Container<const std::shared_ptr<const Link>, NLINKS>>(links)){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        Limb<NJOINTS, NLINKS>::~Limb(){};

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<const Link> Limb<NJOINTS, NLINKS>::getLink(const int linkId) const
        {
            return (*links_)[linkId];
        };
        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<const Joint> Limb<NJOINTS, NLINKS>::getJoint(const int jointId) const
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
        const std::shared_ptr<const Link> Limb<NJOINTS, NLINKS>::getLink(const std::string &name) const
        {
            //Iterate over the array of links to find the link
            for (auto link : *links_)
            {
                if (link->getName().compare(name) == 0)
                {
                    return link;
                }
            }

            //std::cout << "LINK " << name << " NOT FOUND IN LEG " << this->getName() << std::endl;
            return std::shared_ptr<const Link>(nullptr);
        };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<const Joint> Limb<NJOINTS, NLINKS>::getJoint(const std::string &name) const
        {
            //Iterate over the array of links to find the link
            for (auto joint : *joints_)
            {
                if (joint->getName().compare(name) == 0)
                {
                    return joint;
                }
            }

            //std::cout << "JOINT NOT FOUD FROM THE INPUT NAME " << name << std::endl;
            return std::shared_ptr<const Joint>(nullptr);
        };

        template <unsigned int NJOINTS, unsigned int NLINKS>
        const std::shared_ptr<const Link> Limb<NJOINTS, NLINKS>::getEndEffector() const
        {
            return *(--links_->end());
        };

    } // namespace robotlib
} // namespace dls