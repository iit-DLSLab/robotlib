#ifndef _ROBOTLIB_LIMB_HPP_
#define _ROBOTLIB_LIMB_HPP_

#include "limb_base.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Limb : public LimbBase
        {
        public:
            Limb(const std::string &name,
                 const std::array<const std::shared_ptr<const Joint>, NJOINTS> &joints,
                 const std::array<const std::shared_ptr<const Link>, NLINKS> &links);

            virtual ~Limb();

            virtual const std::shared_ptr<const Link> getLink(const int linkId) const override;
            virtual const std::shared_ptr<const Joint> getJoint(const int jointId) const override;
            virtual const int getNLinks() const override;
            virtual const int getNJoints() const override;

            virtual const std::shared_ptr<const Link> getLink(const std::string &name) const override;
            virtual const std::shared_ptr<const Joint> getJoint(const std::string &name) const override;
            virtual const std::shared_ptr<const Link> getEndEffector() const override;

            virtual const std::shared_ptr<const ContainerBase<const std::shared_ptr<const Joint>>> getJoints() { return joints_; };
            virtual const std::shared_ptr<const ContainerBase<const std::shared_ptr<const Link>>> getLinks() { return links_; };

        protected:
            const std::shared_ptr<const Container<const std::shared_ptr<const Joint>, NJOINTS>> joints_;
            const std::shared_ptr<const Container<const std::shared_ptr<const Link>, NLINKS>> links_;
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
