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
                 const std::array<std::shared_ptr<Joint>, NJOINTS> &joints,
                 const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Limb();

            virtual const int getNLinks() const override;
            virtual const int getNJoints() const override;

            virtual const std::shared_ptr<Link> getLink(const std::string &name) const override;
            virtual const std::shared_ptr<Joint> getJoint(const std::string &name) const override;
            virtual const std::shared_ptr<Link> getEndEffector() const override;

            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> getJoints() const { return joints_; };
            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<Link>>> getLinks() const { return links_; };

        protected:
            const std::shared_ptr<const Container<std::shared_ptr<Joint>, NJOINTS>> joints_;
            const std::shared_ptr<const Container<std::shared_ptr<Link>, NLINKS>> links_;
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
