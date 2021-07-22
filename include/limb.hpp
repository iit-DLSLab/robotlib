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
            Limb(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> joints, const std::array<std::shared_ptr<Link>, NLINKS> links);

            virtual ~Limb();

            virtual std::shared_ptr<Frame> getLink(const int linkId) override;
            virtual std::shared_ptr<Frame> getJoint(const int jointId) override;
            virtual const int getNLinks() const override;
            virtual const int getNJoints() const override;

        protected:
            const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;
            const std::array<std::shared_ptr<Link>, NLINKS> links_;

            virtual void setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child);
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
