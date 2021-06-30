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
            Limb(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> &joints, const std::array<std::shared_ptr<Link>, NLINKS> &links);

            virtual ~Limb();

            virtual std::shared_ptr<void> getLink(const int linkId) override;
            virtual std::shared_ptr<void> getJoint(const int jointId) override;
            virtual const int getNumLinks() override;
            virtual const int getNumJoints() override;

        protected:
            const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;
            const std::array<std::shared_ptr<Link>, NLINKS> links_;
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
