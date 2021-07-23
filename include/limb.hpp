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

            //Dummy container for joints (it generalizes the number of joints)
            class Joints : public JointsBase
            {
            public:
                Joints(const std::array<std::shared_ptr<Joint>, NJOINTS> joints) : joints_(joints){};

                virtual ~Joints(){};

                virtual Iterator<const std::shared_ptr<Joint>> begin() const { return Iterator<const std::shared_ptr<Joint>>(&joints_[0]); };
                virtual Iterator<const std::shared_ptr<Joint>> end() const { return Iterator<const std::shared_ptr<Joint>>(&joints_[NJOINTS]); };

                virtual const std::shared_ptr<Joint> operator[](const int id) const { return joints_[id]; };

                virtual const int size() const { return joints_.size(); };

            protected:
                const std::array<std::shared_ptr<Joint>, NJOINTS> joints_;
            };

            virtual std::shared_ptr<Frame> getLink(const int linkId) override;
            virtual const std::shared_ptr<Frame> getJoint(const int jointId) const override;
            virtual const int getNLinks() const override;
            virtual const int getNJoints() const override;

            //virtual const std::shared_ptr<JointsBase> getJoints() { return joints_; };
            virtual const std::shared_ptr<ContainerBase<Joint>> getJoints() { return joints_; };

        protected:
            const std::shared_ptr<Container<Joint, NJOINTS>> joints_;

            const std::array<std::shared_ptr<Link>, NLINKS> links_;

            virtual void setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child);
        };
    } // namespace robotlib
} // namespace dls

#include "limb.tpp"

#endif // _ROBOTLIB_LIMB_HPP_
