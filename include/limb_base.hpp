#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

#include <memory>
#include "frame.hpp"
#include "utils.hpp"

namespace dls
{
    namespace robotlib
    {
        class Link;
        class Joint;
        class LimbBase
        {
        public:
            LimbBase(const std::string &name);

            virtual ~LimbBase();

            class JointsBase
            {
            public:
                JointsBase(){};
                virtual ~JointsBase(){};

                virtual Iterator<const std::shared_ptr<Joint>> begin() const = 0;
                virtual Iterator<const std::shared_ptr<Joint>> end() const = 0;
                virtual const std::shared_ptr<Joint> operator[](const int id) const = 0;
                virtual const int size() const = 0;
            };
            class LinksBase
            {
            };

            const std::string getName() const;

            virtual const int getNLinks() const = 0;
            virtual const int getNJoints() const = 0;
            virtual const std::shared_ptr<Link> getLink(const int linkId) = 0;
            virtual const std::shared_ptr<Joint> getJoint(const int jointId) const = 0;
            //virtual const std::shared_ptr<JointsBase> getJoints() = 0;
            virtual const std::shared_ptr<ContainerBase<Joint>> getJoints() = 0;

            virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) = 0;
            virtual const std::shared_ptr<Link> getLinkFromName(const std::string &name) = 0;
            virtual const std::shared_ptr<Joint> getJointFromName(const std::string &name) = 0;

        private:
            const std::string name_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LIMB_BASE_HPP_
