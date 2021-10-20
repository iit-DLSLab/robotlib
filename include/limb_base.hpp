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

            const std::string getName() const;

            virtual const int getNLinks() const = 0;
            virtual const int getNJoints() const = 0;

            virtual const std::shared_ptr<const Link> getLink(const int linkId) const = 0;
            virtual const std::shared_ptr<const Joint> getJoint(const int jointId) const = 0;

            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<const Joint>>> getJoints() = 0;
            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<const Link>>> getLinks() = 0;

            virtual const std::string jointToChildName(const std::shared_ptr<Joint> joint) const = 0;  //overridden by Glue
            virtual const std::string jointToParentName(const std::shared_ptr<Joint> joint) const = 0; //overridden by Glue

            virtual const std::string linkToChildName(const std::shared_ptr<Link> joint) const = 0;  //overridden by Glue
            virtual const std::string linkToParentName(const std::shared_ptr<Link> joint) const = 0; //overridden by Glue

            virtual const std::shared_ptr<const Link> getLink(const std::string &name) const = 0;
            virtual const std::shared_ptr<const Joint> getJoint(const std::string &name) const = 0;

            virtual const std::shared_ptr<const Link> getEndEffector() const = 0; //overridden by Limb class

        private:
            const std::string name_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LIMB_BASE_HPP_
