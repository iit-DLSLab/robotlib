#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

#include <memory>
#include "frame.hpp"

namespace dls
{
    namespace robotlib
    {
        class LimbBase
        {
        public:
            LimbBase(const std::string &name);

            virtual ~LimbBase();

            const std::string getName() const;

            virtual const int getNLinks() const = 0;
            virtual const int getNJoints() const = 0;
            virtual std::shared_ptr<Frame> getLink(const int linkId) = 0;
            virtual std::shared_ptr<Frame> getJoint(const int jointId) = 0;

        private:
            const std::string name_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LIMB_BASE_HPP_
