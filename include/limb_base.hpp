#ifndef _ROBOTLIB_LIMB_BASE_HPP_
#define _ROBOTLIB_LIMB_BASE_HPP_

#include <memory>

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

            virtual const int getNumLinks() = 0;
            virtual const int getNumJoints() = 0;
            virtual std::shared_ptr<void> getLink(const int linkId) = 0;
            virtual std::shared_ptr<void> getJoint(const int jointId) = 0;

        private:
            const std::string name_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LIMB_BASE_HPP_
