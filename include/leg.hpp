#ifndef _ROBOTLIB_LEG_HPP_
#define _ROBOTLIB_LEG_HPP_

#include "leg_base.hpp"
#include "link.hpp"
#include "joint.hpp"

namespace dls
{
    namespace robotlib
    {
        /**
        * A leg class for robots.
        */
        template <unsigned int NJOINTS, unsigned int NLINKS>
        class Leg : public LegBase
        {
        public:
            Leg(const std::string &name, const std::array<std::shared_ptr<Joint>, NJOINTS> &joints, const std::array<std::shared_ptr<Link>, NLINKS> &links)
                : LegBase(name), joints_(joints), links_(links){};

            ~Leg(){};

            virtual std::shared_ptr<void> getLink(const int linkId) override { return links_[linkId]; };
            virtual std::shared_ptr<void> getJoint(const int jointId) override { return joints_[jointId]; };
            virtual const int getNumLinks() override { return links_.size(); };
            virtual const int getNumJoints() override { return joints_.size(); };

            //forward kinematics

        private:
            const std::array<std::shared_ptr<Joint>, NJOINTS> joints_; //! Array of joints
            const std::array<std::shared_ptr<Link>, NLINKS> links_;    //! Array of links
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LEG_HPP_
