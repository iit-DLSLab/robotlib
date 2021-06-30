#include "robot.hpp"

namespace dls
{
    namespace robotlib
    {
        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::Robot(const std::string &name,
                                                    const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs,
                                                    const std::shared_ptr<ForwardKinematicsBase> &fk)
            : RobotBase(name), legs_(legs), fk_(fk){};

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::~Robot(){};

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::begin()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[0]);
        };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::end()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[NLEGS]);
        };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::array<std::shared_ptr<LimbBase>, NLEGS> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLegs() { return legs_; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLeg(const int id) { return legs_[id]; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLEGS() { return NLEGS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNJOINTS() { return NJOINTS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLINKS() { return NLINKS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        std::shared_ptr<ForwardKinematicsBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getForwardKinematics() { return fk_; };
    } // namespace robotlib
} // namespace dls
