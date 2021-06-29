#include "robot.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::Robot(const std::string &name,
                                                            const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs,
                                                            const std::shared_ptr<ForwardKinematicsBase> &fk)
            : RobotBase(name), legs_(legs), fk_(fk){};

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::~Robot(){};

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::begin()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[0]);
        };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::end()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[NLEGS]);
        };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const std::array<std::shared_ptr<LimbBase>, NLEGS> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getLegs() { return legs_; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const std::shared_ptr<LimbBase> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getLeg(const int id) { return legs_[id]; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNLEGS() { return NLEGS; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNJOINTS() { return NJOINTS_TOT; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNLINKS() { return NLINKS_TOT; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        std::shared_ptr<ForwardKinematicsBase> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getForwardKinematics() { return fk_; };
    } // namespace robotlib
} // namespace dls
