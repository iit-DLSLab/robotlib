#include "robot.hpp"

namespace dls
{
        namespace robotlib
        {
                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::Robot(
                    const std::string &name,
                    const std::shared_ptr<Trunk> trunk,
                    const std::shared_ptr<Container<LimbBase, NLEGS>> &legs,
                    const std::shared_ptr<Container<LimbBase, NARMS>> &arms)
                    : RobotBase(name), legs_(legs), arms_(arms), trunk_(trunk){};

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                Robot<NJOINTS, NLINKS, NLEGS, NARMS>::~Robot(){};

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<ContainerBase<LimbBase>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLegs()
                {
                        return legs_;
                };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<ContainerBase<LimbBase>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getArms()
                {
                        return arms_;
                };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLeg(const int id) { return (*legs_)[id]; };
                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getArm(const int id) { return (*arms_)[id]; };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLEGS()
                {
                        return NLEGS;
                };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNARMS() { return NARMS; };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNJOINTS() { return NJOINTS; };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLINKS() { return NLINKS; };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildrenOfTrunk(const std::shared_ptr<ContainerBase<Joint>> children)
                {
                        trunk_->setChildren(children);
                };
        } // namespace robotlib
} // namespace dls
