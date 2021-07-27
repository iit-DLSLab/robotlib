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

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> child)
                {
                        joint->setChild(child);
                }

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfJoint(const std::shared_ptr<Joint> joint, const std::shared_ptr<Link> parent)
                {
                        joint->setParent(parent);
                }

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> child)
                {
                        link->setChild(child);
                }

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> parent)
                {
                        link->setParent(parent);
                }

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<Link> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLink(const std::string &name)
                {
                        if (name.compare("") == 0)
                                return nullptr;
                        else if (trunk_->getName().compare(name) == 0)
                                return trunk_;
                        else
                        {
                                for (auto leg : *(this->getLegs()))
                                {
                                        std::shared_ptr<Link> link = leg->getLink(name);
                                        if (link != nullptr)
                                                return link;
                                }
                        }

                        std::cout << "LINK NOT FOUND FROM THE INPUT NAME " << name << std::endl;
                        std::cout << "Returning a nullptr... " << std::endl;
                        return std::shared_ptr<Link>(nullptr);
                };

                template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
                const std::shared_ptr<Joint> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getJoint(const std::string &name)
                {

                        if (name.compare("") == 0)
                                return nullptr;
                        for (auto leg : *(this->getLegs()))
                        {
                                std::shared_ptr<Joint> joint = leg->getJoint(name);
                                if (joint != nullptr)
                                        return joint;
                        }
                        std::cout << "JOINT NOT FOUND FROM THE INPUT NAME " << name << std::endl;
                        std::cout << "Returning a nullptr... " << std::endl;
                        return std::shared_ptr<Joint>(nullptr);
                }

        } // namespace robotlib
} // namespace dls
