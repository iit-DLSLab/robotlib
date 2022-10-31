#include "robot.hpp"

namespace robotlib
{
        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::Robot(
            const std::string &name,
            const std::shared_ptr<Trunk> trunk,
            const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NLEGS>> legs,
            const std::shared_ptr<const Container<std::shared_ptr<LimbBase>, NARMS>> arms)
            : RobotBase(name), legs_(legs), arms_(arms), trunk_(trunk){};

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        Robot<NJOINTS, NLINKS, NLEGS, NARMS>::~Robot(){};

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLegs() const { return legs_; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getArms() const { return arms_; };

        // template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>      /// NB: TODO
        // const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNextLeg(const std::shared_ptr<LimbBase>& leg){
        //         for(auto it_leg = (*legs_).begin();it_leg<(*legs_).end(); ++it_leg){
        //                 if (it_leg->getName().compare(leg->getName()) == 0){
        //                         // if (it_leg == ){
        //                         //         std::cout << "You are trying to get the next leg of the last one, which does not exist. The last leg is returned by default";
        //                         //         return legs_[i];
        //                         // }
        //                         // else
        //                                 return *(it_leg+1);
        //                 }
        //         }
        // }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLEGS() { return NLEGS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNARMS() { return NARMS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNJOINTS() { return NJOINTS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const int Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getNLINKS() { return NLINKS; };

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setChildrenOfTrunk(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children)
        {
                trunk_->setChildren(children);

                if(children == nullptr || children->size() < 0 || children->size() > 1)
                        trunk_->setChild(nullptr);
                else if(children->size() == 1)
                {
                        /// TODO: Substitue the for loop with the operator[] for ContainerBase
                        for (auto joint : *(trunk_->getChildren()))
                        {
                                trunk_->setChild(joint);
                        }
                }
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
                link->setChildren(nullptr);

                if(child != nullptr)
                {
                        std::array<std::shared_ptr<Joint>, 1> children{child};
                        link->setChildren(std::make_shared<Container<std::shared_ptr<Joint>, 1>>(children));
                }
        }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setParentOfLink(const std::shared_ptr<Link> link, const std::shared_ptr<Joint> parent)
        {
                link->setParent(parent);
        }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::setJointLimits(const std::shared_ptr<Joint> joint, 
                                                const double q_min, 
                                                const double q_max, 
                                                const double qd_max, 
                                                const double tau_max)
        {
                joint->setMinAngle(q_min);
                joint->setMaxAngle(q_max);
                joint->setMaxVelocity(qd_max);
                joint->setMaxEffort(tau_max);
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
                        for (auto arm : *(this->getArms()))
                        {
                                std::shared_ptr<Link> link = arm->getLink(name);
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
                for (auto arm : *(this->getArms()))
                {
                        std::shared_ptr<Joint> joint = arm->getJoint(name);
                        if (joint != nullptr)
                                return joint;
                }
                std::cout << "JOINT NOT FOUND FROM THE INPUT NAME " << name << std::endl;
                std::cout << "Returning a nullptr... " << std::endl;
                return std::shared_ptr<Joint>(nullptr);
        }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getLeg(const std::string &name)
        {
                for (auto leg : *(this->getLegs()))
                {
                        if (leg->getName().compare(name) == 0)
                                return leg;
                }
                std::cout << "LEG NOT FOUND FROM THE INPUT NAME " << name << std::endl;
                std::cout << "Returning a nullptr... " << std::endl;
                return std::shared_ptr<LimbBase>(nullptr);
        }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const std::shared_ptr<LimbBase> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getArm(const std::string &name)
        {
                for (auto arm : *(this->getArms()))
                {
                        if (arm->getName().compare(name) == 0)
                                return arm;
                }
                std::cout << "ARM NOT FOUND FROM THE INPUT NAME " << name << std::endl;
                std::cout << "Returning a nullptr... " << std::endl;
                return std::shared_ptr<LimbBase>(nullptr);
        }

        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getMinJointAngle(JointState &q_min)
        {
                for(auto leg : *legs_)
                {
                        for (auto joint : *leg->getJoints()) 
                        {
                                q_min[joint] = joint->getMinAngle();
                        }
                }
        }
        
        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getMaxJointAngle(JointState &q_max)
        {
                for(auto leg : *legs_)
                {
                        for (auto joint : *leg->getJoints()) 
                        {
                                q_max[joint] = joint->getMaxAngle();
                        }
                }
        }
        
        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getMaxJointVelocity(JointState &qd_max)
        {
                for(auto leg : *legs_)
                {
                        for (auto joint : *leg->getJoints()) 
                        {
                                qd_max[joint] = joint->getMaxVelocity();
                        }
                }
        }
        
        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        void Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getMaxJointEffort(JointState &tau_max)
        {
                for(auto leg : *legs_)
                {
                        for (auto joint : *leg->getJoints()) 
                        {
                                tau_max[joint] = joint->getMaxEffort();
                        }
                }
        }


        template <int NJOINTS, int NLINKS, unsigned int NLEGS, unsigned int NARMS>
        const Eigen::Matrix<double, 3, 1> Robot<NJOINTS, NLINKS, NLEGS, NARMS>::getTrunkCOM() const
        {
                return trunk_->getCoM();
        };


} // namespace robotlib