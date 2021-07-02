#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_
#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "dyn_params.hpp"
#include "utils.hpp"

#include <iostream>
#include <memory>

namespace dls
{
    namespace robotlib
    {
        class RobotBase
        {
        protected:
            template <class Data>
            class LegDataMap
            {
            public:
                LegDataMap(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    Data *p = new Data[nLegs_];
                    std::shared_ptr<Data> pshrd(p);
                    data_ = pshrd;
                }
                ~LegDataMap(){};

                Iterator<Data> begin() { return Iterator<Data>(&data_.get()[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_.get()[nLegs_]); }

            private:
                std::shared_ptr<Data> data_;
                const int nLegs_;
            };

            template <class Data>
            class JointDataMap
            {
            public:
                JointDataMap(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    Data *p = new Data[nJoints_];
                    std::shared_ptr<Data> pshrd(p);
                    data_ = pshrd;
                }
                ~JointDataMap(){};

                Iterator<Data> begin() { return Iterator<Data>(&data_.get()[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_.get()[nJoints_]); }

            private:
                std::shared_ptr<Data> data_;
                const int nJoints_;
            };

            class JointState : public JointDataMap<double>
            {
            public:
                JointState(RobotBase *robot) : JointDataMap(robot){};
                ~JointState(){};
            };

            template <class Data>
            class LinkDataMap
            {
            public:
                LinkDataMap(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    Data *p = new Data[nLinks_];
                    std::shared_ptr<Data> pshrd(p);
                    data_ = pshrd;
                }
                ~LinkDataMap(){};

                Iterator<Data> begin() { return Iterator<Data>(&data_.get()[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_.get()[nLinks_]); }

            private:
                std::shared_ptr<Data> data_;
                const int nLinks_;
            };

            template <class Data>
            class LegDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

            public:
                LegDataMapPair(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), Data());
                        data_.push_back(pair);
                    }
                }

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLegs_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~LegDataMapPair(){};

            private:
                const int nLegs_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class LinkDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<Link>, Data>;

            public:
                LinkDataMapPair(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nLinks = leg->getNumLinks();
                        for (int j = 0; j < nLinks; j++)
                        {
                            auto link = std::static_pointer_cast<Link>(leg->getLink(j));
                            PairType pair(link, Data());
                            data_.push_back(pair);
                        }
                    }
                }
                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLinks_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~LinkDataMapPair(){};

            private:
                const int nLinks_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class JointDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<Joint>, Data>;

            public:
                JointDataMapPair(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nJoints = leg->getNumJoints();
                        for (int j = 0; j < nJoints; j++)
                        {
                            auto joint = std::static_pointer_cast<Joint>(leg->getJoint(j));
                            PairType pair(joint, Data());
                            data_.push_back(pair);
                        }
                    }
                }

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nJoints_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~JointDataMapPair(){};

            private:
                const int nJoints_;
                std::vector<PairType> data_;
            };

            const std::string name_;

        public:
            RobotBase(const std::string &name) : name_(name){};

            // TODO: Is it correct that the iterators are defined with "{}"?
            virtual Iterator<const std::shared_ptr<LimbBase>> begin(){};
            virtual Iterator<const std::shared_ptr<LimbBase>> end(){};

            // Get functions
            virtual const int getNLEGS() = 0;
            virtual const int getNJOINTS() = 0;
            virtual const int getNLINKS() = 0;

            virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;

            // Plugin typedefs
            typedef std::shared_ptr<RobotBase> createRobot_t();
            typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

            // Create a leg data map
            template <class Data>
            LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this); }

            // Create a joint data map
            template <class Data>
            JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); }

            // Create a link data map
            template <class Data>
            LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); }

            // Create a joint state
            JointState makeJointState() { return JointState(this); }

            // Create a leg data map pair
            template <class Data>
            LegDataMapPair<Data> makeLegDataMapPair() { return LegDataMapPair<Data>(this); }

            // Create a link data map pair
            template <class Data>
            LinkDataMapPair<Data> makeLinkDataMapPair() { return LinkDataMapPair<Data>(this); }

            // Create a joint data map pair
            template <class Data>
            JointDataMapPair<Data> makeJointDataMapPair() { return JointDataMapPair<Data>(this); }

            // Functions for getting info from the robot
            // Names of the Legs
            void getLegsName()
            {
                std::cout << "\n*** LEGS OF " << name_ << " ***" << std::endl;
                for (auto leg : *this)
                {
                    std::cout << leg->getName() << std::endl;
                }
            }
            // Names of the Links for each leg
            void getLinksName()
            {
                std::cout << "\n*** LINKS FOR EACH LEG OF " << name_ << " ***" << std::endl;
                for (auto leg : *this)
                {
                    std::cout << leg->getName() << ":  ";
                    int nLinks = leg->getNumLinks();
                    for (int link = 0; link < nLinks; ++link)
                    {
                        if (link == nLinks - 1)
                            std::cout << std::static_pointer_cast<Link>(leg->getLink(link))->getName() << '\n';
                        else
                            std::cout << std::static_pointer_cast<Link>(leg->getLink(link))->getName() << ", ";
                    }
                }
            }
            // Names of the Joints for each leg
            void getJointsName()
            {
                std::cout << "\n*** JOINTS FOR EACH LEG OF " << name_ << " ***" << std::endl;
                for (auto leg : *this)
                {
                    std::cout << leg->getName() << ":  ";
                    int nJoints = leg->getNumJoints();
                    for (int joint = 0; joint < nJoints; ++joint)
                    {
                        std::cout << std::static_pointer_cast<Joint>(leg->getJoint(joint))->getName() << ", ";
                    }
                    std::cout << '\n';
                }
            }

            virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                     const Frame &origin,
                                                     const Frame &destination) = 0;

            virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                        const Frame &origin,
                                                        const Frame &destination) = 0;

            virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                                 const Frame &origin,
                                                 const Frame &destination) = 0;

            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                    const Frame &foot) = 0;

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const Frame &foot) = 0;

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const Frame &foot) = 0;

            virtual Link getLink(const std::string &name) = 0;

            virtual Joint getJoint(const std::string &name) = 0;

            virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

            std::string getName() { return name_; };
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
