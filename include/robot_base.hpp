#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_

#include "trunk.hpp"
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
        public:
            RobotBase(const std::string &name) : name_(name){};

            template <class Data>
            class LegDataMap
            {
            public:
                friend class RobotBase;

                ~LegDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nLegs_]); }

            private:
                LegDataMap(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    data_ = new Data[nLegs_];
                }

                Data *data_;
                const int nLegs_;
            };

            template <class Data>
            class JointDataMap
            {
            public:
                ~JointDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nJoints_]); }

            protected:
                JointDataMap(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    data_ = new Data[nJoints_];
                }

                Data *data_;
                const int nJoints_;
            };

            class JointState : public JointDataMap<double>
            {
            public:
                friend class RobotBase;

                ~JointState(){};

            private:
                JointState(RobotBase *robot) : JointDataMap(robot){};
            };

            template <class Data>
            class LinkDataMap
            {
            public:
                ~LinkDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nLinks_]); }

            private:
                LinkDataMap(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    data_ = new Data[nLinks_];
                }

                Data *data_;
                const int nLinks_;
            };

            template <class Data>
            class LegDataMapPair
            {
                using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

            public:
                ~LegDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLegs_]); }

                int getSize() { return data_.size(); };

            private:
                LegDataMapPair(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), Data());
                        data_.push_back(pair);
                    }
                }

                const int nLegs_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class LinkDataMapPair
            {
                using PairType = std::pair<std::shared_ptr<Link>, Data>;

            public:
                ~LinkDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLinks_]); }

                // Get functions
                int getSize() { return data_.size(); };

            private: // TODO: private
                LinkDataMapPair(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nLinks = leg->getNLinks();
                        for (int j = 0; j < nLinks; j++)
                        {
                            Link link = leg->getLink(j);
                            PairType pair(link, Data());
                            data_.push_back(pair);
                        }
                    }
                }

                const int nLinks_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class JointDataMapPair
            {
                using PairType = std::pair<std::shared_ptr<Joint>, Data>;

            public:
                ~JointDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nJoints_]); }

                // Get functions
                int getSize() { return data_.size(); };

            private:
                JointDataMapPair(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nJoints = leg->getNJoints();
                        for (int j = 0; j < nJoints; j++)
                        {
                            Joint joint = leg->getJoint(j);
                            PairType pair(joint, Data());
                            data_.push_back(pair);
                        }
                    }
                }

                const int nJoints_;
                std::vector<PairType> data_;
            };

            using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

            class Jacobian : public Map
            {
            public:
                friend class RobotBase;

                ~Jacobian()
                {
                    delete[] data_;
                }

                Map getLinearJacobian() //RT
                {
                    auto linearMatrix{this->block(0, 0, 3, nJoints_)}; // Fixed size matrix!

                    return Map(&linearMatrix(0, 0), 3, nJoints_);
                };

                Map getAngularJacobian() //RT
                {
                    auto linearMatrix{this->block(3, 0, 3, nJoints_)}; // Fixed size matrix!

                    return Map(&linearMatrix(0, 0), 3, nJoints_);
                };

            private:
                Jacobian(const int nJoints) : Map(NULL, 6, nJoints), nJoints_(nJoints)
                {
                    // Data initialization (6: linear and angular part of the jacobian)
                    data_ = new double[6 * nJoints_];
                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = 0;
                    }

                    new (this) Map(data_, 6, nJoints_);
                }

                const int nJoints_;
                double *data_; // Squashed matrix
            };

            const Iterator<const std::shared_ptr<LimbBase>> begin()
            {
                return Iterator<const std::shared_ptr<LimbBase>>(first_limb_);
            }
            const Iterator<const std::shared_ptr<LimbBase>> end()
            {
                return Iterator<const std::shared_ptr<LimbBase>>(last_limb_);
            }

            ~IteratorLimbs(){};

        private:
            IteratorLimbs(RobotBase *robot) : first_limb_(&(robot->getLimb(0))),
                                              last_limb_(&(robot->getLimb(robot->getNLEGS() + robot->getNARMS()))){};

            const std::shared_ptr<LimbBase> *const first_limb_{}, *const last_limb_{};
        };

        // Get functions
        virtual const int getNLEGS() = 0;
        virtual const int getNARMS() = 0;
        virtual const int getNJOINTS() = 0;
        virtual const int getNLINKS() = 0;

        virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;
        virtual const std::shared_ptr<LimbBase> getArm(const int id) = 0;

        virtual const std::shared_ptr<ContainerBase<LimbBase>> getLegs() = 0;
        virtual const std::shared_ptr<ContainerBase<LimbBase>> getArms() = 0;

        // Plugin typedefs
        typedef std::shared_ptr<RobotBase> createRobot_t();
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

        // Create a leg data map
        template <class Data>
        LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this); } // NRT

        // Create a joint data map
        template <class Data>
        JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); } // NRT

        // Create a link data map
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); } // NRT

        // Create a joint state
        JointState makeJointState() { return JointState(this); } // NRT

        // Create a leg data map pair
        template <class Data>
        LegDataMapPair<Data> makeLegDataMapPair() { return LegDataMapPair<Data>(this); } // NRT

        // Create a link data map pair
        template <class Data>
        LinkDataMapPair<Data> makeLinkDataMapPair() { return LinkDataMapPair<Data>(this); } // NRT

        // Create a joint data map pair
        template <class Data>
        JointDataMapPair<Data> makeJointDataMapPair() { return JointDataMapPair<Data>(this); } // NRT

        // TODO
        Jacobian makeJacobian(const Frame &fOrigin, const Frame &fDest) // NRT
        {
            std::cout << "makeJacobian function: TODO\n";
            return Jacobian(1);
        };

        // TODO: it should use makeJacobian
        Jacobian makeFootJacobian(const Frame &frame) // NRT
        {
            // Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

            // const LimbBase *l = foot.getParentLimb();
            // const int nJoints = l->getNJoints();

            // return Jacobian(nJoints);
            std::cout << "makeFootJacobian function: TODO\n";
            return Jacobian(1);
        };

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

        virtual void inverseKinematics(const Eigen::Vector3d &end_effector_position,
                                       const Eigen::Vector3d &end_effector_velocity,
                                       const Eigen::Vector3d &end_effector_acceleration,
                                       Eigen::Vector3d &joint_position,
                                       Eigen::Vector3d &joint_velocity,
                                       Eigen::Vector3d &joint_acceleration,
                                       const Frame &end_effector) = 0;

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;

        std::string getName() { return name_; };

    protected:
        const std::string name_;
    };
} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_ROBOT_BASE_HPP_
