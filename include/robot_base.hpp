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
#include <vector>

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

                // std::shared_ptr<Data> operator[](const std::shared_ptr<LimbBase> leg)
                // {
                //     return data_[leg->toId()]; // leg->toId is overridden in the Glue
                // };
                Data &operator[](int index)
                {
                    assert(index >= 0 && index <= nLegs_);
                    return data_[index];
                }

                const Data &operator[](int index) const
                {
                    assert(index >= 0 && index <= nLegs_);
                    return data_[index];
                }

                void copydata(const LegDataMap &rhs)
                {
                    assert(nLegs_ == rhs.nLegs_); /// TODO: Implement and use getSize() method

                    for (auto i{0}; i < nLegs_; i++)
                    {
                        data_[i] = rhs.data_[i]; /// TODO: Implement and use getData() method
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nLegs_; i++)
                    {
                        data_[i] = value; /// TODO: Implement and use getData() method
                    }
                }

                LegDataMap &operator=(const LegDataMap &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                LegDataMap &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

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
                friend class RobotBase;

                ~JointDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nJoints_]); }

                Data &operator[](int index)
                {
                    assert(index >= 0 && index <= nJoints_);
                    return data_[index];
                }

                const Data &operator[](int index) const
                {
                    assert(index >= 0 && index <= nJoints_);
                    return data_[index];
                }

                void copydata(const JointDataMap &rhs)
                {
                    assert(nJoints_ == rhs.nJoints_); /// TODO: Implement and use getSize() method

                    for (auto i{0}; i < nJoints_; i++)
                    {
                        data_[i] = rhs.data_[i]; /// TODO: Implement and use getData() method
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nJoints_; i++)
                    {
                        data_[i] = value; /// TODO: Implement and use getData() method
                    }
                }

                JointDataMap &operator=(const JointDataMap &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                JointDataMap &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

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

                void copydata(const JointState &rhs)
                {
                    assert(nJoints_ == rhs.nJoints_); /// TODO: Implement and use getSize() method

                    for (auto i{0}; i < nJoints_; i++)
                    {
                        data_[i] = rhs.data_[i]; /// TODO: Implement and use getData() method
                    }
                }

                JointState &operator=(const JointState &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                JointState &operator=(const double &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

            private:
                JointState(RobotBase *robot) : JointDataMap(robot){};
            };

            template <class Data>
            class LinkDataMap
            {
            public:
                friend class RobotBase;
                ~LinkDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nLinks_]); }

                Data &operator[](int index)
                {
                    assert(index >= 0 && index <= nLinks_);
                    return data_[index];
                }

                const Data &operator[](int index) const
                {
                    assert(index >= 0 && index <= nLinks_);
                    return data_[index];
                }

                void copydata(const LinkDataMap &rhs)
                {
                    assert(nLinks_ == rhs.nLinks_); /// TODO: Implement and use getSize() method

                    for (auto i{0}; i < nLinks_; i++)
                    {
                        data_[i] = rhs.data_[i]; /// TODO: Implement and use getData() method
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nLinks_; i++)
                    {
                        data_[i] = value; /// TODO: Implement and use getData() method
                    }
                }

                LinkDataMap &operator=(const LinkDataMap &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                LinkDataMap &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

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
                friend class RobotBase;
                ~LegDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLegs_]); }

                Data &operator[](const std::shared_ptr<LimbBase> leg) // q: shared_ptr or & ?
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(leg->getName()) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                Data &operator[](std::string leg_name)
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(leg_name) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                const Data &operator[](std::string leg_name) const
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(leg_name) == 0)
                            return pair.second;
                    }
                };

                void copydata(const LegDataMapPair &rhs)
                {
                    assert(data_.size() == rhs.data_.size());

                    for (auto i{0}; i < nLegs_; i++)
                    {
                        data_[i].first = rhs.data_[i].first;
                        data_[i].second = rhs.data_[i].second;
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nLegs_; i++)
                    {
                        data_[i].second = value;
                    }
                }

                LegDataMapPair &operator=(const LegDataMapPair &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                LegDataMapPair &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

                int getSize() { return data_.size(); };

            private:
                LegDataMapPair(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), Data()); //shared_pointers?
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
                friend class RobotBase;
                ~LinkDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLinks_]); }

                Data &operator[](const std::shared_ptr<Link> link) // q: shared_ptr or & ?
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(link->getName()) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                Data &operator[](std::string link_name)
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(link_name) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                const Data &operator[](std::string link_name) const
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(link_name) == 0)
                            return pair.second;
                    }
                };

                void copydata(const LinkDataMapPair &rhs)
                {
                    assert(data_.size() == rhs.data_.size());

                    for (auto i{0}; i < nLinks_; i++)
                    {
                        data_[i].first = rhs.data_[i].first;
                        data_[i].second = rhs.data_[i].second;
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nLinks_; i++)
                    {
                        data_[i].second = value;
                    }
                }

                LinkDataMapPair &operator=(const LinkDataMapPair &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                LinkDataMapPair &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

                int getSize() { return data_.size(); };

            private: // TODO: private
                LinkDataMapPair(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (auto leg : *(robot->getLegs()))
                    {
                        for (auto link : *(leg->getLinks()))
                        {
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
                friend class RobotBase;
                ~JointDataMapPair(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nJoints_]); }

                Data &operator[](const std::shared_ptr<Joint> joint) // q: shared_ptr or & ?
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(joint->getName()) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                Data &operator[](std::string joint_name)
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(joint_name) == 0)
                            return pair.second;
                    }
                };

                /// TODO: to understand if we want operator[] with string
                const Data &operator[](std::string joint_name) const
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(joint_name) == 0)
                            return pair.second;
                    }
                };

                void copydata(const JointDataMapPair &rhs)
                {
                    assert(data_.size() == rhs.data_.size());

                    for (auto i{0}; i < nJoints_; i++)
                    {
                        data_[i].first = rhs.data_[i].first;
                        data_[i].second = rhs.data_[i].second;
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < nJoints_; i++)
                    {
                        data_[i].second = value;
                    }
                }

                JointDataMapPair &operator=(const JointDataMapPair &rhs)
                {
                    if (&rhs != this)
                    {
                        copydata(rhs);
                    }
                    return *this;
                }

                JointDataMapPair &operator=(const Data &defaultValue)
                {
                    assignAll(defaultValue);
                    return *this;
                }

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
                            std::shared_ptr<Joint> joint = leg->getJoint(j);
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
                    if (data_ != nullptr)
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

                Jacobian &operator=(const Jacobian &other)            ///NB: the = operator assumes that nJoints of other is equal to this!
                {
                    nJoints_ = other.getNJoints();              ///do this is redundant...

                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = other.data_[i];
                    } 

                    return *this;
                }

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

                Jacobian() : Map(NULL, 0, 0), data_(nullptr){};

                void init(const Jacobian other)
                {
                    nJoints_ = other.getNJoints();

                    data_ = new double[6 * nJoints_];
                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = other.data_[i];
                    }

                    new (this) Map(data_, 6, nJoints_);
                }

                int getNJoints() const { return nJoints_; };

                double *getData() { return data_; };

                int nJoints_;
                double *data_; // Squashed matrix
            };

            // Get functions
            virtual const int getNLEGS() = 0;
            virtual const int getNARMS() = 0;
            virtual const int getNJOINTS() = 0;
            virtual const int getNLINKS() = 0;

            virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;
            virtual const std::shared_ptr<LimbBase> getArm(const int id) = 0;

            // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) = 0;    ///TODO: required for the print inside CGaitTimerHex::run() of Ant Controller

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
            Jacobian makeJacobian(const std::shared_ptr<Frame> fOrigin, const std::shared_ptr<Frame> fDest) // NRT
            {
                std::cout << "makeJacobian function: TODO\n";
                return Jacobian(1);
            };

            // TODO: it should use makeJacobian
            Jacobian makeFootJacobian(const std::shared_ptr<Frame> frame) // NRT
            {
                // Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

                // const LimbBase *l = foot.getParentLimb();
                // const int nJoints = l->getNJoints();

                // return Jacobian(nJoints);
                std::cout << "makeFootJacobian-Input: foot function: TODO\n";
                return Jacobian(1);
            };

            // TODO: it should use makeJacobian
            Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg) // NRT
            {
                return Jacobian(leg->getNJoints());
            };

            LegDataMapPair<Jacobian> makeFeetJacobian() // NRT
            {
                auto feetJac = this->makeLegDataMapPair<Jacobian>();

                for (auto leg : *(this->getLegs()))
                {
                    feetJac[leg].init(makeFootJacobian(leg));
                }
                return feetJac;
            };

            void initFeetJacobians(LegDataMapPair<Jacobian> footJac)
            {
                for (auto leg : *(this->getLegs()))
                {
                    footJac[leg].init(makeFootJacobian(leg));
                }
            }

            virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                     const std::shared_ptr<Frame> origin,
                                                     const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Vector3d getFramePosition(const JointDataMapPair<double> &q,
                                                     const std::shared_ptr<Frame> origin,
                                                     const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix3d getFrameOrientation(const JointDataMapPair<double> &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix4d getFramePose(const JointDataMapPair<double> &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                    const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Vector3d getFootPosition(const JointDataMapPair<double> &q,
                                                    const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix3d getFootOrientation(const JointDataMapPair<double> &q,
                                                       const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix4d getFootPose(const JointDataMapPair<double> &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual void getFootPosition(const JointState &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Eigen::Vector3d &footPos) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual void getFootPosition(const JointDataMapPair<double> &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Eigen::Vector3d &footPos) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix3d getFootOrientation(const JointDataMapPair<double> &q,
                                                       const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual Eigen::Matrix4d getFootPose(const JointDataMapPair<double> &q,
                                                const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            virtual void getFootJacobian(const JointState &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Jacobian &footJac) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMapPair<double>
            virtual void getFootJacobian(const JointDataMapPair<double> &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Jacobian &footJac) = 0; //overridden by Glue

            virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;

            virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;

            virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

            virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                           const JointState &joint_velocity,
                                           const JointState &joint_acceleration,
                                           LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                           LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                           LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) = 0;

            virtual void forwardKinematics(const Eigen::Vector3d &joint_position,
                                           const Eigen::Vector3d &joint_velocity,
                                           const Eigen::Vector3d &joint_acceleration,
                                           Eigen::Vector3d &end_effector_position,
                                           Eigen::Vector3d &end_effector_velocity,
                                           Eigen::Vector3d &end_effector_acceleration,
                                           const std::shared_ptr<Frame> end_effector) = 0; // TODO: Better to use end effector or leg (as in ANT controller)?

            virtual void inverseKinematics(const Eigen::Vector3d &end_effector_position,
                                           const Eigen::Vector3d &end_effector_velocity,
                                           const Eigen::Vector3d &end_effector_acceleration,
                                           Eigen::Vector3d &joint_position,
                                           Eigen::Vector3d &joint_velocity,
                                           Eigen::Vector3d &joint_acceleration,
                                           const std::shared_ptr<Frame> end_effector) = 0; // TODO: Better to use end effector or leg (as in ANT controller)?

            virtual void inverseKinematics(const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                           const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                           const LegDataMapPair<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
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
