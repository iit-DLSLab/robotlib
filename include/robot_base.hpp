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
                using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

            public:
                friend class RobotBase;
                ~LegDataMap(){};

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

                const Data &operator[](const std::shared_ptr<LimbBase> leg) const
                {
                    for (PairType &pair : *this)
                    {
                        if (pair.first->getName().compare(leg->getName()) == 0)
                            return pair.second;
                    }
                };

                void copydata(const LegDataMap &rhs)
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

                int getSize() { return data_.size(); };

            private:
                LegDataMap(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), Data()); //shared_pointers?
                        data_.push_back(pair);
                    }
                }

                LegDataMap(RobotBase *robot, const Data &data) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), data); //shared_pointers?
                        data_.push_back(pair);
                    }
                }
                const int nLegs_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class LinkDataMap
            {
                using PairType = std::pair<std::shared_ptr<Link>, Data>;

            public:
                friend class RobotBase;
                ~LinkDataMap(){};

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

                void copydata(const LinkDataMap &rhs)
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

                int getSize() { return data_.size(); };

            private: // TODO: private
                LinkDataMap(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    for (auto leg : *(robot->getLegs()))
                    {
                        for (auto link : *(leg->getLinks()))
                        {
                            PairType pair(link, Data());
                            data_.push_back(pair);
                        }
                    }
                }
                LinkDataMap(RobotBase *robot, const Data &data) : nLinks_(robot->getNLINKS())
                {
                    for (auto leg : *(robot->getLegs()))
                    {
                        for (auto link : *(leg->getLinks()))
                        {
                            PairType pair(link, Data(data));
                            data_.push_back(pair);
                        }
                    }
                }

                const int nLinks_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class JointDataMap
            {
                using PairType = std::pair<std::shared_ptr<Joint>, Data>;

            public:
                friend class RobotBase;

                ~JointDataMap(){};

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[num_joints_]); }

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

                void copydata(const JointDataMap &rhs)
                {
                    assert(this->getSize() == rhs.getSize());

                    for (auto i{0}; i < num_joints_; i++)
                    {
                        data_[i].first = rhs.data_[i].first;
                        data_[i].second = rhs.data_[i].second;
                    }
                }

                void assignAll(const Data &value)
                {
                    for (auto i{0}; i < num_joints_; i++)
                    {
                        data_[i].second = value;
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

                int getSize() const { return num_joints_; };

            private:
                JointDataMap(RobotBase *robot) : num_joints_(robot->getNJOINTS())
                {
                    const int nLegs = robot->getNLEGS();
                    data_ = new PairType[num_joints_];

                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int num_joints_per_leg = leg->getNJoints();
                        for (int j = 0; j < num_joints_per_leg; j++)
                        {
                            std::shared_ptr<Joint> joint = leg->getJoint(j);
                            PairType pair(joint, Data());
                            data_[i * num_joints_per_leg + j] = pair;
                            // std::cout << data_[i * num_joints_per_leg + j].first
                        }
                    }
                }
                JointDataMap(RobotBase *robot, const Data &data) : num_joints_(robot->getNJOINTS())
                {
                    const int nLegs = robot->getNLEGS();
                    data_ = new PairType[num_joints_];

                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int num_joints_per_leg = leg->getNJoints();
                        for (int j = 0; j < num_joints_per_leg; j++)
                        {
                            std::shared_ptr<Joint> joint = leg->getJoint(j);
                            PairType pair(joint, Data(data));
                            data_[i * num_joints_per_leg + j] = pair;
                        }
                    }
                }
                JointDataMap(const std::shared_ptr<LimbBase> leg) : num_joints_(leg->getNJoints())
                {
                    data_ = new PairType[num_joints_];
                    Iterator<const std::shared_ptr<Joint>> leg_joints_it = leg->getJoints()->begin();

                    for (int i = 0; i < num_joints_; i++)
                    {
                        std::shared_ptr<Joint> joint = *leg_joints_it;
                        data_[i] = PairType(joint, Data());
                        leg_joints_it++;
                    }
                }
                JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data) : num_joints_(leg->getNJoints())
                {
                    data_ = new PairType[num_joints_];
                    Iterator<const std::shared_ptr<Joint>> leg_joints_it = leg->getJoints()->begin();

                    for (int i = 0; i < num_joints_; i++)
                    {
                        std::shared_ptr<Joint> joint = *leg_joints_it;
                        data_[i] = PairType(joint, Data(data));
                        leg_joints_it++;
                    }
                }
                JointDataMap() : num_joints_(0) //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
                {
                    data_ = nullptr;
                }

                void init(const JointDataMap &data)
                {
                    num_joints_ = data.getSize();
                    if (data_ != nullptr)
                    {
                        delete[] data_;
                    }

                    data_ = new PairType[num_joints_];

                    copydata(data);
                }
                int num_joints_;
                PairType *data_;
            };
            class JointState : public LegDataMap<JointDataMap<double>> //public JointDataMap<double>
            {
            public:
                friend class RobotBase;
                using LegDataMap<JointDataMap<double>>::operator=;
                using LegDataMap<JointDataMap<double>>::operator[];

                double &operator[](const std::shared_ptr<Joint> joint) // q: shared_ptr or & ?
                {
                    for (auto leg_pair : *this)
                    {
                        auto joint_data_map_per_leg = leg_pair.second;

                        for (auto joint_pair : joint_data_map_per_leg) //iterate over the JointDataMap
                        {
                            if (joint_pair.first->getName().compare(joint->getName()) == 0)
                            {
                                return joint_data_map_per_leg[joint];
                            }
                        }
                    }
                };

                JointState &operator=(const double data)
                {
                    for (auto leg_pair : *this)
                    {
                        leg_pair.second.assignAll(data);
                    }
                    return *this;
                }

                JointDataMap<double> &getLegJointState(const std::shared_ptr<LimbBase> leg) { return (*this)[leg->getName()]; }

                ~JointState(){};

            private:
                JointState(RobotBase *robot) : LegDataMap<JointDataMap<double>>(robot){};
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

                Jacobian &operator=(const Jacobian &other) ///NB: the = operator assumes that nJoints of other is equal to this!
                {
                    nJoints_ = other.getNJoints(); ///do this is redundant...

                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = other.data_[i];
                    }

                    return *this;
                }

            private:
                Jacobian(const int nJoints, const double data = 0) : Map(NULL, 6, nJoints), nJoints_(nJoints)
                {
                    // Data initialization (6: linear and angular part of the jacobian)
                    data_ = new double[6 * nJoints_];
                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = data;
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

            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;
            virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

            // Plugin typedefs
            typedef std::shared_ptr<RobotBase> createRobot_t();
            typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

            // Create a joint state
            JointState makeJointState()
            {
                JointState joint_state = JointState(this);

                for (auto leg : *this->getLegs())
                {
                    joint_state[leg].init(this->makeJointDataMapPerLeg<double>(leg, 0));
                }

                return joint_state;
            } // NRT

            // Create a leg data map pair
            template <class Data>
            LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this); } // NRT
            // Create a leg data map pair
            template <class Data>
            LegDataMap<Data> makeLegDataMap(const Data &data) { return LegDataMap<Data>(this, data); } // NRT

            // Create a link data map pair
            template <class Data>
            LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); } // NRT
            template <class Data>
            LinkDataMap<Data> makeLinkDataMap(const Data &data) { return LinkDataMap<Data>(this, data); } // NRT

            // Create a joint data map pair
            template <class Data>
            JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); } // NRT
            template <class Data>
            JointDataMap<Data> makeJointDataMap(const Data &data) { return JointDataMap<Data>(this, data); } // NRT

            // Create a joint data map pair
            template <class Data>
            JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg) { return JointDataMap<Data>(leg); } // NRT
            // Create a joint data map pair
            template <class Data>
            JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data &data) { return JointDataMap<Data>(leg, data); } // NRT

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
            Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0) // NRT
            {
                return Jacobian(leg->getNJoints(), data);
            };

            LegDataMap<Jacobian> makeFeetJacobian(const double data = 0) // NRT
            {
                auto feetJac = this->makeLegDataMap<Jacobian>();

                for (auto leg : *(this->getLegs()))
                {
                    feetJac[leg].init(makeFootJacobian(leg, data));
                }
                return feetJac;
            };

            void initFeetJacobians(LegDataMap<Jacobian> footJac)
            {
                for (auto leg : *(this->getLegs()))
                {
                    footJac[leg].init(makeFootJacobian(leg));
                }
            }

            virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                     const std::shared_ptr<Frame> origin,
                                                     const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Vector3d getFramePosition(const JointDataMap<double> &q,
                                                     const std::shared_ptr<Frame> origin,
                                                     const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix3d getFrameOrientation(const JointDataMap<double> &q,
                                                        const std::shared_ptr<Frame> origin,
                                                        const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix4d getFramePose(const JointDataMap<double> &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                    const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Vector3d getFootPosition(const JointDataMap<double> &q,
                                                    const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix3d getFootOrientation(const JointDataMap<double> &q,
                                                       const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix4d getFootPose(const JointDataMap<double> &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

            virtual void getFootPosition(const JointState &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Eigen::Vector3d &footPos) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual void getFootPosition(const JointDataMap<double> &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Eigen::Vector3d &footPos) = 0; //overridden by Glue

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix3d getFootOrientation(const JointDataMap<double> &q,
                                                       const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual Eigen::Matrix4d getFootPose(const JointDataMap<double> &q,
                                                const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

            virtual void getFootJacobian(const JointState &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Jacobian &footJac) = 0; //overridden by Glue

            /// TODO: verify if needed with JointDataMap<double>
            virtual void getFootJacobian(const JointDataMap<double> &q,
                                         const std::shared_ptr<LimbBase> leg,
                                         Jacobian &footJac) = 0; //overridden by Glue

            virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;

            virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;

            virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

            virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                           const JointState &joint_velocity,
                                           const JointState &joint_acceleration,
                                           LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                           LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                           LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) = 0;

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

            virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                           const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                           const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                           JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                           JointState &joint_velocity,
                                           JointState &joint_acceleration) = 0;

            virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                         const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                         const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                         const JointState &joint_position,
                                         const JointState &joint_velocity,
                                         const JointState &joint_acceleration,
                                         const Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                         const JointState &tau_joints) = 0;              ///output

            virtual double getRobotMass() = 0; ///TODO: compute total mass from links and trunk masses

            virtual Eigen::Vector3d getRobotCoM() = 0;

            std::string getName()
            {
                return name_;
            };

        protected:
            const std::string name_;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_ROBOT_BASE_HPP_
