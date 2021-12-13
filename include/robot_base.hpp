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

namespace robotlib
{
    class RobotBase
    {
    public:
        RobotBase(const std::string &name) : name_(name){};
        template <class Key, class Data>
        class DataMap
        {
        private:
            class Pair
            {
            public:
                friend class RobotBase;

                virtual Pair &operator=(const Pair &rhs)
                {
                    this->key_ = rhs.key_;
                    this->data_ = rhs.data_;
                    return *this;
                }

                std::shared_ptr<Key> key_;
                Data data_;

                ~Pair(){};

            private:
                Pair(const std::shared_ptr<Key> key, const Data &data) : key_(key), data_(data){};
                Pair(){};
            };

        public:
            friend class RobotBase;
            virtual ~DataMap()
            {
                delete[] data_;
            };

            virtual Iterator<Pair> begin() { return Iterator<Pair>(&data_[0]); }
            virtual Iterator<Pair> end() { return Iterator<Pair>(&data_[num_data_]); }

            virtual Iterator<const Pair> begin() const { return Iterator<const Pair>(&data_[0]); }
            virtual Iterator<const Pair> end() const { return Iterator<const Pair>(&data_[num_data_]); }

            virtual Data &operator[](const std::shared_ptr<Key> key) // q: shared_ptr or & ?
            {
                for (Pair &pair : *this)
                {
                    if (pair.key_->getName().compare(key->getName()) == 0)
                    {
                        return pair.data_;
                    }
                }
            };

            virtual const Data &operator[](const std::shared_ptr<Key> key) const // q: shared_ptr or & ?
            {
                for (auto &pair : *this)
                {
                    if (pair.key_->getName().compare(key->getName()) == 0)
                        return pair.data_;
                }
            };

            virtual Data &operator[](const std::string &key_name)
            {
                for (Pair &pair : *this)
                {
                    if (pair.key_->getName().compare(key_name) == 0)
                        return pair.data_;
                }
            };

            virtual const Data &operator[](const std::string &key_name) const
            {
                for (auto &pair : *this)
                {
                    if (pair.key_->getName().compare(key_name) == 0)
                        return pair.data_;
                }
            };

            virtual void copydata(const DataMap &rhs)
            {
                assert(this->getSize() == rhs.getSize());

                for (auto i{0}; i < num_data_; i++)
                {
                    data_[i].key_ = rhs.data_[i].key_;
                    data_[i].data_ = rhs.data_[i].data_;
                }
            }

            virtual void assignAll(const Data &value)
            {
                for (auto i{0}; i < num_data_; i++)
                {
                    data_[i].data_ = value;
                }
            }

            virtual DataMap &operator=(const DataMap &rhs)
            {
                if (&rhs != this)
                {
                    copydata(rhs);
                }
                return *this;
            }

            virtual DataMap &operator=(const Data &defaultValue)
            {
                assignAll(defaultValue);
                return *this;
            }

            virtual const int getSize() const { return num_data_; };

        protected:
            DataMap(const int num_data) : num_data_(num_data)
            {
                this->data_ = new Pair[this->num_data_];
            }

            DataMap() : num_data_(0), data_(nullptr) //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
            {
            }

            virtual void init(const DataMap &data)
            {
                num_data_ = data.getSize();

                if (data_ != nullptr)
                {
                    delete[] data_;
                }

                data_ = new Pair[num_data_];

                copydata(data);
            }

            const Pair createPair(const std::shared_ptr<Key> key, const Data &data) const { return Pair(key, data); } //shared_pointers?}
            Pair createPair(const std::shared_ptr<Key> key, const Data &data) { return Pair(key, data); }             //shared_pointers?}

            int num_data_;
            Pair *data_;
        };
        template <class Data>
        class LegDataMap : public DataMap<LimbBase, Data>
        {
        public:
            using DataMap<LimbBase, Data>::operator=;
            friend class RobotBase;

            ~LegDataMap(){};

            /// TODO: Print in new line if data is a vector, matrix, etc... in same line of leg name (as for JointState) if data is a single value
            void print()
            {
                std::cout << "LegDataMap [Name - Value]" << std::endl;
                std::cout << "-------------------------" << std::endl;

                for (auto &leg_pair : *this)
                {
                    std::cout << leg_pair.key_->getName() << " - " << leg_pair.data_ << std::endl;
                }
            }

        protected:
            LegDataMap(RobotBase *robot) : DataMap<LimbBase, Data>(robot->getNLEGS()) //TODO: remove it, leave only the constructor with data
            {
                int count_data = 0;
                for (auto key : *robot->getLegs())
                {
                    this->data_[count_data] = this->createPair(key, Data()); //shared_pointers?
                    count_data++;
                }
            }

            LegDataMap(RobotBase *robot, const Data &data) : DataMap<LimbBase, Data>(robot->getNLEGS())
            {
                int count_data = 0;
                for (auto key : *robot->getLegs())
                {
                    this->data_[count_data] = this->createPair(key, data);
                    count_data++;
                }
            }
        };

        template <class Data>
        class LinkDataMap : public DataMap<Link, Data>
        {

        public:
            using DataMap<Link, Data>::operator=;
            friend class RobotBase;
            ~LinkDataMap(){};

        private:
            LinkDataMap(RobotBase *robot) : DataMap<Link, Data>(robot->getNLINKS())
            {
                int count_data = 0;
                for (auto leg : *(robot->getLegs()))
                {
                    for (auto key : *(leg->getLinks()))
                    {
                        this->data_[count_data] = this->createPair(key, Data());
                        count_data++;
                    }
                }
            }
            LinkDataMap(RobotBase *robot, const Data &data) : DataMap<Link, Data>(robot->getNLINKS())
            {
                int count_data = 0;
                for (auto leg : *(robot->getLegs()))
                {
                    for (auto key : *(leg->getLinks()))
                    {
                        this->data_[count_data] = this->createPair(key, data);
                        count_data++;
                    }
                }
            }
        };

        template <class Data>
        class JointDataMap : public DataMap<Joint, Data>
        {

        public:
            using DataMap<Joint, Data>::operator=;
            friend class RobotBase;

            ~JointDataMap(){};

        private:
            JointDataMap(RobotBase *robot) : DataMap<Joint, Data>(robot->getNJOINTS())
            {
                int count_data = 0;

                for (auto leg : *(robot->getLegs()))
                {
                    for (auto key : *(leg->getJoints()))
                    {
                        this->data_[count_data] = this->createPair(key, Data());
                        count_data++;
                    }
                }
            }
            JointDataMap(RobotBase *robot, const Data &data) : DataMap<Joint, Data>(robot->getNJOINTS())
            {
                int count_data = 0;

                for (auto leg : *(robot->getLegs()))
                {
                    for (auto key : *(leg->getJoints()))
                    {
                        this->data_[count_data] = this->createPair(key, data);
                        count_data++;
                    }
                }
            }
            JointDataMap(const std::shared_ptr<LimbBase> leg) : DataMap<Joint, Data>(leg->getNJoints())
            {
                int count_data = 0;

                for (auto key : *(leg->getJoints()))
                {
                    this->data_[count_data] = this->createPair(key, Data());
                    count_data++;
                }
            }
            JointDataMap(const std::shared_ptr<LimbBase> leg, const Data &data) : DataMap<Joint, Data>(leg->getNJoints())
            {
                int count_data = 0;

                for (auto key : *(leg->getJoints()))
                {
                    this->data_[count_data] = this->createPair(key, data);
                    count_data++;
                }
            }
            JointDataMap() : DataMap<Joint, Data>() //TO BE USED IF AND ONLY IF THE init FUNCTION WANTS TO BE USED!
            {
            }
        };
        class JointState : public LegDataMap<std::shared_ptr<JointDataMap<double>>>
        {
        public:
            friend class RobotBase;
            using LegDataMap<std::shared_ptr<JointDataMap<double>>>::operator=;
            using LegDataMap<std::shared_ptr<JointDataMap<double>>>::operator[];

            double &operator[](const std::shared_ptr<Joint> joint)
            {
                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
                    {
                        if (joint_pair.key_->getName().compare(joint->getName()) == 0)
                        {
                            return joint_pair.data_;
                        }
                    }
                }
            };

            const double &operator[](const std::shared_ptr<Joint> joint) const
            {
                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
                    {
                        if (joint_pair.key_->getName().compare(joint->getName()) == 0)
                        {
                            return joint_pair.data_;
                        }
                    }
                }
            };

            JointState &operator=(const double data)
            {
                for (auto leg_pair : *this)
                {
                    (*leg_pair.data_).assignAll(data);
                }
                return *this;
            }

            void setZero() { *this = 0; }

            const int size() const
            {
                auto size{0};

                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
                    {
                        size++;
                    }
                }

                return size;
            }

            void print()
            {
                std::cout << "JointState [Name - Value]" << std::endl;
                std::cout << "-------------------------" << std::endl;

                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_)
                    {
                        std::cout << joint_pair.key_->getName() << " - " << joint_pair.data_ << std::endl;
                    }
                }
            }

            std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) { return (*this)[leg->getName()]; }
            const std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) const { return (*this)[leg->getName()]; }

            ~JointState(){};

        private:
            JointState(RobotBase *robot) : LegDataMap<std::shared_ptr<JointDataMap<double>>>(robot){};
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

            ///// A new Map is returned but this points to class variable "linear_jacobian_".
            ///// This because if you define the matrix linear_jacobian inside the method you have a pointer to it (linear_jacobian.data())
            ///// that does not exist anymore outside the method and leads to errors.
            //Map getLinearJacobian() //RT?
            //{
            //    for(int i{0}; i<linear_jacobian_.size(); i++)
            //    {
            //        linear_jacobian_(i) = this->data_[i];
            //    }
            //
            //    ///This operation returns a new Map of linear Jacobian and also updates the linear part of the complete Jacobian
            //    return Map(linear_jacobian_.data(), linear_jacobian_.rows(), linear_jacobian_.cols());
            //};

            Map getLinearJacobian() //RT?
            {
                ///This operation returns a new Map of linear Jacobian and also updates the linear part of the complete Jacobian
                return Map(this->data(), 3, nJoints_);
            };

            ///// A new Map is returned but this points to class variable "angular_jacobian_".
            ///// This because if you define the matrix angular_jacobian inside the method you have a pointer to it (angular_jacobian.data())
            ///// that does not exist anymore outside the method and leads to errors.
            //Map getAngularJacobian() //RT?
            //{
            //    int linear_jacobian_size{3*nJoints_};
            //
            //    for(int i{0}; i<angular_jacobian_.size(); i++)
            //    {
            //        angular_jacobian_(i) = this->data_[i + linear_jacobian_size];
            //    }
            //
            //    return Map(angular_jacobian_.data(), angular_jacobian_.rows(), angular_jacobian_.cols());
            //};

            Map getAngularJacobian() //RT?
            {
                int linear_jacobian_size{3 * nJoints_};
                ///This operation returns a new Map of angular Jacobian and also updates the angular part of the complete Jacobian
                return Map(this->data() + linear_jacobian_size, 3, nJoints_);
            };

            Jacobian &operator=(const Jacobian &other) ///NB: the = operator assumes that nJoints of other is equal to this!
            {
                nJoints_ = other.getNJoints(); ///do this is redundant...

                if (other.data_ == nullptr)
                {
                    data_ = nullptr;
                }
                else
                {
                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = other.data_[i];
                    }
                }

                return *this;
            }

            void print()
            {
                std::cout << "Jacobian [Linear]" << std::endl;
                std::cout << "-----------------" << std::endl;

                std::cout << this->getLinearJacobian() << std::endl;

                std::cout << "\nJacobian [Angular]" << std::endl;
                std::cout << "-----------------" << std::endl;

                std::cout << this->getAngularJacobian() << std::endl;
            }

        private:
            Jacobian(const int nJoints, const double data = 0.0) : Map(NULL, 6, nJoints), nJoints_(nJoints)
            {
                // Data initialization (6: linear and angular part of the jacobian)
                data_ = new double[6 * nJoints_];
                for (int i = 0; i < 6 * nJoints_; ++i)
                {
                    data_[i] = data;
                }

                new (this) Map(data_, 6, nJoints_);
            }

            Jacobian() : Map(NULL, 0, 0), nJoints_(0), data_(nullptr){};

            void init(const Jacobian other)
            {
                nJoints_ = other.getNJoints();

                linear_jacobian_.setZero(3, nJoints_);
                angular_jacobian_.setZero(3, nJoints_);

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

            // TODO: Is it ok to declare these here (not initialized) and initialize them later in "init" function?
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> linear_jacobian_, angular_jacobian_;
        };

        // Get functions
        virtual const int getNLEGS() = 0;
        virtual const int getNARMS() = 0;
        virtual const int getNJOINTS() = 0;
        virtual const int getNLINKS() = 0;

        // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) = 0;    ///TODO: required for the print inside CGaitTimerHex::run() of Ant Controller

        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        // Create a joint state
        JointState makeJointState(const double value = 0.0)
        {
            JointState joint_state{this};

            for (auto leg : *this->getLegs())
            {
                JointDataMap<double> *jdm;
                jdm = new JointDataMap<double>(leg, value);
                std::shared_ptr<JointDataMap<double>> ptr(jdm);
                joint_state[leg] = ptr;
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
        Jacobian makeFootJacobian(const std::shared_ptr<LimbBase> leg, const double data = 0.0) // NRT
        {
            return Jacobian(leg->getNJoints(), data);
        };

        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0) // NRT
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

        virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) = 0; //overridden by Glue

        virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;

        virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;

        virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) = 0;

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
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       const LegDataMap<Jacobian> &robot_jacobian,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                     JointState &tau_joints) = 0;              ///output

        virtual double getRobotMass() const = 0;

        virtual const Eigen::Matrix<double, 3, 1>& getTrunkCOM() const = 0;

        virtual Eigen::Vector3d getRobotCoM() = 0;

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM() = 0;

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) = 0;

        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) = 0;

        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const JointState &q,
                                                               const JointState &qd) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const JointState &q,
                                                                 const JointState &qd) = 0;

        virtual void getMinJointAngle(JointState &q_min) = 0;
        virtual void getMaxJointAngle(JointState &q_max) = 0;
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        virtual void getMinJointAngle(const std::shared_ptr<Joint> joint, double &q_min) { q_min = joint->getMinAngle(); };
        virtual void getMaxJointAngle(const std::shared_ptr<Joint> joint, double &q_max) { q_max = joint->getMaxAngle(); };
        virtual void getMaxJointVelocity(const std::shared_ptr<Joint> joint, double &qd_max) { qd_max = joint->getMaxVelocity(); };
        virtual void getMaxJointEffort(const std::shared_ptr<Joint> joint, double &tau_max) { tau_max = joint->getMaxEffort(); };

        std::string getName()
        {
            return name_;
        };

        // Plugin typedefs
        typedef std::shared_ptr<RobotBase> createRobot_t();
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

    protected:
        const std::string name_;
    };
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_HPP_