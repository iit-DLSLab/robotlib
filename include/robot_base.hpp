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
#include <stdexcept>


namespace robotlib
{
    class RobotBase
    {
    public:
        RobotBase(const std::string &name) : name_(name){};
        ~RobotBase(){};

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

            JointState &operator=(const JointState &other)
            {
                for (auto &leg_pair : *this)
                {
                    for(auto &joint_pair: *leg_pair.data_)
                    {   
                        joint_pair.data_ = other[joint_pair.key_];
                    }
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

            /**
             * @brief Return the max value of the joint state
             * @return double
             */
            double max()
            {
                double max_value{0};
                bool first_val{true};

                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
                    {
                        double value = joint_pair.data_;
                        if (first_val==true)
                        {
                            max_value = value;
                            first_val=false;
                        }
                        else
                        {
                            if(value>max_value)
                            {
                                max_value = value;
                            }
                        }
                    }
                }
                return max_value;
            }


            /**
             * @brief Return the min value of the joint state
             * @return double
             */
            double min()
            {
                double min_value{0};
                bool first_val{true};

                for (auto &leg_pair : *this)
                {
                    for (auto &joint_pair : *leg_pair.data_) //iterate over the JointDataMap
                    {
                        double value = joint_pair.data_;
                        if (first_val==true)
                        {
                            min_value = value;
                            first_val=false;
                        }
                        else
                        {
                            if(value<min_value)
                            {
                                min_value = value;
                            }
                        }
                    }
                }
                return min_value;
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
            // FIX ME (& with shared_ptr)
            std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) { return (*this)[leg->getName()]; }
            const std::shared_ptr<JointDataMap<double>> &getLegJointState(const std::shared_ptr<LimbBase> leg) const { return (*this)[leg->getName()]; }

            ~JointState(){};

        private:
            JointState(RobotBase *robot) : LegDataMap<std::shared_ptr<JointDataMap<double>>>(robot){};
        };

        using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
        
        /**
        * @brief Jacobian class. This class allows the definition of jacobian matrices, divided in linear and angular parts, for a robot having an 
        *        arbitrary number of joints. 
        * 
        * It inherits from Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class to handle matrix operations 
        * easily. Its dimension is 6 X #joints, where 6 stands for the linear and angular part of the jacobian. The number of joints is arbitrary, 
        * which means that jacobians corresponding to a different number of joints can be defined. E.g. we can define a jacobian for each robot limb 
        * having a different number of joints.
        * 
        * This calss provides also functions to access only to linear and angular part of the jacobian plus all eigen functions inherited from the 
        * Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class.
        * 
        * All the constructors produce NRT operations and are private, letting only a RobotBase object to use them. The user is therefore forced to 
        * use a RobotBase object to create a Jacobian one, with the purpose of letting the user managing more carefully NRT operations.
        */
        class Jacobian : public Map
        {
        public:
            
            friend class RobotBase; //!< RobotBase is a friend class to let it uses the private costructors of the Jacobian class

            /**
		    * @brief Destructor.
            * 
            * \remark{NOT REAL TIME}
		    */
            ~Jacobian()
            {
                if (data_ != nullptr)
                    delete[] data_;
            }

            /**
		    * @brief Get function. It gets the linear part of the jacobian.
            * 
            * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
            * points to the same memory location associated to the linear part of the jacobian. 
            * 
            * This means that once you do auto linear_jacobian = jacobian.getLinearJacobian(), if you change linear_jacobian it will change also the 
            * linear part of the jacobian object accordingly.
            * 
            * @return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            * 
            * \remark{REAL TIME}
		    */
            Map getLinearJacobian()
            {
                // This operation returns a new Map of linear Jacobian and also updates the linear part of the complete Jacobian
                return Map(this->data(), 3, nJoints_);
            };

            /**
		    * @brief Get function. It gets the angular part of the jacobian.
            * 
            * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
            * points to the same memory location associated to the angular part of the jacobian. 
            * 
            * This means that once you do auto angular_jacobian = jacobian.getAngularJacobian(), if you change angular_jacobian it will change also 
            * the angular part of the jacobian object accordingly.
            * 
            * @return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            * 
            * \remark{REAL TIME}
		    */
            Map getAngularJacobian()
            {
                int linear_jacobian_size{3 * nJoints_};
                // This operation returns a new Map of angular Jacobian and also updates the angular part of the complete Jacobian
                return Map(this->data() + linear_jacobian_size, 3, nJoints_);
            };

            /**
		    * @brief Equal operator. It gets the angular part of the jacobian.
            * 
            * The equality is performed over the data structure stored internally.
            * 
            * @param other Jacobian object to compare with
            * @return Jacobian&
            * 
            * \remark{REAL TIME}
		    */
            Jacobian &operator=(const Jacobian &other)
            {
                if (nJoints_ != other.nJoints_)
                {
                    throw std::invalid_argument("CANNOT USE = OPERATOR FOR JACOBIANS WITH DIFFERENT SIZE. First size: 6x" 
                                            + std::to_string(nJoints_) +", second size: 6x" + std::to_string(other.nJoints_));
                }

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

            /**
		    * @brief Print function.
            * 
            * It prints the Jacobian object.
            * 
            * \remark{NOT REAL TIME}
		    */
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
            /**
		    * @brief Full constructor.
            * 
            * This constructor is used to create a Jacobian object given the number of joints and a default value. When using this constructor, the 
            * init function is not needed.
            * 
            * @param nJoints number of joints
            * @param data value used to initialize the jacobian
            * 
            * \remark{NOT REAL TIME}
		    */
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

            /**
		    * @brief Empy constructor.
            * 
            * This constructor is used to create a LegDataMap<Jacobian> object, with "empty" jacobians. Each jacobian may have different sizes, and 
            * the init function is used to initialize each of them.
            * 
            * \remark{REAL TIME}
            */
            Jacobian() : Map(NULL, 0, 0), nJoints_(0), data_(nullptr){};

            /**
		    * @brief Init function. This function is needed to initialize the jacobians of a LegDataMap<Jacobian> object, where each jacobian may 
            * have different sizes.
            * 
            * For example, you can have a robot with limbs having different number of joints, so each limb has a jacobian of different size.
            * To make real-time code, we created fixed size data structures, like the LegDataMap class, that does not allow you to dinamically change 
            * its length.
            * Therefore, you first create a LegDataMap<Jacobian> object with "empty" jacobians, then you initialize each of them by creating limb 
            * specific jacobian.
            * 
            * @param nJoints number of joints 
            * @param init_value value used to initialize the jacobian
            * 
            * \remark{NOT REAL TIME}
		    */
            
            void init(const int nJoints, const double init_value = 0.0)
            {
                nJoints_ = nJoints;

                data_ = new double[6 * nJoints_];
                for (int i = 0; i < 6 * nJoints_; ++i)
                {
                    data_[i] = init_value;
                }
                new (this) Map(data_, 6, nJoints_);
            }

            int nJoints_;   //!< Number of joints
            double *data_;  //!< Squashed matrix
        };


        // ** FUNCTIONS TO MAKE NRT OBJECTS ** 

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
                feetJac[leg].init(leg->getNJoints(), data);
            }
            return feetJac;
        };

        virtual void forwardKinematics(const JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
                                       const JointState &joint_velocity,
                                       const JointState &joint_acceleration,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration) = 0;

        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;

        // ** INVERSE DYNAMICS ** 

        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                     JointState &tau_joints) = 0;              ///output
        
        /**
        * @brief Function to compute gravity terms.
        * 
        * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized * version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function
        * 
        * @param[in] gravity_vector gravity vector
        * @param[in] joint_position joint angle
        * @param[out] wrench_base wrench of the base
        * @param[out] tau_joints tau of each joint
        * 
        * \remark{NOT REAL TIME / REAL TIME depending on glue code}
        */
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      Eigen::Matrix<double, 6, 1> &wrench_base, ///output
                                                      JointState &tau_joints) = 0;              ///output

        // ** GET FUNCTIONS **

        std::string getName()
        {
            return name_;
        };
        
        virtual const int getNLEGS() = 0;
        virtual const int getNARMS() = 0;
        virtual const int getNJOINTS() = 0;
        virtual const int getNLINKS() = 0;

        // virtual const std::shared_ptr<LimbBase> getNextLeg(const std::shared_ptr<LimbBase>& leg) = 0;    ///TODO: required for the print inside CGaitTimerHex::run() of Ant Controller

        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;

        virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;

        virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) = 0;

        virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

        virtual void getMinJointAngle(JointState &q_min) = 0;
        virtual void getMaxJointAngle(JointState &q_max) = 0;
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        virtual void getMinJointAngle(const std::shared_ptr<Joint> joint, double &q_min) { q_min = joint->getMinAngle(); };
        virtual void getMaxJointAngle(const std::shared_ptr<Joint> joint, double &q_max) { q_max = joint->getMaxAngle(); };
        virtual void getMaxJointVelocity(const std::shared_ptr<Joint> joint, double &qd_max) { qd_max = joint->getMaxVelocity(); };
        virtual void getMaxJointEffort(const std::shared_ptr<Joint> joint, double &tau_max) { tau_max = joint->getMaxEffort(); };

        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) = 0; //overridden by Glue

        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<Frame> foot) = 0; //overridden by Glue

        virtual void getFootPosition(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Eigen::Vector3d &footPos) = 0; //overridden by Glue

        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<LimbBase> leg) = 0; //overridden by Glue

        virtual void getFootJacobian(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg,
                                     Jacobian &footJac) = 0; //overridden by Glue

        virtual void updateLinearJacobian(const JointState &joints_positions,
                                          LegDataMap<Jacobian> &robot_jacobian) = 0; //overridden by Glue


        // TODO: compute total mass from links and trunk masses (it could be even implemented in Robot class)
        virtual double getRobotMass() const = 0;

        virtual double getTrunkMass() const = 0;

        virtual double getLegsMass() const = 0;

        // NB: eventually make the get function void for possible NRT issue
        virtual const Eigen::Matrix<double, 3, 1>& getTrunkCOM() const = 0;

        virtual Eigen::Vector3d getRobotCoM() = 0;

        /**
		 * @brief Compute whole body com in base frame
		 * @return com offset
		 */
        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM() = 0;

        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) = 0;

        virtual Eigen::Vector3d getLegContribution(const JointState &q) = 0;

        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) = 0;

        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &CoM) = 0;

        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVel(const JointState &q,
                                                               const JointState &qd) = 0;
        /**
		 * @brief Compute whole body com velocity in world frame, considering joint influence
         * @param baseVel base velocity in base frame
         * @param rotationMx rotation matrix of base frame expressed in world frame
         * @param q joints angle
         * @param qd joints velocity
		 * @return com velocity in world frame
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const JointState &q,
                                                                 const JointState &qd) = 0;
        /**
		 * @brief Compute whole body com velocity in world frame without considering joint influence
         * @param baseVel base velocity in base frame
         * @param rotationMx rotation matrix of base frame expressed in world frame
         * @param q joints angle
         * @param qd joints velocity
		 * @return com velocity in world frame
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                        const Eigen::Matrix3d &rotationMx,
                                                        const JointState &q) = 0;
        /**
         * @brief Compute whole body com velocity in world frame, without recomputing the com offset, and without considering joint influence
         * @param baseVel base velocity in base frame
         * @param rotationMx rotation matrix of base frame expressed in world frame
         * @param q joints angle
         * @param qd joints velocity
         * @param qd com offset in base frame
         * @return com velocity in world frame
         */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &rotationMx,
                                                                 const Eigen::Vector3d offset_com) = 0;

        // ** SET FUNCTIONS **

        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) = 0;

        virtual void setTrunkMass(const double& trunk_mass) = 0;

		/**
		 * @brief Set inverse kinematics time period.
         * @param period period of the controller
		 */
        virtual void setInvKinTimePeriod(const double& period) = 0;


        // ** PLUGIN TYPEDEFS ** 

        typedef std::shared_ptr<RobotBase> createRobot_t();
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);
        typedef std::shared_ptr<RobotBase> createRobotWithUrdf_t(const std::string&);
        typedef void destroyRobotWithUrdf_t(std::shared_ptr<RobotBase>);

    protected:
        const std::string name_; //!< Robot name
    };
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_HPP_