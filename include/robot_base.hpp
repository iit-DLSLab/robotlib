/*!
 * @file robot_base.hpp
 *
 * @brief RobotBase class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

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
    /*!
     * @brief RobotBase class.
     * @details
     * This class represents a generic abstract robot.  It provides data structures such as LegDataMap, JointDataMap, JointState and Jacobian classes. It also provides the hierarchical structure of limbs as a sequence of joints and links together with utility functions like forwardKinematics, inverseKinematics and inverseDynamics.
     */
    class RobotBase
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] name name of the robot
         *  
         * \remark{NOT REAL TIME}
         */
        RobotBase(const std::string &name) : name_(name){};
        
        /*!
         * @brief Destructor.
         *  
         * \remark{NOT REAL TIME}
         */
        virtual ~RobotBase(){};

        /*!
         * @brief DataMap class.
         * @details
         * This templated class is a wrap around a list of Pair objects, storing a data for each key.
         * @tparam Key class of the keys to which associate data.
         * @tparam Data class of the data associated to keys.
         */
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

            /*!
            * @brief Destructor.
            * @details
            * The Destructor is public.
            */
            virtual ~DataMap()
            {
                delete[] data_;
            };

            /*!
             * @brief Begin function to be used with iterators.
             * @return iterator object pointing to the first data of the data_ array.
             */
            virtual Iterator<Pair> begin() { return Iterator<Pair>(&data_[0]); }

            /*!
             * @brief Begin function to be used with iterators.
             * @details
             * Implementation for constant objects.
             * @return iterator object pointing to the first data of the data_ array.
             */
            virtual Iterator<const Pair> begin() const { return Iterator<const Pair>(&data_[0]); }

            /*!
             * @brief End function to be used with iterators.
             * @return iterator object pointing to the last data of the data_ array.
             */
            virtual Iterator<Pair> end() { return Iterator<Pair>(&data_[num_data_]); }

            /*!
             * @brief End function to be used with iterators.
             * @details
             * Implementation for constant objects.
             * @return iterator object pointing to the last data of the data_ array.
             */
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

            /*!
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


            /*!
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
        
        /*!
         * @brief Jacobian class. This class allows the definition of jacobian matrices, divided in linear and angular parts, for a robot having an 
         *        arbitrary number of joints. 
         * @details
         * It inherits from Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class to handle matrix operations 
         * easily. Its dimension is 6 X #joints, where 6 stands for the linear and angular part of the jacobian. The number of joints is arbitrary, 
         * which means that jacobians corresponding to a different number of joints can be defined. E.g. we can define a jacobian for each robot limb 
         * having a different number of joints.
         * 
         * This class provides also functions to access only to linear and angular part of the jacobian plus all eigen functions inherited from the 
         * Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class.
         * 
         * All the constructors produce NRT operations and are private, letting only a RobotBase object to use them. The user is therefore forced to 
         * use a RobotBase object to create a Jacobian one, with the purpose of letting the user managing more carefully NRT operations.
         */
        class Jacobian : public Map
        {
        public:
            
            friend class RobotBase; //!< RobotBase is a friend class to let it uses the private costructors of the Jacobian class

            /*!
		    * @brief Destructor.
            * \remark{NOT REAL TIME}
		    */
            ~Jacobian()
            {
                if (data_ != nullptr)
                    delete[] data_;
            }

            /*!
		     * @brief Get function. It gets the linear part of the jacobian.
             * @details
             * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
             * points to the same memory location associated to the linear part of the jacobian. 
             * 
             * This means that once you do auto linear_jacobian = jacobian.getLinearJacobian(), if you change linear_jacobian it will change also the 
             * linear part of the jacobian object accordingly.
             * @return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
             * \remark{REAL TIME}
		     */
            Map getLinearJacobian()
            {
                // This operation returns a new Map of linear Jacobian and also updates the linear part of the complete Jacobian
                return Map(this->data(), 3, nJoints_);
            };

            /*!
		     * @brief Get function. It gets the angular part of the jacobian.
             * @details
             * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
             * points to the same memory location associated to the angular part of the jacobian. 
             * 
             * This means that once you do auto angular_jacobian = jacobian.getAngularJacobian(), if you change angular_jacobian it will change also 
             * the angular part of the jacobian object accordingly.
             * @return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
             * \remark{REAL TIME}
		     */
            Map getAngularJacobian()
            {
                int linear_jacobian_size{3 * nJoints_};
                // This operation returns a new Map of angular Jacobian and also updates the angular part of the complete Jacobian
                return Map(this->data() + linear_jacobian_size, 3, nJoints_);
            };

            /*!
		     * @brief Equal operator. It gets the angular part of the jacobian.
             * @details
             * The equality is performed over the data structure stored internally.
             * @param[in] other Jacobian object to compare with
             * @return Jacobian&
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

            /*!
		     * @brief Print function.
             * @details
             * It prints the Jacobian object.
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
            /*!
		     * @brief Full constructor.
             * @details
             * This constructor is used to create a Jacobian object given the number of joints and a default value. When using this constructor, the 
             * init function is not needed.
             * @param nJoints number of joints
             * @param data value used to initialize the jacobian
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

            /*!
		     * @brief Empy constructor.
             * @details
             * This constructor is used to create a LegDataMap<Jacobian> object, with "empty" jacobians. Each jacobian may have different sizes, and 
             * the init function is used to initialize each of them.
             * \remark{REAL TIME}
             */
            Jacobian() : Map(NULL, 0, 0), nJoints_(0), data_(nullptr){};

            /*!
		     * @brief Init function. This function is needed to initialize the jacobians of a LegDataMap<Jacobian> object, where each jacobian may 
             * have different sizes.
             * @details
             * For example, you can have a robot with limbs having different number of joints, so each limb has a jacobian of different size.
             * To make real-time code, we created fixed size data structures, like the LegDataMap class, that does not allow you to dinamically change 
             * its length.
             * Therefore, you first create a LegDataMap<Jacobian> object with "empty" jacobians, then you initialize each of them by creating limb 
             * specific jacobian.
             * @param nJoints number of joints 
             * @param init_value value used to initialize the jacobian
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


        /*!
         * @brief Function to create a JointState object.
         * @param[in] value value used to initialize the joint state.
         * \remark{NOT REAL TIME}
         */
        JointState makeJointState(const double value = 0.0)
        {
            JointState joint_state{this};

            for (auto leg : *this->getLegs())
            {
                JointDataMap<double> *jdm;
                jdm = new JointDataMap<double>(leg, value);
                std::shared_ptr<JointDataMap<double>> ptr(jdm);
                joint_state[leg] = ptr; // TODO: is the memory being pointed by joint_state[leg] being released?
            }

            return joint_state;
        }

        /*!
         * @brief Function to create a LegDataMap object object.
         * @tparam Data data type associated to each leg.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this); }

        /*!
         * @brief Function to create a LegDataMap object.
         * @tparam Data data type associated to each leg.
         * @param[in] data data used to initialize the LegDataMap object.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        LegDataMap<Data> makeLegDataMap(const Data &data) { return LegDataMap<Data>(this, data); }

        /*!
         * @brief Function to create a LinkDataMap object.
         * @tparam Data data type associated to each link.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); }

        /*!
         * @brief Function to create a LinkDataMap object.
         * @tparam Data data type associated to each link.
         * @param[in] data data used to initialize the LinkDataMap object.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        LinkDataMap<Data> makeLinkDataMap(const Data &data) { return LinkDataMap<Data>(this, data); }

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the robot.
         * @tparam Data data type associated to each joint.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); }

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the robot.
         * @tparam Data data type associated to each joint.
         * @param[in] data data used to initialize the JointDataMap object.
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMap(const Data &data) { return JointDataMap<Data>(this, data); }

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the leg in input.
         * @tparam Data data type associated to each joint.
         * @param[in] leg leg whose joints are used to create the JointDataMap object.
         *  
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg) { return JointDataMap<Data>(leg); }

        /*!
         * @brief Function to create a JointDataMap object.
         * @details
         * This function creates a data structure to associate data to each joint of the leg in input.
         * @tparam Data data type associated to each joint.
         * @param[in] leg leg whose joints are used to create the JointDataMap object.
         * @param[in] data data used to initialize the JointDataMap object.
         *  
         * \remark{NOT REAL TIME}
         */
        template <class Data>
        JointDataMap<Data> makeJointDataMapPerLeg(const std::shared_ptr<LimbBase> leg, const Data &data) { return JointDataMap<Data>(leg, data); }

        /*!
         * @brief Function to create a LegDataMap object, associating a Jacobian to each leg.
         * @details
         * Each Jacobian has dimention 6x#n_joints_leg. The rows are 6 to have both linear and angular parts of the jacobian; #n_joints_leg is the  number of joints of the leg to which the Jacobian is associated to.
         * @param[in] leg leg whose joints are used to create the JointDataMap object.
         * @param[in] data data used to initialize the JointDataMap object.
         * \remark{NOT REAL TIME}
         */
        LegDataMap<Jacobian> makeFeetJacobian(const double data = 0.0)
        {
            auto feetJac = this->makeLegDataMap<Jacobian>();

            for (auto leg : *(this->getLegs()))
            {
                feetJac[leg].init(leg->getNJoints(), data);
            }
            return feetJac;
        };

        /*!
          * @brief Forward kinematics.
          * @details
          * It computes the position of each end effector (foot) expressed in base frame.
          * @param[in] joint_position angle of each joint.
          * @param[out] end_effector_position position of each end effector (foot) in base frame.
          * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
          */
        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position) = 0;
        /*!
         * @brief Forward kinematics.
         * @details
         * It computes the position and velocity of each end effector (foot) expressed in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[out] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] end_effector_velocity velocity of each end effector (foot) in base frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void forwardKinematics(const robotlib::RobotBase::JointState &joint_position,
                                       const robotlib::RobotBase::JointState &joint_velocity,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       robotlib::RobotBase::LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity) = 0;
        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle, velocity and acceleration of each joint from the position, velocity and acceleration of each end effector expressed in base frame.
        *
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[in] end_effector_velocity velocity of each end effector (foot) in base frame.
         * @param[in] end_effector_accceleration acceleration of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * @param[out] joint_velocity velocity of each joint.
         * @param[out] joint_acceleration acceleration of each joint.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
                                       const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
                                       JointState &joint_position,
                                       JointState &joint_velocity,
                                       JointState &joint_acceleration) = 0;
        /*!
         * @brief Inverse kinematics.
         * @details
         * It computes the angle of each joint from the position of each end effector expressed in base frame.
         * @param[in] end_effector_position position of each end effector (foot) in base frame.
         * @param[out] joint_position angle of each joint.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
                                       JointState &joint_position) = 0;

        /*!
         * @brief Inverse dynamics.
         * @details
         * It computes the torque of each joint and the wrench at the base.
         * @param[in] robot_velocity velocity of the robot base in base frame.
         * @param[in] robot_acceleration  acceleration of the robot base in base frame.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[in] joint_velocity velocity of each joint.
         * @param[in] joint_acceleration acceleration of each joint.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void inverseDynamics(const Eigen::Matrix<double, 6, 1> &robot_velocity,
                                     const Eigen::Matrix<double, 6, 1> &robot_acceleration,
                                     const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                     const JointState &joint_position,
                                     const JointState &joint_velocity,
                                     const JointState &joint_acceleration,
                                     Eigen::Matrix<double, 6, 1> &wrench_base,
                                     JointState &tau_joints) = 0;

        /*!
         * @brief Compute gravity terms.
         * @details
         * Instead of using the inverseDynamics function, you can use this function to compute gravity terms. In this way you can define an optimized version of their computation, avoiding unnecessary computational cost provided by the inverse dynamics function.
         * @param[in] gravity_vector gravity vector in base frame.
         * @param[in] joint_position angle of each joint.
         * @param[out] wrench_base wrench applied to the base.
         * @param[out] tau_joints torque of each joint.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void computeGravityCompensation(const Eigen::Matrix<double, 6, 1> &gravity_vector,
                                                      const JointState &joint_position,
                                                      Eigen::Matrix<double, 6, 1> &wrench_base,
                                                      JointState &tau_joints) = 0;

        // ** GET FUNCTIONS **

        /*!
         * @brief Get robot name.
         * @return robot name.
         * \remark{TODO}
         */
        std::string getName()
        {
            return name_;
        };

        /*!
         * @brief Get number of robot's legs.
         * @return number of robot's legs.
         * \remark{TODO}
         */
        virtual const int getNLEGS() = 0;

        /*!
         * @brief Get number of robot's arms.
         * @return number of robot's arms.
         * \remark{TODO}
         */
        virtual const int getNARMS() = 0;

        /*!
         * @brief Get number of robot's joints.
         * @return number of robot's joints.
         * \remark{TODO}
         */
        virtual const int getNJOINTS() = 0;

        /*!
         * @brief Get number of robot's links.
         * @return number of robot's links.
         * \remark{TODO}
         */
        virtual const int getNLINKS() = 0;

        /*!
         * @brief Get robot's legs.
         * @return robot's legs as a shared pointer to a ContainerBase object.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getLegs() const = 0;

        /*!
         * @brief Get robot's arms.
         * @return robot's arms as a shared pointer to a ContainerBase object.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> getArms() const = 0;

        /*!
         * @brief Get robot's link from link's name.
         * @param[in] name name of the link.
         * @return a shared pointer pointing to the link.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<Link> getLink(const std::string &name) = 0;

        /*!
         * @brief Get robot's joint from joint's name.
         * @param[in] name name of the joint.
         * @return a shared pointer pointing to the joint.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<Joint> getJoint(const std::string &name) = 0;

        /*!
         * @brief Get robot's leg from leg's name.
         * @param[in] name name of the leg.
         * @return a shared pointer pointing to the leg.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<LimbBase> getLeg(const std::string &name) = 0;

        /*!
         * @brief Get robot's arm from arm's name.
         * @param[in] name name of the arm.
         * @return a shared pointer pointing to the arm.
         * \remark{TODO}
         */
        virtual const std::shared_ptr<LimbBase> getArm(const std::string &name) = 0;

        /*!
         * @brief Get lower angle limit of each joint.
         * @details
         * The constructor of the JointState class is not real-time. So, to make this function real time, a reference to a JointState instance is passed as input and it is set with the lower limits of the joints' angles. This is why this get function has a void return type.
         * @param[out] q_min a joint state object to be filled with the lower limits of the joints' angles.
         * \remark{TODO}
         */
        virtual void getMinJointAngle(JointState &q_min) = 0;

        /*!
         * @brief Get upper angle limit of each joint.
         * @details
         * The constructor of the JointState class is not real-time. So, to make this function real time, a reference to a JointState instance is passed as input and it is set with the upper limits of the joints' angles. This is why this get function has a void return type.
         * @param[out] q_max a joint state object to be filled with the upper limits of the joints' angles.
         * \remark{TODO}
         */
        virtual void getMaxJointAngle(JointState &q_max) = 0;

        /*!
         * @brief Get maximum velocity limit of each joint.
         * @details
         * The constructor of the JointState class is not real-time. So, to make this function real time, a reference to a JointState instance is passed as input and it is set with the maximum velocity limits of the joints. This is why this get function has a void return type.
         * @param[out] qd_max a joint state object to be filled with the maximum velocity limits of the joints.
         * \remark{TODO}
         */
        virtual void getMaxJointVelocity(JointState &qd_max) = 0;

        /*!
         * @brief Get maximum torque limit of each joint.
         * @details
         * The constructor of the JointState class is not real-time. So, to make this function real time, a reference to a JointState instance is passed as input and it is set with the maximum torque limits of the joints. This is why this get function has a void return type.
         * @param[out] tau_max a joint state object to be filled with the maximum torque limits of the joints.
         * \remark{TODO}
         */
        virtual void getMaxJointEffort(JointState &tau_max) = 0;

        /*!
         * @brief Get lower angle limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's lower angle limit.
         * \remark{TODO}
         */
        virtual double getMinJointAngle(const std::shared_ptr<Joint> joint) { return joint->getMinAngle(); };

        /*!
         * @brief Get upper angle limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's upper angle limit.
         * \remark{TODO}
         */
        virtual double getMaxJointAngle(const std::shared_ptr<Joint> joint) { return joint->getMaxAngle(); };

        /*!
         * @brief Get maximum velocity limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's maximum velocity limit.
         * \remark{TODO}
         */
        virtual double getMaxJointVelocity(const std::shared_ptr<Joint> joint) { return joint->getMaxVelocity(); };

        /*!
         * @brief Get maximum torque limit of a joint.
         * @param[in] joint a shared pointer to the joint.
         * @return joint's maximum torque limit.
         * \remark{TODO}
         */
        virtual double getMaxJointEffort(const std::shared_ptr<Joint> joint) { return joint->getMaxEffort(); };

        /*!
         * @brief Get position of the destination frame expressed in the origin one.
         * @param[in] joint angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame position expressed in origin one.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                 const std::shared_ptr<Frame> origin,
                                                 const std::shared_ptr<Frame> destination) = 0;

        /*!
         * @brief Get orientation of the destination frame expressed in the origin one.
         * @param[in] joint angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame orientation expressed in origin one.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                    const std::shared_ptr<Frame> origin,
                                                    const std::shared_ptr<Frame> destination) = 0;

        /*!
         * @brief Get pose of the destination frame expressed in the origin one.
         * @param[in] joint angles of the joints.
         * @param[in] origin origin frame.
         * @param[in] destination destination frame.
         * @return destination frame pose expressed in origin one.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                             const std::shared_ptr<Frame> origin,
                                             const std::shared_ptr<Frame> destination) = 0;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @param[in] joint angles of the joints.
         * @param[in] foot foot frame.
         * @return foot position expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                const std::shared_ptr<Frame> foot) = 0;

        /*!
         * @brief Get foot position with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot position.
         * @param[in] joint angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot position expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                     const std::shared_ptr<LimbBase> leg) = 0;

        /*!
         * @brief Get foot orientation expressed in trunk frame.
         * @param[in] joint angles of the joints.
         * @param[in] foot foot frame.
         * @return foot orientation expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<Frame> foot) = 0;

        /*!
         * @brief Get foot orientation with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot orientation.
         * @param[in] joint angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot orientation expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                   const std::shared_ptr<LimbBase> leg) = 0;

        /*!
         * @brief Get foot pose expressed in trunk frame.
         * @param[in] joint angles of the joints.
         * @param[in] foot foot frame.
         * @return foot pose expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<Frame> foot) = 0;

        /*!
         * @brief Get foot pose with respect to the trunk frame, expressed in trunk frame.
         * @details
         * This function gets the foot corresponding to the leg in input and then it computes the foot pose.
         * @param[in] joint angles of the joints.
         * @param[in] leg leg corresponding to the foot.
         * @return foot pose expressed in trunk frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                            const std::shared_ptr<LimbBase> leg) = 0;

        // virtual void getFootJacobian(const JointState &q,
        //                              const std::shared_ptr<LimbBase> leg,
        //                              Jacobian &footJac) = 0;

        /*!
         * @brief Update the linear part of the jacobians in input.
         * @details
         * Each jacobian is associated to a leg. So it maps all the velocities of the leg's joints to foot velocity.
         * @param[in] joint_position angles of the joints.
         * @param[out] robot_jacobian a LegDataMap object, associating a jacobian to each leg.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void updateLinearJacobian(const JointState &joint_position,
                                          LegDataMap<Jacobian> &robot_jacobian) = 0;

        /*!
         * @brief Get total robot mass.
         * @return total robot mass.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual double getRobotMass() const = 0; //TODO: implement it in Robotlib

        /*!
         * @brief Get trunk mass.
         * @return trunk mass.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual double getTrunkMass() const = 0; //TODO: implement it in Robotlib

        /*!
         * @brief Get total legs' mass.
         * @return total legs' mass.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual double getLegsMass() const = 0; //TODO: implement it in Robotlib

        /*!
         * @brief Get the CoM of the trunk.
         * @return trunk's CoM.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual const Eigen::Matrix<double, 3, 1> getTrunkCOM() const = 0;

        // virtual Eigen::Vector3d getRobotCoM() = 0;

        /*!
         * @brief Compute whole body CoM in base frame.
         * @param[in] joint_state angles of the joints.
         * @return whole body CoM in base frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix<double, 3, 1> getWholeBodyCOM(const JointState &joint_state) = 0;

        /*!
         * @brief Compute CoM legs contribution in base frame.
         * @param[in] q angles of the joints.
         * @return CoM legs contribution in base frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getLegContribution(const JointState &q) = 0;

        /*!
         * @brief Compute robot CoM position in world frame, from base pose in world frame.
         * @param[in] q angles of the joints.
         * @param[in] base_orient base orientation in world frame.
         * @param[in] base_pos base position in world frame.
         * @return CoM position in world frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getCoMFromBase(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &base_pos) = 0;

        /*!
          * @brief Compute robot base position in world frame, from CoM position in world frame.
          * @param[in] q angles of the joints.
          * @param[in] base_orient base orientation in world frame.
          * @param[in] com robot CoM postion in world frame.
          * @return base position in world frame.
          * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Vector3d getBaseFromCoM(const JointState &q,
                                               const Eigen::Vector3d &base_orient,
                                               const Eigen::Vector3d &com) = 0;

        /*!
         * @brief Compute whole body CoM velocity in world frame.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] q angles of the joints.
         * @return CoM velocity in world frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
		 */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                const Eigen::Matrix3d &R,
                                                                const JointState &q) = 0;

        /*!
         * @brief Compute whole body com velocity in world frame, without recomputing the CoM offset.
         * @param[in] baseVel base velocity in base frame.
         * @param[in] R rotation matrix of base frame expressed in world frame.
         * @param[in] offset_com CoM offset in base frame.
         * @return CoM velocity in world frame.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual Eigen::Matrix<double, 6, 1> getWholeBodyCOMVelFB(const Eigen::Matrix<double, 6, 1> &baseVel,
                                                                 const Eigen::Matrix3d &R,
                                                                 const Eigen::Vector3d offset_com) = 0;

        // ** SET FUNCTIONS **

        /*!
         * @brief Set trunk's CoM.
         * @param[in] trunk_com CoM of trunk to be set.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void setTrunkCom(const Eigen::Vector3d &trunk_com) = 0;
        /*!
         * @brief Set trunk's mass.
         * @param[in] trunk_com mass of trunk to be set.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
         */
        virtual void setTrunkMass(const double& trunk_mass) = 0;

		/*!
		 * @brief Set inverse kinematics time period.
         * @details
         * This time period is the controller's loop time period. The time period needs to be set before calling the inverse kinematics.
         * @param[in] period period of the controller.
         * \remark{NOT REAL TIME or REAL TIME depending on glue code implementation}
		 */
        virtual void setInvKinTimePeriod(const double& period) = 0;

        // ** ROBOT INFORMATION **

		/*!
		 * @brief Print robot hierarchy.
         * \remark{TODO}
		 */
        virtual void printRobotHierarchy()
        {
            for(auto leg: *this->getLegs())
            {
                std::cout << "\nLeg: " << leg->getName() << std::endl;

                for(auto joint : *leg->getJoints())
                {
                    std::cout << leg->jointToParentName(joint) << " --> " << joint->getName() << " --> " << leg->jointToChildName(joint) << std::endl;
                }

                for(auto link : *leg->getLinks())
                {
                    std::cout << leg->linkToParentName(link) << " --> " << link->getName() << " --> " << leg->linkToChildName(link) << std::endl;
                }
            }

            for(auto arm: *this->getArms())
            {
                std::cout << "\nArm: " << arm->getName() << std::endl;

                for(auto joint : *arm->getJoints())
                {
                    std::cout << arm->jointToParentName(joint) << " --> " << joint->getName() << " --> " << arm->jointToChildName(joint) << std::endl;
                }

                for(auto link : *arm->getLinks())
                {
                    std::cout << arm->linkToParentName(link) << " --> " << link->getName() << " --> " << arm->linkToChildName(link) << std::endl;
                }
            }
        }

        // ** CLASS FACTORY FUNCTION DECLARATIONS **

        /*!
         * @brief Factory function to load at run-time the glue code, creating a robot object.
         * \remark{NOT REAL TIME}
		 */
        typedef std::shared_ptr<RobotBase> createRobot_t();

        /*!
         * @brief Factory function to load at run-time the glue code, with external urdf in input.
         * @param[in] robot_urdf the urdf of the robot in string format.
         * \remark{NOT REAL TIME}
		 */
        typedef std::shared_ptr<RobotBase> createRobotWithUrdf_t(const std::string& robot_urdf);

         /*!
         * @brief Factory function to destroy the robot object.
         * \remark{NOT REAL TIME}
		 */
        typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

    protected:
        const std::string name_; //!< Robot name
    };
} // namespace robotlib

#endif // _ROBOTLIB_ROBOT_BASE_HPP_