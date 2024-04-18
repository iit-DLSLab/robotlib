#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

#include <memory>
#include <vector>

#include "limb_data_map.hpp"
#include "joint_data_map.hpp"
#include "joint.hpp"
#include <Eigen/Dense>

namespace robotlib
{
    /*!
     * @brief JointState class.
     * @details
     * A joint state is stored as a LimbDataMap object, and a JointDataMap object is associated to each leg.
     */
    class JointState : public LimbDataMap<JointDataMap<double>>
    {
        //! RobotBase is a friend class to let it use the private costructor of the JointDataMap class.
        friend class RobotBase;

    public:

        /*!
         * @brief Copy constructor.
         */
        JointState(const JointState& joint_state);

        /*!
         * @brief Default destructor.
         */
        virtual ~JointState() = default;

        /*!
         * @brief Get a sequece of the joints keys.
         * @return vector of joints keys.
         */
        const std::vector<Joint*> getJoints() const;

        /*!
         * @brief Square brackets operator.
         * @details
         * This method allows to access to the data associated to the joint in input.
         * @param[in] joint the joint.
         * @return reference to the data associated to the joint.
         */
        double& operator[](const Joint& joint);

        /*!
         * @brief Square brackets operator.
         * @details
         * This method allows to access to the data associated to the joint in input.
         * Implementation for constant objects.
         * @param[in] joint the joint.
         * @return reference to the data associated to the joint.
         */
        const double& operator[](const Joint& joint) const;

        /*!
         * @brief Square brackets operator.
         * @details
         * This method allows to access to the data associated to the joint in input.
         * @param[in] joint pointer to the joint.
         * @return reference to the data associated to the joint.
         */
        double& operator[](const std::shared_ptr<Joint>& joint);

        /*!
         * @brief Square brackets operator.
         * @details
         * This method allows to access to the data associated to the joint in input.
         * Implementation for constant objects.
         * @param[in] joint pointer to the joint.
         * @return reference to the data associated to the joint.
         */
        const double& operator[](const std::shared_ptr<Joint>& joint) const;

        /*!
         * @brief Assignment operator.
         * @details
         * This method only assign the data not the keys (the keys couldn't change)
         * @param[in] state JointState object whose data will be copied.
         * @return reference to the JointState object pointed by *this*.
         */
        JointState& operator=(const JointState& state);

        /*!
         * @brief Assignment operator for double type.
         * @details
         * This method only assign the data not the keys (the keys couldn't change)
         * @param[in] data data to be assigned to all the objects pointed by *this*.
         * @return reference to the JointState object pointed by *this*.
         */
        JointState& operator=(const double& data);

        /*!
         * @brief Assignment operator for a vector of double type.
         * @brief The vector should have the same size of the JointState.
         * @param[in] data data to be assigned to all the objects pointed by *this*.
         * @return reference to the JointState object pointed by *this*.
         */
        JointState& operator=(const std::vector<double>& vector);

        /*!
         * @brief Equality operator.
         * @param[in] state JointStae to be compared with *this*.
         * @return true if objects are equal.
         */
        bool operator==(const JointState& other) const;

        /*!
         * @brief Transform a JointState object in an Eigen::VectorXd.
         * @return eigen vector filled by the data associated to each joint.
         */
        Eigen::VectorXd toeig_() const;

        /*!
         * @brief Transform the data associated to the joints of a specific limb in an Eigen::VectorXd.
         * @param[in] limb robot's limb.
         * @return eigen vector filled by the data associated to the joints of the limb in input.
         */
        Eigen::VectorXd toeig_(const std::shared_ptr<LimbBase>& limb) const;

        /*!
         * @brief Transform a JointState object in an std::vector.
         * @return eigen vector filled by the data associated to each joint.
         */
        std::vector<double> tovec_() const;

        /*!
         * @brief Set all values of the object pointed by *this* to 0.
         */
        void setZero();

        /*!
         * @brief Get the dimension of the joint state.
         * @return size of the joint state.
         */
        long unsigned int size() const;

        /**
         * @brief Return the max value of the joint state
         * @return double
         */
        double max();

        /**
         * @brief Return the min value of the joint state
         * @return double
         */
        double min();

        /*!
         * @brief Specification of the operator[] from base class.
         */
        using LimbDataMap<JointDataMap<double>>::operator[];

        /*!
         * @brief Verbose method to get the set of JointStates from an specific limb.
         * @details
         * This method allows to access to the data associated to the the joints of a specific limb.
         * Despite of this method you could simply use the operator[].
         * @param[in] limb pointer to the limb.
         * @return reference to the JointDataMap with joint data associated to the limb.
         */
        JointDataMap<double>& getLimbJointState(const std::shared_ptr<LimbBase> limb);

        /*!
         * @brief Verbose method to get the set of JointStates from an specific limb.
         * @details
         * This method allows to access to the data associated to the the joints of a specific limb.
         * Implementation for constant objects.
         * Despite of this method you could simply use the operator[].
         * @param[in] limb pointer to the limb.
         * @return reference to the JointDataMap with joint data associated to the limb.
         */
        const JointDataMap<double>& getLimbJointState(const std::shared_ptr<LimbBase> limb) const;
 
        // JointState &operator<< (double data);
        // JointState &operator, (double data);

        // IteratorJointState<DataPair<Joint, double>> begin();
    
        // IteratorJointState<DataPair<Joint, double>> end();

    private:

        /*!
         * @brief Main constructor.
         * @details 
         * This constructor is private and could only be used by the RobotBase friend class.
         * To create a JointState you should use the method given by the Robot implementation.
         * For example the makeJointState() method from Robot class.
         */
        JointState(const ContainerBase<LimbBase>& limbs, const double& data);
        

        // Iterator<typename DataMap<Joint, double>::DataPair> attribIt;

        // //! List of pointes to the joints of the robot
        // //! This variable is used to easy and accelerate access to the joints keys
		// std::vector<Joint*> joints_;

        //! Local value with the number of data items inside the map
        long unsigned int total_size;
    };

} //namespace robotlib

robotlib::JointState operator*(const double& data, const robotlib::JointState& joint_state);
robotlib::JointState operator*(const Eigen::VectorXd &vec, const robotlib::JointState& joint_state);

Eigen::Quaterniond operator+=(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2);
Eigen::Quaterniond operator-=(const Eigen::Quaterniond &q1, const Eigen::Quaterniond &q2);



#endif //_ROBOTLIB_JOINT_STATE_HPP_