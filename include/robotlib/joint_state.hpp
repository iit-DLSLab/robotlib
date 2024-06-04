#ifndef _ROBOTLIB_JOINT_STATE_HPP_
#define _ROBOTLIB_JOINT_STATE_HPP_

#include <memory>
#include <vector>

#include "leg_data_map.hpp"
#include "joint_data_map.hpp"
#include "joint.hpp"
#include <Eigen/Dense>

namespace robotlib
{
    /*!
     * @brief JointState class.
     * @details
     * A joint state is stored as a LegDataMap object, and a JointDataMap object is associated to each leg.
     */
    class JointState : public LegDataMap<JointDataMap<double>>
    {
        //! RobotBase is a friend class to let it use the private costructor of the JointDataMap class.
        friend class RobotBase;

    public:
        JointState(const JointState& joint_state);

        /*!
         * @brief Destructor.
         */
        virtual ~JointState();

        using LegDataMap<JointDataMap<double>>::operator[];

        /*!
         * @brief Square brackets operator.
         * @details
         * This function allows to access to the data associated to the joint in input.
         * @param[in] joint shared pointer pointing to the joint.
         * @return reference to the data associated to the joint.
         */
        double &operator[](const std::shared_ptr<Joint> joint);

        /*!
         * @brief Square brackets operator.
         * @details
         * This function allows to access to the data associated to the joint in input.
         * 
         * Implementation for constant objects.
         * @param[in] joint shared pointer pointing to the joint.
         * @return reference to the data associated to the joint.
         */
        const double &operator[](const std::shared_ptr<Joint> joint) const;

        /*!
         * @brief Function getting the joint value from the joint name.
         * @details
         * This function allows to access to the data associated to the joint name in input.
         * 
         * @param[in] joint_name joint name.
         * @return reference to the data associated to the joint.
         */
        double getJointValue(const std::string &joint_name) const;

        /*!
         * @brief Function setting the joint value from the joint name.
         * @details
         * This function allows to set the data associated to the joint name in input.
         * 
         * @param[in] joint_name joint name.
         */
        void setJointValue(const std::string &joint_name, double value);

        /*!
         * @brief Equal operator.
         * @param[in] joint_state JointState object whose data is assigned to the object pointed by *this*.
         * @return reference to the JointState object pointed by *this*.
         */
        JointState &operator=(const JointState& joint_state);

        /*!
         * @brief Equal operator.
         * @param[in] data data to be assigned to the object pointed by *this*.
         * @return reference to the JointState object pointed by *this*.
         */
        JointState &operator=(const double data);
        JointState &operator=(const std::vector<double>& vector);
        JointState &operator=(const Eigen::VectorXd& data);

        JointState operator+(const JointState& joint_state);
        JointState &operator+=(const JointState& joint_state);
        JointState operator-(const JointState& joint_state);
        JointState &operator-=(const JointState& joint_state);

        operator std::vector<double>() const;

        JointState &operator<< (double data);
        JointState &operator, (double data);

        /*!
         * @brief Transform a JointState object in an Eigen::VectorXd.
         * @return eigen vector filled by the data associated to each joint.
         */
        Eigen::VectorXd vec_() const;

        /*!
         * @brief Transform the data associated to the joints of the leg in input in an Eigen::VectorXd.
         * @param[in] leg robot's leg.
         * @return eigen vector filled by the data associated to the jointsof the leg in input.
         */
        Eigen::VectorXd vec_(const std::shared_ptr<LimbBase> leg) const;

        /*!
         * @brief Set all values of the object pointed by *this* to 0.
         */
        void setZero();

        /*!
         * @brief Get the dimension of the joint state.
         * @return size of the joint state.
         */
        int size() const;

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
         * @brief Print joint state information.
         */
        void print() const;

        JointDataMap<double>& getLegJointState(const std::shared_ptr<LimbBase> leg);
        const JointDataMap<double>& getLegJointState(const std::shared_ptr<LimbBase> leg) const;

    private:

        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> limbs);
        JointState(std::shared_ptr<const ContainerBase<std::shared_ptr<LimbBase>>> limbs, double data);
        
        Iterator<DataMap<Joint, double>::Pair> jointIt;
    };

} //namespace robotlib

robotlib::JointState operator*(const double& data, const robotlib::JointState& joint_state);
robotlib::JointState operator*(const Eigen::VectorXd &vec, const robotlib::JointState& joint_state);

#endif //_ROBOTLIB_JOINT_STATE_HPP_