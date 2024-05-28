#ifndef _ROBOTLIB_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_HPP_

#include <memory>
#include "limb_base.hpp"

namespace robotlib
{
    class RobotBase;

    /*!
     * @brief JointDataMap class.
     * @details
     * This templated class is used to store data for each joint.
     * @tparam Data class of the data associated to joint.
     */
    template <class Data>
    class JointDataMap : public DataMap<Joint, Data>
    {
        //! RobotBase is a friend class to let it use the private costructor of the JointDataMap class.
        friend class RobotBase;
        friend class JointState;

    public:
        JointDataMap(const JointDataMap& joint_data_map);
        /*!
         * @brief Destructor.
         */
        virtual ~JointDataMap();

        JointDataMap<Data> &operator=(const JointDataMap<Data> &);
        JointDataMap<Data> operator+(const JointDataMap<Data>&);
        JointDataMap<Data> operator+(const JointDataMap<Data>&) const;
        JointDataMap<Data> &operator+=(const JointDataMap<Data>&);
        JointDataMap<Data> operator*(const JointDataMap<Data>&);
        JointDataMap<Data> operator*(const JointDataMap<Data>&) const;
        JointDataMap<Data> &operator*=(const JointDataMap<Data>&);
        JointDataMap<Data> operator-(const JointDataMap<Data>&);
        JointDataMap<Data> operator-(const JointDataMap<Data>&) const;
        JointDataMap<Data> &operator-=(const JointDataMap<Data>&);
        using DataMap<Joint, Data>::operator=;

    private:
        /*!
         * @brief Constructor.
         * @param[in] robot robot object to be used to get the number of joints.
         */
        JointDataMap(RobotBase* robot_base);
        /*!
         * @brief Constructor.
         * @param[in] robot robot object to be used to get the number of links.
         * @param[in] data data to be associated to each joint.
         */
        JointDataMap(RobotBase* robot_base, const Data& data);
        /*!
         * @brief Constructor.
         * @details
         * It creates a JointDataMap object to associate data to each joint of the leg in input.
         * @param[in] leg shared pointer to the leg object to be used to get its number of joints.
         */
        JointDataMap(const std::shared_ptr<LimbBase> limb_base);
        /*!
         * @brief Constructor.
         * @details
         * It creates a JointDataMap object to associate data to each joint of the leg in input.
         * @param[in] leg shared pointer to the leg object to be used to get its number of joints.
         * @param[in] data data to be associated to each joint of the leg.
         */
        JointDataMap(const std::shared_ptr<LimbBase> limb_base, const Data& data);
    };
}

template <class Data>
robotlib::JointDataMap<Data> operator*(const Eigen::VectorXd &vec, const robotlib::JointDataMap<Data> &state);
template <class Data>
robotlib::JointDataMap<Data> operator*(const double &esc, const robotlib::JointDataMap<Data> &state);

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_