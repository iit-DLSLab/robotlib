#ifndef _ROBOTLIB_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_HPP_

#include "joint.hpp"
namespace robotlib
{
    /*!
     * @brief JointDataMap class.
     * @details
     * This templated class is used to store data for each joint.
     * @tparam Data class of the data associated to joint.
     */
    template <class Data>
    class JointDataMap : public DataMap<Joint, Data>
    {
        //! RobotBase is a friend class to let it use the private costructor of the LinkDataMap class.
        friend class RobotBase;
        friend class JointState;
        
    public:

        JointDataMap(const JointDataMap& other);

        JointDataMap<Data>& operator=(const JointDataMap<Data>& other);

        JointDataMap<Data>& operator=(const std::vector<Data>& other);
        

        /*!
         * @brief Destructor.
         */
        virtual ~JointDataMap() = default;

        // JointDataMap<Data> operator+(const JointDataMap<Data>&);
        JointDataMap<Data> operator+(const JointDataMap<Data>&) const;

        JointDataMap<Data>& operator+=(const JointDataMap<Data>&);
        // JointDataMap<Data> operator*(const JointDataMap<Data>&);
        JointDataMap<Data> operator*(const JointDataMap<Data>&) const;
        JointDataMap<Data>& operator*=(const JointDataMap<Data>&);


        // JointDataMap<Data> operator-(const JointDataMap<Data>&);
        JointDataMap<Data> operator-(const JointDataMap<Data>&) const;

        JointDataMap<Data> &operator-=(const JointDataMap<Data>&);

        /*!
         * @brief Specification of the operator[] from base class.
         */
        using DataMap<Joint, Data>::operator==;
        using DataMap<Joint, Data>::operator[];

   private:

        /*!
         * @brief Constructor.
         * @param[in] joints joints list to be used to map the data.
         * @param[in] data data to be associated all the joints.
         */
        JointDataMap(const ContainerBase<Joint>& joints, const Data& data);

        /*!
         * @brief Constructor.
         * @param[in] joints joints list to be used to map the data.
         * @param[in] data data list to be associated to each joint.
         */
        JointDataMap(const ContainerBase<Joint>& joints, const std::vector<Data>& data);

        /*!
         * @brief Constructor.
         * @param[in] joints list of pointers to joints used to map the data.
         * @param[in] data data to be associated all the joints.
         */
        JointDataMap(const ContainerBase<std::shared_ptr<Joint>> joints, const Data& data);
    };
}

// template <class Data>
// robotlib::JointDataMap<Data> operator*(const Eigen::VectorXd &vec, const robotlib::JointDataMap<Data> &state);

// template <class Data>
// robotlib::JointDataMap<Data> operator*(const double &esc, const robotlib::JointDataMap<Data> &state);

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_