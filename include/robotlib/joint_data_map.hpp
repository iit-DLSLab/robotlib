#ifndef _ROBOTLIB_JOINT_DATA_MAP_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_HPP_

#include <memory>
#include "limb_base.hpp"

namespace robotlib
{
    class RobotBase;

    template <class Data>
    class JointDataMap : public DataMap<Joint, Data>
    {
        friend class RobotBase;
        friend class JointState;

    public:
        JointDataMap(const JointDataMap&);
        ~JointDataMap();

        JointDataMap<Data> &operator=(const JointDataMap<Data> &);
        JointDataMap<Data> operator+(const JointDataMap<Data>&);
        JointDataMap<Data> &operator+=(const JointDataMap<Data>&);
        JointDataMap<Data> operator*(const JointDataMap<Data>&);
        JointDataMap<Data> &operator*=(const JointDataMap<Data>&);
        JointDataMap<Data> operator-(const JointDataMap<Data>&);
        JointDataMap<Data> &operator-=(const JointDataMap<Data>&);
        using DataMap<Joint, Data>::operator=;

    private:
        JointDataMap(RobotBase*);
        JointDataMap(RobotBase*, const Data&);
        JointDataMap(const std::shared_ptr<LimbBase>);
        JointDataMap(const std::shared_ptr<LimbBase>, const Data&);
    };
}

template <class Data>
robotlib::JointDataMap<Data> operator*(const Eigen::VectorXd &vec, const robotlib::JointDataMap<Data> &state);
template <class Data>
robotlib::JointDataMap<Data> operator*(const double &esc, const robotlib::JointDataMap<Data> &state);

#include "joint_data_map.tpp"

#endif //_ROBOTLIB_JOINT_DATA_MAP_HPP_