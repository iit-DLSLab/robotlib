#ifndef _ROBOTLIB_JACOBIAN_HPP_
#define _ROBOTLIB_JACOBIAN_HPP_

#include <Eigen/Dense>

namespace robotlib
{

    using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

    class Jacobian : public Map
    {
    public:
        friend class RobotBase;

        template <class Data> friend class LegDataMap;
        template <class Key, class Data> friend class DataMap;
        friend class Pair;
        
        ~Jacobian();

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

        Map getLinearJacobian();

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

        Map getAngularJacobian();

        Jacobian &operator=(const Jacobian &other);

        void print();

    private:
        Jacobian(const int nJoints, const double data = 0.0);

        /**
        * @brief Empy constructor for the Jacobian class.
        * This constructor is used to create a LegDataMap<Jacobian> object, with "empty" jacobians.
        * Each jacobian may have different sizes, and the init function is used to initialize each of them. 
        */
        Jacobian();

        /**
        * @brief Init function for the Jacobian class. 
        * This function is needed to initialize the jacobians of a LegDataMap<Jacobian> object, where each jacobian may have different sizes. 
        * For example, you can have a robot with limbs having different number of joints, so each limb has a jacobian of different size.
        * To make real-time code, we created fixed size data structures, like the LegDataMap class, that does not allow you to dinamically change its length.
        * Therefore, you first create a LegDataMap<Jacobian> object with "empty" jacobians, then you initialize each of them by creating limb specific jacobian
        * @example makeFeetJacobian function 
        * @param nJoints number of joints 
        * @param init_value value used to initialize the jacobian
        */
        void init(const int nJoints, const double init_value = 0.0);

        int nJoints_;
        double *data_; // Squashed matrix

        // TODO: Is it ok to declare these here (not initialized) and initialize them later in "init" function?
        // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> linear_jacobian_, angular_jacobian_;
    };
}

#endif //_ROBOTLIB_JACOBIAN_HPP_