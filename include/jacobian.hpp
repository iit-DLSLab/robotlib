#ifndef _ROBOTLIB_JACOBIAN_HPP_
#define _ROBOTLIB_JACOBIAN_HPP_

#include <Eigen/Dense>

namespace robotlib
{
    //! Alias for Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>.
    using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

    /*!
     * @brief Jacobian class. This class allows the definition of jacobian matrices, divided in linear and angular parts, for a robot having an 
     *        arbitrary number of joints. 
     * @details
     * It inherits from Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class to handle matrix operations 
     * easily. Its dimension is 6 X num_joints, where 6 stands for the linear and angular part of the jacobian. num_joints is the number of joints and it is arbitrary, which means that jacobians corresponding to a different number of joints can be defined. E.g. we can define a jacobian for each robot limb where each of them can have a different number of joints.
     * 
     * This class provides also functions to access only to linear and angular part of the jacobian plus of course all the eigen functions inherited from the Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class.
     */
    class Jacobian : public Map
    {
        //! RobotBase is a friend class to let it use the private costructors of the Jacobian class.
        friend class RobotBase;

        template <class Data> 
        friend class LegDataMap;

    public:        
        template <class Data> friend class LegDataMap;
        template <class Key, class Data> friend class DataMap;
        friend class Pair;

        /*!
		 * @brief Destructor.
		*/
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

        /*!
		 * @brief Get function. It gets the linear part of the jacobian.
         * @details
         * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
         * points to the same memory location associated to the linear part of the jacobian. 
         * 
         * This means that once you do auto linear_jacobian = jacobian.getLinearJacobian(), if you change linear_jacobian it will change also the 
         * linear part of the jacobian object accordingly.
         * @return linear part of the jacobian.
		 */
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

        /*!
		 * @brief Get function. It gets the angular part of the jacobian.
         * @details
         * The returned object is of type Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>. This means that it 
         * points to the same memory location associated to the angular part of the jacobian. 
         * 
         * This means that once you do auto angular_jacobian = jacobian.getAngularJacobian(), if you change angular_jacobian it will change also 
         * the angular part of the jacobian object accordingly.
         * @return angular part of the jacobian.
		 */
        Map getAngularJacobian();

        /*!
		 * @brief Equal operator.
         * @details
         * The equality is performed over the data structure stored internally.
         * @param[in] jacobian Jacobian object whose values are set to the object pointed by *this*.
         * @return reference to the object pointed by *this*.
		 */
        Jacobian &operator=(const Jacobian &other);

        /*!
		 * @brief Print function.
         * @details
         * It prints the Jacobian object.
		 */
        void print();

    private:
        /*!
		 * @brief Full constructor.
         * @details
         * This constructor is used to create a Jacobian object given the number of joints and a default value. When using this constructor, the 
         * init function is not needed.
         * @param [in] nJoints number of joints.
         * @param [in] data value used to initialize the jacobian.
		 */
        Jacobian(const int nJoints, const double data = 0.0);

        /*!
		 * @brief Empy constructor.
         * @details
         * This constructor is used to create a LegDataMap<Jacobian> object, with "empty" jacobians. Each jacobian may have different sizes, and 
         * the init function is used to initialize each of them.
         */
        Jacobian();

        /*!
		 * @brief Init function. This function is needed to initialize the jacobians of a LegDataMap<Jacobian> object, where each jacobian may 
         * have different sizes.
         * @details
         * For example, you can have a robot with limbs having different number of joints, so each limb has a jacobian of different size.
         * 
         * To avoid dynamic memory allocation, fixed-size data structures are defined in Robotlib, like the LegDataMap class, that does not allow you to dinamically change its length. Therefore, you first create a LegDataMap<Jacobian> object with "empty" jacobians, then you initialize each of them by creating limb specific jacobians.
         * @param [in] nJoints number of joints.
         * @param [in] init_value value used to initialize the jacobian.
		 */
        void init(const int nJoints, const double init_value = 0.0);

        //! Number of joints.
        int nJoints_;

        // Squashed matrix
        double *data_;

        // TODO: Is it ok to declare these here (not initialized) and initialize them later in "init" function?
        // Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> linear_jacobian_, angular_jacobian_;
    };
}

#endif //_ROBOTLIB_JACOBIAN_HPP_