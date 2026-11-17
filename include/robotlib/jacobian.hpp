#ifndef _ROBOTLIB_JACOBIAN_HPP_
#define _ROBOTLIB_JACOBIAN_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace robotlib
{
    //! Alias for Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>.
    // using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
    using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    /*!
     * @brief Jacobian class. This class allows the definition of jacobian matrices, divided in linear and angular parts, for a robot having an 
     *        arbitrary number of joints. 
     * @details
     * It inherits from Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class to handle matrix operations 
     * easily. Its dimension is 6 X num_joints, where 6 stands for the linear and angular part of the jacobian. 
     * num_joints is the number of joints and it is arbitrary, which means that jacobians corresponding to a different number of joints 
     * can be defined. E.g. we can define a jacobian for each robot limb where each of them can have a different number of joints.
     * 
     * This class provides also functions to access only to linear and angular part of the jacobian plus of course all the eigen functions
     * inherited from the Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> class.
     */
    class Jacobian : public Matrix
    {
        //! Robot is a friend class to let it use the private methods of the Joint class.
		friend class Robot;
        friend class RobotBase;

    public:     
        /*!
		 * @brief Copy constructor.
		*/
        Jacobian(const Jacobian& jacobian);

        Jacobian();

        /*!
		 * @brief Destructor.
		*/
        ~Jacobian();

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
        Matrix getLinearJacobian();

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
        Matrix getAngularJacobian();

        using Matrix::operator=;


    private:
        /*!
		 * @brief Full constructor.
         * @details
         * This constructor is used to create a Jacobian object given the number of joints and a default value. 
         * When using this constructor, the init function is not needed.
         * @param [in] nJoints number of joints.
         * @param [in] data value used to initialize the jacobian.
		 */
        Jacobian(const int nJoints, const double& data = 0.0);

    };
}

#endif //_ROBOTLIB_JACOBIAN_HPP_