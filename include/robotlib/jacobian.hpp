#ifndef _ROBOTLIB_JACOBIAN_HPP_
#define _ROBOTLIB_JACOBIAN_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace robotlib
{
    //! Alias for Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>.
    using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;

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
    class Jacobian : public Map
    {
        //! Robot is a friend class to let it use the private methods of the Joint class.
		template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
		friend class Robot;
        friend class RobotBase;

    public:     
        /*!
		 * @brief Copy constructor.
		*/
        // Jacobian(const Jacobian& jacobian);

        /*!
		 * @brief Destructor.
		*/
        ~Jacobian();

        // Jacobian(Map& map)
        //     : Map(map)
        // {
        //     std::cout << "TESTING" << std::endl;
        // }

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


        // Jacobian operator*(std::vector<double>& vec);

        // using Map::operator*;

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

        /*!
		 * @brief Empy constructor.
         * @details
         * This constructor is used to create a LimbDataMap<Jacobian> object, with "empty" jacobians. 
         * Each jacobian may have different sizes, and the init function is used to initialize each of them.
         */
        Jacobian(const std::vector<double>& data);

        //! Number of joints.
        int nJoints_;

        // Squashed matrix
        double *data_;

    };
}

#endif //_ROBOTLIB_JACOBIAN_HPP_