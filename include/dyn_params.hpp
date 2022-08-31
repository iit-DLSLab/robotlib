/**
 * @file dyn_params.hpp
 *
 * @brief DynParams class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_DYN_PARAMS_HPP_
#define _ROBOTLIB_DYN_PARAMS_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace robotlib
{
	/**
	 * @brief DynParams class
	 * @details
	 * This class contains the methods used to get the robot dynamic parameters
	 */
	class DynParams
	{
	public:
		DynParams(const Eigen::Vector3d &com,
				  const double mass,
				  const Eigen::Matrix3d &inertia);

		~DynParams();

		/**
		 * @brief Get the center of mass of the robot
		 * @return const Eigen::Vector3d
		 */
		const Eigen::Vector3d &getCoM() const;
		/**
		 * @brief Get the mass of the robot
		 * @return const double
		 */
		double getMass() const;
		/**
		 * @brief Get the inertia of the robot
		 * @return const Eigen::Matrix3d
		 */
		const Eigen::Matrix3d &getInertia() const;

	private:
		const Eigen::Vector3d com_;
		const double mass_;
		const Eigen::Matrix3d inertia_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_DYN_PARAMS_HPP_