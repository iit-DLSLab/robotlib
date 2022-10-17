/**
 * @file trunk.hpp
 *
 * @brief Trunk class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "link.hpp"
#include "utils.hpp"
#include "joint.hpp"
#include "dyn_params.hpp"

namespace robotlib
{
	/**
	 * @brief Trunk class
	 * @details
	 * This class represents the robot trunk and extends the class Link
	 */
	class Trunk : public Link
	{
	public:
		//TODO: it makes sense only if the dynamic_parameter variable is not constant, so understand if dynamic_parameters can be non constant
		Trunk(const std::string &name);

		Trunk(const std::string &name, const DynParams &dynamic_parameters);

		~Trunk();

		/**
		 * @brief Get the name of the Trunk
		 * @return const std::string
		 */
		virtual const std::string getName() const override;

		/**
		 * @brief Get the center of mass of the trunk
		 * @return const Eigen::Vector3d
		 */
		const Eigen::Vector3d &getCoM() const;

		/**
		 * @brief Get the mass of the trunk
		 * @return const double
		 */
		double getMass() const;

		/**
		 * @brief Get the inertia of the trunk
		 * @return const Eigen::Matrix3d
		 */
		const Eigen::Matrix3d &getInertia() const;

		/**
		 * @brief Get the dynamic parameters of the trunk
		 * @return const DynParams
		 */
		const DynParams &getDynParams() const;

	private:

		/**
		 * @brief Dynamic parameters of the Trunk
		 */
		DynParams dynamic_parameters_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_TRUNK_HPP_