/*!
 * @file trunk.hpp
 *
 * @brief Trunk class definition and functions prototypes.
 *
 * @authors Authors in alphabetical order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Geoff Fink (IIT DLS Lab) - Contact: geoff.fink@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_TRUNK_HPP_
#define _ROBOTLIB_TRUNK_HPP_

#include "link.hpp"
#include "joint.hpp"
#include "dyn_params.hpp"

namespace robotlib
{
	/*!
	 * @brief Trunk class.
	 * @details
	 * This class represents the robot trunk and extends the class Link.
	 */
	class Trunk : public Link
	{
	public:
		/*!
         * @brief Constructor.
         * @param[in] name name of the trunk.
         */
		Trunk(const std::string &name);

		/*!
         * @brief Constructor.
         * @param[in] name name of the trunk.
		 * @param[in] dynamic_parameters dynamic parameters of the trunk.
         */
		Trunk(const std::string &name, const DynParams &dynamic_parameters);

		/*!
         * @brief Destructor.
         */
		virtual ~Trunk();

        /*!
		 * @brief Get the name of the trunk.
		 * @return name of the trunk.
		 */
		virtual const std::string& getName() const override;

		/*!
		 * @brief Get the center of mass of the trunk.
		 * @return trunk's center of mass.
		 */
		virtual const Eigen::Vector3d& getCoM() const;

		/*!
		 * @brief Get the mass of the trunk.
		 * @return trunk's mass.
		 */
		double getMass() const;

		/*!
		 * @brief Get the inertia of the trunk.
		 * @return trunk's inertia.
		 */
		virtual const Eigen::Matrix3d& getInertia() const;

		/*!
		 * @brief Get the dynamic parameters of the trunk.
		 * @return trunk's dynamic parameters.
		 */
		virtual const DynParams& getDynParams() const;

	private:
		//!Dynamic parameters of the Trunk
		const DynParams dynamic_parameters_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_TRUNK_HPP_