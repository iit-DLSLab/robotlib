/*!
 * @file dyn_params.hpp
 *
 * @brief DynParams class definition and functions prototypes.
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

#ifndef _ROBOTLIB_DYN_PARAMS_HPP_
#define _ROBOTLIB_DYN_PARAMS_HPP_

#include <Eigen/Dense>
#include <iostream>

namespace robotlib
{
	/*!
	 * @brief DynParams class.
	 * @details
	 * This class stores the com, mass and inertia as dynamic parameters.
	 */
	class DynParams
	{
	public:
		/*!
         * @brief Constructor.
         * @param[in] com center of mass.
         * @param[in] mass mass.
         * @param[in] inertia inertia.
         */
		DynParams(const Eigen::Vector3d &com,
				  const double mass,
				  const Eigen::Matrix3d &inertia);

	    /*!
         * @brief Destructor.
         */
		virtual ~DynParams();

		/*!
		 * @brief Get the center of mass of the object to which those dynamic parameters are associated to.
		 * @return center of mass.
		 */
		virtual Eigen::Vector3d getCoM() const;

		/*!
		 * @brief Get the mass of the object to which those dynamic parameters are associated to.
		 * @return mass.
		 */
		virtual double getMass() const;

		/*!
		 * @brief Get the inertia of mass of the object to which those dynamic parameters are associated to.
		 * @return inertia.
		 */
		virtual Eigen::Matrix3d getInertia() const;

	private:
		//! Center of mass.
		const Eigen::Vector3d com_;
		//! Mass.
		const double mass_;
		//! Inertia.
		const Eigen::Matrix3d inertia_;
	};
} // namespace robotlib

#endif // _ROBOTLIB_DYN_PARAMS_HPP_