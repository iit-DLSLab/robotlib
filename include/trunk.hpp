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

namespace dls
{
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
			Trunk(const std::string &name);

			~Trunk();

			/**
			 * @brief Get the name of the Trunk
			 * @return const std::string
			 */
			virtual const std::string getName() const override;
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_TRUNK_HPP_
