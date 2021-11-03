/**
 * @file frame.hpp
 *
 * @brief Frame class definition and functions prototypes
 *
 * @author Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 * @author Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#ifndef _ROBOTLIB_FRAME_HPP_
#define _ROBOTLIB_FRAME_HPP_

#include <Eigen/Dense>

namespace dls
{
    namespace robotlib
    {
        /**
		* @brief Frame class
		* @details
		* This class represents a generic robot frame (joint/link) and is extended by Joint and Link classes
		*/
        class Frame
        {
        public:
            Frame(const std::string &name) : name_(name){};
            //Frame(const Eigen::Vector3d &p, const Eigen::Vector3d &ori)
            virtual ~Frame(){};

            /**
			 * @brief Get the name of the Frame
			 * @return const std::string
			 */
            virtual const std::string getName() const = 0;

        protected:
            const std::string name_{};
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_FRAME_HPP_