/*!
 * @file frame.hpp
 *
 * @brief Frame class definition and functions prototypes.
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

#ifndef _ROBOTLIB_FRAME_HPP_
#define _ROBOTLIB_FRAME_HPP_

#include <string>

namespace robotlib
{
    /*!
	 * @brief Frame class
	 * @details
	 * This class represents a generic robot frame (e.g. a joint or a link) and is extended by Joint and Link classes.
	 */
    class Frame
    {
    public:
        /*!
         * @brief Constructor.
         * @param[in] name name of the frame.
         */
        Frame(const std::string &name) : name_(name){};

        /*!
         * @brief Destructor.
         */
        virtual ~Frame(){};

        /*!
		 * @brief Get the name of the Frame.
		 * @return name of the frame.
		 */
        virtual const std::string& getName() const = 0;

    protected:
        //! Name of the frame.
        const std::string name_{};
    };
} // namespace robotlib

#endif // _ROBOTLIB_FRAME_HPP_