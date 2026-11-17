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
#include <memory>

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
        Frame(const std::string &name);

        /*!
         * @brief Destructor.
         */
        virtual ~Frame() = default;

        /*!
		 * @brief Get the name of the Frame.
		 * @return name of the frame.
		 */
        const std::string& getName() const;

        /*!
		 * @brief Equal comparator for Frame.
		 * @return true if the frames are equal.
		 */
		bool operator==(const Frame& rhs) const;

    protected:

        //! Name of the frame.
        std::string name_;
    };

    typedef std::shared_ptr<Frame> FramePtr;
} // namespace robotlib

#endif // _ROBOTLIB_FRAME_HPP_