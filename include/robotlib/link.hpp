/*!
 * @file link.hpp
 *
 * @brief Link class definition and functions prototypes.
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

#ifndef _ROBOTLIB_LINK_HPP_
#define _ROBOTLIB_LINK_HPP_

#include "frame.hpp"
#include "joint.hpp"

#include <memory>

namespace robotlib
{
	class Joint;
	
	/*!
	 * @brief Link class.
	 * @details
	 * This class represents a generic robot link and extends the abstract class Frame.
	 */
	class Link : public Frame
	{
		friend class Robot;
		friend class Joint;
		
	public:

		/*!
         * @brief Constructor.
         * @param[in] name name of the link.
         */
		Link(const std::string &name);

		/*!
         * @brief Constructor.
         * @param[in] name name of the link.
		 * @param[in] parent parent joint.
         */
		// Link(const std::string &name, Joint* parent);

		Link();


		/*!
         * @brief Destructor.
         */
		~Link() = default;
		
		//! Robot is a friend class to let it use the private methods of the Link class.
		friend class Robot;

		// operator==
		bool operator==(const Link& rhs) const;
		
		Link &operator=(const Link &rhs);

		bool isAttached() const;

		int sub_id;
		int id;

	};
	typedef std::shared_ptr<Link> LinkPtr;
} // namespace robotlib

#endif // _ROBOTLIB_LINK_HPP_