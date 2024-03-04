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

#ifndef _ROBOTLIB_FRAME_CPP_
#define _ROBOTLIB_FRAME_CPP_

#include "frame.hpp"

namespace robotlib
{
    Frame::Frame(const std::string &name) 
        : name_(name){};


    const std::string& Frame::getName() const 
    { 
        return name_;
    }


    bool Frame::operator==(const Frame& rhs) const
	{
		return this->getName().compare(rhs.getName());
	}
    
} // namespace robotlib

#endif // _ROBOTLIB_FRAME_CPP_