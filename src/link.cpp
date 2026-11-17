/**
 * @file link.cpp
 *
 * @brief Link class and functions implementation
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

#include "link.hpp"

#include <iostream>

namespace robotlib
{
	Link::Link(const std::string& name) 
		: Frame(name)
		, sub_id(-1)
		, id(-1)
		, limb_id(-1)
	{}

	Link::Link() 
		: Frame("")
		, sub_id(-1)
		, id(-1)
		, limb_id(-1)
	{}

	bool Link::isAttached() const
	{
		if (sub_id == -1 || id == -1 || limb_id == -1)
			return false;
		else
			return true;
	}

	bool Link::operator==(const Link& rhs) const
	{
		if (this->getName() != rhs.getName())
			return false;
		return true;
	}

	Link &Link::operator=(const Link &rhs)
	{
		if (this != &rhs) // self-assignment check
		{
			this->name_ = rhs.name_;
		}
		return *this;
	}
} // namespace robotlib