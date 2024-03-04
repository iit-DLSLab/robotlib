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

namespace robotlib
{
	Link::Link(const std::string& name) 
		: Frame(name)
	{}

	Link::Link(const std::string& name, Joint& parent) 
		: Frame(name)
	{
		this->setParent(parent);
	}

	/// TODO:
	// 0 children - restituisci messaggio + nullptr + gestisci il segfault
	// 1 children - restituisci children con 1 elemento (quando si fa setChild bisogna anche aggiornare i children)
	//				bisogna poi rimuovere la funzione setChildrenForTrunk (deve essere quindi generica per tutti i link)
	// segnalare i possibili segmentation fault da gestire

	std::shared_ptr<const Joint> Link::getParent() const 
	{ 
		return parent_; 
	}

	const std::vector<std::shared_ptr<Joint>>& Link::getChildren() const 
	{ 
		return children_; 
	}
	
	void Link::setParent(Joint& parent) 
	{ 
		parent_ = std::shared_ptr<Joint>(&parent); 
		parent_->setChild(*this);
	}

	void Link::addChild(Joint& child) 
	{ 
		children_.push_back(std::shared_ptr<Joint>(&child)); 
	}
} // namespace robotlib