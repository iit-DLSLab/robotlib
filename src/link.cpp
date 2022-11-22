/**
 * @file link.cpp
 *
 * @brief Link class and functions implementation
 *
 * @authors Authors in alphabetic order:
 *
 *     Gianluca Cerilli (IIT DLS Lab) - Contact: gianluca.cerilli@iit.it
 *
 *     Marco Marchitto (IIT DLS Lab) - Contact: marco.marchitto@iit.it
 *
 * @bug No known bugs.
 */

#include "link.hpp"

namespace robotlib
{
	Link::Link(const std::string &name) : Frame(name){}

	Link::~Link(){}

	/// TODO:
	// 0 children - restituisci messaggio + nullptr + gestisci il segfault
	// 1 children - restituisci children con 1 elemento (quando si fa setChild bisogna anche aggiornare i children)
	//				bisogna poi rimuovere la funzione setChildrenForTrunk (deve essere quindi generica per tutti i link)
	// segnalare i possibili segmentation fault da gestire

	std::string Link::getName() const { return name_; }
	std::shared_ptr<Joint> Link::getParent() const { return parent_; }
	std::shared_ptr<Joint> Link::getChild() const
	{
		if(child_ != nullptr)
			return child_;
		else
			return nullptr;
	}
	std::shared_ptr<const ContainerBase<std::shared_ptr<Joint>>> Link::getChildren() const
	{
		if((children_ != nullptr) && (children_->size() > 0))
			return children_;
		else
			throw std::runtime_error ("RUNTIME ERROR: The link has no children");
	}

	void Link::setParent(const std::shared_ptr<Joint> parent) { parent_ = parent; }
	void Link::setChild(const std::shared_ptr<Joint> child) { child_ = child; }
	void Link::setChildren(const std::shared_ptr<ContainerBase<std::shared_ptr<Joint>>> children) { children_ = children; }
} // namespace robotlib