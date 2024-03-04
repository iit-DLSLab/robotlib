#include "container_base.hpp"

namespace robotlib
{
	template <class Data>
	Iterator<const Data> ContainerBase<Data>::begin() const
	{
		throw std::range_error("You need to implement begin() from ContainerBase");
	}
	
	template <class Data>
	Iterator<const Data> ContainerBase<Data>::end() const
	{
		throw std::range_error("You need to implement end() from ContainerBase");
	}

	template <class Data>
	Iterator<Data> ContainerBase<Data>::begin()
	{
		throw std::range_error("You need to implement unconst begin() from ContainerBase");
	}

    template <class Data>
	Iterator <Data> ContainerBase<Data>::end()
	{
		throw std::range_error("You need to implement unconst end() from ContainerBase");
	}

	template <class Data>
	unsigned int ContainerBase<Data>::length() const
	{
		throw std::range_error("You need to implement length() from ContainerBase");
	}

	template <class Data>
	Data& ContainerBase<Data>::operator[](unsigned int idx) 
	{ 
		throw std::range_error("You need to implement operator[] from ContainerBase");
	};

	// template <class Data>
	// Data& ContainerBase<Data>::operator[](unsigned int idx) const
	// { 
	// 	throw std::range_error("You need to implement operator[] from ContainerBase");
	// };
	
} // namespace robotlib