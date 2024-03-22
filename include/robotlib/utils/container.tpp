#include "container.hpp"

namespace robotlib
{
	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(){}

	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(std::array<std::shared_ptr<Data>, Length> data)
		: data_(data)
	{}

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::begin() const { return &data_[0]; }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::end() const { std::cout << "#### ITERATOR END" << std::endl; return &data_[Length]; }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::begin() { return &data_[0]; }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::end() { std::cout << "#### ITERATOR END CONST" << std::endl; return &data_[Length]; }

	template <class Data, unsigned int Length>
	unsigned int Container<Data, Length>::length() const { return Length; }

	template <class Data, unsigned int Length>
	Data& Container<Data, Length>::operator[](unsigned int idx) { return *(data_[idx]); };

	template <class Data, unsigned int Length>
	std::shared_ptr<Data>& Container<Data, Length>::at(unsigned int idx) { return data_[idx]; };

} // namespace robotlib