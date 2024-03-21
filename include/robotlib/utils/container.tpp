#include "container.hpp"

namespace robotlib
{
	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(){}

	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(std::array<std::shared_ptr<Data>, Length> data)
	{
		for(auto i{0}; i < Length; i++)
		{
			data_[i] = data[i];
		}

	}

	// template <class Data, unsigned int Length>
	// Container<Data, Length>::Container(std::array<Data, Length>& data)
	// {
	// 	for(auto i{0}; i < Length; i++)
	// 	{
	// 		data_[i] = &data[i];
	// 	}

	// }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::begin() const { return IteratorContainer<Data>(&data_[0]); }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::end() const { return IteratorContainer<Data>(&data_[Length]); }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::begin() { return IteratorContainer<Data>(&data_[0]); }

	template <class Data, unsigned int Length>
	IteratorContainer<Data> Container<Data, Length>::end() { return IteratorContainer<Data>(&data_[Length]); }

	template <class Data, unsigned int Length>
	unsigned int Container<Data, Length>::length() const { return Length; }

	template <class Data, unsigned int Length>
	Data& Container<Data, Length>::operator[](unsigned int idx) { return *(data_[idx]); };

	template <class Data, unsigned int Length>
	std::shared_ptr<Data>& Container<Data, Length>::at(unsigned int idx) { return data_[idx]; };

} // namespace robotlib