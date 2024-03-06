#include "container.hpp"

namespace robotlib
{
	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(){}

	template <class Data, unsigned int Length>
	Container<Data, Length>::Container(std::array<std::shared_ptr<Data>, Length> data) : data_(data){}

	template <class Data, unsigned int Length>
	Iterator<const Data> Container<Data, Length>::begin() const { return Iterator<const Data>(data_[0]); }

	template <class Data, unsigned int Length>
	Iterator<const Data> Container<Data, Length>::end() const { return Iterator<const Data>(data_[Length]); }

	template <class Data, unsigned int Length>
	Iterator<Data> Container<Data, Length>::begin() { return Iterator<Data>(data_[0]); }

	template <class Data, unsigned int Length>
	Iterator<Data> Container<Data, Length>::end() { return Iterator<Data>(data_[Length]); }

	template <class Data, unsigned int Length>
	unsigned int Container<Data, Length>::length() const { return Length; }

	template <class Data, unsigned int Length>
	Data& Container<Data, Length>::operator[](unsigned int idx) { return *(data_[idx]); };

	// template <class Data, unsigned int Length>
	// Data& Container<Data, Length>::operator[](unsigned int idx) const { return data_[idx]; };

} // namespace robotlib