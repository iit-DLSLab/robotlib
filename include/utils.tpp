#include "utils.hpp"

namespace robotlib
{
	template <class Data>
	ContainerBase<Data>::ContainerBase(){};

	template <class Data>
	ContainerBase<Data>::~ContainerBase(){};

	template <class Data, unsigned int N>
	Container<Data, N>::Container(const std::array<Data, N> data) : data_(data){};

	template <class Data, unsigned int N>
	Container<Data, N>::~Container(){};

	template <class Data, unsigned int N>
	Iterator<const Data> Container<Data, N>::begin() const
	{
		return Iterator<const Data>(&data_[0]);
	};

	template <class Data, unsigned int N>
	Iterator<const Data> Container<Data, N>::end() const { return Iterator<const Data>(&data_[N]); };

	template <class Data, unsigned int N>
	const int Container<Data, N>::size() const { return data_.size(); };
} // namespace robotlib