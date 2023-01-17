#include "utils/iterator.hpp"

namespace robotlib
{
	template <class Data>
	Iterator::Iterator(pointer ptr) : m_ptr(ptr) {}
	Iterator::reference operator*() const { 
		return *m_ptr;
	}
	Iterator::pointer operator->() {
		return m_ptr;
	}
	Iterator::Iterator &operator++(){
		m_ptr++;
		return *this;
	}
	Iterator::Iterator operator++(int){
		Iterator tmp = *this;
		++(*this);
		return tmp;
	}
	Iterator::Iterator &operator--(){
		m_ptr--;
		return *this;
	}

	Iterator::Iterator operator--(int){
		Iterator tmp = *this;
		--(*this);
		return tmp;
	}

	friend bool Iterator::operator==(const Iterator &a, const Iterator &b) {
		return a.m_ptr == b.m_ptr;
	}
	friend bool Iterator::operator!=(const Iterator &a, const Iterator &b) {
		return a.m_ptr != b.m_ptr;
	}

} // namespace robotlib




