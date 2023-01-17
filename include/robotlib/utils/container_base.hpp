#ifndef _CONTAINER_BASE_HPP_
#define _CONTAINER_BASE_HPP_

#include "utils/iterator.hpp"
#include <memory>
#include <Eigen/Dense>

namespace robotlib
{
    template <class Data>
    class ContainerBase
    {
    public:
        ContainerBase();
        virtual ~ContainerBase();

        virtual Iterator<const Data> begin() const = 0;
        virtual Iterator<const Data> end() const = 0;

        //TODO: define [] operator using a string as input (it implies that all the possible Data classes have a getName function)

        virtual int size() const = 0;
    };
} // namespace robotlib

#include "utils/container_base.tpp"

#endif // _CONTAINER_BASE_HPP_