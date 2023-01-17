#ifndef _CONTAINER_HPP_
#define _CONTAINER_HPP_

#include "utils/container_base.hpp"
#include <memory>
#include <Eigen/Dense>

namespace robotlib
{
    template <class Data, unsigned int N>
    class Container : public ContainerBase<Data>
    {
    public:
        Container(const std::array<Data, N> data);

        virtual ~Container();

        virtual Iterator<const Data> begin() const override;
        virtual Iterator<const Data> end() const override;

        virtual int size() const override;

    protected:
        const std::array<Data, N> data_;
    };
} // namespace robotlib

#include "utils/container.tpp"

#endif // _CONTAINER_HPP_