#ifndef _ROBOTLIB_EIGEN_HPP_
#define _ROBOTLIB_EIGEN_HPP_

#include <map>
#include <Eigen/Dense>

namespace robotlib
{
namespace eigen
    {
        template <typename Key, typename T>
        using aligned_map = std::map<Key, T, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, T>>>;
    } // namespace eigen
} // namespace robotlib

#endif // _ROBOTLIB_EIGEN_HPP_