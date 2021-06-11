
#ifndef _ROBOTLIB_BASE_LEG_DATA_MAP_BASE_HPP
#define _ROBOTLIB_BASE_LEG_DATA_MAP_BASE_HPP

#include <stdexcept>
#include <utils.hpp>

namespace dls {
namespace robot {

/**
 * A very simple container to associate a generic data item to each leg
 * (or anything related to a leg, e.g. a hip)
 */

template<class T>
class LegDataMapBase
{
public:
    virtual Iterator<T> begin () {};
    virtual Iterator<T> end () {};
};
} // namespace robot
} // namespace dls

#endif

