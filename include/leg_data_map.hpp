
#ifndef _ROBOTLIB_LEG_DATA_MAP_HPP_
#define _ROBOTLIB_LEG_DATA_MAP_HPP_

#include <stdexcept>
#include <utils.hpp>

namespace dls {
namespace robot {

/**
 * A very simple container to associate a generic data item to each leg
 * (or anything related to a leg, e.g. a hip)
 */

template<class T>
class LegDataMap{
public:
    LegDataMap(const int nLegs): nLegs_(nLegs){    
        T *p = new T[nLegs_];
        std::shared_ptr<T> pshrd(p);
        data_ = pshrd;
    }
    ~LegDataMap(){};
    
    Iterator<T> begin() { return Iterator<T>(&data_.get()[0]); }
    Iterator<T> end() { return Iterator<T>(&data_.get()[nLegs_]); }
private:
    std::shared_ptr<T> data_;
    
    const int nLegs_;
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LEG_DATA_MAP_HPP_

