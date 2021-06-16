
#ifndef _ROBOTLIB_LEG_DATA_MAP_HPP_
#define _ROBOTLIB_LEG_DATA_MAP_HPP_

#include <stdexcept>
#include "utils.hpp"
#include <memory>

namespace dls {
namespace robot {

/**
 * A very simple container to associate a generic data item to each leg
 * (or anything related to a leg, e.g. a hip)
 */

template<class Data>
class LegDataMap{
public:
    LegDataMap(const int nLegs): nLegs_(nLegs){    
        Data *p = new Data[nLegs_];
        std::shared_ptr<Data> pshrd(p);
        data_ = pshrd;
    }
    ~LegDataMap(){};
    
    Iterator<Data> begin() { return Iterator<Data>(&data_.get()[0]); }
    Iterator<Data> end() { return Iterator<Data>(&data_.get()[nLegs_]); }
private:
    std::shared_ptr<Data> data_;
    
    const int nLegs_;
};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LEG_DATA_MAP_HPP_

