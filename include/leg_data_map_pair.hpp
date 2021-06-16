
#ifndef _ROBOTLIB_LEG_DATA_MAP_PAIR_HPP_
#define _ROBOTLIB_LEG_DATA_MAP_PAIR_HPP_

#include <stdexcept>
#include "utils.hpp"
#include <memory>
#include "robot_base.hpp"

namespace dls {
namespace robot {

 
template<class Data>
class LegDataMapPair{

using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

public:
    LegDataMapPair(std::shared_ptr<RobotBase> robot) : nLegs_(robot->getNLEGS()){    
        for (int i = 0; i<nLegs_ ; ++i){
            PairType pair(robot->getLeg(i),Data());
            data_.push_back(pair);
        }
     }
    
    Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]);}
    Iterator<PairType> end() { return Iterator<PairType>(&data_[nLegs_]);}

     ~LegDataMapPair(){};
    
private:
    const int nLegs_;
    std::vector<PairType> data_;

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LEG_DATA_MAP_PAIR_HPP_

