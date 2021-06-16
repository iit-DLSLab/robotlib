
#ifndef _ROBOTLIB_LINK_DATA_MAP_PAIR_HPP_
#define _ROBOTLIB_LINK_DATA_MAP_PAIR_HPP_

#include <stdexcept>
#include "utils.hpp"
#include <memory>
#include "robot_base.hpp"

namespace dls {
namespace robot {

 
template<class Data>
class LinkDataMapPair{

using PairType = std::pair<std::shared_ptr<Link>, Data>;

public:
    LinkDataMapPair(std::shared_ptr<RobotBase> robot) : nLinks_(robot->getNLINKS()){    
        const int nLegs = robot->getNLEGS();
        for (int i=0;i<nLegs;i++) {
            auto leg = robot->getLeg(i);
            int nLinks = leg->getNumLinks();
            for (int j=0;j<nLinks;j++) {
                auto link = std::static_pointer_cast<Link>(leg->getLink(j));
                PairType pair (link,Data());
                data_.push_back(pair);
            }
        }
     }
    
    Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]);}
    Iterator<PairType> end() { return Iterator<PairType>(&data_[nLinks_]);}

     ~LinkDataMapPair(){};
    
private:
    const int nLinks_;
    std::vector<PairType> data_;

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LINK_DATA_MAP_PAIR_HPP_

