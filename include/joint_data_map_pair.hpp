
#ifndef _ROBOTLIB_JOINT_DATA_MAP_PAIR_HPP_
#define _ROBOTLIB_JOINT_DATA_MAP_PAIR_HPP_

#include <stdexcept>
#include "utils.hpp"
#include <memory>
#include "robot_base.hpp"

namespace dls {
namespace robot {

 
template<class Data>
class JointDataMapPair{

using PairType = std::pair<std::shared_ptr<Joint>, Data>;

public:
    JointDataMapPair(std::shared_ptr<RobotBase> robot) : nJoints_(robot->getNJOINTS()){    
        const int nLegs = robot->getNLEGS();
        for (int i=0;i<nLegs;i++) {
            auto leg = robot->getLeg(i);
            int nJoints = leg->getNumJoints();
            for (int j=0;j<nJoints;j++) {
                auto joint = std::static_pointer_cast<Joint>(leg->getJoint(j));
                PairType pair (joint,Data());
                data_.push_back(pair);
            }
        }
     }
    
    Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]);}
    Iterator<PairType> end() { return Iterator<PairType>(&data_[nJoints_]);}

     ~JointDataMapPair(){};
    
private:
    const int nJoints_;
    std::vector<PairType> data_;

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_JOINT_DATA_MAP_PAIR_HPP_

