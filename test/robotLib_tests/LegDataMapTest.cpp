#include "robotlib/robot.hpp"
#include "robotlib/leg_data_map.hpp"
#include <gtest/gtest.h>

using namespace std;
using namespace dls;
using namespace dls::robot;

enum LegID{LF=0, RF, LH, RH};

#define NLEGS 6
TEST(robotLib, legDataMap){
    
    Eigen::Vector3d fPos;
    fPos.setZero();

    //LegDataMap<NLEGS, Eigen::Vector3d> footPos(fPos);
    //std:: cout << footPos[LegID::LF].transpose() << '\n';

}