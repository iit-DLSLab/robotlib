#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_

// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

// stdlib
#include <fstream>

#include <memory>

#include "limb_base.hpp"
#include "leg.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "trunk.hpp"

#include "utils.hpp"


namespace dls
{
namespace robot
{

// =============================================================================
// Class Interface
// =============================================================================

class RobotBase {
private:

    // LEG DATA MAP CLASS
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

    // LEG DATA MAP PAIR CLASS
    template<class Data>
    class LegDataMapPair{

    using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

    public:
        LegDataMapPair(RobotBase* robot) : nLegs_(robot->getNLEGS()){    
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

    // LINK DATA MAP PAIR CLASS
    template<class Data>
    class LinkDataMapPair{

    using PairType = std::pair<std::shared_ptr<Link>, Data>;

    public:
        LinkDataMapPair(RobotBase* robot) : nLinks_(robot->getNLINKS()){    
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

    // JOINT DATA MAP PAIR CLASS
    template<class Data>
    class JointDataMapPair{

    using PairType = std::pair<std::shared_ptr<Joint>, Data>;

    public:
        JointDataMapPair(RobotBase* robot) : nJoints_(robot->getNJOINTS()){    
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

public:

    virtual Iterator<const std::shared_ptr<LimbBase>> begin() {};
    virtual Iterator<const std::shared_ptr<LimbBase>> end() {}; 

    // Get functions
    virtual const int getNLEGS() = 0;
    virtual const int getNJOINTS() = 0;
    virtual const int getNLINKS() = 0;

    virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;

	// Plugin typedefs
	typedef std::shared_ptr<RobotBase> createRobot_t();
	typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

    
    // Create a leg data map
    template<class Data> LegDataMap<Data> makeLegDataMap(){return LegDataMap<Data>(this->getNLEGS());}

    // Create a joint data map
    template<class Data> LegDataMap<Data> makeJointDataMap(){return LegDataMap<Data>(this->getNJOINTS());}

    // Create a link data map
    template<class Data> LegDataMap<Data> makeLinkDataMap(){return LegDataMap<Data>(this->getNLINKS());}

    // Create a leg data map pair 
    template<class Data> LegDataMapPair<Data> makeLegDataMapPair(){return LegDataMapPair<Data>(this);}

    // Create a link data map pair 
    template<class Data> LinkDataMapPair<Data> makeLinkDataMapPair(){return LinkDataMapPair<Data>(this);}

    // Create a joint data map pair 
    template<class Data> JointDataMapPair<Data> makeJointDataMapPair(){return JointDataMapPair<Data>(this);}

};

} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_LIMB_HPP_
