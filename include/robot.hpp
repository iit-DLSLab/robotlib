/*******************************************************************************
*                                                       ,----,                 *
*                                                     .'   .' \                *
*                                                   ,----,'    |               *
*               ________  ___       ________        |    :  .  ;               *
*              |\   ___ \|\  \     |\   ____\       ;    |.'  /                *
*              \ \  \_|\ \ \  \    \ \  \___|_      `----'/  ;                 *
*               \ \  \ \\ \ \  \    \ \_____  \       /  ;  /                  *
*                \ \  \_\\ \ \  \____\|____|\  \     ;  /  /-,                 *
*                 \ \_______\ \_______\____\_\  \   /  /  /.`|                 *
*                  \|_______|\|_______|\_________\./__;      :                 *
*                                     \|_________||   :    .'                  *
*                                                 ;   | .'                     *
*                                                 `---'                        *
********************************************************************************
* Author:            Marco Marchitto                                           *
* Maintainer:        Marco Marchitto                                           *
* author email:      marco.marchitto@iit.it                                    *
*******************************************************************************/
#ifndef ROBOTLIB_ROBOT_HPP
#define ROBOTLIB_ROBOT_HPP

// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

// stdlib
#include <fstream>

#include <memory>
#include "limb_base.hpp"
#include "leg_base.hpp"
#include "arm_base.hpp"
#include "trunk.hpp"
#include "leg.hpp"
#include "joint.hpp"
#include "leg_data_map_base.hpp"
#include "robot_base.hpp"

//#include "forward_kinematics.hpp"

namespace dls
{
namespace robot
{

// =============================================================================
// Class Interface
// =============================================================================

template<unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS=0>
class Robot : public RobotBase {
public:
	// Constructor
	Robot(const std::array<std::shared_ptr<LimbBase>, NLEGS>& legs): legs_(legs){}; 
    using it = typename std::array<std::shared_ptr<LimbBase>,NLEGS>::iterator;
	
    virtual Iterator<const std::shared_ptr<LimbBase>> begin() override { return Iterator<const std::shared_ptr<LimbBase>>(&legs_[0]); };
    virtual Iterator<const std::shared_ptr<LimbBase>> end() override { return Iterator<const std::shared_ptr<LimbBase>>(&legs_[NLEGS]); };
    
    //it begin()  { return it(&legs_[0]); };
    //it end()  { return it(&legs_[NLEGS]); };

	virtual ~Robot() = default;
 
	// template <class Data>
    // class LegDataMap : public LegDataMapBase<Data>{    TO DO
    // public:
    //     virtual Iterator<Data> begin() override { return Iterator<Data>(&legData_[0]); }
    //     virtual Iterator<Data> end() override { return Iterator<Data>(&legData_[NLEGS]); }
    
    // private: 
    //     std::array<Data,NLEGS> legData_;
    // };

    template <class Data>
    class LegDataMap : public std::array<Data, NLEGS> { };
    
    template <class Data>
    class LinkDataMap : public std::array<Data, NLINKS_TOT> { };

    template <class Data>
    class JointDataMap : public std::array<Data, NJOINTS_TOT> { };

    template <class Data>
    class LegDataMapPair : public std::array<std::pair<std::shared_ptr<LimbBase>, std::shared_ptr<Data>>, NLEGS> {
    public:
        LegDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++){
                this->data()[i] = std::make_pair(robot.legs_[i], std::make_shared<Data>(legData[i]));
            }
        }
    private:
        Data legData[NLEGS];
    };

	template <class Data>
    class LinkDataMapPair : public std::array<std::pair<std::shared_ptr<Link>, std::shared_ptr<Data>>, NLINKS_TOT> {
    public:
        LinkDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++) {
                auto leg = robot.legs_[i];
				int nLinks = leg->getNumLinks();
				for (int j=0;j<nLinks;j++) {
                    auto link = std::static_pointer_cast<Link>(leg->getLink(j)); 
                    this->data()[i*nLinks+j] = std::make_pair(link, std::make_shared<Data>(linkData[i*nLinks+j]));
                }
            }
        }
    private:
        Data linkData[NLINKS_TOT];
    };


    template <class Data>
    class JointDataMapPair : public std::array<std::pair<std::shared_ptr<Joint>, std::shared_ptr<Data>>,NJOINTS_TOT> {
    public:
        JointDataMapPair(Robot &robot) {
            for (int i=0;i<NLEGS;i++) {
                auto leg = robot.legs_[i];
				int nJoints = leg->getNumJoints();
                for (int j=0;j<nJoints;j++) {
                    auto joint = std::static_pointer_cast<Joint>(leg->getJoint(j));
                    this->data()[i*nJoints+j] = std::make_pair(joint,std::make_shared<Data>(jointData[i*nJoints+j]));
                }
            }
        }
    private:
        Data jointData[NJOINTS_TOT];
    }; 

    // Function to create data map objects TO DO or TO REMOVE
    template< class T> LegDataMapBase<T> legDataMap() {return LegDataMap<T>();};


	//*************************************************************************
	// Maybe to be added to robotFactory
	/** Read function
	* @brief: Function for reading (urdf) files
	* @param filename: name of the file to be read
	*/
	static std::string ReadFile(const char *filename);
	/** Check function
	* @brief: Check if the urdf file was correctly parsed
	* @param robot_description: output of the readFile function
	*/
	static urdf::Model init_robot_description(std::string robot_description);
	//*************************************************************************

	// Get functions
	const std::array<std::shared_ptr<LimbBase>, NLEGS> getLegs(){return legs_;};
	const std::shared_ptr<LimbBase> getLeg(const int id){return legs_[id];};

    virtual int getNLEGS() override {return legs_.size();};

	// get leg with LEGID TO DO

protected:
	const std::array<std::shared_ptr<LimbBase>, NLEGS> legs_;				//! Legs of the robot
	//const Trunk trunk_;													//! Trunk of the robot

};
} // namespace dog
} // namespace robot

#endif
