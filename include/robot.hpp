#ifndef _ROBOTLIB_ROBOT_HPP_
#define _ROBOTLIB_ROBOT_HPP_

// =============================================================================
// Includes
// =============================================================================
// ros
#include <urdf/model.h>

// stdlib
#include <fstream>

#include "robot_base.hpp"

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
    
    virtual Iterator<const std::shared_ptr<LimbBase>> begin() override { return Iterator<const std::shared_ptr<LimbBase>>(&legs_[0]); };
    virtual Iterator<const std::shared_ptr<LimbBase>> end() override { return Iterator<const std::shared_ptr<LimbBase>>(&legs_[NLEGS]); };
    
	virtual ~Robot() = default;

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
	const std::shared_ptr<LimbBase> getLeg(const int id) override {return legs_[id];};

    virtual const int getNLEGS() override {return NLEGS;};
    virtual const int getNJOINTS() override {return NJOINTS_TOT;};
    virtual const int getNLINKS() override {return NLINKS_TOT;};

protected:
	const std::array<std::shared_ptr<LimbBase>, NLEGS> legs_;				//! Legs of the robot
	//const Trunk trunk_;													//! Trunk of the robot

};
} // namespace robot
} // namespace dls

#endif // _ROBOTLIB_ROBOT_HPP_
