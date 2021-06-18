// =============================================================================
// Includes
// =============================================================================
#include <memory>
#include "robot.hpp"

// =============================================================================
// Using Declarations
// =============================================================================
using namespace dls::robot;

// =============================================================================
// Constructors
// =============================================================================
// Robot::Robot(	const std::string& 								name,
// 				const std::array<std::shared_ptr<Leg>, NLEGS>   legs) :
// 				name(name),
// 				legs(legs)
// 				{};

// std::string Robot::readFile(const char *filename){

// 	//std::cout << "Reading " << filename << std::endl;

// 	//std::ifstream in(filename, std::ios::in | std::ios::binary);
// 	std::ifstream in(filename, std::ifstream::in);

// 	if (in) {
// 		std::string contents;
// 		in.seekg(0,std::ios::end);
// 		contents.resize(in.tellg());
// 		in.seekg(0,std::ios::beg);
// 		in.read(&contents[0],contents.size());
// 		in.close();
// 		return contents;
// 	}

// 	throw(errno);
// }

// urdf::Model Robot::init_robot_description(std::string robot_description)
// {
// 	urdf::Model robot_model;
// 	if (!robot_model.initString(robot_description))
// 	{
// 		throw std::runtime_error("Failed to parse urdf file");
// 	}
// 	return robot_model;
// }