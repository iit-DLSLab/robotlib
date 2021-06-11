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
* Author:            Legacy Code                                               *
* Maintainer:        Hendrik de Bruin                                          *
* Maintainer email:  hendrik.debruin@iit.it                                    *
*******************************************************************************/
#ifndef ROBOT_FACTORY_CPP_PDOYFAZX
#define ROBOT_FACTORY_CPP_PDOYFAZX
// =============================================================================
// Includes
// =============================================================================
// stdlib
#include <stdexcept>
#include <fstream>
#include <string>
#include <cerrno>

// current class
#include "robot_factory.hpp"

// =============================================================================
// Using declarations
// =============================================================================
using dls::dog::Dog;
using dls::dog::RobotFactory;

// =============================================================================
// Prototypes
// =============================================================================
//urdf::Model          init_robot_description(std::string robot_description);
//std::string readFile(const char *filename);
std::shared_ptr<Dog> buildHyQ();
std::shared_ptr<Dog> buildHyQReal();

// =============================================================================
// Implementiaton
// =============================================================================
std::shared_ptr<Dog> RobotFactory::buildRobot(RobotType robot)
{
	switch(robot)
	{
		case RobotType::HyQ:
		{
			return buildHyQ();
			break;
		}
		case RobotType::HyQReal:
		{
			return buildHyQReal();
			break;
		}
		default:
		{
			throw std::domain_error("Robot type not recognised");
		}
	}
}

// std::string readFile(const char *filename){
	
// 	std::cout << "Reading " << filename << std::endl;

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

// urdf::Model init_robot_description(std::string robot_description)
// {
// 	urdf::Model robot_model;
// 	if (!robot_model.initString(robot_description))
// 	{
// 		throw std::runtime_error("Failed to parse urdf file");
// 	}
// 	return robot_model;
// }

std::shared_ptr<Dog> buildHyQ()
{
	// Load the hyq library
	void* dog = dlopen("../src/doglib/libhyqlib.so", RTLD_LAZY);
	if (!dog) {
		std::cerr << "Cannot load library: " << dlerror() << '\n';
	}
	//else
		//std::cout <<"Shared library is correclty loaded\n";

	// Load the create symbol
	dls::dog::Dog::createDog_t* create_hyq = (dls::dog::Dog::createDog_t*) dlsym(dog, "createDog_t");
	
	const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    }
	//else
		//std::cout <<"Create symbol is correclty loaded\n";

	// // Load the destroy symbol
    // destroyDog_t* destroy_dog = (destroyDog_t*) dlsym(triangle, "destroyDog_t");
    // dlsym_error = dlerror();
    // if (dlsym_error) {
    //     cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    // }
	
	// Create an instance of the class
	return create_hyq();
}

std::shared_ptr<Dog> buildHyQReal()
{	
	// Load the hyqreal library
	void* dog = dlopen("../src/doglib/libhyqReallib.so",RTLD_LAZY);
	if (!dog) {
		std::cerr << "Cannot load library: " << dlerror() << '\n';
	}
	//else
		//std::cout <<"Shared library is correclty loaded\n";
	// Load the create symbol
	dls::dog::Dog::createDog_t* create_hyqreal = (dls::dog::Dog::createDog_t*) dlsym(dog, "createDog_t");

	const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    }
	//else
		//std::cout <<"Create symbol is correclty loaded\n";

	// // Load the destroy symbol
    // destroyDog_t* destroy_dog = (destroyDog_t*) dlsym(triangle, "destroyDog_t");
    // dlsym_error = dlerror();
    // if (dlsym_error) {
    //     cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    // }

	// Create an instance of the class
	return create_hyqreal ();
}

std::shared_ptr<dls::dog::JSSweeper> RobotFactory::buildHyQJSSweeper(double pstep)
{
	// Load the hyqreal library
	void* jss = dlopen("../src/doglib/libhyqlib.so",RTLD_LAZY);
	if (!jss) {
		std::cerr << "Cannot load library: " << dlerror() << '\n';
	}
	//else
		//std::cout <<"Shared library is correclty loaded\n";
	// Load the create symbol
	dls::dog::JSSweeper::createJSSweeper_t* create_jss = (dls::dog::JSSweeper::createJSSweeper_t*) dlsym(jss, "createJSSweeper_t");

	const char* dlsym_error = dlerror();
    if (dlsym_error) {
        std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    }
	//else
		//std::cout <<"Create symbol is correclty loaded\n";
	
	return create_jss (pstep);
}

#endif /* end of include guard: ROBOT_FACTORY_CPP_PDOYFAZX */
