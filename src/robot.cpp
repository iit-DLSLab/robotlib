#include "robot.hpp"

namespace dls
{
    namespace robotlib
    {
        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::Robot(const std::string &name, const std::array<std::shared_ptr<LimbBase>, NLEGS> &legs)
            : RobotBase(name), legs_(legs){};

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::~Robot(){};

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::begin()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[0]);
        };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        Iterator<const std::shared_ptr<LimbBase>> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::end()
        {
            return Iterator<const std::shared_ptr<LimbBase>>(&legs_[NLEGS]);
        };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const std::array<std::shared_ptr<LimbBase>, NLEGS> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getLegs() { return legs_; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const std::shared_ptr<LimbBase> Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getLeg(const int id) { return legs_[id]; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNLEGS() { return NLEGS; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNJOINTS() { return NJOINTS_TOT; };

        template <unsigned int NLEGS, int NJOINTS_TOT, int NLINKS_TOT, unsigned int NARMS>
        const int Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT, NARMS>::getNLINKS() { return NLINKS_TOT; };

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
    } // namespace robotlib
} // namespace dls
