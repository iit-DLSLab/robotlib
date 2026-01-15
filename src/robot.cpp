#include "robotlib/robot.hpp"

namespace robotlib
{
	Robot::Robot(
		const std::string& name,
		const TrunkPtr& trunk,
		const std::vector<LimbPtr>& limbs)
		: RobotBase(name, trunk, limbs)
	{};

	Robot::Robot() : RobotBase() {};

	
	unsigned int Robot::getNLIMBS() const { return limbs_.size(); };

	unsigned int Robot::getNJOINTS() const { return joints_.size(); };

	unsigned int Robot::getNLINKS() const { return links_.size(); };

	
	unsigned int Robot::getNLEGS() const
	{
		unsigned int count{0};
        for(auto limb : limbs_)
        {
            if(limb->type().compare("leg") == 0)
				count++;
        }
        return count;
	};

	
	unsigned int Robot::getNARMS() const 
	{
		unsigned int count{0};
        for(auto limb : limbs_)
        {
            if(limb->type().compare("arm") == 0)
				count++;
        }
        return count;
	};

	
    const TrunkPtr Robot::getTrunk() const
	{
		return trunk_;
	}
	
	void Robot::getMinJointAngle(JointState &q_min)
	{
		for (auto joint : this->getJoints())
		{
			q_min[joint->id] = joint->getMinAngle();
		}
	}

	
	void Robot::getMaxJointAngle(JointState &q_max)
	{
		for (auto joint : this->getJoints())
		{
			q_max[joint->id] = joint->getMaxAngle();
		}
	}

	
	void Robot::getMaxJointVelocity(JointState &qd_max)
	{
		for (auto joint : this->getJoints())
		{
			qd_max[joint->id] = joint->getMaxVelocity();
		}
	}

	
	void Robot::getMaxJointEffort(JointState &tau_max)
	{
		for (auto joint : this->getJoints())
		{
			tau_max[joint->id] = joint->getMaxEffort();
		}
	}
} // namespace robotlib