#include "robotlib/robot.hpp"

namespace robotlib
{
	Robot::Robot(
		const std::string& name,
		const DynParams& dynamic_parameters,
		const std::vector<LimbPtr>& limbs)
		: RobotBase(name, dynamic_parameters, limbs)
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

	
	const JointPtr Robot::getJoint(const std::string &name) const
	{
		for(auto joint : joints_)
        {
            if(joint->getName().compare(name) == 0)
                return joint;
        }
        throw std::range_error("joint name not found");
	}

	
	std::vector<JointPtr>& Robot::getJoints()
	{
		return joints_;
	}

	const std::vector<JointPtr> Robot::getJoints() const
	{
		return joints_;
	}

	
	const LinkPtr Robot::getLink(const std::string& name) const
	{
		for(auto link : links_)
        {
            if(link->getName().compare(name) == 0)
                return link;
        }
        throw std::range_error("link name not found");
	};

	
    std::vector<LinkPtr> Robot::getLinks() const
    {
		return links_; 
    }

	
	const LimbPtr Robot::getLimb(const std::string &name) const
	{
		for (auto limb : limbs_)
		{
			if (limb->getName().compare(name) == 0)
				return limb;
		}
		throw std::range_error("limb name not found");
	}

	
	const std::vector<LimbPtr> Robot::getLimbs() const
	{
		return limbs_; 
	};

	
	std::vector<LimbPtr> Robot::getLegs() const
	{
		std::vector<LimbPtr> out;
		for(auto limb : limbs_)
		{
			if(limb->type().compare("leg") == 0)
				out.push_back(limb);
		}
		return out;
	};

	
	std::vector<LimbPtr> Robot::getArms() const
	{
		std::vector<LimbPtr> out;
		for(auto limb : limbs_)
		{
			if(limb->type().compare("arm") == 0)
				out.push_back(limb);
		}
		return out;
	};

	
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