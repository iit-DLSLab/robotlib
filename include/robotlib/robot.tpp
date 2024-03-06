#include "robotlib/robot.hpp"

namespace robotlib
{
	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	Robot<NLIMBS, NLINKS, NJOINTS>::Robot(
		const std::string& name,
		const Trunk& trunk,
		std::array<std::shared_ptr<LimbBase>, NLIMBS>& limbs)
		: RobotBase(name)
		, trunk_(trunk)
		, limbs_(limbs)
	{
		// unsigned int count_data{0};
		// for(auto& limb : limbs_)
		// {
		// 	for(auto& joint : limb.getJoints())
		// 		this->joints_[count_data++] = std::shared_ptr<Joint>(&joint);
		// }

		// count_data = 0;
		// for(auto& limb : limbs_)
		// {
		// 	for(auto& link : limb.getLinks())
		// 		this->links_[count_data++] = std::shared_ptr<Link>(&link);
		// }

	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	unsigned int Robot<NLIMBS, NLINKS, NJOINTS>::getNLIMBS() const { return NLIMBS; };

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	unsigned int Robot<NLIMBS, NLINKS, NJOINTS>::getNJOINTS() const { return NJOINTS; };

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	unsigned int Robot<NLIMBS, NLINKS, NJOINTS>::getNLINKS() const { return NLINKS; };

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	unsigned int Robot<NLIMBS, NLINKS, NJOINTS>::getNLEGS() const
	{
		unsigned int count{0};
        for(auto& limb : limbs_)
        {
            if(limb.type().compare("leg") == 0)
				count++;
        }
        return count;
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	unsigned int Robot<NLIMBS, NLINKS, NJOINTS>::getNARMS() const 
	{
		unsigned int count{0};
        for(auto& limb : limbs_)
        {
            if(limb.type().compare("arm") == 0)
				count++;
        }
        return count;
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const Joint& Robot<NLIMBS, NLINKS, NJOINTS>::getJoint(const std::string &name) const
	{
		for(auto& joint : joints_)
        {
            if(joint.getName().compare(name) == 0)
                return joint;
        }
        throw std::range_error("joint name not found");
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const ContainerBase<Joint> Robot<NLIMBS, NLINKS, NJOINTS>::getJoints()
	{
		// if(joints_[0] == nullptr)
		// {
		// 	unsigned int count_data{0};
		// 	for(auto& limb : limbs_)
		// 	{
		// 		for(auto& joint : limb.getJoints())
		// 			this->joints_[count_data++] = std::shared_ptr<Joint>(&joint);
		// 	}
		// }

		// count_data = 0;
		// for(auto& limb : limbs_)
		// {
		// 	for(auto& link : limb.getLinks())
		// 		this->links_[count_data++] = std::shared_ptr<Link>(&link);
		// }
		ContainerBase<Joint> out(dynamic_cast<ContainerAbstract<Joint> &>(joints_));
		return out;
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const Link& Robot<NLIMBS, NLINKS, NJOINTS>::getLink(const std::string& name) const
	{
		for(auto& link : links_)
        {
            if(link.getName().compare(name) == 0)
                return link;
        }
        throw std::range_error("link name not found");
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
    const ContainerBase<Link> Robot<NLIMBS, NLINKS, NJOINTS>::getLinks()
    {
		ContainerBase<Link> out(dynamic_cast<ContainerAbstract<Link> &>(links_));
		return out;
    }

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const LimbBase& Robot<NLIMBS, NLINKS, NJOINTS>::getLimb(const std::string &name) const
	{
		for (auto& limb : limbs_)
		{
			if (limb.getName().compare(name) == 0)
				return limb;
		}
		throw std::range_error("limb name not found");
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const ContainerBase<LimbBase> Robot<NLIMBS, NLINKS, NJOINTS>::getLimbs() const
	{
		ContainerBase<LimbBase> out(dynamic_cast<ContainerAbstract<LimbBase>&>(limbs_));
		return out;
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const std::vector<std::shared_ptr<const LimbBase>> Robot<NLIMBS, NLINKS, NJOINTS>::getLegs() const
	{
		std::vector<std::shared_ptr<const LimbBase>> out;
		for(auto& limb : limbs_)
		{
			if(limb.type().compare("leg"))
				out.push_back(std::shared_ptr<const LimbBase>(&limb));
		}
		return out;
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	const std::vector<std::shared_ptr<const LimbBase>> Robot<NLIMBS, NLINKS, NJOINTS>::getArms() const
	{
		std::vector<std::shared_ptr<const LimbBase>> out;
		for(auto& limb : limbs_)
		{
			if(limb.type().compare("arm"))
				out.push_back(std::shared_ptr<const LimbBase>(&limb));
		}
		return out;
	};

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	void Robot<NLIMBS, NLINKS, NJOINTS>::getMinJointAngle(JointState &q_min)
	{
		for (auto& joint : this->getJoints())
		{
			q_min[joint] = joint->getMinAngle();
		}
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	void Robot<NLIMBS, NLINKS, NJOINTS>::getMaxJointAngle(JointState &q_max)
	{
		for (auto& joint : this->getJoints())
		{
			q_max[joint] = joint->getMaxAngle();
		}
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	void Robot<NLIMBS, NLINKS, NJOINTS>::getMaxJointVelocity(JointState &qd_max)
	{
		for (auto& joint : this->getJoints())
		{
			qd_max[joint] = joint->getMaxVelocity();
		}
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	void Robot<NLIMBS, NLINKS, NJOINTS>::getMaxJointEffort(JointState &tau_max)
	{
		for (auto& joint : this->getJoints())
		{
			tau_max[joint] = joint->getMaxEffort();
		}
	}

	template <unsigned int NLIMBS, unsigned int NLINKS, unsigned int NJOINTS>
	Eigen::Vector3d Robot<NLIMBS, NLINKS, NJOINTS>::getTrunkCOM() const
	{
		return trunk_.getCoM();
	};

} // namespace robotlib