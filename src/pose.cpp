#include "pose.hpp"

namespace dls
{
	namespace robotlib
	{
		Pose::Pose(const Eigen::Vector3d &p, const Eigen::Vector3d &ori) : p_(p), ori_(ori){};

		Pose::~Pose(){};

		const Eigen::Vector3d &Pose::getPos() { return p_; };
		const Eigen::Vector3d &Pose::getOri() { return ori_; };
	} // namespace robotlib
} // namespace dls