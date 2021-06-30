#include "dyn_params.hpp"

namespace dls
{
	namespace robotlib
	{
		DynParams::DynParams(const Eigen::Vector3d &com, const double m, const Eigen::Matrix3d &I) : com_(com), m_(m), I_(I){};

		DynParams::~DynParams(){};

		inline const Eigen::Vector3d &DynParams::getCom() { return com_; };
		inline const double DynParams::getM() { return m_; };
		inline const Eigen::Matrix3d &DynParams::getI() { I_; };
	} // namespace robotlib
} // namespace dls