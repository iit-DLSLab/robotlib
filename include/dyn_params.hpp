#ifndef _ROBOTLIB_DYN_PARAMS_HPP_
#define _ROBOTLIB_DYN_PARAMS_HPP_

#include <Eigen/Dense>

namespace dls
{
	namespace robotlib
	{
		class DynParams
		{
		public:
			DynParams(const Eigen::Vector3d &com, const double m, const Eigen::Matrix3d &I);

			~DynParams();

			inline const Eigen::Vector3d &getCom();
			inline const double getM();
			inline const Eigen::Matrix3d &getI();

		private:
			const Eigen::Vector3d com_; //! Center of mass
			const double m_;			//! Mass
			const Eigen::Matrix3d I_;	//! Inertia
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_DYN_PARAMS_HPP_
