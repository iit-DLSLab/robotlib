#ifndef _ROBOTLIB_DYN_PARAMS_HPP_
#define _ROBOTLIB_DYN_PARAMS_HPP_

#include <Eigen/Dense>

namespace dls
{
	namespace robotlib
	{
		/**
 		* A class for robot dynamic parameters (com, mass, inertia).
 		*/
		class DynParams
		{
		public:
			DynParams(const Eigen::Vector3d &com, const double m, const Eigen::Matrix3d &I) : com_(com), m_(m), I_(I){};

			~DynParams(){};

			// Get functions
			inline const Eigen::Vector3d &getCom() { return com_; };
			inline const double getM() { return m_; };
			inline const Eigen::Matrix3d &getI() { I_; };

		private:
			const Eigen::Vector3d com_; //! Center of mass
			const double m_;			//! Mass
			const Eigen::Matrix3d I_;	//! Inertia
		};

	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_DYN_PARAMS_HPP_
