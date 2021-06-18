#ifndef _ROBOTLIB_JOINT_HPP_
#define _ROBOTLIB_JOINT_HPP_

#include "limb_base.hpp"

namespace dls
{
	namespace robotlib
	{
		/**
		 * A joint class for robots.
		 */
		class Joint
		{
		public:
			Joint(LimbBase *parent, const std::string &name) : parent_(parent), name_(name){};
			//Joint (const std::string& name, const DynParams& dparams ): name_(name), dyn_params_(dparams) {};

			~Joint(){};

			// Get functions
			const std::string getName() { return name_; }
			const LimbBase *getParent() const { return parent_; };
			//const DynParams& getDynParams(){return dyn_params_;}

		private:
			const std::string name_; //! Name of the joint
			const LimbBase *parent_; //! Pointer to parent limb
									 //const DynParams dyn_params_;	        //! Dynamic parameter of the joint
									 //joint limits TODO
		};
	} // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_JOINT_HPP_
