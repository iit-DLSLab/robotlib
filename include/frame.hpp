#ifndef _ROBOTLIB_FRAME_HPP_
#define _ROBOTLIB_FRAME_HPP_

#include "limb_base.hpp"
#include <string>

namespace dls
{
    namespace robotlib
    {
        class Frame
        {
        public:
            Frame(){};
            virtual ~Frame(){};

            virtual const std::string getName() = 0;
            virtual const LimbBase *getParent() const = 0;

        protected:
            std::string name_{};
            const LimbBase *parent_{};
            //const DynParams dyn_params_;	        //! Dynamic parameter of the joint
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_FRAME_HPP_