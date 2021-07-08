#ifndef _ROBOTLIB_FRAME_HPP_
#define _ROBOTLIB_FRAME_HPP_

#include <Eigen/Dense>

namespace dls
{
    namespace robotlib
    {
        class Frame
        {
        public:
            Frame(const std::string name) : name_(name){};
            //Frame(const Eigen::Vector3d &p, const Eigen::Vector3d &ori)
            virtual ~Frame(){};

            virtual const std::string getName() = 0;

        protected:
            const std::string name_{};
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_FRAME_HPP_