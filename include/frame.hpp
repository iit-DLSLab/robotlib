#ifndef _ROBOTLIB_FRAME_HPP_
#define _ROBOTLIB_FRAME_HPP_

namespace dls
{
    namespace robotlib
    {
        class Frame
        {
        public:
            Frame(){};
            virtual ~Frame(){};

            virtual bool isIdentifier(std::string &frame_name) = 0;
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_FRAME_HPP_