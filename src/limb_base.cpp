#include "limb_base.hpp"

namespace dls
{
    namespace robotlib
    {
        LimbBase::LimbBase(const std::string &name) : name_(name){};

        LimbBase::~LimbBase(){};

        const std::string LimbBase::getName() const { return name_; };
    } // namespace robotlib
} // namespace dls