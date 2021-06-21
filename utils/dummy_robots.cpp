#include "robot.hpp"
#include "leg.hpp"
#include "limb_base.hpp"
#include "joint.hpp"
#include "link.hpp"

namespace dls
{
    namespace robotlib
    {
        const int NLEGS = 4;
        const int NJOINTS = 3;
        const int NLINKS = 2;
        const int NJOINTS_TOT = 12;
        const int NLINKS_TOT = 8;

        class DummyHyq : public Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT>
        {
        public:
            DummyHyq(const std::string &name, std::array<std::shared_ptr<LimbBase>, NLEGS> legs)
                : Robot<NLEGS, NJOINTS_TOT, NLINKS_TOT>(name, legs)
            {
                name_ = "hyq";
                joints_ = {{std::make_shared<Joint>("HAA"),
                            std::make_shared<Joint>("HFE"),
                            std::make_shared<Joint>("KFE")}};
                links_ = {{std::make_shared<Link>("Upper leg"),
                           std::make_shared<Link>("Lower leg")}};
                legs_ = {{std::make_shared<Leg>("LF", joints_, links_),
                          std::make_shared<Leg>("RF", joints_, links_),
                          std::make_shared<Leg>("LH", joints_, links_),
                          std::make_shared<Leg>("RH", joints_, links_)}};
                name = name_;
                legs = legs_;
            };

            ~DummyHyq();

        private:
            std::string name_;
            std::array<std::shared_ptr<LimbBase>, NLEGS> legs_;
            std::array<std::shared_ptr<Joint>, NJOINTS> joints_;
            std::array<std::shared_ptr<Link>, NLINKS> links_;
        };
    } // namespace hyqlib
} // namespace dls