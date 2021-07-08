#ifndef _ROBOTLIB_ROBOT_BASE_HPP_
#define _ROBOTLIB_ROBOT_BASE_HPP_
#include "limb_base.hpp"
#include "link.hpp"
#include "joint.hpp"
#include "dyn_params.hpp"
#include "utils.hpp"

#include <iostream>
#include <memory>

namespace dls
{
    namespace robotlib
    {
        class RobotBase
        {
        protected:
            template <class Data>
            class LegDataMap
            {
            public:
                LegDataMap(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    data_ = new Data[nLegs_];
                }
                ~LegDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nLegs_]); }

            private:
                Data *data_;
                const int nLegs_;
            };
            template <class Data>
            class JointDataMap
            {
            public:
                JointDataMap(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    data_ = new Data[nJoints_];
                }
                ~JointDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nJoints_]); }

            private:
                Data *data_;
                const int nJoints_;
            };

            class JointState : public JointDataMap<double>
            {
            public:
                JointState(RobotBase *robot) : JointDataMap(robot){};
                ~JointState(){};
            };

            template <class Data>
            class LinkDataMap
            {
            public:
                LinkDataMap(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    data_ = new Data[nLinks_];
                }
                ~LinkDataMap()
                {
                    delete[] data_;
                };

                Iterator<Data> begin() { return Iterator<Data>(&data_[0]); }
                Iterator<Data> end() { return Iterator<Data>(&data_[nLinks_]); }

            private:
                Data *data_;
                const int nLinks_;
            };

            template <class Data>
            class LegDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<LimbBase>, Data>;

            public:
                LegDataMapPair(RobotBase *robot) : nLegs_(robot->getNLEGS())
                {
                    for (int i = 0; i < nLegs_; ++i)
                    {
                        PairType pair(robot->getLeg(i), Data());
                        data_.push_back(pair);
                    }
                }

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLegs_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~LegDataMapPair(){};

            private:
                const int nLegs_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class LinkDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<Link>, Data>;

            public:
                LinkDataMapPair(RobotBase *robot) : nLinks_(robot->getNLINKS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nLinks = leg->getNLinks();
                        for (int j = 0; j < nLinks; j++)
                        {
                            auto link = std::static_pointer_cast<Link>(leg->getLink(j));
                            PairType pair(link, Data());
                            data_.push_back(pair);
                        }
                    }
                }
                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nLinks_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~LinkDataMapPair(){};

            private:
                const int nLinks_;
                std::vector<PairType> data_;
            };

            template <class Data>
            class JointDataMapPair
            {

                using PairType = std::pair<std::shared_ptr<Joint>, Data>;

            public:
                JointDataMapPair(RobotBase *robot) : nJoints_(robot->getNJOINTS())
                {
                    const int nLegs = robot->getNLEGS();
                    for (int i = 0; i < nLegs; i++)
                    {
                        auto leg = robot->getLeg(i);
                        int nJoints = leg->getNJoints();
                        for (int j = 0; j < nJoints; j++)
                        {
                            auto joint = std::static_pointer_cast<Joint>(leg->getJoint(j));
                            PairType pair(joint, Data());
                            data_.push_back(pair);
                        }
                    }
                }

                Iterator<PairType> begin() { return Iterator<PairType>(&data_[0]); }
                Iterator<PairType> end() { return Iterator<PairType>(&data_[nJoints_]); }

                // Get functions
                int getSize() { return data_.size(); };

                ~JointDataMapPair(){};

            private:
                const int nJoints_;
                std::vector<PairType> data_;
            };

            using Map = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
            class Jacobian : public Map
            {
            public:
                Jacobian(const int nJoints) : Map(NULL, 6, nJoints), nJoints_(nJoints)
                {
                    // Data initialization (6: linear and angular part of the jacobian)
                    data_ = new double[6 * nJoints_];
                    for (int i = 0; i < 6 * nJoints_; ++i)
                    {
                        data_[i] = 0;
                    }

                    new (this) Map(data_, 6, nJoints_);
                }
                ~Jacobian()
                {
                    delete[] data_;
                }

                Map getLinearJacobian() //RT
                {
                    auto linearMatrix{this->block(0, 0, 3, nJoints_)}; // Fixed size matrix!

                    return Map(&linearMatrix(0, 0), 3, nJoints_);
                };

                Map getAngularJacobian() //RT
                {
                    auto linearMatrix{this->block(3, 0, 3, nJoints_)}; // Fixed size matrix!

                    return Map(&linearMatrix(0, 0), 3, nJoints_);
                };

            private:
                const int nJoints_;
                double *data_; // Squashed matrix
            };

            const std::string name_;

        public:
            RobotBase(const std::string &name) : name_(name){};

            // TODO: Is it correct that the iterators are defined with "{}"?
            virtual Iterator<const std::shared_ptr<LimbBase>> begin(){};
            virtual Iterator<const std::shared_ptr<LimbBase>> end(){};

            // Get functions
            virtual const int getNLEGS() = 0;
            virtual const int getNJOINTS() = 0;
            virtual const int getNLINKS() = 0;

            virtual const std::shared_ptr<LimbBase> getLeg(const int id) = 0;

            // Plugin typedefs
            typedef std::shared_ptr<RobotBase> createRobot_t();
            typedef void destroyRobot_t(std::shared_ptr<RobotBase>);

            // Create a leg data map
            template <class Data>
            LegDataMap<Data> makeLegDataMap() { return LegDataMap<Data>(this); } // NRT

            // Create a joint data map
            template <class Data>
            JointDataMap<Data> makeJointDataMap() { return JointDataMap<Data>(this); } // NRT

            // Create a link data map
            template <class Data>
            LinkDataMap<Data> makeLinkDataMap() { return LinkDataMap<Data>(this); } // NRT

            // Create a joint state
            JointState makeJointState() { return JointState(this); } // NRT

            // Create a leg data map pair
            template <class Data>
            LegDataMapPair<Data> makeLegDataMapPair() { return LegDataMapPair<Data>(this); } // NRT

            // Create a link data map pair
            template <class Data>
            LinkDataMapPair<Data> makeLinkDataMapPair() { return LinkDataMapPair<Data>(this); } // NRT

            // Create a joint data map pair
            template <class Data>
            JointDataMapPair<Data> makeJointDataMapPair() { return JointDataMapPair<Data>(this); } // NRT

            // TODO
            Jacobian makeJacobian(const Frame &fOrigin, const Frame &fDest) // NRT
            {
                std::cout << "makeJacobian function: TODO\n";
                return Jacobian(1);
            };

            // TODO: it should use makeJacobian
            Jacobian makeFootJacobian(const Frame &frame) // NRT
            {
                Link foot = static_cast<const Link &>(frame); //TODO: try without static_cast

                const LimbBase *l = foot.getParentLimb();
                const int nJoints = l->getNJoints();

                return Jacobian(nJoints);
            };

            virtual Eigen::Vector3d getFramePosition(const JointState &q,
                                                     const Frame &origin,
                                                     const Frame &destination) = 0;

            virtual Eigen::Matrix3d getFrameOrientation(const JointState &q,
                                                        const Frame &origin,
                                                        const Frame &destination) = 0;

            virtual Eigen::Matrix4d getFramePose(const JointState &q,
                                                 const Frame &origin,
                                                 const Frame &destination) = 0;

            virtual Eigen::Vector3d getFootPosition(const JointState &q,
                                                    const Frame &foot) = 0;

            virtual Eigen::Matrix3d getFootOrientation(const JointState &q,
                                                       const Frame &foot) = 0;

            virtual Eigen::Matrix4d getFootPose(const JointState &q,
                                                const Frame &foot) = 0;

            virtual Link getLink(const std::string &name) = 0;

            virtual Joint getJoint(const std::string &name) = 0;

            virtual LegDataMap<std::shared_ptr<Frame>> getFeet() = 0;

            std::string getName() { return name_; };
        };
    } // namespace robotlib
} // namespace dls

#endif // _ROBOTLIB_ROBOT_BASE_HPP_
