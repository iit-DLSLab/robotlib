#include "robot.hpp"
#include "limb.hpp"

namespace dls
{
	namespace robotlib
	{
		class DummyLeg : public Limb<3, 2>
		{
		public:
			DummyLeg(std::string name) : Limb<3, 2>(
											 name,
											 std::array<std::shared_ptr<Joint>, 3>({std::make_shared<Joint>(nullptr, nullptr, name + "_joint1"),
																					std::make_shared<Joint>(nullptr, nullptr, name + "_joint2"),
																					std::make_shared<Joint>(nullptr, nullptr, name + "_joint3")}),
											 std::array<std::shared_ptr<Link>, 2>({std::make_shared<Link>(this, name + "_link1"),
																				   std::make_shared<Link>(this, name + "_link2")})) {}
		};

		class DummyQuadruped : public Robot<12, 8, 4, 0>
		{
		public:
			DummyQuadruped() : Robot<12, 8, 4, 0>(
								   "Quadruped",
								   std::array<std::shared_ptr<LimbBase>, 4>({std::make_shared<DummyLeg>("leg1"),
																			 std::make_shared<DummyLeg>("leg2"),
																			 std::make_shared<DummyLeg>("leg3"),
																			 std::make_shared<DummyLeg>("leg4")}),
								   std::array<std::shared_ptr<LimbBase>, 0>()){};

			Eigen::Vector3d getFramePosition(const JointState &q,
											 const Frame &origin,
											 const Frame &destination) override
			{
				return Eigen::Vector3d().setZero();
			};

			Eigen::Matrix3d getFrameOrientation(const JointState &q,
												const Frame &origin,
												const Frame &destination) override
			{
				return Eigen::Matrix3d().setZero();
			};

			Eigen::Matrix4d getFramePose(const JointState &q,
										 const Frame &origin,
										 const Frame &destination) override
			{
				Eigen::Matrix4d frame_pose{};
				frame_pose.setZero();

				frame_pose.block(0, 3, 3, 1) << getFramePosition(q, origin, destination);
				frame_pose.block(0, 0, 3, 3) << getFrameOrientation(q, origin, destination);
				frame_pose.row(3) << 0, 0, 0, 1;

				return frame_pose;
			};

			Eigen::Vector3d getFootPosition(const JointState &q,
											const Frame &foot) override
			{
				return this->getFramePosition(q, this->getLink("TRUNK"), foot);
			};

			Eigen::Matrix3d getFootOrientation(const JointState &q,
											   const Frame &foot) override
			{
				return this->getFrameOrientation(q, this->getLink("TRUNK"), foot);
			};

			Eigen::Matrix4d getFootPose(const JointState &q,
										const Frame &foot) override
			{
				Eigen::Matrix4d foot_pose{};
				foot_pose.setZero();

				foot_pose.block(0, 3, 3, 1) << getFootPosition(q, foot);
				foot_pose.block(0, 0, 3, 3) << getFootOrientation(q, foot);
				foot_pose.row(3) << 0, 0, 0, 1;

				return foot_pose;
			};

			// TO DO: mapping between name and id
			Link getLink(const std::string &name) override
			{
				return *std::static_pointer_cast<Link>(this->getLeg(0)->getLink(0));
			};

			Joint getJoint(const std::string &name) override
			{
				return Joint(nullptr, nullptr, "joint");
			};

			LegDataMap<std::shared_ptr<Frame>> getFeet() override
			{
				auto feet = this->makeLegDataMap<std::shared_ptr<Frame>>();

				const LimbBase *l;
				for (auto &foot : feet)
					foot = std::make_shared<Link>(l, "link");

				return feet;
			};

			void inverseKinematics(const Eigen::Vector3d &end_effector_position,
								   const Eigen::Vector3d &end_effector_velocity,
								   const Eigen::Vector3d &end_effector_acceleration,
								   Eigen::Vector3d &joint_position,
								   Eigen::Vector3d &joint_velocity,
								   Eigen::Vector3d &joint_acceleration,
								   const Frame &end_effector) override
			{
				std::cout << "Inverse Kinematics 1" << std::endl;
			};

			void inverseKinematics(const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_position,
								   const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_velocity,
								   const LegDataMap<Eigen::Matrix<double, 3, 1>> &end_effector_acceleration,
								   JointState &joint_position, // TODO: In Ant Controller the JointState is an Eigen::Matrix<double, 18, 1>
								   JointState &joint_velocity,
								   JointState &joint_acceleration) override
			{
				std::cout << "Inverse Kinematics 2" << std::endl;
			};
		};

		extern "C" std::shared_ptr<dls::robotlib::RobotBase> createRobot_t()
		{
			return std::make_shared<DummyQuadruped>();
		}

		extern "C" void destroyRobot_t(std::shared_ptr<dls::robotlib::RobotBase> robot)
		{
		}
	} //namespace robotlib
} //namespace dls
