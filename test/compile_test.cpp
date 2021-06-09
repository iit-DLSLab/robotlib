#include "robotlib/hyqreal/HyQReal.hpp"

#include "robotlib/factory/robot_factory.hpp"

using namespace dls::dog;
void testRobot                  (std::shared_ptr<Dog>);
void testRobotLengthsBase       (std::shared_ptr<RobotLengthsBase>);
void testFeetContactForcesBase  (std::shared_ptr<FeetContactForcesBase>);
void testFeetJacobiansBase      (std::shared_ptr<FeetJacobiansBase>);
void testForwardKinematicsBase  (std::shared_ptr<ForwardKinematicsBase>);
void testInertiaPropertiesBase  (std::shared_ptr<InertiaPropertiesBase>);
void testInverseDynamicsBase    (std::shared_ptr<InverseDynamicsBase>);
void testInverseKinematicsBase  (std::shared_ptr<InverseKinematicsBase>);
void testJSIMBase               (std::shared_ptr<JSIMBase>);
void testKinDynParamsBase       (std::shared_ptr<KinDynParamsBase>);
void testLimitsBase             (std::shared_ptr<LimitsBase>);
void testShinJacobiansBase      (std::shared_ptr<ShinJacobiansBase>);

int main(int argc, char **argv)
{
	auto pHyQReal = RobotFactory::buildRobot(RobotFactory::RobotType::HyQReal);
	auto pHyQ = RobotFactory::buildRobot(RobotFactory::RobotType::HyQ);
	testRobot(pHyQ);
	testRobot(pHyQReal);
}

void testRobot(std::shared_ptr<Dog> pRobot)
{
	auto pRobotLengths       =  pRobot->getRobotLengths();
	auto pFeetContactForces  =  pRobot->getFeetContactforces();
	auto pFeetJacobians      =  pRobot->getFeetJacobians();
	auto pForwardKinematics  =  pRobot->getForwardKinematics();
	auto pInertiaProperties  =  pRobot->getInertiaProperties();
	auto pInverseDynamics    =  pRobot->getInverseDynamics();
	auto pInverseKinematics  =  pRobot->getInverseKinematics();
	auto pJSIM               =  pRobot->getJSIM();
	auto pKinDynParams       =  pRobot->getKinDynParams();
	auto pLimits             =  pRobot->getLimits();
	auto pShinJacobians      =  pRobot->getShinJacobians();

	testRobotLengthsBase       (pRobotLengths);
	testFeetContactForcesBase  (pFeetContactForces);
	testFeetJacobiansBase      (pFeetJacobians);
	testForwardKinematicsBase  (pForwardKinematics);
	testInertiaPropertiesBase  (pInertiaProperties);
	testInverseDynamicsBase    (pInverseDynamics);
	testInverseKinematicsBase  (pInverseKinematics);
	testJSIMBase               (pJSIM);
	testKinDynParamsBase       (pKinDynParams);
	testLimitsBase             (pLimits);
	testShinJacobiansBase      (pShinJacobians);
}

void testRobotLengthsBase       (std::shared_ptr<RobotLengthsBase> pRobot)
{
    pRobot->getHAA_x();
    pRobot->getHAA_y();
    pRobot->getHAA_z();

    pRobot->getDist_HAA_HFE();
    pRobot->getDist_HFE_KFE();
    pRobot->getFoot_x();
    pRobot->getFoot_y();
    pRobot->getFoot_z();
}
void testFeetContactForcesBase  (std::shared_ptr<FeetContactForcesBase> pRobot)
{
}
void testFeetJacobiansBase      (std::shared_ptr<FeetJacobiansBase> pRobot)
{
}
void testForwardKinematicsBase  (std::shared_ptr<ForwardKinematicsBase> pRobot)
{
}
void testInertiaPropertiesBase  (std::shared_ptr<InertiaPropertiesBase> pRobot)
{
}
void testInverseDynamicsBase    (std::shared_ptr<InverseDynamicsBase> pRobot)
{
}
void testInverseKinematicsBase  (std::shared_ptr<InverseKinematicsBase> pRobot)
{
}
void testJSIMBase               (std::shared_ptr<JSIMBase> pRobot)
{
}
void testKinDynParamsBase       (std::shared_ptr<KinDynParamsBase> pRobot)
{
}
void testLimitsBase             (std::shared_ptr<LimitsBase> pRobot)
{
}
void testShinJacobiansBase      (std::shared_ptr<ShinJacobiansBase> pRobot)
{
}
