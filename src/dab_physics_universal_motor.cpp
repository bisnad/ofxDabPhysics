/** \file dab_physics_universal_motor.cpp
*/

#include "dab_physics_universal_motor.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_body.h"

using namespace dab;
using namespace dab::physics;

std::array<bool, 3> UniversalMotor::sLinearActive = { true, true, true };
std::array<bool, 3> UniversalMotor::sAngularActive = { true, true, true };
std::array<float, 3> UniversalMotor::sMaxLinearMotorForce = { 10000.0, 10000.0, 10000.0 };
std::array<float, 3> UniversalMotor::sMaxAngularMotorForce = { 10000.0, 10000.0, 10000.0 };
std::array<float, 3> UniversalMotor::sLinearVelocity = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sAngularVelocity = { 0.0, 0.0, 0.0 };
std::array<bool, 3> UniversalMotor::sLinearServoActive = { false, false, false };
std::array<bool, 3> UniversalMotor::sAngularServoActive = { false, false, false };
std::array<float, 3> UniversalMotor::sLinearServoTarget = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sAngularServoTarget = { 0.0, 0.0, 0.0 };
std::array<bool, 3> UniversalMotor::sLinearSpringActive = { false, false, false };
std::array<bool, 3> UniversalMotor::sAngularSpringActive = { false, false, false };
std::array<float, 3> UniversalMotor::sLinearSpringStiffness = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sAngularSpringStiffness = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sLinearSpringDamping = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sAngularSpringDamping = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sLinearSpringRestLength = { 0.0, 0.0, 0.0 };
std::array<float, 3> UniversalMotor::sAngularSpringRestLength = { 0.0, 0.0, 0.0 };

UniversalMotor::UniversalMotor(std::shared_ptr<UniversalJoint> pJoint)
	: BodyMotor(pJoint)
	, mLinearActive(sLinearActive)
	, mAngularActive(sAngularActive)
	, mMaxLinearMotorForce(sMaxLinearMotorForce)
	, mMaxAngularMotorForce(sMaxAngularMotorForce)
	, mLinearVelocity(sLinearVelocity)
	, mAngularVelocity(sAngularVelocity)
	, mLinearServoActive(sLinearServoActive)
	, mAngularServoActive(sAngularServoActive)
	, mLinearServoTarget(sLinearServoTarget)
	, mAngularServoTarget(sAngularServoTarget)
	, mLinearSpringActive(sLinearSpringActive)
	, mAngularSpringActive(sAngularSpringActive)
	, mLinearSpringStiffness(sLinearSpringStiffness)
	, mAngularSpringStiffness(sAngularSpringStiffness)
	, mLinearSpringDamping(sLinearSpringDamping)
	, mAngularSpringDamping(sAngularSpringDamping)
	, mLinearSpringRestLength(sLinearSpringRestLength)
	, mAngularSpringRestLength(sAngularSpringRestLength)
{
	initPhysics();
}

UniversalMotor::~UniversalMotor()
{
	std::cout << "delete UniversalMotor\n";
}

void
UniversalMotor::initPhysics()
{
	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	// translational motor - general settings
	nativeJoint->enableMotor(0, mLinearActive[0]);
	nativeJoint->enableMotor(1, mLinearActive[1]);
	nativeJoint->enableMotor(2, mLinearActive[2]);

	nativeJoint->setMaxMotorForce(0, mMaxLinearMotorForce[0]);
	nativeJoint->setMaxMotorForce(1, mMaxLinearMotorForce[1]);
	nativeJoint->setMaxMotorForce(2, mMaxLinearMotorForce[2]);

	nativeJoint->setBounce(0, mBounce);
	nativeJoint->setBounce(1, mBounce);
	nativeJoint->setBounce(2, mBounce);

	nativeJoint->setDamping(0, mDamping);
	nativeJoint->setDamping(1, mDamping);
	nativeJoint->setDamping(2, mDamping);

	// translational motor - velocity mode settings
	nativeJoint->setTargetVelocity(0, mLinearVelocity[0]);
	nativeJoint->setTargetVelocity(1, mLinearVelocity[1]);
	nativeJoint->setTargetVelocity(2, mLinearVelocity[2]);

	// translational motor - position mode settings
	nativeJoint->setServo(0, mLinearServoActive[0]);
	nativeJoint->setServo(1, mLinearServoActive[1]);
	nativeJoint->setServo(2, mLinearServoActive[2]);

	nativeJoint->setServoTarget(0, mLinearServoTarget[0]);
	nativeJoint->setServoTarget(1, mLinearServoTarget[1]);
	nativeJoint->setServoTarget(2, mLinearServoTarget[2]);

	// translational motor - spring mode settings
	nativeJoint->enableSpring(0, mLinearSpringActive[0]);
	nativeJoint->enableSpring(1, mLinearSpringActive[1]);
	nativeJoint->enableSpring(2, mLinearSpringActive[2]);

	nativeJoint->setStiffness(0, mLinearSpringStiffness[0]);
	nativeJoint->setStiffness(1, mLinearSpringStiffness[1]);
	nativeJoint->setStiffness(2, mLinearSpringStiffness[2]);

	nativeJoint->setDamping(0, mLinearSpringDamping[0]);
	nativeJoint->setDamping(1, mLinearSpringDamping[1]);
	nativeJoint->setDamping(2, mLinearSpringDamping[2]);

	nativeJoint->setEquilibriumPoint(0, mLinearSpringRestLength[0]);
	nativeJoint->setEquilibriumPoint(1, mLinearSpringRestLength[1]);
	nativeJoint->setEquilibriumPoint(2, mLinearSpringRestLength[2]);

	// rotational motor general settings
	nativeJoint->enableMotor(3, mAngularActive[0]);
	nativeJoint->enableMotor(4, mAngularActive[1]);
	nativeJoint->enableMotor(5, mAngularActive[2]);

	nativeJoint->setMaxMotorForce(3, mMaxAngularMotorForce[3]);
	nativeJoint->setMaxMotorForce(4, mMaxAngularMotorForce[4]);
	nativeJoint->setMaxMotorForce(5, mMaxAngularMotorForce[5]);

	nativeJoint->setBounce(3, mBounce);
	nativeJoint->setBounce(4, mBounce);
	nativeJoint->setBounce(5, mBounce);

	nativeJoint->setDamping(3, mDamping);
	nativeJoint->setDamping(4, mDamping);
	nativeJoint->setDamping(5, mDamping);

	// rotational motors - velocity mode settings
	nativeJoint->setTargetVelocity(3, mAngularVelocity[0]);
	nativeJoint->setTargetVelocity(4, mAngularVelocity[1]);
	nativeJoint->setTargetVelocity(5, mAngularVelocity[2]);

	// rotational motors - position mode settings
	nativeJoint->setServo(3, mAngularServoActive[0]);
	nativeJoint->setServo(4, mAngularServoActive[1]);
	nativeJoint->setServo(5, mAngularServoActive[2]);

	nativeJoint->setServoTarget(3, mAngularServoTarget[0]);
	nativeJoint->setServoTarget(4, mAngularServoTarget[1]);
	nativeJoint->setServoTarget(5, mAngularServoTarget[2]);

	// rotational motors - spring mode settings
	nativeJoint->enableSpring(3, mAngularSpringActive[0]);
	nativeJoint->enableSpring(4, mAngularSpringActive[1]);
	nativeJoint->enableSpring(5, mAngularSpringActive[2]);

	nativeJoint->setStiffness(3, mAngularSpringStiffness[0]);
	nativeJoint->setStiffness(4, mAngularSpringStiffness[1]);
	nativeJoint->setStiffness(5, mAngularSpringStiffness[2]);

	nativeJoint->setDamping(3, mAngularSpringDamping[0]);
	nativeJoint->setDamping(4, mAngularSpringDamping[1]);
	nativeJoint->setDamping(5, mAngularSpringDamping[2]);

	nativeJoint->setEquilibriumPoint(3, mAngularSpringRestLength[0]);
	nativeJoint->setEquilibriumPoint(4, mAngularSpringRestLength[1]);
	nativeJoint->setEquilibriumPoint(5, mAngularSpringRestLength[2]);
}

const std::array<bool, 3>& 
UniversalMotor::linearActive() const
{
	return mLinearActive;
}

const std::array<bool, 3>& 
UniversalMotor::angularActive() const
{
	return mAngularActive;
}

const std::array<float, 3>& 
UniversalMotor::linearLowerLimit() const
{
	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	return universalJoint->linearLowerLimit();
}

const std::array<float, 3>& 
UniversalMotor::linearUpperLimit() const
{
	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	return universalJoint->linearUpperLimit();
}

const std::array<float, 3>& 
UniversalMotor::angularLowerLimit() const
{
	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	return universalJoint->angularLowerLimit();
}

const std::array<float, 3>& 
UniversalMotor::angularUpperLimit() const
{
	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	return universalJoint->angularUpperLimit();
}

const std::array<float, 3>& 
UniversalMotor::linearStopERP() const
{
	return mJoint->linearStopERP();
}

const std::array<float, 3>& 
UniversalMotor::angularStopERP() const
{
	return mJoint->angularStopERP();
}

const std::array<float, 3>& 
UniversalMotor::linearStopCFM() const
{
	return mJoint->linearStopCFM();
}

const std::array<float, 3>& 
UniversalMotor::angularStopCFM() const
{
	return mJoint->angularStopCFM();
}

const std::array<float, 3>& 
UniversalMotor::maxLinearMotorForce() const
{
	return mMaxLinearMotorForce;
}

const std::array<float, 3>& 
UniversalMotor::maxAngularMotorForce() const
{
	return mMaxAngularMotorForce;
}

const std::array<float, 3>& 
UniversalMotor::linearVelocity() const
{
	return mLinearVelocity;
}

const std::array<float, 3>& 
UniversalMotor::angularVelocity() const
{
	return mAngularVelocity;
}

const std::array<bool, 3>& 
UniversalMotor::linearServoActive() const
{
	return mLinearServoActive;
}

const std::array<bool, 3>& 
UniversalMotor::angularServoActive() const
{
	return mAngularServoActive;
}

const std::array<float, 3>& 
UniversalMotor::linearServoTarget() const
{
	return mLinearServoTarget;
}

const std::array<float, 3>& 
UniversalMotor::angularServoTarget() const
{
	return mAngularServoTarget;
}

const std::array<bool, 3>& 
UniversalMotor::linearSpringActive() const
{
	return mLinearSpringActive;
}

const std::array<bool, 3>& 
UniversalMotor::angularSpringActive() const
{
	return mAngularSpringActive;
}

const std::array<float, 3>& 
UniversalMotor::linearSpringStiffness() const
{
	return mLinearSpringStiffness;
}

const std::array<float, 3>& 
UniversalMotor::angularSpringStiffness() const
{
	return mAngularSpringStiffness;
}

const std::array<float, 3>& 
UniversalMotor::linearSpringDamping() const
{
	return mLinearSpringDamping;
}

const std::array<float, 3>& 
UniversalMotor::angularSpringDamping() const
{
	return mAngularSpringDamping;
}

const std::array<float, 3>& 
UniversalMotor::linearSpringRestLength() const
{
	return mLinearSpringRestLength;
}

const std::array<float, 3>& 
UniversalMotor::angularSpringRestLength() const
{
	return mAngularSpringRestLength;
}

void 
UniversalMotor::setBounce(float pBounce)
{
	BodyMotor::setBounce(pBounce);

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setBounce(0, mBounce);
		nativeJoint->setBounce(1, mBounce);
		nativeJoint->setBounce(2, mBounce);
		nativeJoint->setBounce(3, mBounce);
		nativeJoint->setBounce(4, mBounce);
		nativeJoint->setBounce(5, mBounce);

		//std::cout << "Body " << mBody->name() << " motor " << name() << " setBounce " << mBounce << "\n";
	}
}

void 
UniversalMotor::setDamping(float pDamping)
{
	BodyMotor::setDamping(pDamping);

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setDamping(0, mDamping);
		nativeJoint->setDamping(1, mDamping);
		nativeJoint->setDamping(2, mDamping);
		nativeJoint->setDamping(3, mDamping);
		nativeJoint->setDamping(4, mDamping);
		nativeJoint->setDamping(5, mDamping);

		//std::cout << "Body " << mBody->name() << " motor " << name() << " setDamping " << mDamping << "\n";
	}
}

void
UniversalMotor::setLinearActive(const std::array<bool, 3>& pLinearActive)
{
	mLinearActive = pLinearActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->enableMotor(0, mLinearActive[0]);
		nativeJoint->enableMotor(1, mLinearActive[1]);
		nativeJoint->enableMotor(2, mLinearActive[2]);

		//std::cout << "Body " << mBody->name() << " motor " << name() << " setLinearActive " << mLinearActive[0] << " " << mLinearActive[1] << " " << mLinearActive[2] << "\n";
	}
}

void
UniversalMotor::setAngularActive(const std::array<bool, 3>& pAngularActive)
{
	mAngularActive = pAngularActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->enableMotor(3, mAngularActive[0]);
		nativeJoint->enableMotor(4, mAngularActive[1]);
		nativeJoint->enableMotor(5, mAngularActive[2]);

		//std::cout << "Body " << mBody->name() << " motor " << name() << " setAngularActive " << mAngularActive[0] << " " << mAngularActive[1] << " " << mAngularActive[2] << "\n";
	}
}

void
UniversalMotor::setLinearLowerLimit(const std::array<float, 3>& pLinearLowerLimit)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setLinearLowerLimit(pLinearLowerLimit);

	//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearLowerLimit " << pLinearLowerLimit[0] << " " << pLinearLowerLimit[1] << " " << pLinearLowerLimit[2] << "\n";
}

void
UniversalMotor::setLinearUpperLimit(const std::array<float, 3>& pLinearUpperLimit)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setLinearUpperLimit(pLinearUpperLimit);

	//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearUpperLimit " << pLinearUpperLimit[0] << " " << pLinearUpperLimit[1] << " " << pLinearUpperLimit[2] << "\n";
}

void
UniversalMotor::setAngularLowerLimit(const std::array<float, 3>& pAngularLowerLimit)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setAngularLowerLimit(pAngularLowerLimit);

	//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularLowerLimit " << pAngularLowerLimit[0] << " " << pAngularLowerLimit[1] << " " << pAngularLowerLimit[2] << "\n";
}

void
UniversalMotor::setAngularUpperLimit(const std::array<float, 3>& pAngularUpperLimit)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setAngularUpperLimit(pAngularUpperLimit);

	//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularUpperLimit " << pAngularUpperLimit[0] << " " << pAngularUpperLimit[1] << " " << pAngularUpperLimit[2] << "\n";
}

void 
UniversalMotor::setLinearStopERP(const std::array<float, 3>& pLinearStopERP)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setLinearStopERP(pLinearStopERP);

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setLinearStopERP " << pLinearStopERP[0] << " " << pLinearStopERP[1] << " " << pLinearStopERP[2] << "\n";
}

void 
UniversalMotor::setAngularStopERP(const std::array<float, 3>& pAngularStopERP)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setAngularStopERP(pAngularStopERP);

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setAngularStopERP " << pAngularStopERP[0] << " " << pAngularStopERP[1] << " " << pAngularStopERP[2] << "\n";
}

void 
UniversalMotor::setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setLinearStopCFM(pLinearStopCFM);

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setLinearStopCFM " << pLinearStopCFM[0] << " " << pLinearStopCFM[1] << " " << pLinearStopCFM[2] << "\n";
}

void 
UniversalMotor::setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM)
{
	std::static_pointer_cast<UniversalJoint>(mJoint)->setAngularStopCFM(pAngularStopCFM);

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setAngularStopCFM " << pAngularStopCFM[0] << " " << pAngularStopCFM[1] << " " << pAngularStopCFM[2] << "\n";
}

void 
UniversalMotor::setMaxLinearMotorForce(const std::array<float, 3>& pMaxLinearMotorForce)
{
	mMaxLinearMotorForce = pMaxLinearMotorForce;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setMaxMotorForce(0, mMaxLinearMotorForce[0]);
		nativeJoint->setMaxMotorForce(1, mMaxLinearMotorForce[1]);
		nativeJoint->setMaxMotorForce(2, mMaxLinearMotorForce[2]);

		//std::cout << "Body " << mBody->name() << " motor " << name() << " setMaxLinearMotorForce " << mMaxLinearMotorForce[0] << " " << mMaxLinearMotorForce[1] << " " << mMaxLinearMotorForce[2] << "\n";
	}
}

void 
UniversalMotor::setMaxAngularMotorForce(const std::array<float, 3>& pMaxAngularMotorForce)
{
	mMaxAngularMotorForce = pMaxAngularMotorForce;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setMaxMotorForce(3, mMaxAngularMotorForce[0]);
		nativeJoint->setMaxMotorForce(4, mMaxAngularMotorForce[1]);
		nativeJoint->setMaxMotorForce(5, mMaxAngularMotorForce[2]);

		//std::cout << "body " << mBody->name() << "motor " << name() << " setMaxAngularMotorForce " << mMaxAngularMotorForce[0] << " " << mMaxAngularMotorForce[1] << " " << mMaxAngularMotorForce[2] << "\n";
	}
}

void 
UniversalMotor::setLinearVelocity(const std::array<float, 3>& pLinearVelocity)
{
	mLinearVelocity = pLinearVelocity;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setTargetVelocity(0, mLinearVelocity[0]);
		nativeJoint->setTargetVelocity(1, mLinearVelocity[1]);
		nativeJoint->setTargetVelocity(2, mLinearVelocity[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearVelocity " << mLinearVelocity[0] << " " << mLinearVelocity[1] << " " << mLinearVelocity[2] << "\n";
	}
}

void 
UniversalMotor::setAngularVelocity(const std::array<float, 3>& pAngularVelocity)
{
	mAngularVelocity = pAngularVelocity;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setTargetVelocity(3, mAngularVelocity[0]);
		nativeJoint->setTargetVelocity(4, mAngularVelocity[1]);
		nativeJoint->setTargetVelocity(5, mAngularVelocity[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularVelocity " << mAngularVelocity[0] << " " << mAngularVelocity[1] << " " << mAngularVelocity[2] << "\n";
	}
}

void 
UniversalMotor::setLinearServoActive(const std::array<bool, 3>& pLinearServoActive)
{
	mLinearServoActive = pLinearServoActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setServo(0, mLinearServoActive[0]);
		nativeJoint->setServo(1, mLinearServoActive[1]);
		nativeJoint->setServo(2, mLinearServoActive[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearServoActive " << mLinearServoActive[0] << " " << mLinearServoActive[1] << " " << mLinearServoActive[2] << "\n";
	}
}

void 
UniversalMotor::setAngularServoActive(const std::array<bool, 3>& pAngularServoActive)
{
	mAngularServoActive = pAngularServoActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setServo(3, mAngularServoActive[0]);
		nativeJoint->setServo(4, mAngularServoActive[1]);
		nativeJoint->setServo(5, mAngularServoActive[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularServoActive " << mAngularServoActive[0] << " " << mAngularServoActive[1] << " " << mAngularServoActive[2] << "\n";
	}
}

void 
UniversalMotor::setLinearServoTarget(const std::array<float, 3>& pLinearServoTarget)
{
	mLinearServoTarget = pLinearServoTarget;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setServoTarget(0, mLinearServoTarget[0]);
		nativeJoint->setServoTarget(1, mLinearServoTarget[1]);
		nativeJoint->setServoTarget(2, mLinearServoTarget[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearServoTarget " << mLinearServoTarget[0] << " " << mLinearServoTarget[1] << " " << mLinearServoTarget[2] << "\n";
	}
}

void 
UniversalMotor::setAngularServoTarget(const std::array<float, 3>& pAngularServoTarget)
{
	mAngularServoTarget = pAngularServoTarget;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setServoTarget(3, mAngularServoTarget[0]);
		nativeJoint->setServoTarget(4, mAngularServoTarget[1]);
		nativeJoint->setServoTarget(5, mAngularServoTarget[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularServoTarget " << mAngularServoTarget[0] << " " << mAngularServoTarget[1] << " " << mAngularServoTarget[2] << "\n";
	}
}

void 
UniversalMotor::setLinearSpringActive(const std::array<bool, 3>& pLinearSpringActive)
{
	mLinearSpringActive = pLinearSpringActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->enableSpring(0, mLinearSpringActive[0]);
		nativeJoint->enableSpring(1, mLinearSpringActive[1]);
		nativeJoint->enableSpring(2, mLinearSpringActive[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearSpringActive " << mLinearSpringActive[0] << " " << mLinearSpringActive[1] << " " << mLinearSpringActive[2] << "\n";
	}
}

void 
UniversalMotor::setAngularSpringActive(const std::array<bool, 3>& pAngularSpringActive)
{
	mAngularSpringActive = pAngularSpringActive;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->enableSpring(3, mAngularSpringActive[0]);
		nativeJoint->enableSpring(4, mAngularSpringActive[1]);
		nativeJoint->enableSpring(5, mAngularSpringActive[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularSpringActive " << mAngularSpringActive[0] << " " << mAngularSpringActive[1] << " " << mAngularSpringActive[2] << "\n";
	}
}

void 
UniversalMotor::setLinearSpringStiffness(const std::array<float, 3>& pLinearSpringStiffness)
{
	mLinearSpringStiffness = pLinearSpringStiffness;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setStiffness(0, mLinearSpringStiffness[0]);
		nativeJoint->setStiffness(1, mLinearSpringStiffness[1]);
		nativeJoint->setStiffness(2, mLinearSpringStiffness[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearSpringStiffness " << mLinearSpringStiffness[0] << " " << mLinearSpringStiffness[1] << " " << mLinearSpringStiffness[2] << "\n";
	}
}

void 
UniversalMotor::setAngularSpringStiffness(const std::array<float, 3>& pAngularSpringStiffness)
{
	mAngularSpringStiffness = pAngularSpringStiffness;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setStiffness(3, mAngularSpringStiffness[0]);
		nativeJoint->setStiffness(4, mAngularSpringStiffness[1]);
		nativeJoint->setStiffness(5, mAngularSpringStiffness[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularSpringStiffness " << mAngularSpringStiffness[0] << " " << mAngularSpringStiffness[1] << " " << mAngularSpringStiffness[2] << "\n";
	}
}

void 
UniversalMotor::setLinearSpringDamping(const std::array<float, 3>& pLinearSpringDamping)
{
	mLinearSpringDamping = pLinearSpringDamping;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setDamping(0, mLinearSpringDamping[0]);
		nativeJoint->setDamping(1, mLinearSpringDamping[1]);
		nativeJoint->setDamping(2, mLinearSpringDamping[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearSpringDamping " << mLinearSpringDamping[0] << " " << mLinearSpringDamping[1] << " " << mLinearSpringDamping[2] << "\n";
	}
}

void 
UniversalMotor::setAngularSpringDamping(const std::array<float, 3>& pAngularSpringDamping)
{
	mAngularSpringDamping = pAngularSpringDamping;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setDamping(3, mAngularSpringDamping[0]);
		nativeJoint->setDamping(4, mAngularSpringDamping[1]);
		nativeJoint->setDamping(5, mAngularSpringDamping[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularSpringDamping " << mAngularSpringDamping[0] << " " << mAngularSpringDamping[1] << " " << mAngularSpringDamping[2] << "\n";
	}
}

void 
UniversalMotor::setLinearSpringRestLength(const std::array<float, 3>& pLinearSpringRestLength)
{
	mLinearSpringRestLength = pLinearSpringRestLength;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setEquilibriumPoint(0, mLinearSpringRestLength[0]);
		nativeJoint->setEquilibriumPoint(1, mLinearSpringRestLength[1]);
		nativeJoint->setEquilibriumPoint(2, mLinearSpringRestLength[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setLinearSpringRestLength " << mLinearSpringRestLength[0] << " " << mLinearSpringRestLength[1] << " " << mLinearSpringRestLength[2] << "\n";
	}
}

void 
UniversalMotor::setAngularSpringRestLength(const std::array<float, 3>& pAngularSpringRestLength)
{
	// TODO: figure out why does has no effect at all

	mAngularSpringRestLength = pAngularSpringRestLength;

	std::shared_ptr<UniversalJoint> universalJoint = std::static_pointer_cast<UniversalJoint>(mJoint);
	btGeneric6DofSpring2Constraint* nativeJoint = universalJoint->mNativeJoint;

	if (nativeJoint != nullptr)
	{
		nativeJoint->setEquilibriumPoint(3, mAngularSpringRestLength[0]);
		nativeJoint->setEquilibriumPoint(4, mAngularSpringRestLength[1]);
		nativeJoint->setEquilibriumPoint(5, mAngularSpringRestLength[2]);

		//std::cout << "body " << mBody->name() << " motor " << name() << " setAngularSpringRestLength " << mAngularSpringRestLength[0] << " " << mAngularSpringRestLength[1] << " " << mAngularSpringRestLength[2] << "\n";
	}
}

void 
UniversalMotor::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	try
	{
		if (pParName == "bounce")
		{
			setBounce(pValue);
		}
		else if (pParName == "damping")
		{
			setDamping(pValue);
		}
		else if (pParName == "linearActive")
		{
			setLinearActive(pValue);
		}
		else if (pParName == "angularActive")
		{
			setAngularActive(pValue);
		}
		else if (pParName == "linearLowerLimit")
		{
			setLinearLowerLimit(pValue);
		}
		else if (pParName == "linearUpperLimit")
		{
			setLinearUpperLimit(pValue);
		}
		else if (pParName == "angularLowerLimit")
		{
			setAngularLowerLimit(pValue);
		}
		else if (pParName == "angularUpperLimit")
		{
			setAngularUpperLimit(pValue);
		}
		else if (pParName == "linearStopERP")
		{
			setLinearStopERP(pValue);
		}
		else if (pParName == "angularStopERP")
		{
			setAngularStopERP(pValue);
		}
		else if (pParName == "linearStopCFM")
		{
			setLinearStopCFM(pValue);
		}
		else if (pParName == "angularStopCFM")
		{
			setAngularStopCFM(pValue);
		}
		else if (pParName == "maxLinearMotorForce")
		{
			setMaxLinearMotorForce(pValue);
		}
		else if (pParName == "maxAngularMotorForce")
		{
			setMaxAngularMotorForce(pValue);
		}
		else if (pParName == "linearVelocity")
		{
			setLinearVelocity(pValue);
		}
		else if (pParName == "angularVelocity")
		{
			setAngularVelocity(pValue);
		}
		else if (pParName == "linearServoActive")
		{
			setLinearServoActive(pValue);
		}
		else if (pParName == "angularServoActive")
		{
			setAngularServoActive(pValue);
		}
		else if (pParName == "linearServoTarget")
		{
			setLinearServoTarget(pValue);
		}
		else if (pParName == "angularServoTarget")
		{
			setAngularServoTarget(pValue);
		}
		else if (pParName == "linearSpringActive")
		{
			setLinearSpringActive(pValue);
		}
		else if (pParName == "angularSpringActive")
		{
			setAngularSpringActive(pValue);
		}
		else if (pParName == "linearSpringStiffness")
		{
			setLinearSpringStiffness(pValue);
		}
		else if (pParName == "angularSpringStiffness")
		{
			setAngularSpringStiffness(pValue);
		}		
		else if (pParName == "linearSpringDamping")
		{
			setLinearSpringDamping(pValue);
		}
		else if (pParName == "angularSpringDamping")
		{
			setAngularSpringDamping(pValue);
		}
		else if (pParName == "linearSpringRestLength")
		{
			setLinearSpringRestLength(pValue);
		}
		else if (pParName == "angularSpringRestLength")
		{
			setAngularSpringRestLength(pValue);
		}
		else
		{
			throw dab::Exception("Physics Error: parameter name " + pParName + " not recognized by universal motor", __FILE__, __FUNCTION__, __LINE__);

		}
	}
	catch (Exception& e)
	{
		e += Exception("Physics Error: failed to set parameter " + pParName + " for universal motor " + name(), __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}