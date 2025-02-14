/** \file dab_physics_random_rotation_target.cpp
*/

#include "dab_physics_behavior_random_rotation_target.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_universal_motor.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_body_joint.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

RandomRotationTargetBehavior::RandomRotationTargetBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mTargetMin({ 0.0, 0.0, 0.0 })
	, mTargetMax({ 0.0, 0.0, 0.0 })
	, mSpeed(0.1)
	, mApplicationInterval(1.0)
	, mRandomizationInterval(1000.0)
{
	// only use motors of type Universal2Motor
	std::vector<std::shared_ptr<BodyMotor>> _motors;
	int motorCount = pMotors.size();
	for (int mI = 0; mI < motorCount; ++mI)
	{
		if (std::dynamic_pointer_cast<UniversalMotor>(mMotors[mI]) != nullptr)
		{
			_motors.push_back(mMotors[mI]);
		}
	}
	mMotors = _motors;

	init();
}

void
RandomRotationTargetBehavior::init()
{
	mParameters["minTarget"] = &mTargetMin;
	mParameters["maxTarget"] = &mTargetMax;
	mParameters["speed"] = &mSpeed;
	mParameters["appInterval"] = &mApplicationInterval;
	mParameters["randInterval"] = &mRandomizationInterval;

	mTargets = std::vector< std::array<float, 3>>(mMotors.size(), std::array<float, 3>({0.0, 0.0, 0.0}));

	mLastApplicationTime = ofGetElapsedTimeMillis();
	mLastRandomizationTime = mLastApplicationTime;
}

void
RandomRotationTargetBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
RandomRotationTargetBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& targetMin = mTargetMin;
	const std::array<float, 3>& targetMax = mTargetMax;
	const float& speed = mSpeed;
	const float& applicationInterval = mApplicationInterval;
	const float& randomizationInterval = mRandomizationInterval;

	int motorCount = mMotors.size();

	double currentTime = ofGetElapsedTimeMillis();

	//std::cout << "RandomRotationTargetBehavior currentTime " << currentTime << " rotationTargetChangeTime " << (mLastRotationTargetChangeTime + rotationTargetChangeInterval) << "\n";

	// set new rotation targets
	if (currentTime > mLastRandomizationTime + randomizationInterval)
	{
		//std::cout << "set new rotation targets\n";

		mLastRandomizationTime = currentTime;

		float rotationTarget;

		for (int mI = 0; mI < motorCount; ++mI)
		{
			for (int d = 0; d < 3; ++d)
			{
				mTargets[mI][d] = targetMin[d] + ofRandomuf() * (targetMax[d] - targetMin[d]);
			}

			//std::cout << "new rot target for motor " << mMotors[mI]->name() << " : " << mTargets[mI][0] << " " << mTargets[mI][1] << " " << mTargets[mI][2] << "\n";
		}
	}

	// apply rotation target
	if (currentTime > mLastApplicationTime + applicationInterval)
	{
		//std::cout << "apply rotation targets\n";

		mLastApplicationTime = currentTime;

		std::array<float, 3> currentAngle;
		std::array<float, 3> targetAngleDiff;
		std::array<float, 3> nextAngleTarget;

		for (int mI = 0; mI < motorCount; ++mI)
		{
			std::shared_ptr<UniversalMotor> _motor = std::static_pointer_cast<UniversalMotor>(mMotors[mI]);

			// determine incremented target angle
			std::shared_ptr<UniversalJoint> _joint = std::static_pointer_cast<UniversalJoint>(_motor->joint());
			currentAngle = _joint->angles();

			const std::array<float, 3>& target = mTargets[mI];

			//std::cout << "mI " << mI << " target " << target[0] << " " << target[1] << " " << target[2] << "\n";

			for (int d = 0; d < 3; ++d)
			{
				targetAngleDiff[d] = target[d] - currentAngle[d];
				if (fabs(targetAngleDiff[d]) < speed) nextAngleTarget[d] = target[d];
				else if (targetAngleDiff[d] < 0.0) nextAngleTarget[d] = currentAngle[d] - speed;
				else nextAngleTarget[d] = currentAngle[d] + speed;
			}

			//std::cout << "mI " << mI << " curr " << currentAngle[0] << " " << currentAngle[1] << " " << currentAngle[2] << "\n";
			//std::cout << "mI " << mI << " diff " << targetAngleDiff[0] << " " << targetAngleDiff[1] << " " << targetAngleDiff[2] << "\n";
			//std::cout << "mI " << mI << " next " << nextAngleTarget[0] << " " << nextAngleTarget[1] << " " << nextAngleTarget[2] << "\n";

			// update motor target angle
			const std::array<bool, 3>& angularServoActive = _motor->angularServoActive();
			const std::array<bool, 3>& angularSpringActive = _motor->angularSpringActive();

			for (int d = 0; d < 3; ++d)
			{
				if (angularServoActive[d] == true)
				{
					_motor->setAngularServoTarget(nextAngleTarget);
					//std::cout << "mI " << mI << " setAngularServoTarget " << nextAngleTarget[0] << " " << nextAngleTarget[1] << " " << nextAngleTarget[2] << "\n";
					break;
				}
				else if (angularSpringActive[d] == true)
				{
					_motor->setAngularSpringRestLength(nextAngleTarget);
					//std::cout << "mI " << mI << " setAngularSpringRestLength " << nextAngleTarget[0] << " " << nextAngleTarget[1] << " " << nextAngleTarget[2] << "\n";
					break;
				}
			}

		}
	}
}

