/** \file dab_physics_rotation_target.cpp
*/

#include "dab_physics_behavior_rotation_target.h"
#include "dab_physics_body.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_universal_motor.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_body_joint.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

RotationTargetBehavior::RotationTargetBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mTarget({ 0.0, 0.0, 0.0 })
	, mSpeed(0.1)
	, mApplicationInterval(1.0)
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
RotationTargetBehavior::init()
{
	mParameters["target"] = &mTarget;
	mParameters["speed"] = &mSpeed;
	mParameters["appInterval"] = &mApplicationInterval;

	mLastApplicationTime = ofGetElapsedTimeMillis();
}

void
RotationTargetBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
RotationTargetBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& target = mTarget;
	const float& speed = mSpeed;
	const float& applicationInterval = mApplicationInterval;

	//std::cout << "body " << mBody->name() << " target " << target[0] << " " << target[1] << " " << target[2] << " speed " << speed << " inter " << applicationInterval << "\n";

	int motorCount = mMotors.size();

	double currentTime = ofGetElapsedTimeMillis();

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

			//std::cout << "mI " << mI << " mTarget " << mTarget[0] << " " << mTarget[1] << " " << mTarget[2] << "\n";

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

