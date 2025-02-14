/** \file dab_physics_torque.cpp
*/

#include "dab_physics_behavior_torque.h"
#include "dab_physics_body_part.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

TorqueBehavior::TorqueBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mTorque({ 0.0, 0.0, 0.0 })
	, mTorqueScale(1.0)
	, mApplicationInterval(1.0)
{
	init();
}

void
TorqueBehavior::init()
{
	mParameters["torque"] = &mTorque;
	mParameters["scale"] = &mTorqueScale;
	mParameters["appInterval"] = &mApplicationInterval;

	mLastApplicationTime = ofGetElapsedTimeMillis();
}

void
TorqueBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
TorqueBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& torque = mTorque;
	float scale = mTorqueScale;
	float applicationInterval = mApplicationInterval;
	int partCount = mParts.size();

	double currentTime = ofGetElapsedTimeMillis();

	if (currentTime > mLastApplicationTime + applicationInterval)
	{
		glm::vec3 _torque(torque[0] * scale, torque[1] * scale, torque[2] * scale);
		//std::cout << "apply torque " << _torque[0] << " " << _torque[1] << " " << _torque[2] << "\n";

		mLastApplicationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			mParts[pI]->applyTorque(_torque);
		}
	}
}

