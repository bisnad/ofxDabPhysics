/** \file dab_physics_random_torque.cpp
*/

#include "dab_physics_behavior_random_torque.h"
#include "dab_physics_body_part.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

RandomTorqueBehavior::RandomTorqueBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mTorqueMin({ -1.0, -1.0, -1.0 })
	, mTorqueMax({ 1.0, 1.0, 1.0 })
	, mTorqueScale(1.0)
	, mApplicationInterval(1.0)
	, mRandomizationInterval(1000.0)
{
	init();
}

void
RandomTorqueBehavior::init()
{
	mParameters["minTorque"] = &mTorqueMin;
	mParameters["maxTorque"] = &mTorqueMax;
	mParameters["scale"] = &mTorqueScale;
	mParameters["appInterval"] = &mApplicationInterval;
	mParameters["randInterval"] = &mRandomizationInterval;

	mTorques = std::vector<glm::vec3>(mParts.size(), glm::vec3(0.0, 0.0, 0.0));

	mLastApplicationTime = ofGetElapsedTimeMillis();
	mLastRandomizationTime = mLastApplicationTime;
}

void
RandomTorqueBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
RandomTorqueBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& torqueMin = mTorqueMin;
	const std::array<float, 3>& torqueMax = mTorqueMax;
	float scale = mTorqueScale;
	float applicationInterval = mApplicationInterval;
	float randomizationInterval = mRandomizationInterval;
	int partCount = mParts.size();

	double currentTime = ofGetElapsedTimeMillis();

	//std::cout << "RandomTorqueBehavior currentTime " << currentTime << " forceChangeTime " << (mLastForceChangeTime + forceChangeInterval) << " forceApplyTime " << (mLastForceApplicationTime + forceApplicationInterval) << "\n";

	if (currentTime > mLastRandomizationTime + randomizationInterval)
	{
		mLastRandomizationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			glm::vec3& torque = mTorques[pI];

			for (int d = 0; d < 3; ++d)
			{
				torque[d] = torqueMin[d] + ofRandomuf() * (torqueMax[d] - torqueMin[d]);
				torque[d] *= scale;
			}

			//std::cout << "new torque " << tI << " : " << torque[0] << " " << torque[1] << " " << torque[2] << "\n";
		}
	}

	if (currentTime > mLastApplicationTime + applicationInterval)
	{
		//std::cout << "apply mForce " << mForce[0] << " " << mForce[1] << " " << mForce[2] << "\n";

		mLastApplicationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			mParts[pI]->applyTorque(mTorques[pI]);
		}
	}
}

