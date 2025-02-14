/** \file dab_physics_random_force.cpp
*/

#include "dab_physics_behavior_force.h"
#include "dab_physics_body_part.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

ForceBehavior::ForceBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mForceDirection({ 1.0, 0.0, 0.0 })
	, mForceAmplitude(0.0)
	, mForceApplicationInterval(1.0)
	, mForce(0.0, 0.0, 0.0)
{
	init();
}

void
ForceBehavior::init()
{
	mParameters["dir"] = &mForceDirection;
	mParameters["amp"] = &mForceAmplitude;
	mParameters["appInterval"] = &mForceApplicationInterval;

	mLastForceApplicationTime = ofGetElapsedTimeMillis();
}

void
ForceBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";

	if (pParName == "dir" || pParName == "amp")
	{
		const std::array<float, 3>& forceDirection = mForceDirection;
		float forceAmplitude = mForceAmplitude;

		glm::vec3 force(forceDirection[0], forceDirection[1], forceDirection[2]);
		float forceLength = glm::length(force);
		if (forceLength > 0.0000001) force /= forceLength;
		force *= forceAmplitude;

		mForce = force;
	}
}

void
ForceBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& forceDirection = mForceDirection;
	float forceAmplitude = mForceAmplitude;
	float forceApplicationInterval = mForceApplicationInterval;

	int partCount = mParts.size();

	double currentTime = ofGetElapsedTimeMillis();

	if (currentTime > mLastForceApplicationTime + forceApplicationInterval)
	{
		//std::cout << "apply mForce " << mForce[0] << " " << mForce[1] << " " << mForce[2] << "\n";

		mLastForceApplicationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			mParts[pI]->applyForce(mForce);
		}
	}
}

