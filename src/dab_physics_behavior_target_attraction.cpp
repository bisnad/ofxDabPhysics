/** \file dab_physics_behavior_target_attraction.cpp
*/

#include "dab_physics_behavior_target_attraction.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_universal_motor.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

TargetAttractionBehavior::TargetAttractionBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mTargetPosition({ 0.0, 0.0, 0.0 })
	, mMinDistance(0.1)
	, mMaxDistance(10.0)
	, mForceAmplitudeMin(0.0)
	, mForceAmplitudeMax(10.0)
	, mForceApplicationInterval(1.0)
{
	init();
}

void
TargetAttractionBehavior::init()
{
	mParameters["targetPos"] = &mTargetPosition;
	mParameters["minDist"] = &mMinDistance;
	mParameters["maxDist"] = &mMaxDistance;
	mParameters["minAmp"] = &mForceAmplitudeMin;
	mParameters["maxAmp"] = &mForceAmplitudeMax;
	mParameters["appInterval"] = &mForceApplicationInterval;

	mForces = std::vector<glm::vec3>(mParts.size(), glm::vec3(0.0, 0.0, 0.0));

	mLastForceApplicationTime = ofGetElapsedTimeMillis();
}

void
TargetAttractionBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
TargetAttractionBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& _targetPosition = mTargetPosition;
	glm::vec3 targetPosition(_targetPosition[0], _targetPosition[1], _targetPosition[2]);

	float minDistance = mMinDistance;
	float maxDistance = mMaxDistance;
	float minForceAmplitude = mForceAmplitudeMin;
	float maxForceAmplitude = mForceAmplitudeMax;
	float forceApplicationInterval = mForceApplicationInterval;
	int partCount = mParts.size();

	double currentTime = ofGetElapsedTimeMillis();

	if (currentTime > mLastForceApplicationTime + forceApplicationInterval)
	{
		//std::cout << "apply mForce " << mForce[0] << " " << mForce[1] << " " << mForce[2] << "\n";

		mLastForceApplicationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			std::shared_ptr<BodyPart> part = mParts[pI];
			glm::vec3 partPos = part->position();

			glm::vec3 posDiff = targetPosition - partPos;
			float posDist = glm::length(posDiff);
			if (posDist > maxDistance) continue;
			if (posDist < minDistance) continue;

			glm::vec3 forceDir = posDiff / posDist;

			float forceScale = 1.0 - posDist / maxDistance;
			float forceAmp = minForceAmplitude + (maxForceAmplitude - minForceAmplitude) * forceScale;

			glm::vec3 forceVec = forceDir * forceAmp;

			mForces[pI] = forceVec;

			part->applyForce(forceVec);
		}
	}
}




