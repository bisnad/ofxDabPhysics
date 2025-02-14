/** \file dab_physics_random_force.cpp
*/

#include "dab_physics_behavior_volume.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

VolumeBehavior::VolumeBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mMaxDistance({ 1.0f })
	, mMinForceAmplitude(0.0f)
	, mMaxForceAmplitude(10.0f)
	, mForceApplicationInterval(1.0f)
{
	init();
}

void
VolumeBehavior::init()
{
	mParameters["maxDist"] = &mMaxDistance;
	mParameters["minAmp"] = &mMinForceAmplitude;
	mParameters["maxAmp"] = &mMaxForceAmplitude;
	mParameters["appInterval"] = &mForceApplicationInterval;

	mForces = std::vector<glm::vec3>(mParts.size() / 2, glm::vec3(0.0, 0.0, 0.0));

	mLastForceApplicationTime = ofGetElapsedTimeMillis();
}

void
VolumeBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
VolumeBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	float maxDistance = mMaxDistance;
	float minForceAmplitude = mMinForceAmplitude;
	float maxForceAmplitude = mMaxForceAmplitude;
	float forceApplicationInterval = mForceApplicationInterval;
	int partCount = mParts.size();
	int pairCount = partCount / 2;

	//std::cout << "body " << mBody->name() << " minF " << minForceAmplitude << " maxF " << maxForceAmplitude << " maxD " << maxDistance << "\n";

	double currentTime = ofGetElapsedTimeMillis();

	//std::cout << "RandomForceBehavior currentTime " << currentTime << " forceChangeTime " << (mLastForceChangeTime + forceChangeInterval) << " forceApplyTime " << (mLastForceApplicationTime + forceApplicationInterval) << "\n";

	if (currentTime > mLastForceApplicationTime + forceApplicationInterval)
	{
		//// clear forces
		//for (int pI = 0; pI < pairCount; ++pI)
		//{
		//	mForces[0] = glm::vec3(0.0);
		//}

		// calculate forces between pairs of body parts
		for (int pI = 0; pI < pairCount; ++pI)
		{
			std::shared_ptr<BodyPart> part1 = mParts[pI*2];
			std::shared_ptr<BodyPart> part2 = mParts[pI*2+1];

			glm::vec3 part1Pos = part1->position();
			glm::vec3 part2Pos = part2->position();

			glm::vec3 posDiff = part2Pos - part1Pos;
			float posDist = glm::length(posDiff);

			if (posDist > maxDistance) continue;

			glm::vec3 forceDir = posDiff / posDist;
			float forceScale = 1.0 - posDist / maxDistance;
			float forceAmp = minForceAmplitude + (maxForceAmplitude - minForceAmplitude) * forceScale;

			glm::vec3 forceVec = forceDir * forceAmp;

			mForces[pI] = forceVec;

			part1->applyForce(forceVec * -1.0);
			part2->applyForce(forceVec * 1.0);
		}

		mLastForceApplicationTime = currentTime;
	}
}

