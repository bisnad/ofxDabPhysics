/** \file dab_physics_random_force.cpp
*/

#include "dab_physics_behavior_random_force.h"
#include "dab_physics_body_part.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

RandomForceBehavior::RandomForceBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mForceDirectionsMin({ -1.0, -1.0, -1.0 })
	, mForceDirectionsMax({ 1.0, 1.0, 1.0 })
	, mForceAmplitudeMin(0.0)
	, mForceAmplitudeMax(10.0)
	, mForceApplicationInterval(1.0)
	, mForceChangeInterval(1000.0)
{
	init();
}

void
RandomForceBehavior::init()
{
	mParameters["minDir"] = &mForceDirectionsMin;
	mParameters["maxDir"] = &mForceDirectionsMax;
	mParameters["minAmp"] = &mForceAmplitudeMin;
	mParameters["maxAmp"] = &mForceAmplitudeMax;
	mParameters["appInterval"] = &mForceApplicationInterval;
	mParameters["randInterval"] = &mForceChangeInterval;

	mForces = std::vector<glm::vec3>(mParts.size(), glm::vec3(0.0, 0.0, 0.0));

	mLastForceApplicationTime = ofGetElapsedTimeMillis();
	mLastForceChangeTime = mLastForceApplicationTime;
}

void
RandomForceBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
RandomForceBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	const std::array<float, 3>& forceDirectionsMin = mForceDirectionsMin;
	const std::array<float, 3>& forceDirectionsMax = mForceDirectionsMax;
	float forceAmplitudeMin = mForceAmplitudeMin;
	float forceAmplitudeMax = mForceAmplitudeMax;
	float forceApplicationInterval = mForceApplicationInterval;
	float forceChangeInterval = mForceChangeInterval;
	int partCount = mParts.size();

	double currentTime = ofGetElapsedTimeMillis();

	//std::cout << "RandomForceBehavior currentTime " << currentTime << " forceChangeTime " << (mLastForceChangeTime + forceChangeInterval) << " forceApplyTime " << (mLastForceApplicationTime + forceApplicationInterval) << "\n";

	if (currentTime > mLastForceChangeTime + forceChangeInterval)
	{
		mLastForceChangeTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			glm::vec3& force = mForces[pI];

			for (int d = 0; d < 3; ++d) force[d] = forceDirectionsMin[d] + ofRandomuf() * (forceDirectionsMax[d] - forceDirectionsMin[d]);
			float forceAmplitude = forceAmplitudeMin + ofRandomuf() * (forceAmplitudeMax - forceAmplitudeMin);
			force = glm::normalize(force);
			force *= forceAmplitude;

			//std::cout << "new force " << fI << " : " << force[0] << " " << force[1] << " " << force[2] << "\n";
		}
	}

	if (currentTime > mLastForceApplicationTime + forceApplicationInterval)
	{
		//std::cout << "apply mForce " << mForce[0] << " " << mForce[1] << " " << mForce[2] << "\n";

		mLastForceApplicationTime = currentTime;

		for (int pI = 0; pI < partCount; ++pI)
		{
			mParts[pI]->applyForce(mForces[pI]);
		}
	}
}

