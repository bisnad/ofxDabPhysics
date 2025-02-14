/** \file dab_physics_behavior_speed.cpp
*/

#include "dab_physics_behavior_speed.h"
#include "dab_physics_body_part.h"
#include "ofUtils.h"
#include "ofMath.h"

using namespace dab;
using namespace dab::physics;

SpeedBehavior::SpeedBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mSpeed(1.0)
	, mAmount(1.0)
{
	init();
}


void
SpeedBehavior::init()
{
	mParameters["speed"] = &mSpeed;
	mParameters["amount"] = &mAmount;
}

void
SpeedBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
SpeedBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	float targetSpeed = mSpeed;
	float amount = mAmount;

	int partCount = mParts.size();

	for (int pI = 0; pI < partCount; ++pI)
	{
		const glm::vec3& linearVelocity = mParts[pI]->linearVelocity();

		float currentSpeed = glm::length(linearVelocity);
		float speedDifference = targetSpeed - currentSpeed;

		glm::vec3 force = glm::normalize(linearVelocity);
		force *= speedDifference;
		force *= amount;

		mParts[pI]->applyForce(force);
	}
}

