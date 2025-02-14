/** \file dab_physics_neighbor_behavior_alignment.cpp
*/

#include "dab_physics_neighbor_behavior_alignment.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"

using namespace dab;
using namespace dab::physics;

AlignmentBehavior::AlignmentBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyPart>>& pNeighborParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyJoint>>& pNeighborJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors, const std::vector<std::shared_ptr<BodyMotor>>& pNeighborMotors)
	: NeighborBehavior(pName, pParts, pNeighborParts, pJoints, pNeighborJoints, pMotors, pNeighborMotors)
	, mMinDist(0.0)
	, mMaxDist(10.0)
	, mLinearAmount(1.0)
	, mAngularAmount(1.0)
	, mAmount(1.0)
{
	init();
}

void
AlignmentBehavior::init()
{
	mParameters["minDist"] = &mMinDist;
	mParameters["maxDist"] = &mMaxDist;
	mParameters["linearAmount"] = &mLinearAmount;
	mParameters["angularAmount"] = &mAngularAmount;
	mParameters["amount"] = &mAmount;
}

void
AlignmentBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
AlignmentBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	float minDist = mMinDist;
	float maxDist = mMaxDist;
	float linearAmount = mLinearAmount;
	float angularAmount = mAngularAmount;
	float amount = mAmount;

	int partCount = mParts.size();
	int neighborPartCount = mNeighborParts.size();

	for (int pI = 0; pI < partCount; ++pI)
	{
		const glm::vec3& partPos = mParts[pI]->position();
		const glm::vec3& partLinearVelocity = mParts[pI]->linearVelocity();
		const glm::vec3& partAngularVelocity = mParts[pI]->angularVelocity();

		//std::cout << "pav " << mParts[pI]->body()->name() << " - " << mParts[pI]->name() << " " << partAngularVelocity << "\n";


		glm::vec3 avgNeighborDirection(0.0);
		glm::vec3 avgNeighborLinearVelocity(0.0);
		glm::vec3 avgNeighborAngularVelocity(0.0);

		int neighborWithinDistanceCount = 0;


		for (int npI = 0; npI < neighborPartCount; ++npI)
		{
			const glm::vec3& neighborPartPos = mNeighborParts[npI]->position(); 
			const glm::vec3& neighborLinearVelocity = mNeighborParts[npI]->linearVelocity();
			const glm::vec3& neighborAngularVelocity = mNeighborParts[npI]->angularVelocity();

			glm::vec3 neighborDir = neighborPartPos - partPos;
			float neighborDist = glm::length(neighborDir);

			if (neighborDist < minDist || neighborDist > mMaxDist) continue;

			avgNeighborDirection += neighborDir;
			avgNeighborLinearVelocity += neighborLinearVelocity;
			avgNeighborAngularVelocity += neighborAngularVelocity;

			neighborWithinDistanceCount++;

			//std::cout << "npav " << mNeighborParts[npI]->body()->name() << " - " << mNeighborParts[npI]->name() << " " << neighborAngularVelocity << "\n";
		}

		if (neighborWithinDistanceCount == 0) continue;

		avgNeighborDirection /= static_cast<float>(neighborWithinDistanceCount);
		avgNeighborLinearVelocity /= static_cast<float>(neighborWithinDistanceCount);
		avgNeighborAngularVelocity /= static_cast<float>(neighborWithinDistanceCount);

		float avgNeighborDist = glm::length(avgNeighborDirection);

		float forceScale;
		if (maxDist >= 0.0 && minDist >= 0.0 && maxDist != minDist) forceScale = 1.0 - (avgNeighborDist - minDist) / (maxDist - minDist);
		else forceScale = 1.0;

		//std::cout << "forceScale " << forceScale << " avgNeighborDist " << avgNeighborDist << " minDist " << minDist << " maxDist " << maxDist << "\n";

		//float forceScale = 1.0;

		glm::vec3 linearForce = avgNeighborLinearVelocity - partLinearVelocity;
		linearForce *= forceScale;
		linearForce *= linearAmount;
		linearForce *= amount;

		//std::cout << "lv " << partLinearVelocity << " nlv " << avgNeighborLinearVelocity << " lf " << linearForce << "\n";

		glm::vec3 angularForce = avgNeighborAngularVelocity - partAngularVelocity;
		angularForce *= forceScale;
		angularForce *= angularAmount;
		angularForce *= amount;

		//std::cout << "av " << partAngularVelocity << " nav " << avgNeighborAngularVelocity << " af " << angularForce << "\n";

		mParts[pI]->applyForce(linearForce);
		mParts[pI]->applyTorque(angularForce);
	}
}