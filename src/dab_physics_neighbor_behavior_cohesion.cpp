/** \file dab_physics_neighbor_behavior_cohesion.cpp
*/

#include "dab_physics_neighbor_behavior_cohesion.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"

using namespace dab;
using namespace dab::physics;

CohesionBehavior::CohesionBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyPart>>& pNeighborParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyJoint>>& pNeighborJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors, const std::vector<std::shared_ptr<BodyMotor>>& pNeighborMotors)
	: NeighborBehavior(pName, pParts, pNeighborParts, pJoints, pNeighborJoints, pMotors, pNeighborMotors)
	, mMinDist(0.0)
	, mMaxDist(10.0)
	, mAmount(1.0)
{
	init();
}

void
CohesionBehavior::init()
{
	mParameters["minDist"] = &mMinDist;
	mParameters["maxDist"] = &mMaxDist;
	mParameters["amount"] = &mAmount;

	//// debug
	//std::cout << "CohesionBehavior parts: ";
	//int partCount = mParts.size();
	//for (int pI = 0; pI < partCount; ++pI)
	//{
	//	std::cout << mParts[pI]->body()->name() << "-" << mParts[pI]->name() << " ";
	//}
	//std::cout << "\n";
	//int neighborPartCount = mNeighborParts.size();
	//std::cout << "CohesionBehavior nparts: ";
	//for (int npI = 0; npI < neighborPartCount; ++npI)
	//{
	//	std::cout << mNeighborParts[npI]->body()->name() << "-" << mNeighborParts[npI]->name() << " ";
	//}
	//std::cout << "\n";
	//// debug done
}

void
CohesionBehavior::notifyParameterChange(const std::string& pParName)
{
	//std::cout << "parameter " << pParName << " set to " << *(mParameters[pParName]) << "\n";
}

void
CohesionBehavior::update()
{
	bool active = mActive;
	if (active == false) return;

	float minDist = mMinDist;
	float maxDist = mMaxDist;
	float amount = mAmount;

	int partCount = mParts.size();
	int neighborPartCount = mNeighborParts.size();

	for (int pI = 0; pI < partCount; ++pI)
	{
		const glm::vec3& partPos = mParts[pI]->position();

		glm::vec3 avgNeighborDirection(0.0);
		int neighborWithinDistanceCount = 0;

		for (int npI = 0; npI < neighborPartCount; ++npI)
		{
			const glm::vec3& neighborPartPos = mNeighborParts[npI]->position();
			glm::vec3 neighborDir = neighborPartPos - partPos;
			float neighborDist = glm::length(neighborDir);

			if (neighborDist < minDist || neighborDist > mMaxDist) continue;

			avgNeighborDirection += neighborDir;
			neighborWithinDistanceCount++;
		}

		if (neighborWithinDistanceCount == 0) continue;

		avgNeighborDirection /= static_cast<float>(neighborWithinDistanceCount);
		float avgNeighborDist = glm::length(avgNeighborDirection);

		float forceScale;
		if (maxDist > 0.0 && minDist > 0.0) forceScale = (avgNeighborDist - minDist) / (maxDist - minDist);
		else forceScale = 1.0;

		glm::vec3 force = glm::normalize(avgNeighborDirection);
		force *= forceScale;
		force *= amount;

		mParts[pI]->applyForce(force);
	}
}