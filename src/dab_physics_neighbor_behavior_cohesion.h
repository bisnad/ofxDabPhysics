/** \file dab_physics_neighbor_behavior_cohesion.h
*/

#pragma once

#include "dab_physics_neighbor_behavior.h"
#include "ofVectorMath.h"

namespace dab
{

namespace physics
{

class CohesionBehavior : public NeighborBehavior
{
public:
	CohesionBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyPart>>& pNeighborParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyJoint>>& pNeighborJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors, const std::vector<std::shared_ptr<BodyMotor>>& pNeighborMotors);

	void update() throw (Exception);

protected:

	void init();
	void notifyParameterChange(const std::string& pParName);

	Value<float> mMinDist;
	Value<float> mMaxDist;
	Value<float> mAmount;
};

};

};