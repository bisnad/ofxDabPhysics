/** \file dab_physics_neighbor_behavior.cpp
*/

#include "dab_physics_neighbor_behavior.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_simulation.h"

using namespace dab;
using namespace dab::physics;

NeighborBehavior::NeighborBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyPart>>& pNeighborParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyJoint>>& pNeighborJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors, const std::vector<std::shared_ptr<BodyMotor>>& pNeighborMotors)
	: Behavior(pName, pParts, pJoints, pMotors)
	, mNeighborParts(pNeighborParts)
	, mNeighborJoints(pNeighborJoints)
	, mNeighborMotors(pNeighborMotors)
{}

const std::vector<std::shared_ptr<BodyPart>>
NeighborBehavior::neighborParts() const
{
	return mNeighborParts;
}

const std::vector<std::shared_ptr<BodyJoint>>
NeighborBehavior::neighborJoints() const
{
	return mNeighborJoints;
}

const std::vector<std::shared_ptr<BodyMotor>>
NeighborBehavior::neighborMotors() const
{
	return mNeighborMotors;
}

void
NeighborBehavior::notifyParameterChange(const std::string& pParName)
{}