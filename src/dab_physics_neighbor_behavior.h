/** \file dab_physics_neighbor_behavior.h
*/

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "dab_value.h"
#include "dab_exception.h"
#include "dab_physics_behavior.h"

namespace dab
{

	namespace physics
	{

		class Simulation;
		class Body;
		class BodyPart;
		class BodyJoint;
		class BodyMotor;

		class NeighborBehavior : public Behavior
		{
		public:
			friend class Simulation;

			NeighborBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyPart>>& pNeighborParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyJoint>>& pNeighborJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors, const std::vector<std::shared_ptr<BodyMotor>>& pNeighborMotors);

			const std::vector<std::shared_ptr<BodyPart>> neighborParts() const;
			const std::vector<std::shared_ptr<BodyJoint>> neighborJoints() const;
			const std::vector<std::shared_ptr<BodyMotor>> neighborMotors() const;

		protected:
			virtual void notifyParameterChange(const std::string& pParName);

			std::vector<std::shared_ptr<BodyPart>> mNeighborParts;
			std::vector<std::shared_ptr<BodyJoint>> mNeighborJoints;
			std::vector<std::shared_ptr<BodyMotor>> mNeighborMotors;
		};
	};
};
