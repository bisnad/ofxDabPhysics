/** \file dab_physics_behavior_random_force.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class RandomForceBehavior : public Behavior
		{
		public:
			RandomForceBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			//Behavior* copy() const;

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mForceDirectionsMin;
			Value< std::array<float, 3> > mForceDirectionsMax;
			Value<float> mForceAmplitudeMin;
			Value<float> mForceAmplitudeMax;
			Value<float> mForceApplicationInterval;
			Value<float> mForceChangeInterval;

			std::vector<glm::vec3> mForces;
			double mLastForceApplicationTime;
			double mLastForceChangeTime;
		};

	};

};