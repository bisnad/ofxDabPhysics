/** \file dab_physics_behavior_force.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"
#include <array>

namespace dab
{
	namespace physics
	{

		class ForceBehavior : public Behavior
		{
		public:
			ForceBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			//Behavior* copy() const;

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mForceDirection;
			Value<float> mForceAmplitude;
			Value<float> mForceApplicationInterval;

			glm::vec3 mForce;

			double mLastForceApplicationTime;
		};

	};

};