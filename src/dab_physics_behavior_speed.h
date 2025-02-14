/** \file dab_physics_behavior_speed.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class SpeedBehavior : public Behavior
		{
		public:
			SpeedBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value<float> mSpeed;
			Value<float> mAmount;

		};

	};

};