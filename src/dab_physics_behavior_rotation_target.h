/** \file dab_physics_behavior_rotation_target.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{
		class BodyMotor;

		class RotationTargetBehavior : public Behavior
		{
		public:
			RotationTargetBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mTarget;
			Value<float> mSpeed;
			Value<float> mApplicationInterval;

			double mLastApplicationTime;
		};

	};

};