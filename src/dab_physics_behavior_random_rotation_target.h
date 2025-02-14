/** \file dab_physics_behavior_random_rotation_target.h
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

		class RandomRotationTargetBehavior : public Behavior
		{
		public:
			RandomRotationTargetBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			//Behavior* copy() const;

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float,3> > mTargetMin;
			Value< std::array<float,3> > mTargetMax;
			std::vector< std::array<float, 3> > mTargets;

			Value<float> mSpeed;
			Value<float> mApplicationInterval;
			Value<float> mRandomizationInterval;

			double mLastRandomizationTime;
			double mLastApplicationTime;
		};

	};

};