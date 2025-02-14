/** \file dab_physics_behavior_target_attraction.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class TargetAttractionBehavior : public Behavior
		{
		public:
			TargetAttractionBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mTargetPosition;
			Value<float> mMinDistance;
			Value<float> mMaxDistance;
			Value<float> mForceAmplitudeMin;
			Value<float> mForceAmplitudeMax;
			Value<float> mForceApplicationInterval;

			std::vector<glm::vec3> mForces;
			double mLastForceApplicationTime;
		};

	};

};