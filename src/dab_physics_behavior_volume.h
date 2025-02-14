/** \file dab_physics_behavior_volume.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class VolumeBehavior : public Behavior
		{
		public:
			VolumeBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value<float> mMaxDistance;
			Value<float> mMinForceAmplitude;
			Value<float> mMaxForceAmplitude;
			Value<float> mForceApplicationInterval;

			std::vector<glm::vec3> mForces;
			double mLastForceApplicationTime;
		};

	};

};