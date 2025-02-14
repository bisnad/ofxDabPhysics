/** \file dab_physics_behavior_torque.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class TorqueBehavior : public Behavior
		{
		public:
			TorqueBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mTorque;
			Value<float> mTorqueScale;
			Value<float> mApplicationInterval;

			std::vector<glm::vec3> mTorques;
			double mLastApplicationTime;
		};

	};

};