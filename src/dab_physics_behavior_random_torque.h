/** \file dab_physics_behavior_random_torque.h
*/

#pragma once

#include "dab_physics_behavior.h"
#include "ofVectorMath.h"

#include <array>

namespace dab
{
	namespace physics
	{

		class RandomTorqueBehavior : public Behavior
		{
		public:
			RandomTorqueBehavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors);

			//Behavior* copy() const;

			void update() throw (Exception);

		protected:

			void init();
			void notifyParameterChange(const std::string& pParName);

			Value< std::array<float, 3> > mTorqueMin;
			Value< std::array<float, 3> > mTorqueMax;
			Value<float> mTorqueScale;
			Value<float> mApplicationInterval;
			Value<float> mRandomizationInterval;

			std::vector<glm::vec3> mTorques;
			double mLastApplicationTime;
			double mLastRandomizationTime;
		};

	};

};