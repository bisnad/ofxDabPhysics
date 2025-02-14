/** \file dab_physics_universal_motor.h
*/

#pragma once

#include "dab_physics_body_motor.h"
#include "dab_value.h"

namespace dab
{

	namespace physics
	{

		class UniversalJoint;

		class UniversalMotor : public BodyMotor
		{
		public:
			friend class Simulation;

			UniversalMotor(std::shared_ptr<UniversalJoint> pJoint);
			virtual ~UniversalMotor();

			const std::array<bool, 3>& linearActive() const;
			const std::array<bool, 3>& angularActive() const;

			const std::array<float, 3>& linearLowerLimit() const;
			const std::array<float, 3>& linearUpperLimit() const;
			const std::array<float, 3>& angularLowerLimit() const;
			const std::array<float, 3>& angularUpperLimit() const;

			const std::array<float, 3>& linearStopERP() const;
			const std::array<float, 3>& angularStopERP() const;
			const std::array<float, 3>& linearStopCFM() const;
			const std::array<float, 3>& angularStopCFM() const;

			const std::array<float, 3>& maxLinearMotorForce() const;
			const std::array<float, 3>& maxAngularMotorForce() const;

			const std::array<float, 3>& linearVelocity() const;
			const std::array<float, 3>& angularVelocity() const;

			const std::array<bool, 3>& linearServoActive() const;
			const std::array<bool, 3>& angularServoActive() const;
			const std::array<float, 3>& linearServoTarget() const;
			const std::array<float, 3>& angularServoTarget() const;

			const std::array<bool, 3>& linearSpringActive() const;
			const std::array<bool, 3>& angularSpringActive() const;
			const std::array<float, 3>& linearSpringStiffness() const;
			const std::array<float, 3>& angularSpringStiffness() const;
			const std::array<float, 3>& linearSpringDamping() const;
			const std::array<float, 3>& angularSpringDamping() const;
			const std::array<float, 3>& linearSpringRestLength() const;
			const std::array<float, 3>& angularSpringRestLength() const;

			void setBounce(float pBounce);
			void setDamping(float pDamping);

			void setLinearActive(const std::array<bool, 3>& pLinearActive);
			void setAngularActive(const std::array<bool, 3>& pAngularActive);

			void setLinearLowerLimit(const std::array<float, 3>& pLinearLowerLimit);
			void setLinearUpperLimit(const std::array<float, 3>& pLinearUpperLimit);
			void setAngularLowerLimit(const std::array<float, 3>& pAngularLowerLimit);
			void setAngularUpperLimit(const std::array<float, 3>& pAngularUpperLimit);

			void setLinearStopERP(const std::array<float, 3>& pLinearStopERP);
			void setAngularStopERP(const std::array<float, 3>& pAngularStopERP);
			void setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM);
			void setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM);

			void setMaxLinearMotorForce(const std::array<float, 3>& pMaxLinearMotorForce);
			void setMaxAngularMotorForce(const std::array<float, 3>& pMaxAngularMotorForce);

			void setLinearVelocity(const std::array<float, 3>& pLinearVelocity);
			void setAngularVelocity(const std::array<float, 3>& pAngularVelocity);

			void setLinearServoActive(const std::array<bool, 3>& pLinearServoActive);
			void setAngularServoActive(const std::array<bool, 3>& pAngularServoActive);
			void setLinearServoTarget(const std::array<float, 3>& pLinearServoTarget);
			void setAngularServoTarget(const std::array<float, 3>& pAngularServoTarget);

			void setLinearSpringActive(const std::array<bool, 3>& pLinearSpringActive);
			void setAngularSpringActive(const std::array<bool, 3>& pAngularSpringActive);
			void setLinearSpringStiffness(const std::array<float, 3>& pLinearSpringStiffness);
			void setAngularSpringStiffness(const std::array<float, 3>& pAngularSpringStiffness);
			void setLinearSpringDamping(const std::array<float, 3>& pLinearSpringDamping);
			void setAngularSpringDamping(const std::array<float, 3>& pAngularSpringDamping);
			void setLinearSpringRestLength(const std::array<float, 3>& pLinearSpringRestLength);
			void setAngularSpringRestLength(const std::array<float, 3>& pAngularSpringRestLength);

			void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);

		protected:
			static std::array<bool, 3> sLinearActive;
			static std::array<bool, 3> sAngularActive;
			static std::array<float, 3> sMaxLinearMotorForce;
			static std::array<float, 3> sMaxAngularMotorForce;
			static std::array<float, 3> sLinearVelocity;
			static std::array<float, 3> sAngularVelocity;
			static std::array<bool, 3> sLinearServoActive;
			static std::array<bool, 3> sAngularServoActive;
			static std::array<float, 3> sLinearServoTarget;
			static std::array<float, 3> sAngularServoTarget;
			static std::array<bool, 3> sLinearSpringActive;
			static std::array<bool, 3> sAngularSpringActive;
			static std::array<float, 3> sLinearSpringStiffness;
			static std::array<float, 3> sAngularSpringStiffness;
			static std::array<float, 3> sLinearSpringDamping;
			static std::array<float, 3> sAngularSpringDamping;
			static std::array<float, 3> sLinearSpringRestLength;
			static std::array<float, 3> sAngularSpringRestLength;

			std::array<bool, 3> mLinearActive;
			std::array<bool, 3> mAngularActive;
			std::array<float, 3> mMaxLinearMotorForce;
			std::array<float, 3> mMaxAngularMotorForce;
			std::array<float, 3> mLinearVelocity;
			std::array<float, 3> mAngularVelocity;
			std::array<bool, 3> mLinearServoActive;
			std::array<bool, 3> mAngularServoActive;
			std::array<float, 3> mLinearServoTarget;
			std::array<float, 3> mAngularServoTarget;
			std::array<bool, 3> mLinearSpringActive;
			std::array<bool, 3> mAngularSpringActive;
			std::array<float, 3> mLinearSpringStiffness;
			std::array<float, 3> mAngularSpringStiffness;
			std::array<float, 3> mLinearSpringDamping;
			std::array<float, 3> mAngularSpringDamping;
			std::array<float, 3> mLinearSpringRestLength;
			std::array<float, 3> mAngularSpringRestLength;

			void initPhysics();
		};

	};

};
