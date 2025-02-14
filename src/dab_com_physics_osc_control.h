/** \file dab_com_physics_osc_control.h
*/

#pragma once

#include "dab_singleton.h"
#include "dab_osc_receiver.h"
#include <mutex>

namespace dab
{

namespace physics
{
	class BodyPart;
	class BodyJoint;
	class BodyMotor;
}

namespace com
{

class PhysicsOscControl : public OscListener
{
public:
	PhysicsOscControl();
	~PhysicsOscControl();

	void notify(std::shared_ptr<OscMessage> pMessage);
	void update();
	void update(std::shared_ptr<OscMessage> pMessage);

protected:
	unsigned int mMaxMessageQueueLength = 2048;
	std::deque< std::shared_ptr<OscMessage> > mMessageQueue;

	// simulation control
	void setSimGravity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setSimTimeStep(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setSimSubSteps(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	// part control
	void setPartMass(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setPartLinearDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setPartAngularDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setPartFriction(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setPartRollingFriction(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setPartResitution(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	void applyPartForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	// joint control
	void setJointBasis(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointPrevPos(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointNextPos(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setLinearStopERP(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setAngularStopERP(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setLinearStopCFM(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setAngularStopCFM(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointLinearLowerLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointLinearUpperLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointAngularLowerLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setJointAngularUpperLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	// motor control
	void setMotorBounce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorMaxLinearForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorMaxAngularForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearVelocity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularVelocity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearServoActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularServoActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearServoTarget(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularServoTarget(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);	
	void setMotorLinearSpringActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularSpringActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearSpringStiffness(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularSpringStiffness(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearSpringDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularSpringDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorLinearSpringRestLength(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);
	void setMotorAngularSpringRestLength(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	// behavior control
	void setBehaviorParameter(const std::vector<_OscArg*>& pArgs) throw (dab::Exception);

	// helper functions
	std::vector<std::shared_ptr<physics::BodyPart>> getParts(const std::string& pName);
	std::vector<std::shared_ptr<physics::BodyJoint>> getJoints(const std::string& pName);
	std::vector<std::shared_ptr<physics::BodyMotor>> getMotors(const std::string& pName);

	std::mutex mLock;
};

};

};
