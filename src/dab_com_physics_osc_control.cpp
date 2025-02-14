/** \file dab_com_physics_osc_control.cpp
*/

#include "dab_com_physics_osc_control.h"
#include "dab_physics_simulation.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_universal_motor.h"
#include "dab_physics_behavior.h"
#include "ofVectorMath.h"
#include <array>
#include <memory>

using namespace dab;
using namespace dab::com;

PhysicsOscControl::PhysicsOscControl()
	: OscListener()
{}

PhysicsOscControl::~PhysicsOscControl()
{}

void
PhysicsOscControl::notify(std::shared_ptr<OscMessage> pMessage)
{
	mLock.lock();

	mMessageQueue.push_back(pMessage);
	if (mMessageQueue.size() > mMaxMessageQueueLength) mMessageQueue.pop_front();

	mLock.unlock();
}

void
PhysicsOscControl::update()
{
	mLock.lock();

	while (mMessageQueue.size() > 0)
	{
		std::shared_ptr< OscMessage > oscMessage = mMessageQueue[0];

		update(oscMessage);

		mMessageQueue.pop_front();
	}

	mLock.unlock();
}

void
PhysicsOscControl::update(std::shared_ptr<OscMessage> pMessage)
{
	try
	{
		std::string address = pMessage->address();

		//std::cout << "address " << address << "\n";

		const std::vector<_OscArg*>& arguments = pMessage->arguments();

		if (address.compare("/physics/sim/gravity") == 0) setSimGravity(arguments);
		else if (address.compare("/physics/sim/timestep") == 0) setSimTimeStep(arguments);
		else if (address.compare("/physics/sim/substeps") == 0) setSimSubSteps(arguments);
		else if (address.compare("/physics/part/mass") == 0) setPartMass(arguments);
		else if (address.compare("/physics/part/lineardamping") == 0) setPartLinearDamping(arguments);
		else if (address.compare("/physics/part/angulardamping") == 0) setPartAngularDamping(arguments);
		else if (address.compare("/physics/part/friction") == 0) setPartFriction(arguments);
		else if (address.compare("/physics/part/rollingfriction") == 0) setPartRollingFriction(arguments);
		else if (address.compare("/physics/part/restitution") == 0) setPartResitution(arguments);
		else if (address.compare("/physics/part/force") == 0) applyPartForce(arguments);
		else if (address.compare("/physics/joint/basis") == 0) setJointBasis(arguments);
		else if (address.compare("/physics/joint/prevpos") == 0) setJointPrevPos(arguments);
		else if (address.compare("/physics/joint/nextpos") == 0) setJointNextPos(arguments);
		else if (address.compare("/physics/joint/linearstoperp") == 0) setLinearStopERP(arguments);
		else if (address.compare("/physics/joint/angularstoperp") == 0) setAngularStopERP(arguments);
		else if (address.compare("/physics/joint/linearstopcfm") == 0) setLinearStopCFM(arguments);
		else if (address.compare("/physics/joint/angularstopcfm") == 0) setAngularStopCFM(arguments);
		else if (address.compare("/physics/joint/linearlowerlimit") == 0) setJointLinearLowerLimit(arguments);
		else if (address.compare("/physics/joint/linearupperlimit") == 0) setJointLinearUpperLimit(arguments);
		else if (address.compare("/physics/joint/angularlowerlimit") == 0) setJointAngularLowerLimit(arguments);
		else if (address.compare("/physics/joint/angularupperlimit") == 0) setJointAngularUpperLimit(arguments);
		else if (address.compare("/physics/motor/bounce") == 0) setMotorBounce(arguments);
		else if (address.compare("/physics/motor/damping") == 0) setMotorDamping(arguments);
		else if (address.compare("/physics/motor/linearactive") == 0) setMotorLinearActive(arguments);
		else if (address.compare("/physics/motor/angularactive") == 0) setMotorAngularActive(arguments);
		else if (address.compare("/physics/motor/maxlinearforce") == 0) setMotorMaxLinearForce(arguments);
		else if (address.compare("/physics/motor/maxangularforce") == 0) setMotorMaxAngularForce(arguments);
		else if (address.compare("/physics/motor/linearvelocity") == 0) setMotorLinearVelocity(arguments);
		else if (address.compare("/physics/motor/angularvelocity") == 0) setMotorAngularVelocity(arguments);
		else if (address.compare("/physics/motor/linearservoactive") == 0) setMotorLinearServoActive(arguments);
		else if (address.compare("/physics/motor/angularservoactive") == 0) setMotorAngularServoActive(arguments);
		else if (address.compare("/physics/motor/linearservoposition") == 0) setMotorLinearServoTarget(arguments);
		else if (address.compare("/physics/motor/angularservoposition") == 0) setMotorAngularServoTarget(arguments);
		else if (address.compare("/physics/motor/linearspringactive") == 0) setMotorLinearSpringActive(arguments);
		else if (address.compare("/physics/motor/angularspringactive") == 0) setMotorAngularSpringActive(arguments);
		else if (address.compare("/physics/motor/linearspringstiffness") == 0) setMotorLinearSpringStiffness(arguments);
		else if (address.compare("/physics/motor/angularspringstiffness") == 0) setMotorAngularSpringStiffness(arguments);
		else if (address.compare("/physics/motor/linearspringdamping") == 0) setMotorLinearSpringDamping(arguments);
		else if (address.compare("/physics/motor/angularspringdamping") == 0) setMotorAngularSpringDamping(arguments);
		else if (address.compare("/physics/motor/linearspringrestlength") == 0) setMotorLinearSpringRestLength(arguments);
		else if (address.compare("/physics/motor/angularspringrestlength") == 0) setMotorAngularSpringRestLength(arguments);
		else if (address.compare("/physics/behavior/parameter") == 0) setBehaviorParameter(arguments);
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}
}

void 
PhysicsOscControl::setSimGravity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 1)
	{
		float gravity = *pArgs[0];

		physics::Simulation& physics = physics::Simulation::get();

		physics.setGravity(glm::vec3(0.0, 0.0, gravity));
	}
	else if (pArgs.size() == 3)
	{
		glm::vec3 gravity;
		gravity[0] = *pArgs[0];
		gravity[1] = *pArgs[1];
		gravity[2] = *pArgs[2];

		physics::Simulation& physics = physics::Simulation::get();

		physics.setGravity(gravity);
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/sim/gravity", __FILE__, __FUNCTION__, __LINE__));
}

void 
PhysicsOscControl::setSimTimeStep(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 1)
	{
		float timstep = *pArgs[0];

		physics::Simulation& physics = physics::Simulation::get();

		physics.setTimeStep(timstep);
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/sim/timestep", __FILE__, __FUNCTION__, __LINE__));
}

void 
PhysicsOscControl::setSimSubSteps(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	if (pArgs.size() == 1)
	{
		int substeps = *pArgs[0];

		physics::Simulation& physics = physics::Simulation::get();

		physics.setSupSteps(substeps);
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/sim/substeps", __FILE__, __FUNCTION__, __LINE__));
}

void 
PhysicsOscControl::setPartMass(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float mass;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			mass = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			mass = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			mass = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/mass", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			// don't change mass of parts with zero mass
			// TODO: probably use another mechanism to prevent masses from being changed, such as a fixed mass attribute
			if (part->mass() == 0.0) continue;

			part->setMass(mass);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part mass", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setPartLinearDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float damping;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			damping = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			damping = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			damping = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/damping", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			part->setLinearDamping(damping);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part damping", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setPartAngularDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float damping;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			damping = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			damping = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			damping = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/angulardamping", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			part->setAngularDamping(damping);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part angular damping", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setPartFriction(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float friction;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			friction = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			friction = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			friction = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/friction", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			part->setFriction(friction);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part friction", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void
PhysicsOscControl::setPartRollingFriction(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float rollingFriction;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			rollingFriction = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			rollingFriction = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			rollingFriction = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/rollingfriction", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			part->setRollingFriction(rollingFriction);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part rolling friction", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setPartResitution(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		float restitution;

		if (pArgs.size() == 3) // body name / part name / mass
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			restitution = *pArgs[2];

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 2) // body or part name / mass
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			restitution = *pArgs[1];

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 1) // mass
		{
			restitution = *pArgs[0];

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/restitution", __FILE__, __FUNCTION__, __LINE__));

		for (auto part : parts)
		{
			part->setRestitution(restitution);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set part restitution", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::applyPartForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyPart>> parts;
		glm::vec3 force;
		glm::vec3 pos;
		bool useForcePos;

		if (pArgs.size() == 5) // body name, part name, force_x, force_y, force_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			force[0] = *pArgs[2];
			force[1] = *pArgs[3];
			force[2] = *pArgs[4];

			useForcePos = false;

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 8) // body name, part name, force_x, force_y, force_z, pos_x, pos_y, pos_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string partName = pArgs[1]->operator const std::string&();
			force[0] = *pArgs[2];
			force[1] = *pArgs[3];
			force[2] = *pArgs[4];
			pos[0] = *pArgs[5];
			pos[1] = *pArgs[6];
			pos[2] = *pArgs[7];

			useForcePos = true;

			parts.push_back(physics.body(bodyName)->part(partName));
		}
		else if (pArgs.size() == 4) // body or part name, force_x, force_y, force_z
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			force[0] = *pArgs[1];
			force[1] = *pArgs[2];
			force[2] = *pArgs[3];

			useForcePos = false;

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 7) // body or part name, force_x, force_y, force_z, pos_x, pos_y, pos_z
		{
			std::string bodyOrPartName = pArgs[0]->operator const std::string&();
			force[0] = *pArgs[1];
			force[1] = *pArgs[2];
			force[2] = *pArgs[3];
			pos[0] = *pArgs[4];
			pos[1] = *pArgs[5];
			pos[2] = *pArgs[6];

			useForcePos = true;

			parts = getParts(bodyOrPartName);
		}
		else if (pArgs.size() == 3) // force_x, force_y, force_z
		{
			force[0] = *pArgs[0];
			force[1] = *pArgs[1];
			force[2] = *pArgs[2];

			useForcePos = false;

			parts = physics.parts();
		}
		else if (pArgs.size() == 6) // force_x, force_y, force_z, pos_x, pos_y, pos_z
		{
			force[0] = *pArgs[0];
			force[1] = *pArgs[1];
			force[2] = *pArgs[2];

			pos[0] = *pArgs[3];
			pos[1] = *pArgs[4];
			pos[2] = *pArgs[5];

			useForcePos = true;

			parts = physics.parts();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/part/force", __FILE__, __FUNCTION__, __LINE__));
	
		if (useForcePos)
		{
			for (auto part : parts)
			{
				part->applyForce(force, pos);
			}
		}
		else
		{
			for (auto part : parts)
			{
				part->applyForce(force);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to apply part force", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointBasis(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> jointBasis;

		if (pArgs.size() == 5) // body_name, joint name, basis_x, basis_y, basis_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			jointBasis[0] = *pArgs[2];
			jointBasis[1] = *pArgs[3];
			jointBasis[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, basis_x, basis_y, basis_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			jointBasis[0] = *pArgs[1];
			jointBasis[1] = *pArgs[2];
			jointBasis[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // basis_x, basis_y, basis_z
		{
			jointBasis[0] = *pArgs[0];
			jointBasis[1] = *pArgs[1];
			jointBasis[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/basis", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setBasis(jointBasis);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint basis", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointPrevPos(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		glm::vec3 jointPos;

		if (pArgs.size() == 5) // body name, joint name, pos_x, pos_y, pos_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			jointPos[0] = *pArgs[2];
			jointPos[1] = *pArgs[3];
			jointPos[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or part name, pos_x, pos_y, pos_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			jointPos[0] = *pArgs[1];
			jointPos[1] = *pArgs[2];
			jointPos[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // pos_x, pos_y, pos_z
		{
			jointPos[0] = *pArgs[0];
			jointPos[1] = *pArgs[1];
			jointPos[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/prevpos", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setPrevJointPos(jointPos);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set previous joint position", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointNextPos(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		glm::vec3 jointPos;

		if (pArgs.size() == 5) // body name, joint name, pos_x, pos_y, pos_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			jointPos[0] = *pArgs[2];
			jointPos[1] = *pArgs[3];
			jointPos[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or part name, pos_x, pos_y, pos_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			jointPos[0] = *pArgs[1];
			jointPos[1] = *pArgs[2];
			jointPos[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // pos_x, pos_y, pos_z
		{
			jointPos[0] = *pArgs[0];
			jointPos[1] = *pArgs[1];
			jointPos[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/nextpos", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setNextJointPos(jointPos);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set next joint position", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setLinearStopERP(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> erp;

		if (pArgs.size() == 5) // body name, joint name, erp_x, erp_y, erp_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			erp[0] = *pArgs[2];
			erp[1] = *pArgs[3];
			erp[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, erp_x, erp_y, erp_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			erp[0] = *pArgs[1];
			erp[1] = *pArgs[2];
			erp[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // erp_x, erp_y, erp_z
		{
			erp[0] = *pArgs[0];
			erp[1] = *pArgs[1];
			erp[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/linearstoperp", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setLinearStopERP(erp);

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setLinearStopERP(erp);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint linear stop erp", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setAngularStopERP(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> erp;

		if (pArgs.size() == 5) // body name, joint name, erp_x, erp_y, erp_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			erp[0] = *pArgs[2];
			erp[1] = *pArgs[3];
			erp[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, erp_x, erp_y, erp_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			erp[0] = *pArgs[1];
			erp[1] = *pArgs[2];
			erp[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // erp_x, erp_y, erp_z
		{
			erp[0] = *pArgs[0];
			erp[1] = *pArgs[1];
			erp[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/angularstoperp", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setAngularStopERP(erp);

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setAngularStopERP(erp);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint angular stop erp", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setLinearStopCFM(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> cfm;

		if (pArgs.size() == 5) // body name, joint name, cfm_x, cfm_y, cfm_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			cfm[0] = *pArgs[2];
			cfm[1] = *pArgs[3];
			cfm[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, cfm_x, cfm_y, cfm_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			cfm[0] = *pArgs[1];
			cfm[1] = *pArgs[2];
			cfm[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // cfm_x, cfm_y, cfm_z
		{
			cfm[0] = *pArgs[0];
			cfm[1] = *pArgs[1];
			cfm[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/linearstopcfm", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setLinearStopCFM(cfm);

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setLinearStopCFM(cfm);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint linear stop cfm", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setAngularStopCFM(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> cfm;

		if (pArgs.size() == 5) // body name, joint name, cfm_x, cfm_y, cfm_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			cfm[0] = *pArgs[2];
			cfm[1] = *pArgs[3];
			cfm[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, cfm_x, cfm_y, cfm_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			cfm[0] = *pArgs[1];
			cfm[1] = *pArgs[2];
			cfm[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // cfm_x, cfm_y, cfm_z
		{
			cfm[0] = *pArgs[0];
			cfm[1] = *pArgs[1];
			cfm[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/angularstopcfm", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			joint->setAngularStopCFM(cfm);

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setAngularStopCFM(cfm);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint angular stop cfm", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointLinearLowerLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> limit;

		if (pArgs.size() == 5) // body name, joint name, limit_x, limit_y, limit_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			limit[0] = *pArgs[2];
			limit[1] = *pArgs[3];
			limit[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, limit_x, limit_y, limit_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			limit[0] = *pArgs[1];
			limit[1] = *pArgs[2];
			limit[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // limit_x, limit_y, limit_z
		{
			limit[0] = *pArgs[0];
			limit[1] = *pArgs[1];
			limit[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/linearlowerlimit", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			// TODO: check for other joint classes that also have setLinearLowerLimit function
			if (std::dynamic_pointer_cast<physics::UniversalJoint>(joint) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalJoint>(joint)->setLinearLowerLimit(limit);
			}

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setLinearLowerLimit(limit);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint linear lower limit", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointLinearUpperLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> limit;

		if (pArgs.size() == 5) // body name, joint name, limit_x, limit_y, limit_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			limit[0] = *pArgs[2];
			limit[1] = *pArgs[3];
			limit[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, limit_x, limit_y, limit_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			limit[0] = *pArgs[1];
			limit[1] = *pArgs[2];
			limit[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // limit_x, limit_y, limit_z
		{
			limit[0] = *pArgs[0];
			limit[1] = *pArgs[1];
			limit[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/linearupperlimit", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			// TODO: check for other joint classes that also have setLinearUpperLimit function
			if (std::dynamic_pointer_cast<physics::UniversalJoint>(joint) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalJoint>(joint)->setLinearUpperLimit(limit);
			}

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setLinearUpperLimit(limit);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint linear upper limit", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointAngularLowerLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> limit;

		if (pArgs.size() == 5) // body name, joint name, limit_x, limit_y, limit_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			limit[0] = *pArgs[2];
			limit[1] = *pArgs[3];
			limit[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, limit_x, limit_y, limit_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			limit[0] = *pArgs[1];
			limit[1] = *pArgs[2];
			limit[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // limit_x, limit_y, limit_z
		{
			limit[0] = *pArgs[0];
			limit[1] = *pArgs[1];
			limit[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/angularlowerlimit", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			// TODO: check for other joint classes that also have setAngularLowerLimit function
			if (std::dynamic_pointer_cast<physics::UniversalJoint>(joint) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalJoint>(joint)->setAngularLowerLimit(limit);
			}

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setAngularLowerLimit(limit);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint angular lower limit", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setJointAngularUpperLimit(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyJoint>> joints;
		std::array<float, 3> limit;

		if (pArgs.size() == 5) // body name, joint name, limit_x, limit_y, limit_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string jointName = pArgs[1]->operator const std::string&();
			limit[0] = *pArgs[2];
			limit[1] = *pArgs[3];
			limit[2] = *pArgs[4];

			joints.push_back(physics.body(bodyName)->joint(jointName));
		}
		else if (pArgs.size() == 4) // body or joint name, limit_x, limit_y, limit_z
		{
			std::string bodyOrJointName = pArgs[0]->operator const std::string&();
			limit[0] = *pArgs[1];
			limit[1] = *pArgs[2];
			limit[2] = *pArgs[3];

			joints = getJoints(bodyOrJointName);
		}
		else if (pArgs.size() == 3) // limit_x, limit_y, limit_z
		{
			limit[0] = *pArgs[0];
			limit[1] = *pArgs[1];
			limit[2] = *pArgs[2];

			joints = physics.joints();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/joint/angularupperlimit", __FILE__, __FUNCTION__, __LINE__));

		for (auto joint : joints)
		{
			// TODO: check for other joint classes that also have setAngularUpperLimit function
			if (std::dynamic_pointer_cast<physics::UniversalJoint>(joint) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalJoint>(joint)->setAngularUpperLimit(limit);
			}

			if (joint->motor() != nullptr && std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor()) != nullptr)
			{
				std::shared_ptr<physics::UniversalMotor> motor = std::dynamic_pointer_cast<physics::UniversalMotor>(joint->motor());
				motor->setAngularUpperLimit(limit);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set joint angular upper limit", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorBounce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		float bounce;

		if (pArgs.size() == 3) // body name, motor name, bounce
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			bounce = *pArgs[2];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 2) // body or motor name, bounce
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			bounce = *pArgs[1];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 1) // bounce
		{
			bounce = *pArgs[0];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/bounce", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			motor->setBounce(bounce);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor bounce", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		float damping;

		if (pArgs.size() == 3) // body name, motor name, bounce
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			damping = *pArgs[2];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 2) // body or motor name, bounce
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			damping = *pArgs[1];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 1) // bounce
		{
			damping = *pArgs[0];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/damping", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			motor->setDamping(damping);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor damping", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorMaxLinearForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> maxForce;

		if (pArgs.size() == 5) // body name, motor name, force_x, force_y, force_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			maxForce[0] = *pArgs[2];
			maxForce[1] = *pArgs[3];
			maxForce[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, force_x, force_y, force_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			maxForce[0] = *pArgs[1];
			maxForce[1] = *pArgs[2];
			maxForce[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // force_x, force_y, force_z
		{
			maxForce[0] = *pArgs[0];
			maxForce[1] = *pArgs[1];
			maxForce[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/maxlinearforce", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setMaxLinearMotorForce function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setMaxLinearMotorForce(maxForce);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor max linear force", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorMaxAngularForce(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> maxForce;

		if (pArgs.size() == 5) // body name, motor name, force_x, force_y, force_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			maxForce[0] = *pArgs[2];
			maxForce[1] = *pArgs[3];
			maxForce[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, force_x, force_y, force_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			maxForce[0] = *pArgs[1];
			maxForce[1] = *pArgs[2];
			maxForce[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // force_x, force_y, force_z
		{
			maxForce[0] = *pArgs[0];
			maxForce[1] = *pArgs[1];
			maxForce[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/maxangularforce", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setMaxLinearMotorForce function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setMaxAngularMotorForce(maxForce);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor max angular force", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearVelocity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> velocity;

		if (pArgs.size() == 5) // body name, motor name, velocity_x, velocity_y, velocity_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			velocity[0] = *pArgs[2];
			velocity[1] = *pArgs[3];
			velocity[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, velocity_x, velocity_y, velocity_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			velocity[0] = *pArgs[1];
			velocity[1] = *pArgs[2];
			velocity[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // velocity_x, velocity_y, velocity_z
		{
			velocity[0] = *pArgs[0];
			velocity[1] = *pArgs[1];
			velocity[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearvelocity", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearVelocity function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearVelocity(velocity);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear velocity", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularVelocity(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> velocity;

		if (pArgs.size() == 5) // body name, motor name, velocity_x, velocity_y, velocity_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			velocity[0] = *pArgs[2];
			velocity[1] = *pArgs[3];
			velocity[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, velocity_x, velocity_y, velocity_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			velocity[0] = *pArgs[1];
			velocity[1] = *pArgs[2];
			velocity[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // velocity_x, velocity_y, velocity_z
		{
			velocity[0] = *pArgs[0];
			velocity[1] = *pArgs[1];
			velocity[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularvelocity", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularVelocity function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularVelocity(velocity);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular velocity", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearServoActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearservoactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearServoActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearServoActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear servo active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularServoActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularservoactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularServoActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularServoActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular servo active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearServoTarget(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> position;

		if (pArgs.size() == 5) // body name, motor name, pos_x, pos_y, pos_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			position[0] = *pArgs[2];
			position[1] = *pArgs[3];
			position[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, pos_x, pos_y, pos_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			position[0] = *pArgs[1];
			position[1] = *pArgs[2];
			position[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // pos_x, pos_y, pos_z
		{
			position[0] = *pArgs[0];
			position[1] = *pArgs[1];
			position[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearservoposition", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearServoTarget function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearServoTarget(position);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear servo position", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularServoTarget(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> position;

		if (pArgs.size() == 5) // body name, motor name, pos_x, pos_y, pos_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			position[0] = *pArgs[2];
			position[1] = *pArgs[3];
			position[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, pos_x, pos_y, pos_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			position[0] = *pArgs[1];
			position[1] = *pArgs[2];
			position[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // pos_x, pos_y, pos_z
		{
			position[0] = *pArgs[0];
			position[1] = *pArgs[1];
			position[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularservoposition", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularServoTarget function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularServoTarget(position);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular servo position", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearSpringActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearspringactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearSpringActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearSpringActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear spring active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularSpringActive(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<bool, 3> active;
		std::array<int, 3> tmp;

		if (pArgs.size() == 5) // body name, motor name, active_x, active_y, active_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			tmp[0] = *pArgs[2];
			tmp[1] = *pArgs[3];
			tmp[2] = *pArgs[4];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, active_x, active_y, active_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			tmp[0] = *pArgs[1];
			tmp[1] = *pArgs[2];
			tmp[2] = *pArgs[3];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // active_x, active_y, active_z
		{
			tmp[0] = *pArgs[0];
			tmp[1] = *pArgs[1];
			tmp[2] = *pArgs[2];
			active[0] = static_cast<bool>(tmp[0]);
			active[1] = static_cast<bool>(tmp[1]);
			active[2] = static_cast<bool>(tmp[2]);

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularspringactive", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularSpringActive function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularSpringActive(active);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular spring active", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearSpringStiffness(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> stiffness;

		if (pArgs.size() == 5) // body name, motor name, stiff_x, stiff_y, stiff_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			stiffness[0] = *pArgs[2];
			stiffness[1] = *pArgs[3];
			stiffness[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, stiff_x, stiff_y, stiff_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			stiffness[0] = *pArgs[1];
			stiffness[1] = *pArgs[2];
			stiffness[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // stiff_x, stiff_y, stiff_z
		{
			stiffness[0] = *pArgs[0];
			stiffness[1] = *pArgs[1];
			stiffness[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearspringstiffness", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearSpringStiffness function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearSpringStiffness(stiffness);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear spring stiffness", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularSpringStiffness(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> stiffness;

		if (pArgs.size() == 5) // body name, motor name, stiff_x, stiff_y, stiff_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			stiffness[0] = *pArgs[2];
			stiffness[1] = *pArgs[3];
			stiffness[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, stiff_x, stiff_y, stiff_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			stiffness[0] = *pArgs[1];
			stiffness[1] = *pArgs[2];
			stiffness[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // stiff_x, stiff_y, stiff_z
		{
			stiffness[0] = *pArgs[0];
			stiffness[1] = *pArgs[1];
			stiffness[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularspringstiffness", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularSpringStiffness function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularSpringStiffness(stiffness);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular spring stiffness", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearSpringDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> damping;

		if (pArgs.size() == 5) // body name, motor name, damping_x, damping_y, damping_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			damping[0] = *pArgs[2];
			damping[1] = *pArgs[3];
			damping[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, damping_x, damping_y, damping_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			damping[0] = *pArgs[1];
			damping[1] = *pArgs[2];
			damping[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // damping_x, damping_y, damping_z
		{
			damping[0] = *pArgs[0];
			damping[1] = *pArgs[1];
			damping[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearspringdamping", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearSpringDamping function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearSpringDamping(damping);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear spring damping", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularSpringDamping(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> damping;

		if (pArgs.size() == 5) // body name, motor name, damping_x, damping_y, damping_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			damping[0] = *pArgs[2];
			damping[1] = *pArgs[3];
			damping[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, damping_x, damping_y, damping_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			damping[0] = *pArgs[1];
			damping[1] = *pArgs[2];
			damping[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // damping_x, damping_y, damping_z
		{
			damping[0] = *pArgs[0];
			damping[1] = *pArgs[1];
			damping[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularspringdamping", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularSpringDamping function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularSpringDamping(damping);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular spring damping", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorLinearSpringRestLength(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> restLength;

		if (pArgs.size() == 5) // body name, motor name, length_x, length_y, length_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			restLength[0] = *pArgs[2];
			restLength[1] = *pArgs[3];
			restLength[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, length_x, length_y, length_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			restLength[0] = *pArgs[1];
			restLength[1] = *pArgs[2];
			restLength[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // length_x, length_y, length_z
		{
			restLength[0] = *pArgs[0];
			restLength[1] = *pArgs[1];
			restLength[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/linearspringrestlength", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setLinearSpringRestLength function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setLinearSpringRestLength(restLength);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor linear spring rest length", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setMotorAngularSpringRestLength(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	try
	{
		physics::Simulation& physics = physics::Simulation::get();
		std::vector<std::shared_ptr<physics::BodyMotor>> motors;
		std::array<float, 3> restLength;

		if (pArgs.size() == 5) // body name, motor name, length_x, length_y, length_z
		{
			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string motorName = pArgs[1]->operator const std::string&();
			restLength[0] = *pArgs[2];
			restLength[1] = *pArgs[3];
			restLength[2] = *pArgs[4];

			motors.push_back(physics.body(bodyName)->motor(motorName));
		}
		else if (pArgs.size() == 4) // body or motor name, length_x, length_y, length_z
		{
			std::string bodyOrMotorName = pArgs[0]->operator const std::string&();
			restLength[0] = *pArgs[1];
			restLength[1] = *pArgs[2];
			restLength[2] = *pArgs[3];

			motors = getMotors(bodyOrMotorName);
		}
		else if (pArgs.size() == 3) // length_x, length_y, length_z
		{
			restLength[0] = *pArgs[0];
			restLength[1] = *pArgs[1];
			restLength[2] = *pArgs[2];

			motors = physics.motors();
		}
		else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/motor/angularspringrestlength", __FILE__, __FUNCTION__, __LINE__));

		for (auto motor : motors)
		{
			// TODO: check for other joint classes that also have setAngularSpringRestLength function
			if (std::dynamic_pointer_cast<physics::UniversalMotor>(motor) != nullptr)
			{
				std::dynamic_pointer_cast<physics::UniversalMotor>(motor)->setAngularSpringRestLength(restLength);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Osc Error: failed to set motor angular spring rest length", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
PhysicsOscControl::setBehaviorParameter(const std::vector<_OscArg*>& pArgs) throw (dab::Exception)
{
	int argCount = pArgs.size();

	if (argCount > 3 && pArgs[2]->oscType() == OSC_TYPE_STRING && pArgs[3]->oscType() == OSC_TYPE_FLOAT) // body name, behavior name, parameter name, parameter value(s)
	{
		try
		{
			physics::Simulation& physics = physics::Simulation::get();

			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string behaviorName = pArgs[1]->operator const std::string&();

			std::shared_ptr<physics::Behavior> behavior = physics.behavior(bodyName, behaviorName);

			std::string parameterName = pArgs[2]->operator const std::string&();

			std::vector<float> parameterValues;
			float parameterValue;

			//std::cout << "PhysicsOscControl::setBehaviorParameter Behavior " << behaviorName << " parameter " << parameterName << " set\n";

			for (int aI = 3; aI < argCount; ++aI)
			{
				parameterValue = *pArgs[aI];
				parameterValues.push_back(parameterValue);
			}

			behavior->set(parameterName, parameterValues);
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set behavior parameter", __FILE__, __FUNCTION__, __LINE__);
			throw e;
		}
	}
	else if (argCount > 2 && pArgs[1]->oscType() == OSC_TYPE_STRING && pArgs[2]->oscType() == OSC_TYPE_FLOAT) // behavior name, parameter name, parameter value(s)
	{
		try
		{
			physics::Simulation& physics = physics::Simulation::get();

			std::string behaviorName = pArgs[0]->operator const std::string&();
			std::string parameterName = pArgs[1]->operator const std::string&();

			std::vector<float> parameterValues;
			float parameterValue;

			//std::cout << "PhysicsOscControl::setBehaviorParameter Behavior " << behaviorName << " parameter " << parameterName << " set: ";

			for (int aI = 2; aI < argCount; ++aI)
			{
				parameterValue = *pArgs[aI];
				parameterValues.push_back(parameterValue);

				//std::cout << parameterValue << " ";
			}
			//std::cout << "\n";

			const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

			for (auto body : bodies)
			{
				if (body->behaviorExists(behaviorName) == false) continue;
				body->behavior(behaviorName)->set(parameterName, parameterValues);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set behavior parameter", __FILE__, __FUNCTION__, __LINE__);
			throw e;
		}
	}
	else if (argCount > 3 && pArgs[2]->oscType() == OSC_TYPE_STRING && pArgs[3]->oscType() == OSC_TYPE_INT32) // body name, behavior name, parameter name, parameter value(s)
	{
		try
		{
			physics::Simulation& physics = physics::Simulation::get();

			std::string bodyName = pArgs[0]->operator const std::string&();
			std::string behaviorName = pArgs[1]->operator const std::string&();
			std::shared_ptr<physics::Behavior> behavior = physics.behavior(bodyName, behaviorName);

			std::string parameterName = pArgs[2]->operator const std::string&();

			// special case: parameterName == active requires boolean value
			if(parameterName == "active")
			{
				std::vector<bool> parameterValues;
				int parameterValue = *pArgs[3];
				parameterValues.push_back(static_cast<bool>(parameterValue));

				behavior->set(parameterName, parameterValues);
			}
			else
			{ 
				std::vector<int> parameterValues;
				int parameterValue;

				//std::cout << "PhysicsOscControl::setBehaviorParameter Behavior " << behaviorName << " parameter " << parameterName << " set\n";

				for (int aI = 3; aI < argCount; ++aI)
				{
					parameterValue = *pArgs[aI];
					parameterValues.push_back(parameterValue);
				}

				behavior->set(parameterName, parameterValues);
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set behavior parameter", __FILE__, __FUNCTION__, __LINE__);
			throw e;
		}
	}
	else if (argCount > 2 && pArgs[1]->oscType() == OSC_TYPE_STRING && pArgs[2]->oscType() == OSC_TYPE_INT32) // behavior name, parameter name, parameter value(s)
	{
		try
		{
			physics::Simulation& physics = physics::Simulation::get();

			std::string behaviorName = pArgs[0]->operator const std::string&();
			std::string parameterName = pArgs[1]->operator const std::string&();

			// special case: parameterName == active: requires boolean value
			if (parameterName == "active")
			{
				std::vector<bool> parameterValues;
				int parameterValue = *pArgs[2];
				parameterValues.push_back(static_cast<bool>(parameterValue));

				const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

				for (auto body : bodies)
				{
					if (body->behaviorExists(behaviorName) == false) continue;
					body->behavior(behaviorName)->set(parameterName, parameterValues);
				}
			}
			else
			{
				std::vector<int> parameterValues;
				int parameterValue;

				//std::cout << "PhysicsOscControl::setBehaviorParameter Behavior " << behaviorName << " parameter " << parameterName << " set\n";

				for (int aI = 2; aI < argCount; ++aI)
				{
					parameterValue = *pArgs[aI];
					parameterValues.push_back(parameterValue);
				}

				const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

				for (auto body : bodies)
				{
					if (body->behaviorExists(behaviorName) == false) continue;
					body->behavior(behaviorName)->set(parameterName, parameterValues);
				}
			}
		}
		catch (dab::Exception& e)
		{
			e += dab::Exception("Osc Error: failed to set behavior parameter", __FILE__, __FUNCTION__, __LINE__);
			throw e;
		}
	}
	else throw(dab::Exception("Osc Error: Wrong Parameters for /physics/behavior/parameter", __FILE__, __FUNCTION__, __LINE__));
}

std::vector<std::shared_ptr<physics::BodyPart>> 
PhysicsOscControl::getParts(const std::string& pName)
{
	physics::Simulation& physics = physics::Simulation::get();

	// check if pName is name of a body, if yes, return all parts of that body
	if (physics.bodyExists(pName)) return physics.body(pName)->parts();

	// check if pName is name of a part, if yes, return all parts with that name from all bodies
	std::vector<std::shared_ptr<physics::BodyPart>> _parts;
	const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

	int bodyCount = bodies.size();

	for (int bI = 0; bI < bodyCount; ++bI)
	{
		if (bodies[bI]->partExists(pName)) _parts.push_back(bodies[bI]->part(pName));
	}

	return _parts;
}

std::vector<std::shared_ptr<physics::BodyJoint>> 
PhysicsOscControl::getJoints(const std::string& pName)
{
	physics::Simulation& physics = physics::Simulation::get();

	// check if pName is name of a body, if yes, return all joints of that body
	if (physics.bodyExists(pName)) return physics.body(pName)->joints();

	// check if pName is name of a joint, if yes, return all joints with that name from all bodies
	std::vector<std::shared_ptr<physics::BodyJoint>> _joints;
	const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

	int bodyCount = bodies.size();

	for (int bI = 0; bI < bodyCount; ++bI)
	{
		if (bodies[bI]->jointExists(pName)) _joints.push_back(bodies[bI]->joint(pName));
	}

	return _joints;
}

std::vector<std::shared_ptr<physics::BodyMotor>> 
PhysicsOscControl::getMotors(const std::string& pName)
{
	physics::Simulation& physics = physics::Simulation::get();

	// check if pName is name of a body, if yes, return all motors of that body
	if (physics.bodyExists(pName)) return physics.body(pName)->motors();

	// check if pName is name of a motor, if yes, return all motors with that name from all bodies
	std::vector<std::shared_ptr<physics::BodyMotor>> _motors;
	const std::vector<std::shared_ptr<physics::Body>>& bodies = physics.bodies();

	int bodyCount = bodies.size();

	for (int bI = 0; bI < bodyCount; ++bI)
	{
		if (bodies[bI]->motorExists(pName)) _motors.push_back(bodies[bI]->motor(pName));
	}

	return _motors;
}