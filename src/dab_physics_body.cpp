/** \file dab_physics_body.cpp
*/

#include "dab_physics_body.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_behavior.h"
#include "dab_physics_simulation.h"

using namespace dab;
using namespace dab::physics;

Body::Body(const std::string& pName)
	: mName(pName)
	, mRootPart(nullptr)
{
	//initBody();
}

Body::~Body()
{
	std::cout << "delete body " << mName << "\n";
}

const std::string& 
Body::name() const
{
	return mName;
}

std::shared_ptr<BodyPart> 
Body::rootPart() const
{
	return mRootPart;
}

const std::vector<std::shared_ptr<BodyPart>>& 
Body::parts() const
{
	return mParts.values();
}

const std::vector<std::shared_ptr<BodyJoint>>& 
Body::joints() const
{
	return mJoints.values();
}

const std::vector<std::shared_ptr<BodyMotor>>& 
Body::motors() const
{
	return mMotors.values();
}

const std::vector<std::shared_ptr<Behavior>>& 
Body::behaviors() const
{
	return mBehaviors.values();
}

bool 
Body::partExists(const std::string& pPartName) const
{
	return mParts.contains(pPartName);
}

bool 
Body::jointExists(const std::string& pJointName) const
{
	return mJoints.contains(pJointName);
}

bool 
Body::motorExists(const std::string& pMotorName) const
{
	return mMotors.contains(pMotorName);
}

bool 
Body::behaviorExists(const std::string& pBehaviorName) const
{
	return mBehaviors.contains(pBehaviorName);
}

std::shared_ptr<BodyPart> 
Body::part(const std::string& pPartName) throw (dab::Exception)
{
	try
	{
		return mParts[pPartName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve part, body " + mName + " has no part " + pPartName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyPart> 
Body::part(const std::string& pPartName) const throw (dab::Exception)
{
	try
	{
		return mParts[pPartName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve part, body " + mName + " has no part " + pPartName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<BodyJoint> 
Body::joint(const std::string& pJointName) throw (dab::Exception)
{
	try
	{
		return mJoints[pJointName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve joint, body " + mName + " has no joint " + pJointName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyJoint> 
Body::joint(const std::string& pJointName) const throw (dab::Exception)
{
	try
	{
		return mJoints[pJointName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve joint, body " + mName + " has no joint " + pJointName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<BodyMotor> 
Body::motor(const std::string& pMotorName) throw (dab::Exception)
{
	try
	{
		return mMotors[pMotorName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve motor, body " + mName + " has no motor " + pMotorName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyMotor> 
Body::motor(const std::string& pMotorName) const throw (dab::Exception)
{
	try
	{
		return mMotors[pMotorName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve motor, body " + mName + " has no motor " + pMotorName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<Behavior> 
Body::behavior(const std::string& pBehaviorName) throw (dab::Exception)
{
	try
	{
		return mBehaviors[pBehaviorName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve behavior, body " + mName + " has no behavior " + pBehaviorName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<Behavior> 
Body::behavior(const std::string& pBehaviorName) const throw (dab::Exception)
{
	try
	{
		return mBehaviors[pBehaviorName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve behavior, body " + mName + " has no behavior " + pBehaviorName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
Body::setRootPart(std::shared_ptr<BodyPart> pPart) throw (dab::Exception)
{
	if (mRootPart != nullptr) throw dab::Exception("Physics Error: Body " + mName + " already has root part", __FILE__, __FUNCTION__, __LINE__);

	mRootPart = pPart;
	addPart(mRootPart);
}

void
Body::addPart(std::shared_ptr<BodyPart> pPart) throw (dab::Exception)
{
	if(mParts.contains(pPart->name()))  throw dab::Exception("Physics Error: Body " + mName + " already has part " + pPart->name(), __FILE__, __FUNCTION__, __LINE__);

	mParts.insert(pPart->name(), pPart);
}

void
Body::addJoint(std::shared_ptr<BodyJoint> pJoint) throw (dab::Exception)
{
	if (mJoints.contains(pJoint->name()))  throw dab::Exception("Physics Error: Body " + mName + " already has joint " + pJoint->name(), __FILE__, __FUNCTION__, __LINE__);

	mJoints.insert(pJoint->name(), pJoint);
}

void
Body::addMotor(std::shared_ptr<BodyMotor> pMotor)
{
	if (mMotors.contains(pMotor->name()))  throw dab::Exception("Physics Error: Body " + mName + " already has motor " + pMotor->name(), __FILE__, __FUNCTION__, __LINE__);

	mMotors.insert(pMotor->name(), pMotor);
}

void
Body::addBehavior(std::shared_ptr<Behavior> pBehavior)
{
	if (mBehaviors.contains(pBehavior->name()))  throw dab::Exception("Physics Error: Body " + mName + " already has behavior " + pBehavior->name(), __FILE__, __FUNCTION__, __LINE__);

	mBehaviors.insert(pBehavior->name(), pBehavior);
}

//void 
//Body::initBody()
//{
//	addPart(mRootPart);
//}

//void
//Body::addPart(std::shared_ptr<BodyPart> pPart)
//{
//	std::cout << "Body::addPart " << pPart->name() << "\n";
//
//	mParts.insert(pPart->name(), pPart);
//	
//	const std::vector<std::shared_ptr<BodyJoint>>& joints = pPart->nextJoints();
//
//	std::cout << "next joint count " << joints.size() << "\n";
//	std::cout << "prev joint count " << pPart->prevJoints().size() << "\n";
//
//	for (auto joint : joints)
//	{
//		addJoint(joint);
//	}
//}

//void
//Body::addJoint(std::shared_ptr<BodyJoint> pJoint)
//{
//	std::cout << "Body::addJoint " << pJoint->name() << "\n";
//
//	mJoints.insert(pJoint->name(), pJoint);
//	if (pJoint->motor() != nullptr) addMotor(pJoint->motor());
//
//	addPart(pJoint->nextPart());
//}

//void
//Body::addMotor(std::shared_ptr<BodyMotor> pMotor)
//{
//	std::cout << "Body::addMotor " << pMotor->name() << "\n";
//
//	mMotors.insert(pMotor->name(), pMotor);
//}