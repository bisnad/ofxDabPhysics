/** \file dab_physics_behavior.cpp
*/

#include "dab_physics_behavior.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_simulation.h"

using namespace dab;
using namespace dab::physics;

Behavior::Behavior(const std::string& pName, const std::vector<std::shared_ptr<BodyPart>>& pParts, const std::vector<std::shared_ptr<BodyJoint>>& pJoints, const std::vector<std::shared_ptr<BodyMotor>>& pMotors)
	: mName(pName)
	, mBody(nullptr)
	, mParts(pParts)
	, mJoints(pJoints)
	, mMotors(pMotors)
	, mActive(1)
{
	mParameters["active"] = &mActive;
}

std::shared_ptr<Body> 
Behavior::body() const
{
	return mBody;
}

const std::string& 
Behavior::name() const
{
	return mName;
}

const std::vector<std::shared_ptr<BodyPart>> 
Behavior::parts() const
{
	return mParts;
}

const std::vector<std::shared_ptr<BodyJoint>> 
Behavior::joints() const
{
	return mJoints;
}

const std::vector<std::shared_ptr<BodyMotor>> 
Behavior::motors() const
{
	return mMotors;
}

const std::map<std::string, AbstractValue* > 
Behavior::parameters() const
{
	return mParameters;
}

void 
Behavior::notifyParameterChange(const std::string& pParName)
{}

void 
Behavior::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	if (mParameters.find(pParName) == mParameters.end()) throw Exception("Physic Error: behavior " + mName + " does not contain parameter " + pParName, __FILE__, __FUNCTION__, __LINE__);

	//if (mBody != nullptr)
	//{
	//	std::cout << "body " << mBody->name() << " ";
	//}
	//std::cout << " behavior " << name() << " parameter " << pParName << " value " << pValue << "\n";


	try
	{
		*(mParameters.at(pParName)) = pValue;
		notifyParameterChange(pParName);
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}