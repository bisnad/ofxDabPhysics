/** \file dab_physics_body_motor.cpp
*/

#include "dab_physics_body_motor.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body.h"

using namespace dab;
using namespace dab::physics;

float BodyMotor::sBounce = 0.0;
float BodyMotor::sDamping = 0.1;
float BodyMotor::sSoftness = 0.01;

BodyMotor::BodyMotor(std::shared_ptr<BodyJoint> pJoint)
	: mJoint(pJoint)
	, mBody(nullptr)
	, mBounce(sBounce)
	, mDamping(sDamping)
	, mSoftness(sSoftness)
{}

BodyMotor::~BodyMotor()
{
	std::cout << "delete motor\n";
}

std::shared_ptr<Body>
BodyMotor::body()
{
	return mBody;
}

std::shared_ptr<BodyJoint> 
BodyMotor::joint()
{
	return mJoint;
}

int 
BodyMotor::id() const
{
	return mJoint->id();
}

const std::string& 
BodyMotor::name() const
{
	return mJoint->name();
}

float
BodyMotor::bounce() const
{
	return mBounce;
}

float
BodyMotor::damping() const
{
	return mDamping;
}

float
BodyMotor::softness() const
{
	return mSoftness;
}

void
BodyMotor::setBounce(float pBounce)
{
	mBounce = pBounce;

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setBounce " << mBounce << "\n";
}

void
BodyMotor::setDamping(float pDamping)
{
	mDamping = pDamping;

	//std::cout << "Body " << mBody->name() << " motor " << name() << " setDamping " << mDamping << "\n";
}

void
BodyMotor::setSoftness(float pSoftness)
{
	mSoftness = pSoftness;
}

void 
BodyMotor::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	// only implemented for UniversalMotor
}