/** \file dab_physics_body_motor.h
*/

#pragma once

#include <iostream>
#include <memory>
#include <array>
#include "dab_value.h"

namespace dab
{

namespace physics
{

class Body;
class BodyJoint;

class BodyMotor
{
public:
	BodyMotor(std::shared_ptr<BodyJoint> pJoint);
	virtual ~BodyMotor();

	std::shared_ptr<Body> body();
	std::shared_ptr<BodyJoint> joint();

	int id() const;
	const std::string& name() const;

	float bounce() const;
	float damping() const;
	float softness() const;

	virtual void setBounce(float pBounce);
	virtual void setDamping(float pDamping);
	virtual void setSoftness(float pSoftness);;

	virtual void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);

protected:
	static float sBounce;
	static float sDamping;
	static float sSoftness;

	std::shared_ptr<Body> mBody;
	std::shared_ptr<BodyJoint> mJoint;

	float mBounce;
	float mDamping;
	float mSoftness;

	virtual void initPhysics() = 0;
};

};

};