/** \file dab_physics_body_joint.h
*/

#pragma once

#include <iostream>
#include <array>
#include <btBulletDynamicsCommon.h>
#include "ofVectorMath.h"
#include "dab_value.h"

namespace dab
{

namespace physics
{

class Body;
class BodyPart;
class BodyMotor;

class BodyJoint
{
public:
	friend class Simulation;
	friend class BodyMotor;

	BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos);
	BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot);
	BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pParentPart, std::shared_ptr<BodyPart> pChildPart, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild);

	virtual ~BodyJoint();

	int id() const;
	const std::string& name() const;
	std::shared_ptr<Body> body() const;
	std::shared_ptr<BodyMotor> motor() const;

	std::shared_ptr<BodyPart> prevPart() const;
	std::shared_ptr<BodyPart> nextPart() const;

	btTransform transform() const;
	const btTransform& prevTransform() const;
	const btTransform& nextTransform() const;

	const std::array<float, 3>& basis() const;
	const glm::vec3& prevJointPos() const;
	const glm::vec3& nextJointPos() const;
	const glm::quat& prevJointRot() const;
	const glm::quat& nextJointRot() const;

	const std::array<float, 3>& linearStopERP() const;
	const std::array<float, 3>& angularStopERP() const;
	const std::array<float, 3>& linearStopCFM() const;
	const std::array<float, 3>& angularStopCFM() const;

	void setMotor(std::shared_ptr<BodyMotor> pMotor);

	virtual void setBasis(const std::array<float, 3>& pBasis);
	virtual void setPrevJointPos(const glm::vec3& pPrevJointPos);
	virtual void setNextJointPos(const glm::vec3& pNextJointPos);
	virtual void setPrevJointRot(const glm::vec3& pPrevJointRot);
	virtual void setNextJointRot(const glm::vec3& pNextJointRot);

	virtual void setLinearStopERP(const std::array<float, 3>& pLinearStopERP);
	virtual void setAngularStopERP(const std::array<float, 3>& pAngularStopERP);
	virtual void setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM);
	virtual void setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM);

	virtual void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);

protected:
	static int sId;
	static std::array<float, 3> sBasis;
	static std::array<float, 3> sLinearStopERP;
	static std::array<float, 3> sAngularStopERP;
	static std::array<float, 3> sLinearStopCFM;
	static std::array<float, 3> sAngularStopCFM;

	int mId;
	std::string mName;
	std::shared_ptr<Body> mBody;
	std::shared_ptr<BodyMotor> mMotor;
	std::shared_ptr<BodyPart> mPrevPart;
	std::shared_ptr<BodyPart> mNextPart;
	glm::vec3 mPrevJointPos;
	glm::vec3 mNextJointPos;
	glm::quat mPrevJointRot;
	glm::quat mNextJointRot;
	btTransform mPrevTransform;
	btTransform mNextTransform;

	std::array<float, 3> mBasis;
	std::array<float, 3> mLinearStopERP;
	std::array<float, 3> mAngularStopERP;
	std::array<float, 3> mLinearStopCFM;
	std::array<float, 3> mAngularStopCFM;

	virtual void initPhysics();
};

};

};