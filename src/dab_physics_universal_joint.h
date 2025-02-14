/** \file dab_physics_universal2_joint.h
*/

#pragma once

#include "dab_physics_body_joint.h"
#include "dab_value.h"
#include "ofMathConstants.h"
#include <array>

namespace dab
{

namespace physics
{

class UniversalJoint : public BodyJoint
{
public:
	friend class Simulation;
	friend class UniversalMotor;

	UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos);
	UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot, RotateOrder pRotateOrder = RO_XYZ);
	UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pParentPart, std::shared_ptr<BodyPart> pChildPart, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, RotateOrder pRotateOrder);
	~UniversalJoint();

	btGeneric6DofSpring2Constraint* nativeJoint();
	const btGeneric6DofSpring2Constraint* nativeJoint() const;

	RotateOrder rotateOrder() const;
	const std::array<float, 3>& linearLowerLimit() const;
	const std::array<float, 3>& linearUpperLimit() const;
	const std::array<float, 3>& angularLowerLimit() const;
	const std::array<float, 3>& angularUpperLimit() const;

	std::array<float, 3> angles() const;
	float angle(int pAxisIndex) const;

	void setBasis(const std::array<float, 3>& pBasis);
	void setPrevJointPos(const glm::vec3& pPrevJointPos);
	void setNextJointPos(const glm::vec3& pNextJointPos);

	void setLinearLowerLimit(const std::array<float, 3>& pLinearLowerLimit);
	void setLinearUpperLimit(const std::array<float, 3>& pLinearUpperLimit);
	void setAngularLowerLimit(const std::array<float, 3>& pAngularLowerLimit);
	void setAngularUpperLimit(const std::array<float, 3>& pAngularUpperLimit);

	void setLinearStopERP(const std::array<float, 3>& pLinearStopERP);
	void setAngularStopERP(const std::array<float, 3>& pAngularStopERP);
	void setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM);
	void setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM);

	void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);

protected:
	btGeneric6DofSpring2Constraint* mNativeJoint;

	static RotateOrder sRotateOrder;
	static std::array<float, 3> sLinearLowerLimit;
	static std::array<float, 3> sLinearUpperLimit;
	static std::array<float, 3> sAngularLowerLimit;
	static std::array<float, 3> sAngularUpperLimit;

	RotateOrder mRotateOrder;
	std::array<float, 3> mLinearLowerLimit;
	std::array<float, 3> mLinearUpperLimit;
	std::array<float, 3> mAngularLowerLimit;
	std::array<float, 3> mAngularUpperLimit;

	void initPhysics();
};

};

};
