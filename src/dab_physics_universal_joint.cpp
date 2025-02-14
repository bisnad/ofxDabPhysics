/** \file dab_physics_universal2_joint.cpp
*/

#include "dab_physics_universal_joint.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"

using namespace dab;
using namespace dab::physics;

RotateOrder UniversalJoint::sRotateOrder = RO_XYZ;
std::array<float, 3> UniversalJoint::sLinearLowerLimit = { -1.0, -1.0, -1.0 };
std::array<float, 3> UniversalJoint::sLinearUpperLimit = { 1.0, 1.0, 1.0 };
std::array<float, 3> UniversalJoint::sAngularLowerLimit = { -PI, -PI, -PI };
std::array<float, 3> UniversalJoint::sAngularUpperLimit = { PI, PI, PI };

UniversalJoint::UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos)
	: BodyJoint(pName, pPrevPart, pNextPart, pPrevJointPos, pNextJointPos)
	, mRotateOrder(sRotateOrder)
	, mNativeJoint(nullptr)
	, mLinearLowerLimit(sLinearLowerLimit)
	, mLinearUpperLimit(sLinearUpperLimit)
	, mAngularLowerLimit(sAngularLowerLimit)
	, mAngularUpperLimit(sAngularUpperLimit)
{
	initPhysics();
}

UniversalJoint::UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pParentPart, std::shared_ptr<BodyPart> pChildPart, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, RotateOrder pRotateOrder)
	: BodyJoint(pName, pParentPart, pChildPart, pOffsetInParent, pOffsetInChild)
	, mRotateOrder(pRotateOrder)
	, mNativeJoint(nullptr)
	, mLinearLowerLimit(sLinearLowerLimit)
	, mLinearUpperLimit(sLinearUpperLimit)
	, mAngularLowerLimit(sAngularLowerLimit)
	, mAngularUpperLimit(sAngularUpperLimit)
{
	initPhysics();
}

UniversalJoint::UniversalJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot, RotateOrder pRotateOrder)
	: BodyJoint(pName, pPrevPart, pNextPart, pPrevJointPos, pPrevJointRot, pNextJointPos, pNextJointRot)
	, mRotateOrder(pRotateOrder)
	, mNativeJoint(nullptr)
	, mLinearLowerLimit(sLinearLowerLimit)
	, mLinearUpperLimit(sLinearUpperLimit)
	, mAngularLowerLimit(sAngularLowerLimit)
	, mAngularUpperLimit(sAngularUpperLimit)
{
	initPhysics();
}

UniversalJoint::~UniversalJoint()
{
	std::cout << "delete UniversalJoint\n";
}

void
UniversalJoint::initPhysics()
{
	//mJoint = new btGeneric6DofSpring2Constraint(*(mNextPart->nativeBody()), *(mPrevPart->nativeBody()), mNextTransform, mPrevTransform, mRotateOrder);
	mNativeJoint = new btGeneric6DofSpring2Constraint(*(mPrevPart->nativeBody()), *(mNextPart->nativeBody()), mPrevTransform, mNextTransform, mRotateOrder);

	//std::cout << "Universal2Joint name " << mName << "\n";
	//std::cout << "prevBody name " << mPrevPart->name() << "\n";
	//std::cout << "nextBody name " << mNextPart->name() << "\n";

	//btVector3 prevPos = mPrevTransform.getOrigin();
	//btVector3 nextPos = mNextTransform.getOrigin();
	//btQuaternion prevRot = mPrevTransform.getRotation();
	//btQuaternion nextRot = mNextTransform.getRotation();
	//btMatrix3x3 prevBasis = mPrevTransform.getBasis();
	//btMatrix3x3 nextBasis = mNextTransform.getBasis();

	//std::cout << "prevPos " << prevPos.x() << " " << prevPos.y() << " " << prevPos.z() << "\n";
	//std::cout << "prevRot " << prevRot.x() << " " << prevRot.y() << " " << prevRot.z() << " " << prevRot.w() << "\n";
	////std::cout << "prevBasis 0 " << prevBasis[0].x() << " " << prevBasis[0].y() << " " << prevBasis[0].z() << "\n";
	////std::cout << "prevBasis 1 " << prevBasis[1].x() << " " << prevBasis[1].y() << " " << prevBasis[1].z() << "\n";
	////std::cout << "prevBasis 2 " << prevBasis[2].x() << " " << prevBasis[2].y() << " " << prevBasis[2].z() << "\n";

	//std::cout << "nextPos " << nextPos.x() << " " << nextPos.y() << " " << nextPos.z() << "\n";
	//std::cout << "nextRot " << nextRot.x() << " " << nextRot.y() << " " << nextRot.z() << " " << nextRot.w() << "\n";
	////std::cout << "nextBasis 0 " << nextBasis[0].x() << " " << nextBasis[0].y() << " " << nextBasis[0].z() << "\n";
	////std::cout << "nextBasis 1 " << nextBasis[1].x() << " " << nextBasis[1].y() << " " << nextBasis[1].z() << "\n";
	////std::cout << "nextBasis 2 " << nextBasis[2].x() << " " << nextBasis[2].y() << " " << nextBasis[2].z() << "\n";


	//mJoint->setLinearLowerLimit(btVector3(mLinearLowerLimit[0], mLinearLowerLimit[1], mLinearLowerLimit[2]));
	//mJoint->setLinearUpperLimit(btVector3(mLinearUpperLimit[0], mLinearUpperLimit[1], mLinearUpperLimit[2]));
	//mJoint->setAngularLowerLimit(btVector3(mAngularLowerLimit[0], mAngularLowerLimit[1], mAngularLowerLimit[2]));
	//mJoint->setAngularUpperLimit(btVector3(mAngularUpperLimit[0], mAngularUpperLimit[1], mAngularUpperLimit[2]));

	////std::cout << "lll " << mLinearLowerLimit[0] << " " << mLinearLowerLimit[1] << " " << mLinearLowerLimit[2] << "\n";
	////std::cout << "lol " << mLinearUpperLimit[0] << " " << mLinearUpperLimit[1] << " " << mLinearUpperLimit[2] << "\n";
	////std::cout << "all " << mAngularLowerLimit[0] << " " << mAngularLowerLimit[1] << " " << mAngularLowerLimit[2] << "\n";
	////std::cout << "aol " << mAngularUpperLimit[0] << " " << mAngularUpperLimit[1] << " " << mAngularUpperLimit[2] << "\n";

	mNativeJoint->setEnabled(true);

	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[0], 0);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[1], 1);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[2], 2);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[0], 3);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[1], 4);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[2], 5);

	//The ERP specifies what proportion of the joint error will be fixed during the next simulation step.
	//If ERP=0 then no correcting force is applied and the bodies will eventually drift apart as the simulation proceeds.
	//If ERP=1 then the simulation will attempt to fix all joint error during the next time step.
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[0], 0);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[1], 1);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[2], 2);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[0], 3);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[1], 4);
	mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[2], 5);

	mNativeJoint->setLimit(0, mLinearLowerLimit[0], mLinearUpperLimit[0]);
	mNativeJoint->setLimit(1, mLinearLowerLimit[1], mLinearUpperLimit[1]);
	mNativeJoint->setLimit(2, mLinearLowerLimit[2], mLinearUpperLimit[2]);

	mNativeJoint->setLimit(3, mAngularLowerLimit[0], mAngularUpperLimit[0]);
	mNativeJoint->setLimit(4, mAngularLowerLimit[1], mAngularUpperLimit[1]);
	mNativeJoint->setLimit(5, mAngularLowerLimit[2], mAngularUpperLimit[2]);
}

btGeneric6DofSpring2Constraint* 
UniversalJoint::nativeJoint()
{
	return mNativeJoint;
}

const btGeneric6DofSpring2Constraint* 
UniversalJoint::nativeJoint() const
{
	return mNativeJoint;
}

RotateOrder 
UniversalJoint::rotateOrder() const
{
	return mRotateOrder;
}

const std::array<float, 3>&
UniversalJoint::linearLowerLimit() const
{
	return mLinearLowerLimit;
}

const std::array<float, 3>&
UniversalJoint::linearUpperLimit() const
{
	return mLinearUpperLimit;
}

const std::array<float, 3>&
UniversalJoint::angularLowerLimit() const
{
	return mAngularLowerLimit;
}

const std::array<float, 3>&
UniversalJoint::angularUpperLimit() const
{
	return mAngularUpperLimit;
}

std::array<float, 3> 
UniversalJoint::angles() const
{
	std::array<float, 3> _angles;
	_angles[0] = mNativeJoint->getAngle(0);
	_angles[1] = mNativeJoint->getAngle(1);
	_angles[2] = mNativeJoint->getAngle(2);

	return _angles;
}

float 
UniversalJoint::angle(int pAxisIndex) const
{
	pAxisIndex = std::max(std::min(0, pAxisIndex), 2);
	return mNativeJoint->getAngle(pAxisIndex);
}

void
UniversalJoint::setBasis(const std::array<float, 3> &pBasis)
{
	BodyJoint::setBasis(pBasis);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->getFrameOffsetA().getBasis().setEulerZYX(mBasis[0], mBasis[1], mBasis[2]);
		mNativeJoint->getFrameOffsetB().getBasis().setEulerZYX(mBasis[0], mBasis[1], mBasis[2]);
	}
}

void
UniversalJoint::setPrevJointPos(const glm::vec3 &pPrevJointPos)
{
	BodyJoint::setPrevJointPos(pPrevJointPos);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setFrames(mNextTransform, mPrevTransform);
	}
}

void
UniversalJoint::setNextJointPos(const glm::vec3 &pNextJointPos)
{
	BodyJoint::setNextJointPos(pNextJointPos);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setFrames(mNextTransform, mPrevTransform);
	}
}

void
UniversalJoint::setLinearLowerLimit(const std::array<float, 3>& pLinearLowerLimit)
{
	mLinearLowerLimit = pLinearLowerLimit;

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setLimit(0, mLinearLowerLimit[0], mLinearUpperLimit[0]);
		mNativeJoint->setLimit(1, mLinearLowerLimit[1], mLinearUpperLimit[1]);
		mNativeJoint->setLimit(2, mLinearLowerLimit[2], mLinearUpperLimit[2]);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setLinearLowerLimit " << mLinearLowerLimit[0] << " " << mLinearLowerLimit[1] << " " << mLinearLowerLimit[2] << "\n";
	}
}

void
UniversalJoint::setLinearUpperLimit(const std::array<float, 3>& pLinearUpperLimit)
{
	mLinearUpperLimit = pLinearUpperLimit;

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setLimit(0, mLinearLowerLimit[0], mLinearUpperLimit[0]);
		mNativeJoint->setLimit(1, mLinearLowerLimit[1], mLinearUpperLimit[1]);
		mNativeJoint->setLimit(2, mLinearLowerLimit[2], mLinearUpperLimit[2]);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setLinearUpperLimit " << mLinearUpperLimit[0] << " " << mLinearUpperLimit[1] << " " << mLinearUpperLimit[2] << "\n";
	}
}

void
UniversalJoint::setAngularLowerLimit(const std::array<float, 3>& pAngularLowerLimit)
{
	mAngularLowerLimit = pAngularLowerLimit;

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setLimit(3, mAngularLowerLimit[0], mAngularUpperLimit[0]);
		mNativeJoint->setLimit(4, mAngularLowerLimit[1], mAngularUpperLimit[1]);
		mNativeJoint->setLimit(5, mAngularLowerLimit[2], mAngularUpperLimit[2]);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setAngularLowerLimit " << mAngularLowerLimit[0] << " " << mAngularLowerLimit[1] << " " << mAngularLowerLimit[2] << "\n";
	}
}

void
UniversalJoint::setAngularUpperLimit(const std::array<float, 3>& pAngularUpperLimit)
{
	mAngularUpperLimit = pAngularUpperLimit;

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setLimit(3, mAngularLowerLimit[0], mAngularUpperLimit[0]);
		mNativeJoint->setLimit(4, mAngularLowerLimit[1], mAngularUpperLimit[1]);
		mNativeJoint->setLimit(5, mAngularLowerLimit[2], mAngularUpperLimit[2]);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setAngularUpperLimit " << mAngularUpperLimit[0] << " " << mAngularUpperLimit[1] << " " << mAngularUpperLimit[2] << "\n";
	}
}

void
UniversalJoint::setLinearStopERP(const std::array<float, 3>& pLinearStopERP)
{
	BodyJoint::setLinearStopERP(pLinearStopERP);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[0], 0);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[1], 1);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mLinearStopERP[2], 2);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setLinearStopERP " << mLinearStopERP[0] << " " << mLinearStopERP[1] << " " << mLinearStopERP[2] << "\n";
	}
}

void
UniversalJoint::setAngularStopERP(const std::array<float, 3>& pAngularStopERP)
{
	BodyJoint::setAngularStopERP(pAngularStopERP);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[0], 3);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[1], 4);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_ERP, mAngularStopERP[2], 5);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setAngularStopERP " << mAngularStopERP[0] << " " << mAngularStopERP[1] << " " << mAngularStopERP[2] << "\n";
	}
}

void
UniversalJoint::setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM)
{
	BodyJoint::setLinearStopCFM(pLinearStopCFM);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[0], 0);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[1], 1);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mLinearStopCFM[2], 2);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setLinearStopCFM " << mLinearStopCFM[0] << " " << mLinearStopCFM[1] << " " << mLinearStopCFM[2] << "\n";
	}
}

void
UniversalJoint::setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM)
{
	BodyJoint::setAngularStopCFM(pAngularStopCFM);

	if (mNativeJoint != nullptr)
	{
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[0], 3);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[1], 4);
		mNativeJoint->setParam(BT_CONSTRAINT_STOP_CFM, mAngularStopCFM[2], 5);

		//std::cout << "body " << mBody->name() << " joint " << name() << " setAngularStopCFM " << mAngularStopCFM[0] << " " << mAngularStopCFM[1] << " " << mAngularStopCFM[2] << "\n";
	}
}

void 
UniversalJoint::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	try
	{
		if (pParName == "linearLowerLimit")
		{
			setLinearLowerLimit(pValue);
		}
		else if (pParName == "linearUpperLimit")
		{
			setLinearUpperLimit(pValue);
		}
		else if (pParName == "angularLowerLimit")
		{
			setAngularLowerLimit(pValue);
		}
		else if (pParName == "angularUpperLimit")
		{
			setAngularUpperLimit(pValue);
		}
		else if (pParName == "linearStopERP")
		{
			setLinearStopERP(pValue);
		}
		else if (pParName == "angularStopERP")
		{
			setAngularStopERP(pValue);
		}
		else if (pParName == "linearStopCFM")
		{
			setLinearStopCFM(pValue);
		}
		else if (pParName == "angularStopCFM")
		{
			setAngularStopCFM(pValue);
		}
		else
		{
			throw dab::Exception("Physics Error: parameter name " + pParName + " not recognized by universal joint", __FILE__, __FUNCTION__, __LINE__);

		}
	}
	catch (Exception& e)
	{
		e += Exception("Physics Error: failed to set parameter " + pParName + " for universal joint " + mName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}