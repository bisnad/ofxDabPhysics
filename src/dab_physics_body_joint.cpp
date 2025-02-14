/** \file dab_physics_body_joint.cpp
*/

#include "dab_physics_body_joint.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body.h"

using namespace dab;
using namespace dab::physics;

int BodyJoint::sId = 0;
std::array<float, 3> BodyJoint::sBasis = { 0.0, 0.0, 0.0 };
//std::array<float, 3> BodyJoint::sLinearStopERP = { 1.0, 1.0, 1.0 };
//std::array<float, 3> BodyJoint::sAngularStopERP = { 1.0, 1.0, 1.0 };
std::array<float, 3> BodyJoint::sLinearStopCFM = { 0.1, 0.1, 0.1 };
std::array<float, 3> BodyJoint::sAngularStopCFM = { 0.1, 0.1, 0.1 };
std::array<float, 3> BodyJoint::sLinearStopERP = { 0.9, 0.9, 0.9 };
std::array<float, 3> BodyJoint::sAngularStopERP = { 0.9, 0.9, 0.9 };
//std::array<float, 3> BodyJoint::sLinearStopCFM = { 0.0, 0.0, 0.0 };
//std::array<float, 3> BodyJoint::sAngularStopCFM = { 0.0, 0.0, 0.0 };

BodyJoint::BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos)
	: mId(sId++)
	, mName(pName)
	, mBody(nullptr)
	, mMotor(nullptr)
	, mPrevPart(pPrevPart)
	, mNextPart(pNextPart)
	, mPrevJointPos(pPrevJointPos)
	, mNextJointPos(pNextJointPos)
	, mBasis(sBasis)
	, mLinearStopERP(sLinearStopERP)
	, mAngularStopERP(sAngularStopERP)
	, mLinearStopCFM(sLinearStopCFM)
	, mAngularStopCFM(sAngularStopCFM)
{
	initPhysics();
}

BodyJoint::BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pPrevPart, std::shared_ptr<BodyPart> pNextPart, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot)
	: mId(sId++)
	, mName(pName)
	, mBody(nullptr)
	, mMotor(nullptr)
	, mPrevPart(pPrevPart)
	, mNextPart(pNextPart)
	, mPrevJointPos(pPrevJointPos)
	, mNextJointPos(pNextJointPos)
	, mPrevJointRot(pPrevJointRot)
	, mNextJointRot(pNextJointRot)
	, mBasis(sBasis)
	, mLinearStopERP(sLinearStopERP)
	, mAngularStopERP(sAngularStopERP)
	, mLinearStopCFM(sLinearStopCFM)
	, mAngularStopCFM(sAngularStopCFM)
{
	initPhysics();
}


BodyJoint::BodyJoint(const std::string& pName, std::shared_ptr<BodyPart> pParentPart, std::shared_ptr<BodyPart> pChildPart, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild)
	: mId(sId++)
	, mName(pName)
	, mBody(nullptr)
	, mMotor(nullptr)
	, mPrevPart(pParentPart)
	, mNextPart(pChildPart)
	, mPrevTransform(pOffsetInParent)
	, mNextTransform(pOffsetInChild)
	, mLinearStopERP(sLinearStopERP)
	, mAngularStopERP(sAngularStopERP)
	, mLinearStopCFM(sLinearStopCFM)
	, mAngularStopCFM(sAngularStopCFM)
{
	//std::cout << "joint addr " << this << "\n";
	//std::cout << "joint " << mName << "\n";
	//std::cout << "prev origin " << mPrevTransform.getOrigin().x() << " " << mPrevTransform.getOrigin().y() << " " << mPrevTransform.getOrigin().z() << "\n";
	//std::cout << "prev rot " << mPrevTransform.getRotation().x() << " " << mPrevTransform.getRotation().y() << " " << mPrevTransform.getRotation().z() << " " << mPrevTransform.getRotation().w() << "\n";
	//std::cout << "next origin " << mNextTransform.getOrigin().x() << " " << mNextTransform.getOrigin().y() << " " << mNextTransform.getOrigin().z() << "\n";
	//std::cout << "next rot " << mNextTransform.getRotation().x() << " " << mNextTransform.getRotation().y() << " " << mNextTransform.getRotation().z() << " " << mNextTransform.getRotation().w() << "\n";
}

BodyJoint::~BodyJoint()
{
	std::cout << "delete joint " << mName << " body " << mBody->name() << "\n";
}

void
BodyJoint::initPhysics()
{
	mPrevTransform = btTransform::getIdentity();
	mPrevTransform.setOrigin(btVector3(mPrevJointPos[0], mPrevJointPos[1], mPrevJointPos[2]));
	mPrevTransform.setRotation(btQuaternion(mPrevJointRot.x, mPrevJointRot.y, mPrevJointRot.z, mPrevJointRot.w));

	mNextTransform = btTransform::getIdentity();
	mNextTransform.setOrigin(btVector3(mNextJointPos[0], mNextJointPos[1], mNextJointPos[2]));
	mNextTransform.setRotation(btQuaternion(mNextJointRot.x, mNextJointRot.y, mNextJointRot.z, mNextJointRot.w));

	//std::cout << "joint addr " << this << "\n";
	//std::cout << "joint " << mName << "\n";
	//std::cout << "prev origin " << mPrevTransform.getOrigin().x() << " " << mPrevTransform.getOrigin().y() << " " << mPrevTransform.getOrigin().z() << "\n";
	//std::cout << "prev rot " << mPrevTransform.getRotation().x() << " " << mPrevTransform.getRotation().y() << " " << mPrevTransform.getRotation().z() << " " << mPrevTransform.getRotation().w() << "\n";
	//std::cout << "next origin " << mNextTransform.getOrigin().x() << " " << mNextTransform.getOrigin().y() << " " << mNextTransform.getOrigin().z() << "\n";
	//std::cout << "next rot " << mNextTransform.getRotation().x() << " " << mNextTransform.getRotation().y() << " " << mNextTransform.getRotation().z() << " " << mNextTransform.getRotation().w() << "\n";
}

int
BodyJoint::id() const
{
	return mId;
}

const std::string&
BodyJoint::name() const
{
	return mName;
}

std::shared_ptr<Body>
BodyJoint::body() const
{
	return mBody;
}

std::shared_ptr<BodyMotor> 
BodyJoint::motor() const
{
	return mMotor;
}

std::shared_ptr<BodyPart> 
BodyJoint::prevPart() const
{
	return mPrevPart;
}

std::shared_ptr<BodyPart> 
BodyJoint::nextPart() const
{
	return mNextPart;
}

btTransform 
BodyJoint::transform() const
{
	// TODO: untested
	const btTransform& partTransform = mPrevPart->transform();
	btTransform jointTransform;
	jointTransform.mult(partTransform, mPrevTransform);
	//jointTransform.mult(mPrevTransform, partTransform);

	//// debug
	//const btVector3& partPos = partTransform.getOrigin();
	//const btQuaternion& partRot = partTransform.getRotation();

	//const btVector3& prevPos = mPrevTransform.getOrigin();
	//const btQuaternion& prevRot = mPrevTransform.getRotation();

	//const btVector3& jointPos = jointTransform.getOrigin();
	//const btQuaternion& jointRot = jointTransform.getRotation();

	//std::cout << "joint " << mName << "\n";
	//std::cout << "partPos " << partPos[0] << " " << partPos[1] << " " << partPos[2] << " partRot " << partRot[0] << " " << partRot[1] << " " << partRot[2] << " " << partRot[3] << "\n";
	//std::cout << "prevPos " << prevPos[0] << " " << prevPos[1] << " " << prevPos[2] << " prevRot " << prevRot[0] << " " << prevRot[1] << " " << prevRot[2] << " " << prevRot[3] << "\n";
	//std::cout << "jointPos " << jointPos[0] << " " << jointPos[1] << " " << jointPos[2] << " jointRot " << jointRot[0] << " " << jointRot[1] << " " << jointRot[2] << " " << jointRot[3] << "\n";
	//// debug

	return jointTransform;
}

const btTransform& 
BodyJoint::prevTransform() const 
{
	return mPrevTransform;
}

const btTransform& 
BodyJoint::nextTransform() const
{
	return mNextTransform;
}

const glm::vec3&
BodyJoint::prevJointPos() const
{
	return mPrevJointPos;
}

const glm::vec3&
BodyJoint::nextJointPos() const
{
	return mNextJointPos;
}

const glm::quat&
BodyJoint::prevJointRot() const
{
	return mPrevJointRot;
}

const glm::quat&
BodyJoint::nextJointRot() const
{
	return mNextJointRot;
}

const std::array<float, 3>&
BodyJoint::basis() const
{
	return mBasis;
}

void
BodyJoint::setBasis(const std::array<float, 3> &pBasis)
{
	mBasis = pBasis;

	//std::cout << "Body " << mBody->name() << " joint " << mName << " setBasis " << mBasis[0] << " " << mBasis[1] << " " << mBasis[2] << "\n";
}

const std::array<float, 3>&
BodyJoint::linearStopERP() const
{
	return mLinearStopERP;
}

const std::array<float, 3>&
BodyJoint::angularStopERP() const
{
	return mAngularStopERP;
}

const std::array<float, 3>&
BodyJoint::linearStopCFM() const
{
	return mLinearStopCFM;
}

const std::array<float, 3>&
BodyJoint::angularStopCFM() const
{
	return mAngularStopCFM;
}

void 
BodyJoint::setMotor(std::shared_ptr<BodyMotor> pMotor)
{
	mMotor = pMotor;
}

void
BodyJoint::setPrevJointPos(const glm::vec3 &pPrevJointPos)
{
	mPrevJointPos = pPrevJointPos;
	mPrevTransform.setOrigin(btVector3(mPrevJointPos.x, mPrevJointPos.y, mPrevJointPos.z));

	//std::cout << "Body " << mBody->name() << " joint " << mName << " setPrevJointPos " << mPrevJointPos[0] << " " << mPrevJointPos[1] << " " << mPrevJointPos[2] << "\n";
}

void
BodyJoint::setNextJointPos(const glm::vec3 &pNextJointPos)
{
	mNextJointPos = pNextJointPos;
	mNextTransform.setOrigin(btVector3(mNextJointPos.x, mNextJointPos.y, mNextJointPos.z));

	//std::cout << "Body " << mBody->name() << " joint " << mName << " setNextJointPos " << mNextJointPos[0] << " " << mNextJointPos[1] << " " << mNextJointPos[2] << "\n";
}

void 
BodyJoint::setPrevJointRot(const glm::vec3& pPrevJointRot)
{
	mPrevJointRot = pPrevJointRot;
	mPrevTransform.setRotation(btQuaternion(mPrevJointRot.x, mPrevJointRot.y, mPrevJointRot.z, mPrevJointRot.w));
}

void 
BodyJoint::setNextJointRot(const glm::vec3& pNextJointRot)
{
	mNextJointRot = pNextJointRot;
	mNextTransform.setRotation(btQuaternion(mNextJointRot.x, mNextJointRot.y, mNextJointRot.z, mNextJointRot.w));
}

void
BodyJoint::setLinearStopERP(const std::array<float, 3>& pLinearStopERP)
{
	mLinearStopERP = pLinearStopERP;
}

void
BodyJoint::setAngularStopERP(const std::array<float, 3>& pAngularStopERP)
{
	mAngularStopERP = pAngularStopERP;
}

void
BodyJoint::setLinearStopCFM(const std::array<float, 3>& pLinearStopCFM)
{
	mLinearStopCFM = pLinearStopCFM;
}

void
BodyJoint::setAngularStopCFM(const std::array<float, 3>& pAngularStopCFM)
{
	mAngularStopCFM = pAngularStopCFM;
}

void 
BodyJoint::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	// currently only implemented for UniversalJoint
}