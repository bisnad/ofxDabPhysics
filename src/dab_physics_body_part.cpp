/** \file dab_physics_body_part.cpp
*/

#include "dab_physics_body_part.h"
#include "dab_physics_body_shape.h"
#include "dab_physics_body.h"
#include <memory>

using namespace dab;
using namespace dab::physics;

int BodyPart::sId = 0;
float BodyPart::sMass = 1.0;
float BodyPart::sLinearDamping = 0.5;
float BodyPart::sAngularDamping = 0.5;
float BodyPart::sFriction = 0.1;
float BodyPart::sRollingFriction = 0.1;
float BodyPart::sRestitution = 0.0;

BodyPart::BodyPart()
{}

BodyPart::BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass)
	: mId(sId++)
	, mName(pName)
	, mShape(pShape)
	, mBody(nullptr)
	, mMass(pMass)
	, mLinearDamping(sLinearDamping)
	, mAngularDamping(sAngularDamping)
	, mFriction(sFriction)
	, mRollingFriction(sRollingFriction)
	, mRestitution(sRestitution)
	, mNativeBody(nullptr)
{
	initPhysics();
}

BodyPart::BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass, const glm::vec3& pPosition, const glm::quat& pOrientation)
	: mId(sId++)
	, mName(pName)
	, mShape(pShape)
	, mBody(nullptr)
	, mMass(pMass)
	, mLinearDamping(sLinearDamping)
	, mAngularDamping(sAngularDamping)
	, mFriction(sFriction)
	, mRollingFriction(sRollingFriction)
	, mRestitution(sRestitution)
	, mNativeBody(nullptr)
{
	initPhysics(pPosition, pOrientation);
}

BodyPart::BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass, const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans)
	: mId(sId++)
	, mName(pName)
	, mShape(pShape)
	, mBody(nullptr)
	, mMass(pMass)
	, mLinearDamping(sLinearDamping)
	, mAngularDamping(sAngularDamping)
	, mFriction(sFriction)
	, mRollingFriction(sRollingFriction)
	, mRestitution(sRestitution)
	, mNativeBody(nullptr)
{
	initPhysics(pLocalInertiaDiagonal, pInitialWorldTrans);
}

BodyPart::~BodyPart()
{
	std::cout << "delete part " << mName << " from body " << mBody->name() << "\n";
}

int
BodyPart::id() const
{
	return mId;
}

const std::string&
BodyPart::name() const
{
	return mName;
}

std::shared_ptr<Body>
BodyPart::body() const
{
	return mBody;
}

bool 
BodyPart::isCompound() const
{
	return std::dynamic_pointer_cast<BodyCompoundShape>(mShape) != nullptr;
}

float
BodyPart::mass() const
{
	return mMass;
}

float
BodyPart::linearDamping() const
{
	return mLinearDamping;
}

float
BodyPart::angularDamping() const
{
	return mAngularDamping;
}

float
BodyPart::friction() const
{
	return mFriction;
}

float
BodyPart::rollingFriction() const
{
	return mRollingFriction;
}

float
BodyPart::restitution() const
{
	return mRestitution;
}

std::shared_ptr<BodyShape>
BodyPart::shape() const
{
	return mShape;
}

btRigidBody* 
BodyPart::nativeBody()
{
	return mNativeBody;
}

const btRigidBody*
BodyPart::nativeBody() const
{
	return mNativeBody;
}

const std::vector< std::shared_ptr<BodyJoint> >& 
BodyPart::nextJoints()
{
	return mNextJoints;
}

const std::vector< std::shared_ptr<BodyJoint> >& 
BodyPart::prevJoints()
{
	return mPrevJoints;
}

void
BodyPart::setMass(float pMass)
{
	mMass = pMass;

	if (mNativeBody != nullptr)
	{
		btVector3 inertia(0.0, 0.0, 0.0);

		if (mMass > 0.0)
		{
			calcInertia(inertia);
		}

		mNativeBody->setMassProps(mMass, inertia);

		//std::cout << "Body " << mBody->name() << " BodyPart " << mName << " setMass " << mMass << "\n";
	}
}

void
BodyPart::setLinearDamping(float pLinearDamping)
{
	mLinearDamping = pLinearDamping;

	if (mBody != nullptr)
	{
		mNativeBody->setDamping(mLinearDamping, mAngularDamping);

		//std::cout << "Body " << mBody->name() << " part " << mName << " setLinearDamping " << mLinearDamping << "\n";
	}
}

void
BodyPart::setAngularDamping(float pAngularDamping)
{
	mAngularDamping = pAngularDamping;

	if (mNativeBody != nullptr)
	{
		mNativeBody->setDamping(mLinearDamping, mAngularDamping);

		//std::cout << "Body " << mBody->name() << " part " << mName << " setAngularDamping " << mAngularDamping << "\n";
	}
}

void
BodyPart::setFriction(float pFriction)
{
	mFriction = pFriction;

	if (mNativeBody != nullptr)
	{
		mNativeBody->setFriction(mFriction);

		//std::cout << "Body " << mBody->name() << " part " << mName << " setFriction " << mFriction << "\n";
	}
}

void
BodyPart::setRollingFriction(float pRollingFriction)
{
	mRollingFriction = pRollingFriction;

	if (mNativeBody != nullptr)
	{
		mNativeBody->setRollingFriction(mRollingFriction);

		//std::cout << "Body " << mBody->name() << " part " << mName << " setRollingFriction " << mRollingFriction << "\n";
	}
}

void
BodyPart::setRestitution(float pRestitution)
{
	mRestitution = pRestitution;

	if (mNativeBody != nullptr)
	{
		mNativeBody->setRestitution(mRestitution);

		//std::cout << "Body " << mBody->name() << " part " << mName << " setRestitution " << mRestitution << "\n";
	}
}

void 
BodyPart::set(const std::string& pParName, const AbstractValue& pValue) throw (Exception)
{
	try
	{
		if (pParName == "mass")
		{
			if (mMass == 0.0) return; // don't change fixed masses
			setMass(pValue);
		}
		else if (pParName == "linearDamping")
		{
			setLinearDamping(pValue);
		}
		else if (pParName == "angularDamping")
		{
			setAngularDamping(pValue);
		}
		else if (pParName == "friction")
		{
			setFriction(pValue);
		}
		else if (pParName == "rollingFriction")
		{
			setRollingFriction(pValue);
		}
		else if (pParName == "restitution")
		{
			setRestitution(pValue);
		}
		else
		{
			throw dab::Exception("Physics Error: parameter name " + pParName + " not recognized by body part", __FILE__, __FUNCTION__, __LINE__);
		}
	}
	catch (Exception& e)
	{
		e += Exception("Physics Error: failed to set parameter " + pParName + " for body part " + mName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const btTransform& 
BodyPart::transform() const
{
	return mNativeBody->getWorldTransform();
}

glm::vec3
BodyPart::position() const
{
	const btVector3& _pos = mNativeBody->getWorldTransform().getOrigin();
	return glm::vec3(_pos[0], _pos[1], _pos[2]);
}

glm::quat
BodyPart::rotation() const
{
	const btQuaternion& _rot = mNativeBody->getOrientation();
	return glm::quat(_rot.w(), _rot.x(), _rot.y(), _rot.z());
}

const glm::mat4&
BodyPart::matrix()
{
	btTransform wTransform = mNativeBody->getWorldTransform();
	float glTransform[16];
	wTransform.getOpenGLMatrix(glTransform);
	mMatrix = glm::make_mat4(glTransform);

	return mMatrix;
}

glm::vec3
BodyPart::linearVelocity() const
{
	const btVector3& _vel = mNativeBody->getLinearVelocity();
	return glm::vec3(_vel.x(), _vel.y(), _vel.z());
}

glm::vec3
BodyPart::angularVelocity() const
{
	const btVector3& _vel = mNativeBody->getAngularVelocity();
	return glm::vec3(_vel.x(), _vel.y(), _vel.z());
}

void
BodyPart::setPosition(const glm::vec3& pPosition)
{
	//std::cout << "set position body " << mBody->name() << " part " << mName << " pos " << pPosition.x << " " << pPosition.y << " " << pPosition.z << "\n";

	mNativeBody->setActivationState(ACTIVE_TAG);
	mNativeBody->setWorldTransform(btTransform(mNativeBody->getOrientation(), btVector3(pPosition.x, pPosition.y, pPosition.z)));
	mNativeBody->getMotionState()->setWorldTransform(btTransform(mNativeBody->getOrientation(), btVector3(pPosition.x, pPosition.y, pPosition.z)));
}

void
BodyPart::setPosition(const btVector3& pPosition)
{
	mNativeBody->setActivationState(ACTIVE_TAG);
	mNativeBody->setWorldTransform(btTransform(mNativeBody->getOrientation(), pPosition));
	mNativeBody->getMotionState()->setWorldTransform(btTransform(mNativeBody->getOrientation(), btVector3(pPosition[0], pPosition[1], pPosition[2])));
}

void
BodyPart::setRotation(const glm::quat& pRotation)
{
	mNativeBody->setActivationState(ACTIVE_TAG);
	mNativeBody->setWorldTransform(btTransform(btQuaternion(pRotation.x, pRotation.y, pRotation.z, pRotation.w), mNativeBody->getWorldTransform().getOrigin()));
	mNativeBody->getMotionState()->setWorldTransform(btTransform(btQuaternion(pRotation.x, pRotation.y, pRotation.z, pRotation.w), mNativeBody->getWorldTransform().getOrigin()));
}

void
BodyPart::setRotation(const btQuaternion& pRotation)
{
	mNativeBody->setActivationState(ACTIVE_TAG);
	mNativeBody->setWorldTransform(btTransform(pRotation, mNativeBody->getWorldTransform().getOrigin()));
	mNativeBody->getMotionState()->setWorldTransform(btTransform(pRotation, mNativeBody->getWorldTransform().getOrigin()));
}

void
BodyPart::setLinearVelocity(const glm::vec3& pVelocity)
{
	mNativeBody->setLinearVelocity(btVector3(pVelocity.x, pVelocity.y, pVelocity.z));
}

void
BodyPart::setLinearVelocity(const btVector3& pVelocity)
{
	mNativeBody->setLinearVelocity(pVelocity);
}

void
BodyPart::setAngularVelocity(const glm::vec3& pVelocity)
{
	mNativeBody->setAngularVelocity(btVector3(pVelocity.x, pVelocity.y, pVelocity.z));
}

void
BodyPart::setAngularVelocity(const btVector3& pVelocity)
{
	mNativeBody->setAngularVelocity(pVelocity);
}

void 
BodyPart::applyForce(const glm::vec3& pForce)
{
	mNativeBody->applyCentralForce(btVector3(pForce[0], pForce[1], pForce[2]));

	//std::cout << "body " << mBody->name() << " part " << mName << " apply force " << pForce[0] << " " << pForce[1] << " " << pForce[2] << "\n";
}

void 
BodyPart::applyForce(const glm::vec3& pForce, const glm::vec3& pPosition)
{
	mNativeBody->applyForce(btVector3(pForce[0], pForce[1], pForce[2]), btVector3(pPosition[0], pPosition[1], pPosition[2]));

	//std::cout << "body " << mBody->name() << " part " << mName << " apply force " << pForce[0] << " " << pForce[1] << " " << pForce[2] << " pos " << pPosition[0] << " " << pPosition[1] << " " << pPosition[2] << "\n";
}

void 
BodyPart::applyTorque(const glm::vec3& pTorque)
{
	mNativeBody->applyTorque(btVector3(pTorque[0], pTorque[1], pTorque[2]));
}

void
BodyPart::clearForces()
{
	mNativeBody->clearForces();
}

void
BodyPart::initPhysics()
{
	//std::cout << "BodyPart::initPhysics()\n";

	btTransform transform;
	transform.setIdentity();
	btMotionState* _motionState = new btDefaultMotionState(transform);

	btVector3 inertia(0, 0, 0);

	if (mMass > 0.0)
	{
		calcInertia(inertia);
	}

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mMass, _motionState, mShape->mShape, inertia);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mMass,nullptr, mShape->mShape,inertia);
	mNativeBody = new btRigidBody(rbInfo);
	mNativeBody->setActivationState(DISABLE_DEACTIVATION);
	mNativeBody->setDamping(mLinearDamping, mAngularDamping);
	mNativeBody->setFriction(mFriction);
	mNativeBody->setRollingFriction(mRollingFriction);
	mNativeBody->setRestitution(mRestitution);
	mNativeBody->setUserPointer(this);
}

void
BodyPart::initPhysics(const glm::vec3& pPosition, const glm::quat& pOrientation)
{
	//std::cout << "BodyPart::initPhysics(const glm::vec3& pPosition, const glm::quat& pOrientation)\n";

	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(pPosition.x, pPosition.y, pPosition.z));
	transform.setRotation(btQuaternion(pOrientation.x, pOrientation.y, pOrientation.z, pOrientation.w));
	btMotionState* _motionState = new btDefaultMotionState(transform);

	btVector3 inertia(0, 0, 0);
	if (mMass > 0.0)
	{
		calcInertia(inertia);
	}

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mMass, _motionState, mShape->mShape, inertia);
	mNativeBody = new btRigidBody(rbInfo);
	mNativeBody->setActivationState(DISABLE_DEACTIVATION);
	mNativeBody->setDamping(mLinearDamping, mAngularDamping);
	mNativeBody->setFriction(mFriction);
	mNativeBody->setRollingFriction(mRollingFriction);
	mNativeBody->setRestitution(mRestitution);
	mNativeBody->setUserPointer(this);
}

void
BodyPart::initPhysics(const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans)
{
	//std::cout << "BodyPart::initPhysics(const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans)\n";

	btMotionState* _motionState = new btDefaultMotionState(pInitialWorldTrans);

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mMass, _motionState, mShape->mShape, pLocalInertiaDiagonal);
	//rbInfo.m_startWorldTransform = pInitialWorldTrans;
	btScalar sleep_threshold = btScalar(0.22360679775);
	rbInfo.m_angularSleepingThreshold = sleep_threshold;
	rbInfo.m_linearSleepingThreshold = sleep_threshold;

	mNativeBody = new btRigidBody(rbInfo);
	mNativeBody->setActivationState(DISABLE_DEACTIVATION);
	mNativeBody->setDamping(mLinearDamping, mAngularDamping);
	mNativeBody->setFriction(mFriction);
	mNativeBody->setRollingFriction(mRollingFriction);
	mNativeBody->setRestitution(mRestitution);
	mNativeBody->setUserPointer(this);

	//std::cout << "BodyPart init physics\n";
	//std::cout << "name " << mName << "\n";
	//std::cout << "col " << mShape->name() << "\n";


	/*
		btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
	rbci.m_startWorldTransform = initialWorldTrans;
	btScalar sleep_threshold = btScalar(0.22360679775);//sqrt(0.05) to be similar to btMultiBody (SLEEP_THRESHOLD)
	rbci.m_angularSleepingThreshold = sleep_threshold;
	rbci.m_linearSleepingThreshold = sleep_threshold;
	
	btRigidBody* body = new btRigidBody(rbci);
	if (m_rigidBody == 0)
	{
		//only store the root of the multi body
		m_rigidBody = body;
	}
	return body;
	*/
}

void
BodyPart::calcInertia(btVector3& pInertia)
{
	mShape->mShape->calculateLocalInertia(mMass, pInertia);
}

const btTransform& 
BodyPart::nativeShapeTransform() const
{
	return mShapeTransform;
}

const glm::mat4& 
BodyPart::shapeMatrix() const
{
	return mShapeMatrix;
}

void 
BodyPart::setShapeTransform(const glm::vec3& pOrigin, const glm::vec3& pBasis)
{
	mShapeTransform.setOrigin(btVector3(pOrigin.x, pOrigin.y, pOrigin.z));

	btMatrix3x3 _basis;
	_basis.setEulerYPR(pBasis.x, pBasis.y, pBasis.z);
	mShapeTransform.setBasis(_basis);

	float glTransform[16];
	mShapeTransform.getOpenGLMatrix(glTransform);
	mShapeMatrix = glm::make_mat4(glTransform);
}