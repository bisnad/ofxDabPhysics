/** \file dab_physics_body_part.h
*/

#pragma once

#include <btBulletDynamicsCommon.h>
#include "dab_value.h"
#include "ofVectorMath.h"

namespace dab
{

namespace physics
{

class Simulation;
class Body;
class BodyShape;
class BodyJoint;

class BodyPart
{
public:
	BodyPart();
	BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass);
	BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass, const glm::vec3& pPosition, const glm::quat& pOrientation);
	BodyPart(const std::string& pName, std::shared_ptr<BodyShape> pShape, float pMass, const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans);

	~BodyPart();

	int id() const;
	const std::string& name() const;
	bool isCompound() const;
	float mass() const;
	const std::vector<float>& masses() const;
	float linearDamping() const;
	float angularDamping() const;
	float friction() const;
	float rollingFriction() const;
	float restitution() const;

	std::shared_ptr<BodyShape> shape() const;
	std::shared_ptr<Body> body() const;
	btRigidBody* nativeBody();
	const btRigidBody* nativeBody() const;

	const std::vector< std::shared_ptr<BodyJoint> >& nextJoints();
	const std::vector< std::shared_ptr<BodyJoint> >& prevJoints();

	void setMass(float pMass);
	void setLinearDamping(float pLinearDamping);
	void setAngularDamping(float pAngularDamping);
	void setFriction(float pFriction);
	void setRollingFriction(float pRollingFriction);
	void setRestitution(float pRestitution);

	void set(const std::string& pParName, const AbstractValue& pValue) throw (Exception);

	const btTransform& transform() const;
	glm::vec3 position() const;
	glm::quat rotation() const;
	const glm::mat4& matrix();
	glm::vec3 linearVelocity() const;
	glm::vec3 angularVelocity() const;

	void setPosition(const glm::vec3& pPosition);
	void setPosition(const btVector3& pPosition);
	void setRotation(const glm::quat& pRotation);
	void setRotation(const btQuaternion& pRotation);
	void setLinearVelocity(const glm::vec3& pVelocity);
	void setLinearVelocity(const btVector3& pVelocity);
	void setAngularVelocity(const glm::vec3& pVelocity);
	void setAngularVelocity(const btVector3& pVelocity);

	void applyForce(const glm::vec3& pForce);
	void applyForce(const glm::vec3& pForce, const glm::vec3& pPosition);
	void applyTorque(const glm::vec3& pTorque);
	void clearForces();

	friend class Simulation;
	friend class BodyJoint;
	friend class BodyMotor;

	const glm::mat4& shapeMatrix() const;
	const btTransform& nativeShapeTransform() const;
	void setShapeTransform(const glm::vec3& pOrigin, const glm::vec3& pBasis);

protected:
	static int sId;
	static float sMass;
	static float sLinearDamping;
	static float sAngularDamping;
	static float sFriction;
	static float sRollingFriction;
	static float sRestitution;

	int mId;
	std::string mName;
	float mMass;

	float mLinearDamping;
	float mAngularDamping;
	float mFriction;
	float mRollingFriction;
	float mRestitution;

	glm::mat4 mMatrix;

	std::shared_ptr<BodyShape> mShape;
	std::shared_ptr<Body> mBody;
	std::vector< std::shared_ptr<BodyJoint> > mNextJoints;
	std::vector< std::shared_ptr<BodyJoint> > mPrevJoints;

	btRigidBody* mNativeBody;

	void initPhysics();
	void initPhysics(const glm::vec3& pPosition, const glm::quat& pOrientation);
	void initPhysics(const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans);

	void calcInertia(btVector3& pInertia);

	btTransform mShapeTransform;
	glm::mat4 mShapeMatrix;
};

};

};