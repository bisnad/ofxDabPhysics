/** \file dab_physics_body_shape.h
*/

#pragma once

#include <btBulletDynamicsCommon.h>
#include "ofVectorMath.h"

namespace dab
{

namespace physics
{

#pragma mark BodyShape definition

class BodyShape 
{
public:
	BodyShape(const std::string& pName);
	virtual ~BodyShape();

	int id() const;
	const std::string& name() const;
	btCollisionShape* nativeShape();
	const btCollisionShape* nativeShape() const;

	virtual void initPhysics() = 0;

	friend class BodyPart;
	friend class BodyCompoundShape;

protected:
	static int sId;

	int mId;
	std::string mName;

	btCollisionShape* mShape;
};

#pragma mark BodyPlaneShape definition

class BodyPlaneShape : public BodyShape
{
public:
	BodyPlaneShape(const std::string& pName);
	BodyPlaneShape(const std::string& pName, const glm::vec3& pNormal);
	~BodyPlaneShape();

	void initPhysics();

protected:
	static glm::vec3 sNormal;

	glm::vec3 mNormal;
};

#pragma mark BodySphereShape definition

class BodySphereShape : public BodyShape
{
public:
	BodySphereShape(const std::string& pName);
	BodySphereShape(const std::string& pName, float pRadius);
	~BodySphereShape();

	void initPhysics();

protected:
	static float sRadius;

	float mRadius;
};

#pragma mark BodyBoxShape definition

class BodyBoxShape : public BodyShape
{
public:
	BodyBoxShape(const std::string& pName);
	BodyBoxShape(const std::string& pName, const glm::vec3& pSize);
	~BodyBoxShape();

	void initPhysics();

protected:
	static glm::vec3 sSize;

	glm::vec3 mSize;
};

#pragma mark BodyCylinderShape definition

class BodyCylinderShape : public BodyShape
{
public:
	BodyCylinderShape(const std::string& pName);
	BodyCylinderShape(const std::string& pName, const glm::vec3& pSize);
	~BodyCylinderShape();

	void initPhysics();

protected:
	static glm::vec3 sSize;

	glm::vec3 mSize;
};

#pragma mark BodyCapsuleShape definition

class BodyCapsuleShape : public BodyShape
{
public:
	BodyCapsuleShape(const std::string& pName);
	BodyCapsuleShape(const std::string& pName, float pRadius, float pHeight);
	~BodyCapsuleShape();

	void initPhysics();

protected:
	static float sRadius;
	static float sHeight;

	float mRadius;
	float mHeight;
};

#pragma mark BodyConeShape definition

class BodyConeShape : public BodyShape
{
public:
	BodyConeShape(const std::string& pName);
	BodyConeShape(const std::string& pName, float pRadius, float pHeight);
	~BodyConeShape();

	void initPhysics();

protected:
	static float sRadius;
	static float sHeight;

	float mRadius;
	float mHeight;
};

#pragma mark BodyConvexMeshShape definition

class BodyConvexMeshShape : public BodyShape
{
public:
	BodyConvexMeshShape(const std::string& pName, const std::string& pMeshName, const std::vector<glm::vec3>& pVertices);
	~BodyConvexMeshShape();

	void initPhysics();

	const std::string& meshName() const;

protected:
	std::string mMeshName;
	std::vector<glm::vec3> mVertices;
};

#pragma mark ConcaveMeshShape definition

// uses internally btGImpactMeshShape
// TODO: figure out how to implement this

class ConcaveMeshShape : public BodyShape
{
	/*
	btGImpactMeshShape(btStridingMeshInterface* meshInterface)
	*/
};

#pragma mark BodyCompoundShape definition

class BodyCompoundShape : public BodyShape
{
public:
	BodyCompoundShape(const std::string& pName);
	~BodyCompoundShape();

	int childCount() const;
	const std::vector< btTransform >& childTransforms() const;
	const std::vector< std::shared_ptr<BodyShape> >& childShapes() const;

	void addChildShape(std::shared_ptr<BodyShape> pShape, const glm::vec3& pPosition, const glm::quat& pOrientation);
	void addChildShape(std::shared_ptr<BodyShape> pShape, const btTransform& pTransform);

	void initPhysics();

	friend class BodyPart;
protected:
	std::vector< btTransform > mChildTransforms;
	std::vector< std::shared_ptr<BodyShape> > mChildShapes;
};

};

};