/** \file dab_physics_body_shape.cpp
*/

#include "dab_physics_body_shape.h"

using namespace dab;
using namespace dab::physics;

#pragma mark BodyShape implementation

int BodyShape::sId = 0;
BodyShape::BodyShape(const std::string& pName)
	: mId(sId++)
	, mName(pName)
	, mShape(nullptr)
{}

BodyShape::~BodyShape()
{
	if (mShape != nullptr) delete mShape;
}

int
BodyShape::id() const
{
	return mId;
}

const std::string&
BodyShape::name() const
{
	return mName;
}

btCollisionShape*
BodyShape::nativeShape()
{
	return mShape;
}

const btCollisionShape*
BodyShape::nativeShape() const
{
	return mShape;
}

#pragma mark BodyPlaneShape implementation

glm::vec3 BodyPlaneShape::sNormal = glm::vec3(0.0, 1.0, 0.0);

BodyPlaneShape::BodyPlaneShape(const std::string& pName)
	: BodyShape(pName)
	, mNormal(sNormal)
{
	initPhysics();
}

BodyPlaneShape::BodyPlaneShape(const std::string& pName, const glm::vec3& pNormal)
	: BodyShape(pName)
	, mNormal(pNormal)
{
	initPhysics();
}

BodyPlaneShape::~BodyPlaneShape()
{}

void 
BodyPlaneShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btStaticPlaneShape(btVector3(mNormal.x, mNormal.y, mNormal.z), 0);
}


#pragma mark BodySphereShape implementation

float BodySphereShape::sRadius = 1.0;

BodySphereShape::BodySphereShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
{
	initPhysics();
}

BodySphereShape::BodySphereShape(const std::string& pName, float pRadius)
	: BodyShape(pName)
	, mRadius(pRadius)
{
	initPhysics();
}

BodySphereShape::~BodySphereShape()
{}

void
BodySphereShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btSphereShape(mRadius);
}

#pragma mark BodyBoxShape implementation

glm::vec3 BodyBoxShape::sSize = glm::vec3(1.0, 1.0, 1.0);

BodyBoxShape::BodyBoxShape(const std::string& pName)
	: BodyShape(pName)
	, mSize(sSize)
{
	initPhysics();
}

BodyBoxShape::BodyBoxShape(const std::string& pName, const glm::vec3& pSize)
	: BodyShape(pName)
	, mSize(pSize)
{
	initPhysics();
}

BodyBoxShape::~BodyBoxShape()
{}

void
BodyBoxShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btBoxShape(btVector3(mSize[0] * 0.5, mSize[1] * 0.5, mSize[2] * 0.5));
}

#pragma mark BodyCylinderShape implementation

glm::vec3 BodyCylinderShape::sSize = glm::vec3(1.0, 1.0, 1.0);

BodyCylinderShape::BodyCylinderShape(const std::string& pName)
	: BodyShape(pName)
	, mSize(sSize)
{
	initPhysics();
}

BodyCylinderShape::BodyCylinderShape(const std::string& pName, const glm::vec3& pSize)
	: BodyShape(pName)
	, mSize(pSize)
{
	initPhysics();
}

BodyCylinderShape::~BodyCylinderShape()
{}

void
BodyCylinderShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btCylinderShape(btVector3(mSize[0] * 0.5, mSize[1] * 0.5, mSize[2] * 0.5));
}

#pragma mark BodyCapsuleShape implementation

float BodyCapsuleShape::sRadius = 1.0;
float BodyCapsuleShape::sHeight = 1.0;

BodyCapsuleShape::BodyCapsuleShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mHeight(sHeight)
{
	initPhysics();
}

BodyCapsuleShape::BodyCapsuleShape(const std::string& pName, float pRadius, float pHeight)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mHeight(pHeight)
{
	initPhysics();
}

BodyCapsuleShape::~BodyCapsuleShape()
{}

void
BodyCapsuleShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btCapsuleShape(mRadius, mHeight);
}

#pragma mark BodyConeShape implementation

float BodyConeShape::sRadius = 1.0;
float BodyConeShape::sHeight = 1.0;

BodyConeShape::BodyConeShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mHeight(sHeight)
{
	initPhysics();
}

BodyConeShape::BodyConeShape(const std::string& pName, float pRadius, float pHeight)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mHeight(pHeight)
{
	initPhysics();
}

BodyConeShape::~BodyConeShape()
{}

void
BodyConeShape::initPhysics()
{
	if (mShape != nullptr) return;

	mShape = new btConeShape(mRadius, mHeight);
}

#pragma mark BodyConvexMeshShape implementation

BodyConvexMeshShape::BodyConvexMeshShape(const std::string& pName, const std::string& pMeshName, const std::vector<glm::vec3>& pVertices)
	: BodyShape(pName)
	, mMeshName(pMeshName)
	, mVertices(pVertices)
{
	initPhysics();
}

BodyConvexMeshShape::~BodyConvexMeshShape()
{}

void 
BodyConvexMeshShape::initPhysics()
{
	if (mShape != nullptr) return;

	btConvexHullShape* _shape = new btConvexHullShape();

	int vertexCount = mVertices.size();
	for (auto vertex : mVertices)
	{
		_shape->addPoint( btVector3(vertex.x, vertex.y, vertex.z) );
	}

	_shape->optimizeConvexHull();
	_shape->initializePolyhedralFeatures();
	mShape = _shape;
}

const std::string& 
BodyConvexMeshShape::meshName() const
{
	return mMeshName;
}

#pragma mark BodyCompoundShape implementation

BodyCompoundShape::BodyCompoundShape(const std::string& pName)
	: BodyShape(pName)
{
	initPhysics();
}

BodyCompoundShape::~BodyCompoundShape()
{
	mChildShapes.clear();
}

int
BodyCompoundShape::childCount() const
{
	return mChildShapes.size();
}

const std::vector< btTransform >&
BodyCompoundShape::childTransforms() const
{
	return mChildTransforms;
}

const std::vector< std::shared_ptr<BodyShape> >&
BodyCompoundShape::childShapes() const
{
	return mChildShapes;
}

void
BodyCompoundShape::addChildShape(std::shared_ptr<BodyShape> pShape, const glm::vec3& pPosition, const glm::quat& pOrientation)
{
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(btVector3(pPosition[0], pPosition[1], pPosition[2]));
	transform.setRotation(btQuaternion(pOrientation[0], pOrientation[1], pOrientation[2], pOrientation[3]));

	mChildTransforms.push_back(transform);
	mChildShapes.push_back(pShape);

	btCompoundShape* _compoundShape = static_cast<btCompoundShape*>(mShape);
	_compoundShape->addChildShape(transform, pShape->nativeShape());
}

void 
BodyCompoundShape::addChildShape(std::shared_ptr<BodyShape> pShape, const btTransform& pTransform)
{
	mChildTransforms.push_back(pTransform);
	mChildShapes.push_back(pShape);

	btCompoundShape* _compoundShape = static_cast<btCompoundShape*>(mShape);
	_compoundShape->addChildShape(pTransform, pShape->nativeShape());

	//// debug
	//{
	//	btVector3 prevPos = pTransform.getOrigin();
	//	btQuaternion prevRot = pTransform.getRotation();
	//	btMatrix3x3 prevBasis = pTransform.getBasis();

	//	std::cout << "pos " << prevPos.x() << " " << prevPos.y() << " " << prevPos.z() << "\n";
	//	std::cout << "rot " << prevRot.x() << " " << prevRot.y() << " " << prevRot.z() << prevRot.w() << "\n";
	//	std::cout << "basis 0 " << prevBasis[0].x() << " " << prevBasis[0].y() << " " << prevBasis[0].z() << "\n";
	//	std::cout << "basis 1 " << prevBasis[1].x() << " " << prevBasis[1].y() << " " << prevBasis[1].z() << "\n";
	//	std::cout << "basis 2 " << prevBasis[2].x() << " " << prevBasis[2].y() << " " << prevBasis[2].z() << "\n";
	//}
}

void
BodyCompoundShape::initPhysics()
{
	if (mShape != nullptr) return;

	btCompoundShape* compoundShape = new btCompoundShape();

	//int childCount = mChildShapes.size();

	//for (int cI = 0; cI < childCount; ++cI)
	//{
	//	btTransform& transform = mChildTransforms[cI];
	//	mChildShapes[cI]->initPhysics();
	//	btCollisionShape* shape = mChildShapes[cI]->mShape;

	//	compoundShape->addChildShape(transform, shape);
	//}

	mShape = compoundShape;
}