/** \file dab_vis_body_shape.cpp
*/

#include "dab_vis_body_shape.h"
#include "dab_math_vec.h"

using namespace dab;
using namespace dab::vis;

int BodyShape::sId = 0;

BodyShape::BodyShape(const std::string& pName)
	: mId(sId++)
	, mName(pName)
	, mVisPrimitive(nullptr)
{}


BodyShape::~BodyShape()
{}

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

BodyTransform& 
BodyShape::transform()
{
	return mTransform;
}

const BodyTransform& 
BodyShape::transform() const
{
	return mTransform;
}

Material&
BodyShape::material()
{
	return mMaterial;
}

void 
BodyShape::drawFaces(const glm::mat4& pPreTransform, const ofShader& pShader)
{
	if (mVisPrimitive == nullptr) return;

	glm::mat4 _transform = pPreTransform * mTransform.matrix();

	pShader.setUniformMatrix4f("ModelMatrix", _transform);
	pShader.setUniform1f("Alpha", 1.0 - mMaterial.transparency());
	pShader.setUniform1f("AmbientScale", mMaterial.ambientScale());
	pShader.setUniform1f("DiffuseScale", mMaterial.diffuseScale());
	pShader.setUniform1f("SpecularScale", mMaterial.specularScale());
	pShader.setUniform1f("SpecularPow", mMaterial.specularPow());
	pShader.setUniform3f("AmbientColor", mMaterial.ambientColor());
	pShader.setUniform3f("DiffuseColor", mMaterial.diffuseColor());

	mVisPrimitive->drawFaces();
}

void 
BodyShape::drawWireframe(const glm::mat4& pPreTransform, const ofShader& pShader)
{
	if (mVisPrimitive == nullptr) return;

	glm::mat4 _transform = pPreTransform * mTransform.matrix();

	pShader.setUniformMatrix4f("ModelMatrix", _transform);
	pShader.setUniform1f("Alpha", 1.0 - mMaterial.transparency());
	pShader.setUniform1f("AmbientScale", mMaterial.ambientScale());
	pShader.setUniform1f("DiffuseScale", mMaterial.diffuseScale());
	pShader.setUniform1f("SpecularScale", mMaterial.specularScale());
	pShader.setUniform1f("SpecularPow", mMaterial.specularPow());
	pShader.setUniform3f("AmbientColor", mMaterial.ambientColor());
	pShader.setUniform3f("DiffuseColor", mMaterial.diffuseColor());

	mVisPrimitive->drawWireframe();
}


#pragma mark BodyPlaneShape implementation

float BodyPlaneShape::sSize = 100.0;
int BodyPlaneShape::sResolution = 10;
glm::vec3 BodyPlaneShape::sNormal = glm::vec3(0.0, 1.0, 0.0);

BodyPlaneShape::BodyPlaneShape(const std::string& pName)
	: BodyShape(pName)
	, mSize(sSize)
	, mResolution(sResolution)
	, mNormal(sNormal)
{
	initVisuals();
}

BodyPlaneShape::BodyPlaneShape(const std::string& pName, const glm::vec3& pNormal)
	: BodyShape(pName)
	, mSize(sSize)
	, mResolution(sResolution)
	, mNormal(pNormal)
{
	initVisuals();
}

BodyPlaneShape::~BodyPlaneShape()
{}

void
BodyPlaneShape::initVisuals()
{
	glm::quat shapeRot = math::VectorMath::get().makeQuaternion(glm::vec3(0.0, 1.0, 0.0), mNormal);
	mTransform.setOrientation(shapeRot);

	mVisPrimitive = std::make_shared<ofPlanePrimitive>(mSize, mSize, mResolution, mResolution);
}

void 
BodyPlaneShape::setSize(float pSize)
{
	sSize = pSize;
}

void 
BodyPlaneShape::setResolution(int pResolution)
{
	sResolution = pResolution;
}

void 
BodyPlaneShape::setNormal(const glm::vec3& pNormal)
{
	sNormal = pNormal;
}

#pragma mark BodySphereShape implementation

float BodySphereShape::sRadius = 1.0;
int BodySphereShape::sResolution = 24;

BodySphereShape::BodySphereShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mResolution(sResolution)
{
	initVisuals();
}

BodySphereShape::BodySphereShape(const std::string& pName, float pRadius)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mResolution(sResolution)
{
	initVisuals();
}

BodySphereShape::~BodySphereShape()
{}

void
BodySphereShape::initVisuals()
{
	mVisPrimitive = std::make_shared<ofSpherePrimitive>(mRadius, mResolution);
}

void 
BodySphereShape::setRadius(float pRadius)
{
	sRadius = pRadius;
}

void 
BodySphereShape::setResolution(int pResolution)
{
	sResolution = pResolution;
}

#pragma mark BodyBoxShape implementation

std::array<float, 3>  BodyBoxShape::sSize = {1.0, 1.0, 1.0};
std::array<int, 3> BodyBoxShape::sResolution = { 1, 1, 1 };

BodyBoxShape::BodyBoxShape(const std::string& pName)
	: BodyShape(pName)
	, mSize(sSize)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyBoxShape::BodyBoxShape(const std::string& pName, const std::array<float, 3>& pSize)
	: BodyShape(pName)
	, mSize(pSize)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyBoxShape::~BodyBoxShape()
{}

void
BodyBoxShape::initVisuals()
{
	mVisPrimitive = std::make_shared<ofBoxPrimitive>(mSize[0], mSize[1], mSize[2], mResolution[0], mResolution[1], mResolution[2]);
}

void 
BodyBoxShape::setSize(const std::array<float, 3>& pSize)
{
	sSize = pSize;
}

void 
BodyBoxShape::setResolution(const std::array<int, 3>& pResolution)
{
	sResolution = pResolution;
}

#pragma mark BodyCylinderShape implementation

float BodyCylinderShape::sRadius = 1.0;
float BodyCylinderShape::sHeight = 1.0;
std::array<int, 2> BodyCylinderShape::sResolution = { 24, 1 };


BodyCylinderShape::BodyCylinderShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mHeight(sHeight)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyCylinderShape::BodyCylinderShape(const std::string& pName, float pRadius, float pHeight)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mHeight(pHeight)
	, mResolution(sResolution)
{
	initVisuals();
}
	
BodyCylinderShape::~BodyCylinderShape()
{}

void
BodyCylinderShape::initVisuals()
{
	mVisPrimitive = std::make_shared<ofCylinderPrimitive>(mRadius, mHeight, sResolution[0], sResolution[1]);
}

void 
BodyCylinderShape::setRadius(float pRadius)
{
	sRadius = pRadius;
}

void 
BodyCylinderShape::setHeight(float pHeight)
{
	sHeight = pHeight;
}

void 
BodyCylinderShape::setResolution(const std::array<int, 2>& pResolution)
{
	sResolution = pResolution;
}

#pragma mark BodyCapsuleShape implementation

float BodyCapsuleShape::sRadius = 1.0;
float BodyCapsuleShape::sHeight = 1.0;
std::array<int, 2> BodyCapsuleShape::sResolution = { 24, 1 };


BodyCapsuleShape::BodyCapsuleShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mHeight(sHeight)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyCapsuleShape::BodyCapsuleShape(const std::string& pName, float pRadius, float pHeight)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mHeight(pHeight)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyCapsuleShape::~BodyCapsuleShape()
{}

void
BodyCapsuleShape::initVisuals()
{
	//TODO: create a custom capsule mesh instead of a cylinder primite
	mVisPrimitive = std::make_shared<ofCylinderPrimitive>(mRadius, mHeight, sResolution[0], sResolution[1]);
}

void
BodyCapsuleShape::setRadius(float pRadius)
{
	sRadius = pRadius;
}

void
BodyCapsuleShape::setHeight(float pHeight)
{
	sHeight = pHeight;
}

void
BodyCapsuleShape::setResolution(const std::array<int, 2>& pResolution)
{
	sResolution = pResolution;
}

#pragma mark BodyConeShape implementation

float BodyConeShape::sRadius = 1.0;
float BodyConeShape::sHeight = 1.0;
std::array<int, 3> BodyConeShape::sResolution = { 24, 1, 1 };


BodyConeShape::BodyConeShape(const std::string& pName)
	: BodyShape(pName)
	, mRadius(sRadius)
	, mHeight(sHeight)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyConeShape::BodyConeShape(const std::string& pName, float pRadius, float pHeight)
	: BodyShape(pName)
	, mRadius(pRadius)
	, mHeight(pHeight)
	, mResolution(sResolution)
{
	initVisuals();
}

BodyConeShape::~BodyConeShape()
{}

void
BodyConeShape::initVisuals()
{
	// of cone shape is flipped along y axis compared to bullet cone shape 
	mTransform.setScale(glm::vec3(1.0, -1.0, 1.0));

	mVisPrimitive = std::make_shared<ofConePrimitive>(mRadius, mHeight, sResolution[0], sResolution[1], sResolution[2]);
}

void
BodyConeShape::setRadius(float pRadius)
{
	sRadius = pRadius;
}

void
BodyConeShape::setHeight(float pHeight)
{
	sHeight = pHeight;
}

void
BodyConeShape::setResolution(const std::array<int, 3>& pResolution)
{
	sResolution = pResolution;
}


#pragma mark BodyMeshShape implementation

BodyMeshShape::BodyMeshShape(const std::string& pName, const std::string& pMeshName, std::shared_ptr<ofMesh> pMesh)
	: BodyShape(pName)
	, mMeshName(pMeshName)
	, mMesh(pMesh)
{
	initVisuals();
}

BodyMeshShape::~BodyMeshShape()
{}

void
BodyMeshShape::initVisuals() throw (dab::Exception)
{
	mVisPrimitive = std::make_shared<of3dPrimitive>(*mMesh);
}

const std::string& 
BodyMeshShape::meshName() const
{
	return mMeshName;
}

#pragma mark BodyCompoundShape implementation

BodyCompoundShape::BodyCompoundShape(const std::string& pName)
	: BodyShape(pName)
{}

BodyCompoundShape::~BodyCompoundShape()
{}

void
BodyCompoundShape::initVisuals()
{}

int 
BodyCompoundShape::childCount() const
{
	return mChildShapes.size();
}

const std::vector< BodyTransform >& 
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
	BodyTransform _transform;
	_transform.setPosition(pPosition);
	_transform.setOrientation(pOrientation);

	addChildShape(pShape, _transform);
}

void 
BodyCompoundShape::addChildShape(std::shared_ptr<BodyShape> pShape, const BodyTransform& pTransform)
{
	mChildShapes.push_back(pShape);
	mChildTransforms.push_back(pTransform);
}

void 
BodyCompoundShape::drawFaces(const glm::mat4& pPreTransform, const ofShader& pShader)
{
	int childCount = mChildShapes.size();

	for (int cI = 0; cI < childCount; ++cI)
	{
		const BodyTransform& childTransform = mChildTransforms[cI];

		glm::mat4 _transform = pPreTransform * mTransform.matrix() * childTransform.matrix();

		mChildShapes[cI]->drawFaces(_transform, pShader);
	}
}

void 
BodyCompoundShape::drawWireframe(const glm::mat4& pPreTransform, const ofShader& pShader)
{
	int childCount = mChildShapes.size();

	for (int cI = 0; cI < childCount; ++cI)
	{
		const BodyTransform& childTransform = mChildTransforms[cI];

		glm::mat4 _transform = pPreTransform * mTransform.matrix() * childTransform.matrix();

		mChildShapes[cI]->drawWireframe(_transform, pShader);
	}
}

