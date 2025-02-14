/** \file dab_vis_body_visualization.cpp
*/

#include "dab_vis_body_visualization.h"
#include "dab_geom_mesh_manager.h"
#include "dab_vis_camera.h"
#include "dab_vis_body.h"
#include "dab_vis_body_shape.h"
#include "dab_vis_body_part.h"
#include "dab_physics_simulation.h"
#include "dab_physics_body.h"
#include "dab_physics_body_shape.h"
#include "dab_physics_body_part.h"

using namespace dab;
using namespace dab::vis;

BodyVisualization::BodyVisualization()
	: mCamera(new Camera())
{
	loadShader(mShapeShader, "shaders/shape_shader.vert", "shaders/shape_shader.frag");

	mVisTargetPositionMaterial = std::shared_ptr<Material>(new Material());
	mVisTargetPositionMaterial->setTransparency(0.0);
	mVisTargetPositionMaterial->setAmbientScale(1.0);
	mVisTargetPositionMaterial->setDiffuseScale(0.5);
	mVisTargetPositionMaterial->setSpecularScale(1.0);
	mVisTargetPositionMaterial->setSpecularPow(4.0);
	mVisTargetPositionMaterial->setAmbientColor(glm::vec3(1.0, 0.0, 0.0));
	mVisTargetPositionMaterial->setDiffuseColor(glm::vec3(1.0, 1.0, 1.0));
}

BodyVisualization::~BodyVisualization()
{}

const std::vector<std::shared_ptr<Body>>& 
BodyVisualization::bodies() const
{
	return mBodies.values();
}

const std::vector<std::shared_ptr<BodyShape>>& 
BodyVisualization::shapes() const
{
	return mShapes.values();
}

const std::vector<std::shared_ptr<BodyPart>>& 
BodyVisualization::parts() const
{
	return mParts;
}

const std::vector<std::shared_ptr<Material>>& 
BodyVisualization::materials() const
{
	return mMaterials.values();
}

bool
BodyVisualization::bodyExists(const std::string& pBodyName)
{
	return mBodies.contains(pBodyName);
}

bool 
BodyVisualization::shapeExists(const std::string& pShapeName)
{
	return mShapes.contains(pShapeName);
}

bool 
BodyVisualization::partExists(const std::string& pBodyName, const std::string& pPartName)
{
	if (mBodies.contains(pBodyName) == false) return false;

	return mBodies[pBodyName]->mParts.contains(pPartName);
}

bool
BodyVisualization::materialExists(const std::string& pMaterialName)
{
	return mMaterials.contains(pMaterialName);
}

std::shared_ptr<Camera> 
BodyVisualization::camera()
{
	return mCamera;
}

std::shared_ptr<Body>
BodyVisualization::body(const std::string& pBodyName) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Vis Error: body " + pBodyName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mBodies[pBodyName];
}

std::shared_ptr<BodyShape>
BodyVisualization::shape(const std::string& pShapeName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == false) throw dab::Exception("Vis Error: body shape " + pShapeName + " not found", __FILE__, __FUNCTION__, __LINE__);
		
	return mShapes[pShapeName];
}

std::shared_ptr<BodyPart>
BodyVisualization::part(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception)
{
	if(mBodies.contains(pBodyName) == false) throw dab::Exception("Vis Error: body " + pBodyName + " not found", __FILE__, __FUNCTION__, __LINE__);
	if (mBodies[pBodyName]->mParts.contains(pPartName) == false) throw dab::Exception("Vis Error: body part " + pPartName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mBodies[pBodyName]->mParts[pPartName];
}

std::shared_ptr<Material>
BodyVisualization::material(const std::string& pMaterialName) throw (dab::Exception)
{
	if (mMaterials.contains(pMaterialName) == false) throw dab::Exception("Vis Error: material " + pMaterialName + " not found", __FILE__, __FUNCTION__, __LINE__);

	return mMaterials[pMaterialName];
}

std::shared_ptr<Body>
BodyVisualization::addBody(const std::string& pBodyName) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == true) throw dab::Exception("Vis Error: failed to create body " + pBodyName + " since it already exists", __FILE__, __FUNCTION__, __LINE__);

	physics::Simulation& physics = physics::Simulation::get();
	std::shared_ptr<physics::Body> physicsBody;

	try
	{
		physicsBody = physics.body(pBodyName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("VIS ERROR: failed to create body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	std::shared_ptr<Body> _body(new Body(physicsBody));
	mBodies.add(pBodyName, _body);

	return _body;
}

std::shared_ptr<Body> 
BodyVisualization::copyBody(const std::string& pSourceBodyName, const std::string& pTargetBodyName) throw (dab::Exception)
{
	if (mBodies.contains(pSourceBodyName) == false) throw dab::Exception("Vis Error: body " + pTargetBodyName + " not found", __FILE__, __FUNCTION__, __LINE__);
	
	try
	{
		std::shared_ptr<Body> _sourceBody = mBodies[pSourceBodyName];
		std::shared_ptr<Body> _targetBody = addBody(pTargetBodyName);

		int partCount = _sourceBody->mParts.size();

		for (int pI=0; pI< partCount; ++pI)
		{
			std::shared_ptr<BodyPart> _sourcePart = _sourceBody->mParts[pI];
			std::shared_ptr<BodyPart> _targetPart = addBodyPart(pTargetBodyName, _sourcePart->physicsBodyPart()->name(), _sourcePart->bodyShape()->name());
		
			// TODO: copy visual settings
		}


		return _targetBody;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("VIS ERROR: failed to copy body from source body " + pSourceBodyName + " to target body " + pTargetBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
	
}

std::shared_ptr<BodyPlaneShape> 
BodyVisualization::addPlaneShape(const std::string& pShapeName, const glm::vec3& pNormal) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create plane shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyPlaneShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyPlaneShape>(new BodyPlaneShape(pShapeName, { pNormal.x, pNormal.y, pNormal.z } ));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add plane shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodySphereShape> 
BodyVisualization::addSphereShape(const std::string& pShapeName, float pRadius) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create sphere shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodySphereShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodySphereShape>(new BodySphereShape(pShapeName, pRadius));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add sphere shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyBoxShape> 
BodyVisualization::addBoxShape(const std::string& pShapeName, const std::array<float, 3>& pSize) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create box shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyBoxShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyBoxShape>(new BodyBoxShape(pShapeName, pSize));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add box shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyCylinderShape> 
BodyVisualization::addCylinderShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create cylinder shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCylinderShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyCylinderShape>(new BodyCylinderShape(pShapeName, pRadius, pHeight));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add cylinder shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyCapsuleShape> 
BodyVisualization::addCapsuleShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create capsule shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCapsuleShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyCapsuleShape>(new BodyCapsuleShape(pShapeName, pRadius, pHeight));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add capsule shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyConeShape> 
BodyVisualization::addConeShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create cone shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyConeShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyConeShape>(new BodyConeShape(pShapeName, pRadius, pHeight));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add cone shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyMeshShape>
BodyVisualization::addMeshShape(const std::string& pShapeName, const std::string& pMeshName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create mesh shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyMeshShape> _shape = nullptr;

	try
	{
		std::shared_ptr<ofMesh> _mesh = geom::MeshManager::get().mesh(pMeshName);
		_shape = std::shared_ptr<BodyMeshShape>(new BodyMeshShape(pShapeName, pMeshName, _mesh));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add mesh shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<BodyCompoundShape> 
BodyVisualization::addCompoundShape(const std::string& pShapeName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create cone shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCompoundShape> _shape = nullptr;

	try
	{
		_shape = std::shared_ptr<BodyCompoundShape>(new BodyCompoundShape(pShapeName));
		mShapes.add(pShapeName, _shape);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to add comound shape " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}


std::shared_ptr<BodyShape> 
BodyVisualization::addBodyShape(const std::string& pShapeName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Vis Error: failed to create shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	physics::Simulation& physics = physics::Simulation::get();
	std::shared_ptr<physics::BodyShape> physicsBodyShape = nullptr;
	std::shared_ptr<BodyShape> visualBodyShape = nullptr;

	try
	{
		physicsBodyShape = physics.shape(pShapeName);

		if (std::dynamic_pointer_cast<physics::BodySphereShape>(physicsBodyShape) != nullptr) visualBodyShape = addSphereShape(std::static_pointer_cast<physics::BodySphereShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyBoxShape>(physicsBodyShape) != nullptr) visualBodyShape = addBoxShape(std::static_pointer_cast<physics::BodyBoxShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyCylinderShape>(physicsBodyShape) != nullptr) visualBodyShape = addCylinderShape(std::static_pointer_cast<physics::BodyCylinderShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyCapsuleShape>(physicsBodyShape) != nullptr) visualBodyShape = addCapsuleShape(std::static_pointer_cast<physics::BodyCapsuleShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyConeShape>(physicsBodyShape) != nullptr) visualBodyShape = addConeShape(std::static_pointer_cast<physics::BodyConeShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyConvexMeshShape>(physicsBodyShape) != nullptr) visualBodyShape = addMeshShape(std::static_pointer_cast<physics::BodyConvexMeshShape>(physicsBodyShape));
		else if (std::dynamic_pointer_cast<physics::BodyCompoundShape>(physicsBodyShape) != nullptr) visualBodyShape = addCompoundShape(std::static_pointer_cast<physics::BodyCompoundShape>(physicsBodyShape));

		if (visualBodyShape != nullptr)
		{
			mShapes.add(pShapeName, visualBodyShape);
		}

	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to create body shape with name " + pShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return visualBodyShape;
}

std::shared_ptr<BodyPart> 
BodyVisualization::addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName) throw (dab::Exception)
{
	//std::cout << "BodyVisualization::addBodyPart body " << pBodyName << " part " << pPartName << " shape " << pShapeName << "\n";

	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Vis Error: Body " + pBodyName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mParts.contains(pPartName)) return _body->mParts[pPartName];

	physics::Simulation& physics = physics::Simulation::get();
	std::shared_ptr<physics::BodyPart> physicsBodyPart;

	try
	{
		physicsBodyPart = physics.part(pBodyName, pPartName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("VIS ERROR: failed to create body part " + pPartName + " for body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	if (mShapes.contains(pShapeName) == false)
	{
		throw dab::Exception("VIS ERROR: body shape " + pShapeName + " not found", __FILE__, __FUNCTION__, __LINE__);
	}


	std::shared_ptr<BodyShape> visualBodyShape = mShapes[pShapeName];
	std::shared_ptr<BodyPart> visualBodyPart(new BodyPart(physicsBodyPart, visualBodyShape));

	_body->mParts.add(pPartName, visualBodyPart);
	mParts.push_back(visualBodyPart);
	visualBodyPart->mBody = _body;

	return visualBodyPart;
}

std::shared_ptr<BodyPart> 
BodyVisualization::addBodyPart(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Vis Error: Body " + pBodyName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mParts.contains(pPartName)) return _body->mParts[pPartName];

	physics::Simulation& physics = physics::Simulation::get();
	std::shared_ptr<physics::BodyPart> physicsBodyPart;

	try
	{
		physicsBodyPart = physics.part(pBodyName, pPartName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("VIS ERROR: failed to create body part " + pPartName + " for body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	const std::shared_ptr<physics::BodyShape> physicsBodyShape = physicsBodyPart->shape();
	std::string shapeName = physicsBodyShape->name();
	std::shared_ptr<BodyShape> visualBodyShape;
	
	try
	{
		visualBodyShape = addBodyShape(shapeName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("VIS ERROR: failed to create body part " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	std::shared_ptr<BodyPart> visualBodyPart(new BodyPart(physicsBodyPart, visualBodyShape));

	_body->mParts.add(pBodyName, visualBodyPart);
	mParts.push_back(visualBodyPart);
	visualBodyPart->mBody = _body;

	return visualBodyPart;
}

void
BodyVisualization::addTargetPositions(int pPositionCount)
{
	std::shared_ptr<of3dPrimitive> _pos(new ofSpherePrimitive(0.05, 20));
	mVisTargetPositions.push_back(_pos);
}

void
BodyVisualization::setTargetPosition(int pPositionIndex, const glm::vec3& pTargetPosition) throw (dab::Exception)
{
	int targetPosCount = mVisTargetPositions.size();

	if (pPositionIndex >= targetPosCount) throw dab::Exception("Vis Error: index " + std::to_string(pPositionIndex) + " exceeds number of target positions " + std::to_string(targetPosCount), __FILE__, __FUNCTION__, __LINE__);

	mVisTargetPositions[pPositionIndex]->setGlobalPosition(pTargetPosition);
}


void 
BodyVisualization::update()
{
	int bodyPartCount = mParts.size();
	for (int bI = 0; bI < bodyPartCount; ++bI)
	{
		mParts[bI]->update();
	}
}

void 
BodyVisualization::draw()
{
	dab::vis::BodyVisualization& visuals = dab::vis::BodyVisualization::get();

	const glm::vec4& camProjection = visuals.camera()->projection();
	const glm::vec3& camPosition = visuals.camera()->position();
	const glm::vec3& camRotation = visuals.camera()->rotation();

	glm::mat4 projectionMatrix;
	glm::mat4 viewMatrix;

	projectionMatrix = glm::perspective(camProjection[0], camProjection[1], camProjection[2], camProjection[3]);

	viewMatrix = glm::translate(glm::mat4(1.0), camPosition);
	viewMatrix = viewMatrix * glm::toMat4(glm::angleAxis(glm::radians(camRotation[0]), glm::vec3(1.0, 0.0, 0.0)));
	viewMatrix = viewMatrix * glm::toMat4(glm::angleAxis(glm::radians(camRotation[1]), glm::vec3(0.0, 1.0, 0.0)));
	viewMatrix = viewMatrix * glm::toMat4(glm::angleAxis(glm::radians(camRotation[2]), glm::vec3(0.0, 0.0, 1.0)));

	glm::vec3 lightPosition(1.0, 2.0, -1.0);

	//draw bodies
	int bodyPartCount = mParts.size();

	mShapeShader.begin();

	mShapeShader.setUniformMatrix4f("ProjectionMatrix", projectionMatrix);
	mShapeShader.setUniformMatrix4f("ViewMatrix", viewMatrix);
	mShapeShader.setUniform3f("LightPosition", lightPosition);

	for (int bI = 0; bI < bodyPartCount; ++bI)
	{
		mParts[bI]->draw(mShapeShader);
	}

	// draw target
	int targetPosCount = mVisTargetPositions.size();
	for (int pI = 0; pI < targetPosCount; ++pI)
	{
		std::shared_ptr<of3dPrimitive> _visTargetPosition = mVisTargetPositions[pI];

		glm::mat4 modelMatrix = _visTargetPosition->getGlobalTransformMatrix();
		mShapeShader.setUniformMatrix4f("ModelMatrix", modelMatrix);
		mShapeShader.setUniform1f("Alpha", 1.0 - mVisTargetPositionMaterial->transparency());
		mShapeShader.setUniform1f("AmbientScale", mVisTargetPositionMaterial->ambientScale());
		mShapeShader.setUniform1f("DiffuseScale", mVisTargetPositionMaterial->diffuseScale());
		mShapeShader.setUniform1f("SpecularScale", mVisTargetPositionMaterial->specularScale());
		mShapeShader.setUniform1f("SpecularPow", mVisTargetPositionMaterial->specularPow());
		mShapeShader.setUniform3f("AmbientColor", mVisTargetPositionMaterial->ambientColor());
		mShapeShader.setUniform3f("DiffuseColor", mVisTargetPositionMaterial->diffuseColor());

		_visTargetPosition->drawFaces();
	}

	mShapeShader.end();
}

bool
BodyVisualization::loadShader(ofShader& pShader, const std::string& pVertexShader, const std::string& pFragShader)
{
	bool success;

	success = pShader.setupShaderFromFile(GL_VERTEX_SHADER, pVertexShader);
	if (success == false) std::cout << "shader " << pVertexShader << " failed: " << shaderErrorMessage(pShader, GL_VERTEX_SHADER) << "\n";

	success = pShader.setupShaderFromFile(GL_FRAGMENT_SHADER, pFragShader);
	if (success == false) std::cout << "shader " << pFragShader << " failed: " << shaderErrorMessage(pShader, GL_FRAGMENT_SHADER) << "\n";

	success = pShader.linkProgram();
	//std::cout << "link shader " << success << "\n";

	return success;
}

std::string
BodyVisualization::shaderErrorMessage(ofShader& pShader, GLenum pShaderType)
{
	GLsizei logLength;
	GLchar  errorLog[1024];
	glGetShaderInfoLog(pShader.getShader(pShaderType), sizeof(errorLog), &logLength, errorLog);

	std::cout << "Shader info log:\n" << errorLog << "\n";

	std::string errorString = errorLog;

	return errorString;
}

std::shared_ptr<BodySphereShape> 
BodyVisualization::addSphereShape(std::shared_ptr<physics::BodySphereShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const btCollisionShape* colShape = pPhysicsBodyShape->nativeShape();
	const btSphereShape* sphereShape = static_cast<const btSphereShape*>(colShape);
	float sphereRadius = sphereShape->getRadius();

	return std::shared_ptr<BodySphereShape>(new BodySphereShape(shapeName, sphereRadius));
}

std::shared_ptr<BodyBoxShape> 
BodyVisualization::addBoxShape(std::shared_ptr<physics::BodyBoxShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const btCollisionShape* colShape = pPhysicsBodyShape->nativeShape();
	const btBoxShape* boxShape = static_cast<const btBoxShape*>(colShape);
	const btVector3& boxHalfSize = boxShape->getHalfExtentsWithoutMargin();

	return std::shared_ptr<BodyBoxShape>(new BodyBoxShape(shapeName, { boxHalfSize[0] * 2.0f, boxHalfSize[1] * 2.0f, boxHalfSize[2] * 2.0f }));
}

std::shared_ptr<BodyCylinderShape> 
BodyVisualization::addCylinderShape(std::shared_ptr<physics::BodyCylinderShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const btCollisionShape* colShape = pPhysicsBodyShape->nativeShape();
	const btCylinderShape* cylinderShape = static_cast<const btCylinderShape*>(colShape);
	const btVector3& cylinderHalfSize = cylinderShape->getHalfExtentsWithoutMargin();

	return std::shared_ptr<BodyCylinderShape>(new BodyCylinderShape(shapeName, cylinderHalfSize[0], cylinderHalfSize[1] * 2.0f));
}

std::shared_ptr<BodyCapsuleShape>
BodyVisualization::addCapsuleShape(std::shared_ptr<physics::BodyCapsuleShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const btCollisionShape* colShape = pPhysicsBodyShape->nativeShape();
	const btCylinderShape* cylinderShape = static_cast<const btCylinderShape*>(colShape);
	const btVector3& cylinderHalfSize = cylinderShape->getHalfExtentsWithoutMargin();

	return std::shared_ptr<BodyCapsuleShape>(new BodyCapsuleShape(shapeName, cylinderHalfSize[0], cylinderHalfSize[1] * 2.0f));
}

std::shared_ptr<BodyConeShape> 
BodyVisualization::addConeShape(std::shared_ptr<physics::BodyConeShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const btCollisionShape* colShape = pPhysicsBodyShape->nativeShape();
	const btConeShape* coneShape = static_cast<const btConeShape*>(colShape);
	float radius = coneShape->getRadius();
	float height = coneShape->getHeight();

	return std::shared_ptr<BodyConeShape>(new BodyConeShape(shapeName, radius, height));
}

std::shared_ptr<BodyMeshShape>
BodyVisualization::addMeshShape(std::shared_ptr<physics::BodyConvexMeshShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();
	const std::string& meshName = pPhysicsBodyShape->meshName();

	std::shared_ptr<ofMesh> _mesh = geom::MeshManager::get().mesh(meshName);

	return std::shared_ptr<BodyMeshShape>(new BodyMeshShape(shapeName, meshName, _mesh));
}

std::shared_ptr<BodyCompoundShape> 
BodyVisualization::addCompoundShape(std::shared_ptr<physics::BodyCompoundShape> pPhysicsBodyShape)
{
	const std::string& shapeName = pPhysicsBodyShape->name();

	std::shared_ptr<BodyCompoundShape> _visCompoundShape(new BodyCompoundShape(shapeName));

	int childCount = pPhysicsBodyShape->childCount();
	const std::vector< btTransform >& childPhysicsTransforms = pPhysicsBodyShape->childTransforms();
	const std::vector< std::shared_ptr<physics::BodyShape> >& childPhysicsShapes = pPhysicsBodyShape->childShapes();

	for (int cI = 0; cI < childCount; ++cI)
	{
		const btTransform& physicsTransform = childPhysicsTransforms[cI];
		const btVector3& physicsPosition = physicsTransform.getOrigin();
		const btQuaternion& physicsRotation = physicsTransform.getRotation();

		BodyTransform visTransform;
		visTransform.setPosition(glm::vec3(physicsPosition.x(), physicsPosition.y(), physicsPosition.z()));
		visTransform.setOrientation(glm::quat(physicsRotation.w(), physicsRotation.x(), physicsRotation.y(), physicsRotation.z()));

		std::shared_ptr<physics::BodyShape> childPhysicsShape = childPhysicsShapes[cI];

		std::shared_ptr<BodyShape> _visShape = nullptr;

		if (std::dynamic_pointer_cast<physics::BodySphereShape>(childPhysicsShape) != nullptr) _visShape = addSphereShape(std::static_pointer_cast<physics::BodySphereShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyBoxShape>(childPhysicsShape) != nullptr) _visShape = addBoxShape(std::static_pointer_cast<physics::BodyBoxShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyCylinderShape>(childPhysicsShape) != nullptr) _visShape = addCylinderShape(std::static_pointer_cast<physics::BodyCylinderShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyCapsuleShape>(childPhysicsShape) != nullptr) _visShape = addCapsuleShape(std::static_pointer_cast<physics::BodyCapsuleShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyConeShape>(childPhysicsShape) != nullptr) _visShape = addConeShape(std::static_pointer_cast<physics::BodyConeShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyConvexMeshShape>(childPhysicsShape) != nullptr) _visShape = addMeshShape(std::static_pointer_cast<physics::BodyConvexMeshShape>(childPhysicsShape));
		else if (std::dynamic_pointer_cast<physics::BodyCompoundShape>(childPhysicsShape) != nullptr) _visShape = addCompoundShape(std::static_pointer_cast<physics::BodyCompoundShape>(childPhysicsShape));

		if (_visShape != nullptr)
		{
			_visCompoundShape->addChildShape(_visShape, visTransform);
		}
	}

	return _visCompoundShape;
}

void
BodyVisualization::addMaterial(const std::string& pMaterialName, const Material& pMaterial)  throw (dab::Exception)
{
	if (mMaterials.contains(pMaterialName) == true) throw dab::Exception("Vis Error: failed to create material, material with name " + pMaterialName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Material> _material( new Material(pMaterial) );
	mMaterials.add(pMaterialName, _material);
}

void
BodyVisualization::changeMaterial(const std::string& pMaterialName, const Material& pMaterial)  throw (dab::Exception)
{
	if (mMaterials.contains(pMaterialName) == false) throw dab::Exception("Vis Error: material with name " + pMaterialName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	*(mMaterials[pMaterialName]) = pMaterial;
}

void 
BodyVisualization::registerShapeMaterial(const std::string& pShapeName, const std::string& pMaterialName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == false) throw dab::Exception("Vis Error: shape with name " + pShapeName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (mMaterials.contains(pMaterialName) == false) throw dab::Exception("Vis Error: material with name " + pMaterialName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if(mShapeMaterialMap.contains(pShapeName) == true) throw dab::Exception("Vis Error: shape with name " + pShapeName + " already registered", __FILE__, __FUNCTION__, __LINE__);

	mShapeMaterialMap.add(pShapeName, pMaterialName);
}

void 
BodyVisualization::updateShapeMaterials()
{
	try
	{
		int shapeCount = mShapeMaterialMap.size();
		for (int sI = 0; sI < shapeCount; ++sI)
		{
			std::shared_ptr<BodyShape> _shape = shape(mShapeMaterialMap.key(sI));
			std::shared_ptr<Material> _material = material(mShapeMaterialMap.value(sI));

			_shape->material() = *_material;
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Vis Error: failed to update shape materials", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}
