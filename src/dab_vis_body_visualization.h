/** \file dab_vis_body_visualization.h
*/

#pragma once

#include "dab_singleton.h"
#include "dab_index_map.h"
#include "ofMesh.h"
#include "ofShader.h"
#include "ofxAssimpModelLoader.h"

namespace dab
{

namespace physics
{
class Body;
class BodyShape;
class BodyPlaneShape;
class BodySphereShape;
class BodyBoxShape;
class BodyCylinderShape;
class BodyCapsuleShape;
class BodyConeShape;
class BodyConvexMeshShape;
class BodyCompoundShape;
class BodyPart;
};

namespace vis
{

class Camera;
class Body;
class BodyShape;
class BodyPlaneShape;
class BodySphereShape;
class BodyBoxShape;
class BodyCylinderShape;
class BodyCapsuleShape;
class BodyConeShape;
class BodyMeshShape;
class BodyCompoundShape;
class BodyPart;
class Material;

class BodyVisualization : public Singleton<BodyVisualization>
{
public:
	BodyVisualization();
	~BodyVisualization();

	const std::vector<std::shared_ptr<Body>>& bodies() const;
	const std::vector<std::shared_ptr<BodyShape>>& shapes() const;
	const std::vector<std::shared_ptr<BodyPart>>& parts() const;
	const std::vector<std::shared_ptr<Material>>& materials() const;

	bool bodyExists(const std::string& pBodyName);
	bool shapeExists(const std::string& pShapeName);
	bool materialExists(const std::string& pMaterialName);
	bool partExists(const std::string& pBodyName, const std::string& pPartName);

	std::shared_ptr<Camera> camera();
	std::shared_ptr<Body> body(const std::string& pBodyName) throw (dab::Exception);
	std::shared_ptr<BodyShape> shape(const std::string& pShapeName) throw (dab::Exception);
	std::shared_ptr<Material> material(const std::string& pMaterialName) throw (dab::Exception);
	std::shared_ptr<BodyPart> part(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception);

	// functions to add visual shapes with custom specifications
	std::shared_ptr<BodyPlaneShape> addPlaneShape(const std::string& pShapeName, const glm::vec3& pNormal) throw (dab::Exception);
	std::shared_ptr<BodySphereShape> addSphereShape(const std::string& pShapeName, float pRadius) throw (dab::Exception);
	std::shared_ptr<BodyBoxShape> addBoxShape(const std::string& pShapeName, const std::array<float, 3>& pSize) throw (dab::Exception);
	std::shared_ptr<BodyCylinderShape> addCylinderShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception);
	std::shared_ptr<BodyCapsuleShape> addCapsuleShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception);
	std::shared_ptr<BodyConeShape> addConeShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception);
	std::shared_ptr<BodyMeshShape> addMeshShape(const std::string& pShapeName, const std::string& pMeshName) throw (dab::Exception);
	std::shared_ptr<BodyCompoundShape> addCompoundShape(const std::string& pShapeName) throw (dab::Exception);

	// function to add visual bodies based on physical bodies
	std::shared_ptr<Body> addBody(const std::string& pBodyName) throw (dab::Exception);
	std::shared_ptr<Body> copyBody(const std::string& pSourceBodyName, const std::string& pTargetBodyName) throw (dab::Exception);

	// function to add visual shapes based on physical shapes
	std::shared_ptr<BodyShape> addBodyShape(const std::string& pShapeName) throw (dab::Exception);
	
	// functions to add visual parts based on physical parts
	std::shared_ptr<BodyPart> addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName) throw (dab::Exception);
	std::shared_ptr<BodyPart> addBodyPart(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception);

	// debug: add axis aligned bouding box for shape
	void addBBox(const std::string& pShapeName);

	// functions to add materials
	void addMaterial(const std::string& pMaterialName, const Material& pMaterial) throw (dab::Exception);
	void changeMaterial(const std::string& pMaterialName, const Material& pMaterial) throw (dab::Exception);
	void registerShapeMaterial(const std::string& pShapeName, const std::string& pMaterialName) throw (dab::Exception);
	void updateShapeMaterials();

	void update();
	void draw();

	// debug vis target
	void addTargetPositions(int pPositionCount);
	void setTargetPosition(int pPositionIndex, const glm::vec3& pTargetPosition) throw (dab::Exception);

protected:
	bool loadShader(ofShader& pShader, const std::string& pVertexShader, const std::string& pFragShader);
	std::string shaderErrorMessage(ofShader& pShader, GLenum pShaderType);

	// functions to create visual shapes that mimic physical shapes
	std::shared_ptr<BodySphereShape> addSphereShape(std::shared_ptr<physics::BodySphereShape> pPhysicsBodyShape);
	std::shared_ptr<BodyBoxShape> addBoxShape(std::shared_ptr<physics::BodyBoxShape> pPhysicsBodyShape);
	std::shared_ptr<BodyCylinderShape> addCylinderShape(std::shared_ptr<physics::BodyCylinderShape> pPhysicsBodyShape);
	std::shared_ptr<BodyCapsuleShape> addCapsuleShape(std::shared_ptr<physics::BodyCapsuleShape> pPhysicsBodyShape);
	std::shared_ptr<BodyConeShape> addConeShape(std::shared_ptr<physics::BodyConeShape> pPhysicsBodyShape);
	std::shared_ptr<BodyMeshShape> addMeshShape(std::shared_ptr<physics::BodyConvexMeshShape> pPhysicsBodyShape);
	std::shared_ptr<BodyCompoundShape> addCompoundShape(std::shared_ptr<physics::BodyCompoundShape> pPhysicsBodyShape);

	std::shared_ptr<Camera> mCamera;
	IndexMap< std::string, std::shared_ptr<Body> > mBodies;
	IndexMap< std::string, std::shared_ptr<BodyShape> > mShapes;
	IndexMap< std::string, std::shared_ptr<Material> > mMaterials;
	IndexMap< std::string, std::string > mShapeMaterialMap;
	std::vector<std::shared_ptr<BodyPart> > mParts;

	// shape shader
	ofShader mShapeShader;

	// vis target
	std::vector<std::shared_ptr<of3dPrimitive>> mVisTargetPositions;
	std::shared_ptr<Material> mVisTargetPositionMaterial;
};

};

};