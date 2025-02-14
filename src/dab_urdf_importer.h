/** \file dab_urdf_importer.h
*/

#pragma once

#include "dab_singleton.h"
#include "dab_exception.h"
#include "dab_urdf_parser.h"
#include <btBulletDynamicsCommon.h>

namespace dab
{

namespace physics
{
	class Body;
	class BodyShape;
	class BodyPart;
	class BodyJoint;
};

namespace vis
{
	class BodyShape;
	class BodyPart;
}

class UrdfImporter : public Singleton<UrdfImporter>
{
public:
	UrdfImporter();
	~UrdfImporter();

	void loadURDF(std::string& pFileName, bool pForceFixedBase = false) throw (dab::Exception);
	void loadSDF(std::string& pFileName, bool pForceFixedBase = false) throw (dab::Exception);

protected:
	static btScalar sDefaultCollisionMargin;

	void parseURDF(std::string& pFileName, bool pForceFixedBase = false) throw (dab::Exception);
	void parseSDF(std::string& pFileName, bool pForceFixedBase = false) throw (dab::Exception);

	void processUrdf(UrdfModel& pUrdfModel, std::string& pPathPrefix, bool pUseSDF, bool pUseMJCF) throw (dab::Exception);
	btTransform processUrdf(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, const btTransform& pParentTransformInWorldSpace, std::string& pPathPrefix, bool pUseSDF, bool pUseMJCF) throw (dab::Exception);

	std::shared_ptr<physics::BodyShape> createPhysicsShape(UrdfLink& pUrdfLink, const btTransform& pLocalInertiaFrame, std::string& pPathPrefix) throw (dab::Exception);
	std::shared_ptr<physics::BodyShape> createPhysicsShape(UrdfLink& pUrdfLink, UrdfCollision& pUrdCollision, std::string& pPathPrefix) throw (dab::Exception);

	std::shared_ptr<vis::BodyShape> createVisualShape(UrdfLink& pUrdfLink, const btTransform& pLocalInertiaFrame, std::string& pPathPrefix) throw (dab::Exception);
	std::shared_ptr<vis::BodyShape> createVisualShape(UrdfLink& pUrdfLink, UrdfVisual& pUrdVisual, std::string& pPathPrefix) throw (dab::Exception);
	
	std::shared_ptr<physics::BodyPart> createPhysicsLink(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, std::shared_ptr<physics::BodyShape> pShape, bool pIsBodyRoot) throw (dab::Exception);
	std::shared_ptr<vis::BodyPart> createVisualLink(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, std::shared_ptr<vis::BodyShape> pShape) throw (dab::Exception);
	
	std::shared_ptr<physics::BodyJoint> createJoint(UrdfModel& pUrdfModel, UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild);
	std::shared_ptr<physics::BodyJoint> createRevoluteJoint(UrdfModel& pUrdfModel, UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild) throw (dab::Exception);
	//std::shared_ptr<physics::BodyJoint> createRevoluteJoint(UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, const btVector3& pJointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit) throw (dab::Exception);

	//void loadMeshes() throw (dab::Exception);
	//void loadMesh(const std::string& pFilePath) throw (dab::Exception);

	std::string getPathPrefix(const std::string& pFilePath);
	std::string getMeshFileName(const std::string& pMeshFileString);
	std::string getMeshName(const std::string& pMeshFileString);
	std::string getTextureName(const std::string& pTextureFileString);
	void getMassAndInertia(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame);
	bool getJointInfo(UrdfLink& pUrdfLink, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity);
	void processContactParameters(const URDFLinkContactInfo& contactInfo, btCollisionObject* col);

	static float sMassScale;
};

};