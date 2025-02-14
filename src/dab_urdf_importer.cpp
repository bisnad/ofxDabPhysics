/** \file dab_urdf_importer.cpp
*/

#include "dab_urdf_importer.h"
#include "dab_file_io.h"
#include "dab_geom_mesh_manager.h"
#include "dab_physics_simulation.h"
#include "dab_vis_body_visualization.h"
#include "dab_physics_body_shape.h"
#include "dab_physics_body.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_universal_motor.h"
#include "dab_vis_body_shape.h"
#include "dab_vis_body_part.h"
#include "dab_vis_material.h"
#include <algorithm>

using namespace dab;

//btScalar UrdfImporter::sDefaultCollisionMargin = -0.001;
//float UrdfImporter::sMassScale = 0.001;
btScalar UrdfImporter::sDefaultCollisionMargin = 0.01;
float UrdfImporter::sMassScale = 1.0;

UrdfImporter::UrdfImporter()
{}

UrdfImporter::~UrdfImporter()
{}

void 
UrdfImporter::loadURDF(std::string& pFileName, bool pForceFixedBase) throw (dab::Exception)
{
	try
	{
		dab::UrdfParser& urdfParser = dab::UrdfParser::get();

		parseURDF(pFileName, pForceFixedBase);

		std::string pathPrefix = getPathPrefix(pFileName);

		dab::UrdfModel& urdfModel = urdfParser.getModel();

		processUrdf(urdfModel, pathPrefix, false, false);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to load file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
UrdfImporter::loadSDF(std::string& pFileName, bool pForceFixedBase) throw (dab::Exception)
{
	try
	{
		dab::UrdfParser& urdfParser = dab::UrdfParser::get();

		parseSDF(pFileName, pForceFixedBase);

		std::string pathPrefix = getPathPrefix(pFileName);

		int modelCount = urdfParser.getNumModels();

		for (int mI = 0; mI < modelCount; ++mI)
		{
			dab::UrdfModel& urdfModel = urdfParser.getModelByIndex(mI);
			processUrdf(urdfModel, pathPrefix, true, false);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to load file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
UrdfImporter::parseURDF(std::string& pFileName, bool pForceFixedBase) throw (dab::Exception)
{
	try
	{
		dab::UrdfParser& urdfParser = dab::UrdfParser::get();
		dab::FileIO& fileIO = dab::FileIO::get();
		std::string urdfString;

		urdfParser.setParseSDF(false);

		fileIO.read(pFileName, urdfString);
		bool success = urdfParser.loadUrdf(urdfString.c_str(), false, false);

		if (success == false) throw dab::Exception("Urdf Error: failed to parse file " + pFileName, __FILE__, __FUNCTION__, __LINE__);

		// merge fixed links
		if (urdfParser.getModel().m_rootLinks.size())
		{
			urdfParser.mergeFixedLinks(urdfParser.getModel(), urdfParser.getModel().m_rootLinks[0], pForceFixedBase, 0);
			urdfParser.getModel().m_links.clear();
			urdfParser.getModel().m_joints.clear();
			urdfParser.recreateModel(urdfParser.getModel(), urdfParser.getModel().m_rootLinks[0]);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to parse file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
UrdfImporter::parseSDF(std::string& pFileName, bool pForceFixedBase) throw (dab::Exception)
{
	try
	{
		dab::UrdfParser& urdfParser = dab::UrdfParser::get();
		dab::FileIO& fileIO = dab::FileIO::get();
		std::string urdfString;

		urdfParser.setParseSDF(true);

		fileIO.read(pFileName, urdfString);
		bool success = urdfParser.loadSDF(urdfString.c_str());

		if (success == false) throw dab::Exception("Urdf Error: failed to parse file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to parse file " + pFileName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

void 
UrdfImporter::processUrdf(UrdfModel& pUrdfModel, std::string& pPathPrefix, bool pUseSDF, bool pUseMJCF) throw (dab::Exception)
{
	if (pUrdfModel.m_rootLinks.size() != 1) throw dab::Exception("Urdf Error: number of root links is not 1", __FILE__, __FUNCTION__, __LINE__ );

	// create body
	const std::string& bodyName = pUrdfModel.m_name;

	try
	{
		physics::Simulation::get().addBody(bodyName);
		vis::BodyVisualization::get().addBody(bodyName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create body " + bodyName, __FILE__, __FUNCTION__, __LINE__);
	}

	UrdfLink* rootLink = pUrdfModel.m_rootLinks[0];
	btTransform rootTransform = pUrdfModel.m_rootTransformInWorld;

	try
	{
		processUrdf(pUrdfModel, *rootLink, rootTransform, pPathPrefix, pUseSDF, pUseMJCF);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create links", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

btTransform
UrdfImporter::processUrdf(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, const btTransform& pParentTransformInWorldSpace, std::string& pPathPrefix, bool pUseSDF, bool pUseMJCF) throw (dab::Exception)
{
	//std::cout << "processUrdf link " << pUrdfLink.m_name << "\n";

	physics::Simulation& physics = physics::Simulation::get();
	vis::BodyVisualization& visuals = vis::BodyVisualization::get();

	btTransform linkTransformInWorldSpace;
	linkTransformInWorldSpace.setIdentity();

	try
	{
		//std::cout << "retrieve body " << pUrdfModel.m_name << "\n";

		const std::string& bodyName = pUrdfModel.m_name;
		std::shared_ptr<physics::Body> _body = physics.body(bodyName);

		UrdfLink* urdfParentLink = pUrdfLink.m_parentLink;

		std::shared_ptr<physics::BodyPart> _parentBodyPart = nullptr;

		btTransform parentLocalInertialFrame;
		parentLocalInertialFrame.setIdentity();
		btScalar parentMass(1);
		btVector3 parentLocalInertiaDiagonal(1, 1, 1);

		if (urdfParentLink != nullptr)
		{
			//std::cout << "parent link " << urdfParentLink->m_name << "\n";

			_parentBodyPart = _body->part(urdfParentLink->m_name);
			getMassAndInertia(pUrdfModel, *urdfParentLink, parentMass, parentLocalInertiaDiagonal, parentLocalInertialFrame);
		}

		btScalar mass = 0;
		btTransform localInertialFrame;
		localInertialFrame.setIdentity();
		btVector3 localInertiaDiagonal(0, 0, 0);

		getMassAndInertia(pUrdfModel, pUrdfLink, mass, localInertiaDiagonal, localInertialFrame);

		btTransform parent2joint;
		parent2joint.setIdentity();

		int jointType;
		btVector3 jointAxisInJointSpace;
		btScalar jointLowerLimit;
		btScalar jointUpperLimit;
		btScalar jointDamping;
		btScalar jointFriction;
		btScalar jointMaxForce;
		btScalar jointMaxVelocity;

		bool hasParentJoint = getJointInfo(pUrdfLink, parent2joint, linkTransformInWorldSpace, jointAxisInJointSpace, jointType, jointLowerLimit, jointUpperLimit, jointDamping, jointFriction, jointMaxForce, jointMaxVelocity);
		
		if (pUseSDF)
		{
			parent2joint = pParentTransformInWorldSpace.inverse() * linkTransformInWorldSpace;
		}
		else
		{
			if (pUseMJCF)
			{
				linkTransformInWorldSpace = pParentTransformInWorldSpace * linkTransformInWorldSpace;
			}
			else
			{
				linkTransformInWorldSpace = pParentTransformInWorldSpace * parent2joint;
			}
		}

		std::shared_ptr<physics::BodyShape> _collisionShape = nullptr;
		std::shared_ptr<vis::BodyShape> _visualShape = nullptr;

		// create collision shape
		_collisionShape = createPhysicsShape(pUrdfLink, localInertialFrame, pPathPrefix);

		// get single child shape out of compound shape if it contains only one child and this has identity transfrom
		std::shared_ptr<physics::BodyCompoundShape> _colCompShape = std::dynamic_pointer_cast<physics::BodyCompoundShape>(_collisionShape);
		if (_colCompShape != nullptr && _colCompShape->childCount() == 1 && _colCompShape->childTransforms()[0] == btTransform::getIdentity())
		{
			_collisionShape = _colCompShape->childShapes()[0];
		}

		if (_collisionShape != nullptr && mass > 0.0)
		{
			_collisionShape->nativeShape()->calculateLocalInertia(mass, localInertiaDiagonal);
			btAssert(localInertiaDiagonal[0] < 1e10);
			btAssert(localInertiaDiagonal[1] < 1e10);
			btAssert(localInertiaDiagonal[2] < 1e10);

			URDFLinkContactInfo& contactInfo = pUrdfLink.m_contactInfo;
			//temporary inertia scaling until we load inertia from URDF
			if (contactInfo.m_flags & URDF_CONTACT_HAS_INERTIA_SCALING)
			{
				localInertiaDiagonal *= contactInfo.m_inertiaScaling;
			}
		}

		// create visual shape
		_visualShape = createVisualShape(pUrdfLink, localInertialFrame, pPathPrefix);

		// create physics body part (link)
		btTransform inertialFrameInWorldSpace = linkTransformInWorldSpace * localInertialFrame;
		std::shared_ptr<physics::BodyPart> _bodyPart = createPhysicsLink(pUrdfModel, pUrdfLink, mass, localInertiaDiagonal, inertialFrameInWorldSpace, _collisionShape, urdfParentLink == nullptr);
		URDFLinkContactInfo contactInfo = pUrdfLink.m_contactInfo;
		processContactParameters(contactInfo, _bodyPart->nativeBody());

		// create visual body part (link)
		std::shared_ptr<vis::BodyPart> _visualPart = createVisualLink(pUrdfModel, pUrdfLink, _visualShape);

		// create physics joint
		if (hasParentJoint)
		{
			btTransform offsetInA, offsetInB;
			offsetInA = parentLocalInertialFrame.inverse() * parent2joint;
			offsetInB = localInertialFrame.inverse();

			std::shared_ptr<physics::BodyJoint> _joint = createJoint(pUrdfModel, *pUrdfLink.m_parentJoint, offsetInA, offsetInB);
		}

		// recursion into child links
		int childLinkCount = pUrdfLink.m_childLinks.size();
		for (int lI = 0; lI < childLinkCount; ++lI)
		{
			processUrdf(pUrdfModel, *(pUrdfLink.m_childLinks[lI]), linkTransformInWorldSpace, pPathPrefix, pUseSDF, pUseMJCF);
		}

	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to process urdf link " + pUrdfLink.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return linkTransformInWorldSpace;
}

std::shared_ptr<physics::BodyShape>
UrdfImporter::createPhysicsShape(UrdfLink& pUrdfLink, const btTransform& pLocalInertiaFrame, std::string& pPathPrefix) throw (dab::Exception)
{
	//// debug
	//std::cout << "createPhysicsShape for link " << pUrdfLink.m_name << "\n";


	//btVector3 prevPos = pLocalInertiaFrame.getOrigin();
	//btQuaternion prevRot = pLocalInertiaFrame.getRotation();
	//btMatrix3x3 prevBasis = pLocalInertiaFrame.getBasis();

	//std::cout << "pos " << prevPos.x() << " " << prevPos.y() << " " << prevPos.z() << "\n";
	//std::cout << "rot " << prevRot.x() << " " << prevRot.y() << " " << prevRot.z() << " " << prevRot.w() << "\n";
	//std::cout << "basis 0 " << prevBasis[0].x() << " " << prevBasis[0].y() << " " << prevBasis[0].z() << "\n";
	//std::cout << "basis 1 " << prevBasis[1].x() << " " << prevBasis[1].y() << " " << prevBasis[1].z() << "\n";
	//std::cout << "basis 2 " << prevBasis[2].x() << " " << prevBasis[2].y() << " " << prevBasis[2].z() << "\n";
	//// debug

	physics::Simulation& physics = physics::Simulation::get();

	// always create a compund shape that contains the actual shape(s) as child shape(s)
	// this allows to apply transforms to the actual shape(s)
	std::shared_ptr<physics::BodyCompoundShape> _compoundShape = nullptr;
	std::string compoundShapeName = pUrdfLink.m_name + "_comp_col_shape";
	if (physics.shapeExists(compoundShapeName) == true) return physics.shape(compoundShapeName);

	try
	{
		// create compound shape to which the actual shape(s) will be added, irrespective if there is only a single collision shape or multiple collision shapes
		//std::cout << "create compound collision shape " << compoundShapeName << " for link " << pUrdfLink.m_name << "\n";
		_compoundShape = physics.addCompoundShape(compoundShapeName);
		_compoundShape->nativeShape()->setMargin(sDefaultCollisionMargin);

		// TODO: check why both the collision frame is wrong and the local inertial frame as well

		// iterate trough all collision shapes and add them to compound shape
		btArray<UrdfCollision>& collisionArray = pUrdfLink.m_collisionArray;
		int collisionCount = collisionArray.size();
	 
		for (int cI = 0; cI < collisionCount; ++cI)
		{
			UrdfCollision& urdfCollision = collisionArray[cI];

			//btVector3 ciPos = urdfCollision.m_linkLocalFrame.getOrigin();
			//btQuaternion ciRot = urdfCollision.m_linkLocalFrame.getRotation();
			//btMatrix3x3 ciBasis = urdfCollision.m_linkLocalFrame.getBasis();

			//std::cout << "col pos " << ciPos.x() << " " << ciPos.y() << " " << ciPos.z() << "\n";
			//std::cout << "col rot " << ciRot.x() << " " << ciRot.y() << " " << ciRot.z() << " " << ciRot.w() << "\n";
			//std::cout << "col basis 0 " << ciBasis[0].x() << " " << ciBasis[0].y() << " " << ciBasis[0].z() << "\n";
			//std::cout << "col basis 1 " << ciBasis[1].x() << " " << ciBasis[1].y() << " " << ciBasis[1].z() << "\n";
			//std::cout << "col basis 2 " << ciBasis[2].x() << " " << ciBasis[2].y() << " " << ciBasis[2].z() << "\n";


			std::shared_ptr<physics::BodyShape> _shape = createPhysicsShape(pUrdfLink, urdfCollision, pPathPrefix);

			if (_shape != nullptr)
			{
				btTransform childTrans = urdfCollision.m_linkLocalFrame;

				//std::cout << "add child shape " << _shape->name() << " to compound shape " << _compoundShape->name() << "\n";

				//btVector3 liPos = pLocalInertiaFrame.getOrigin();
				//btQuaternion liRot = pLocalInertiaFrame.getRotation();
				//btMatrix3x3 liBasis = pLocalInertiaFrame.getBasis();

				//std::cout << "li pos " << liPos.x() << " " << liPos.y() << " " << liPos.z() << "\n";
				//std::cout << "li rot " << liRot.x() << " " << liRot.y() << " " << liRot.z() << " " << liRot.w() << "\n";
				//std::cout << "li basis 0 " << liBasis[0].x() << " " << liBasis[0].y() << " " << liBasis[0].z() << "\n";
				//std::cout << "li basis 1 " << liBasis[1].x() << " " << liBasis[1].y() << " " << liBasis[1].z() << "\n";
				//std::cout << "li basis 2 " << liBasis[2].x() << " " << liBasis[2].y() << " " << liBasis[2].z() << "\n";

				//btVector3 ciPos = childTrans.getOrigin();
				//btQuaternion ciRot = childTrans.getRotation();
				//btMatrix3x3 ciBasis = childTrans.getBasis();

				//std::cout << "ci pos " << ciPos.x() << " " << ciPos.y() << " " << ciPos.z() << "\n";
				//std::cout << "ci rot " << ciRot.x() << " " << ciRot.y() << " " << ciRot.z() << " " << ciRot.w() << "\n";
				//std::cout << "ci basis 0 " << ciBasis[0].x() << " " << ciBasis[0].y() << " " << ciBasis[0].z() << "\n";
				//std::cout << "ci basis 1 " << ciBasis[1].x() << " " << ciBasis[1].y() << " " << ciBasis[1].z() << "\n";
				//std::cout << "ci basis 2 " << ciBasis[2].x() << " " << ciBasis[2].y() << " " << ciBasis[2].z() << "\n";

				_compoundShape->addChildShape(_shape, pLocalInertiaFrame.inverse() * childTrans);
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create collision shape " + compoundShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _compoundShape;
}

std::shared_ptr<physics::BodyShape> 
UrdfImporter::createPhysicsShape(UrdfLink& pUrdfLink, UrdfCollision& pUrdfCollision, std::string& pPathPrefix) throw (dab::Exception)
{
	physics::Simulation& physics = physics::Simulation::get();

	std::shared_ptr<physics::BodyShape> _shape = nullptr;


	// create name for collision shape if it doesn't exsist already
	if (pUrdfCollision.m_name == "")
	{
		// name based on mesh file
		if (pUrdfCollision.m_geometry.m_type == URDF_GEOM_MESH || pUrdfCollision.m_geometry.m_type == URDF_GEOM_MESH)
		{
			std::string& meshFileName = getMeshFileName(pUrdfCollision.m_geometry.m_meshFileName);
			std::string& meshName = getMeshName(meshFileName);
			pUrdfCollision.m_name = meshName + "_col_shape";
		}
		else // name based on link name
		{
			pUrdfCollision.m_name = pUrdfLink.m_name + "_col_shape";
		}
	}

	if (physics.shapeExists(pUrdfCollision.m_name) == true) return physics.shape(pUrdfCollision.m_name);



	try
	{
		switch (pUrdfCollision.m_geometry.m_type)
		{
		case URDF_GEOM_PLANE:
		{
			std::cout << "create plane for collision shape " << pUrdfCollision.m_name << "\n";

			btVector3 planeNormal = pUrdfCollision.m_geometry.m_planeNormal;
			_shape = physics.addPlaneShape(pUrdfCollision.m_name, glm::vec3(planeNormal.x(), planeNormal.y(), planeNormal.z()));
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CAPSULE:
		{
			std::cout << "create capsule for collision shape " << pUrdfCollision.m_name << "\n";

			btScalar radius = pUrdfCollision.m_geometry.m_capsuleRadius;
			btScalar height = pUrdfCollision.m_geometry.m_capsuleHeight;
			_shape = physics.addCapsuleShape(pUrdfCollision.m_name, radius, height);
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CYLINDER:
		{
			std::cout << "create cylinder for collision shape " << pUrdfCollision.m_name << "\n";

			btScalar cylRadius = pUrdfCollision.m_geometry.m_capsuleRadius;
			btScalar cylHalfLength = 0.5 * pUrdfCollision.m_geometry.m_capsuleHeight;
			btVector3 halfExtents(cylRadius, cylRadius, cylHalfLength);
			_shape = physics.addCylinderShape(pUrdfCollision.m_name, glm::vec3(halfExtents.x(), halfExtents.y(), halfExtents.z()));
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_BOX:
		{
			std::cout << "create box for collision shape " << pUrdfCollision.m_name << "\n";

			btVector3 halfExtents = pUrdfCollision.m_geometry.m_boxSize * 0.5;
			_shape = physics.addBoxShape(pUrdfCollision.m_name, glm::vec3(halfExtents.x(), halfExtents.y(), halfExtents.z()));
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_SPHERE:
		{
			std::cout << "create sphere for collision shape " << pUrdfCollision.m_name << "\n";

			btScalar radius = pUrdfCollision.m_geometry.m_sphereRadius;
			_shape = physics.addSphereShape(pUrdfCollision.m_name, radius);
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			break;
		}
		case URDF_GEOM_CDF:
		{
			std::cout << "create cdf for collision shape " << pUrdfCollision.m_name << "\n";

			throw dab::Exception("Urdf Error: CDF shape not implemented", __FILE__, __FUNCTION__, __LINE__);

			//const std::string& meshFileName = pUrdfCollision.m_geometry.m_meshFileName;
			//std::string meshFilePath = pPathPrefix + meshFileName;

			//btAlignedObjectArray<char> sdfData;
			//{
			//	std::streampos fsize = 0;
			//	std::ifstream file(meshFilePath, std::ios::binary);
			//	if (file.good())
			//	{
			//		fsize = file.tellg();
			//		file.seekg(0, std::ios::end);
			//		fsize = file.tellg() - fsize;
			//		file.seekg(0, std::ios::beg);
			//		sdfData.resize(fsize);
			//		int bytesRead = file.rdbuf()->sgetn(&sdfData[0], fsize);
			//		btAssert(bytesRead == fsize);
			//		file.close();
			//	}
			//}

			//if (sdfData.size())
			//{
			//	// TODO: implement this shape class in physics_body_shape.h/cpp
			//	// TODO: and add creator function to dab_physics_simulation.h/cpp

			//	// here original implementation from BulletUrdfImporter.cpp

			//	//btSdfCollisionShape* sdfShape = new btSdfCollisionShape();
			//	//bool valid = sdfShape->initializeSDF(&sdfData[0], sdfData.size());
			//	//btAssert(valid);

			//	//if (valid)
			//	//{
			//	//	shape = sdfShape;
			//	//}
			//	//else
			//	//{
			//	//	delete sdfShape;
			//	//}
			//}
			break;
		}
		case URDF_GEOM_MESH:
		{
			//std::cout << "create collision shape for link " << pUrdfLink.m_name << "\n";

			//std::cout << "create mesh for collision shape " << pUrdfCollision.m_name << "\n";


			geom::MeshManager& meshManager = geom::MeshManager::get();

			std::string& meshFileName = getMeshFileName(pUrdfCollision.m_geometry.m_meshFileName);
			std::string meshFilePath = pPathPrefix + meshFileName;
			std::string& meshName = getMeshName(meshFileName);

			//std::cout << "link " << pUrdfLink.m_name << " meshFileName " << meshFileName << " meshName " << meshName << "\n";

			if (meshManager.meshExists(meshName) == false) meshManager.loadMesh(meshName, meshFilePath, 0);

			_shape = physics.addConvexMeshShape(pUrdfCollision.m_name, meshName);
			_shape->nativeShape()->setMargin(sDefaultCollisionMargin);
			static_cast<btConvexHullShape*>(_shape->nativeShape())->recalcLocalAabb();

			break;
		}
		default:
			throw dab::Exception("Urdf Error: unknown collision geometry type " + std::to_string(pUrdfCollision.m_geometry.m_type), __FILE__, __FUNCTION__, __LINE__);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create physics shape " + pUrdfCollision.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _shape;
}

std::shared_ptr<vis::BodyShape>
UrdfImporter::createVisualShape(UrdfLink& pUrdfLink, const btTransform& pLocalInertiaFrame, std::string& pPathPrefix) throw (dab::Exception)
{
	vis::BodyVisualization& visuals = vis::BodyVisualization::get();

	std::shared_ptr<vis::BodyCompoundShape> _compoundShape = nullptr;
	std::string compoundShapeName = pUrdfLink.m_name + "_comp_vis_shape";
	if (visuals.shapeExists(compoundShapeName) == true) return visuals.shape(compoundShapeName);

	try
	{		
		// create compound shape to which the actual shape(s) will be added, irrespective if there is only a single collision shape or multiple collision shapes
		//std::cout << "create compound visual shape " << compoundShapeName << " for link " << pUrdfLink.m_name << "\n";
		_compoundShape = visuals.addCompoundShape(compoundShapeName);

		if (pUrdfLink.m_visualArray.size() == 0) // copy properties from collision shapes into visual shapes
		{
			//std::cout << "iterate trough all collision shapes and add them to compound shape\n";

			// iterate trough all collision shapes and add them to compound shape
			btArray<UrdfCollision>& collisionArray = pUrdfLink.m_collisionArray;
			int collisionCount = collisionArray.size();

			for (int cI = 0; cI < collisionCount; ++cI)
			{
				std::shared_ptr<vis::BodyShape> _shape = nullptr;

				const UrdfCollision& urdfCollision = collisionArray[cI];

				UrdfVisual tmpUrdfVisual;
				tmpUrdfVisual.m_sourceFileLocation = urdfCollision.m_sourceFileLocation;
				tmpUrdfVisual.m_linkLocalFrame = urdfCollision.m_linkLocalFrame;
				tmpUrdfVisual.m_geometry = urdfCollision.m_geometry;
				tmpUrdfVisual.m_name = "";
				tmpUrdfVisual.m_materialName = "";

				_shape = createVisualShape(pUrdfLink, tmpUrdfVisual, pPathPrefix);

				if (_shape != nullptr)
				{
					btTransform childTrans = urdfCollision.m_linkLocalFrame;
					btTransform shapeTransform = pLocalInertiaFrame.inverse() * childTrans;
					//const btVector3& shapeOrigin = shapeTransform.getOrigin();
					//const btQuaternion& shapeRotation = shapeTransform.getRotation();

					//vis::BodyTransform visTransform;
					//visTransform.setPosition(glm::vec3(shapeOrigin.x(), shapeOrigin.y(), shapeOrigin.z()));
					//visTransform.setOrientation(glm::quat(shapeRotation.w(), shapeRotation.x(), shapeRotation.y(), shapeRotation.z()));

					// create opengl transform
					vis::BodyTransform visTransform;

					float glTransform[16];
					shapeTransform.getOpenGLMatrix(glTransform);
					glm::mat4 glMatrix = glm::make_mat4(glTransform);

					visTransform.setMatrix(glMatrix);

					_compoundShape->addChildShape(_shape, visTransform);
				}
			}
		}
		else
		{
			//std::cout << "iterate trough all visual shapes and add them to compound shape\n";

			// iterate trough all visual shapes and add them to compound shape
			btArray<UrdfVisual>& visualArray = pUrdfLink.m_visualArray;
			int visualCount = visualArray.size();

			for (int cI = 0; cI < visualCount; ++cI)
			{
				std::shared_ptr<vis::BodyShape> _shape = nullptr;

				UrdfVisual& urdfVisual = visualArray[cI];

				_shape = createVisualShape(pUrdfLink, urdfVisual, pPathPrefix);

				if (_shape != nullptr)
				{
					btTransform childTrans = urdfVisual.m_linkLocalFrame;
					btTransform shapeTransform = pLocalInertiaFrame.inverse() * childTrans;
					//const btVector3& shapeOrigin = shapeTransform.getOrigin();
					//const btQuaternion& shapeRotation = shapeTransform.getRotation();

					//vis::BodyTransform visTransform;

					//visTransform.setPosition(glm::vec3(shapeOrigin.x(), shapeOrigin.y(), shapeOrigin.z()));
					//visTransform.setOrientation(glm::quat(shapeRotation.w(), shapeRotation.x(), shapeRotation.y(), shapeRotation.z()));

					// create opengl transform
					vis::BodyTransform visTransform;

					float glTransform[16];
					shapeTransform.getOpenGLMatrix(glTransform);
					glm::mat4 glMatrix = glm::make_mat4(glTransform);

					visTransform.setMatrix(glMatrix);

					_compoundShape->addChildShape(_shape, visTransform);
				}
			}
		}


	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create visual shape " + compoundShapeName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _compoundShape;
}

std::shared_ptr<vis::BodyShape> 
UrdfImporter::createVisualShape(UrdfLink& pUrdfLink, UrdfVisual& pUrdVisual, std::string& pPathPrefix) throw (dab::Exception)
{
	vis::BodyVisualization& visuals = vis::BodyVisualization::get();

	// create name for visual shape if it doesn't exsist already
	if (pUrdVisual.m_name == "")
	{
		// name based on mesh file
		if (pUrdVisual.m_geometry.m_type == URDF_GEOM_MESH || pUrdVisual.m_geometry.m_type == URDF_GEOM_MESH)
		{
			std::string& meshFileName = getMeshFileName(pUrdVisual.m_geometry.m_meshFileName);
			std::string& meshName = getMeshName(meshFileName);
			pUrdVisual.m_name = meshName + "_vis_shape";
		}
		else // name based on link name
		{
			pUrdVisual.m_name = pUrdfLink.m_name + "_vis_shape";
		}
	}

	if (visuals.shapeExists(pUrdVisual.m_name) == true) return visuals.shape(pUrdVisual.m_name);

	//std::cout << "create visual shape " << pUrdVisual.m_name << " for link " << pUrdfLink.m_name << "\n";

	std::shared_ptr<vis::BodyShape> _shape = nullptr;

	try
	{
		switch (pUrdVisual.m_geometry.m_type)
		{
		case URDF_GEOM_PLANE:
		{
			std::cout << "create plane for visual shape " << pUrdVisual.m_name << "\n";

			btVector3 planeNormal = pUrdVisual.m_geometry.m_planeNormal;
			_shape = visuals.addPlaneShape(pUrdVisual.m_name, glm::vec3(planeNormal.x(), planeNormal.y(), planeNormal.z()));

			break;
		}
		case URDF_GEOM_CYLINDER:
		{
			std::cout << "create cylinder for visual shape " << pUrdVisual.m_name << "\n";

			btScalar cylRadius = pUrdVisual.m_geometry.m_capsuleRadius;
			btScalar cylHalfLength = pUrdVisual.m_geometry.m_capsuleHeight;
			_shape = visuals.addCylinderShape(pUrdVisual.m_name, cylRadius, cylHalfLength);

			break;
		}
		case URDF_GEOM_CAPSULE:
		{
			std::cout << "create capsule for visual shape " << pUrdVisual.m_name << "\n";

			btScalar radius = pUrdVisual.m_geometry.m_capsuleRadius;
			btScalar height = pUrdVisual.m_geometry.m_capsuleHeight;
			_shape = visuals.addCapsuleShape(pUrdVisual.m_name, radius, height);

			break;
		}
		case URDF_GEOM_BOX:
		{
			std::cout << "create box for visual shape " << pUrdVisual.m_name << "\n";

			btVector3 extents = pUrdVisual.m_geometry.m_boxSize;
			_shape = visuals.addBoxShape(pUrdVisual.m_name, { extents.x(), extents.y(), extents.z() });

			break;
		}
		case URDF_GEOM_SPHERE:
		{
			std::cout << "create sphere for visual shape " << pUrdVisual.m_name << "\n";

			btScalar radius = pUrdVisual.m_geometry.m_sphereRadius;
			_shape = visuals.addSphereShape(pUrdVisual.m_name, radius);

			break;
		}
		case URDF_GEOM_CDF:
		{
			//std::cout << "create cdf for visual shape " << pUrdVisual.m_name << "\n";

			throw dab::Exception("Urdf Error: CDF shape not implemented", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		case URDF_GEOM_MESH:
		{
			//std::cout << "create mesh for visual shape " << pUrdVisual.m_name << "\n";

			geom::MeshManager& meshManager = geom::MeshManager::get();

			std::string& meshFileName = getMeshFileName(pUrdVisual.m_geometry.m_meshFileName);
			std::string meshFilePath = pPathPrefix + meshFileName;
			std::string& meshName = getMeshName(meshFileName);

			//std::cout << "link " << pUrdfLink.m_name << " meshFileName " << meshFileName << " meshName " << meshName << "\n";

			if (meshManager.meshExists(meshName) == false) meshManager.loadMesh(meshName, meshFilePath, 0);

			_shape = visuals.addMeshShape(pUrdVisual.m_name, meshName);

			break;
		}
		default:
			throw dab::Exception("Urdf Error: unknown visual geometry type " + std::to_string(pUrdVisual.m_geometry.m_type), __FILE__, __FUNCTION__, __LINE__);
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create visual shape " + pUrdVisual.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	// create material
	// TODO: load texture (involves creating a texture manager class)
	if (_shape != nullptr && pUrdVisual.m_geometry.m_hasLocalMaterial)
	{
		const UrdfMaterial& urdfMaterial = pUrdVisual.m_geometry.m_localMaterial;
		vis::Material _material;

		const std::string& textureFileName = urdfMaterial.m_textureFilename;
		const btVector4& urdfRGBAColor = urdfMaterial.m_matColor.m_rgbaColor;
		const btVector3& urdfSpecularColor = urdfMaterial.m_matColor.m_specularColor;

		std::string textureName = getTextureName(textureFileName);

		_material.setAmbientColor(glm::vec3(urdfRGBAColor.x(), urdfRGBAColor.y(), urdfRGBAColor.z()));
		_material.setDiffuseColor(glm::vec3(urdfSpecularColor.x(), urdfSpecularColor.y(), urdfSpecularColor.z()));
		_material.setTransparency(1.0 - urdfRGBAColor.w());
		_material.setTextureName(textureName);

		_shape->material() = _material;

		if (pUrdVisual.m_materialName != "")
		{
			visuals.addMaterial(pUrdVisual.m_materialName, _material);
			visuals.registerShapeMaterial(_shape->name(), pUrdVisual.m_materialName);
		}
	}

	return _shape;
}


std::shared_ptr<physics::BodyPart> 
UrdfImporter::createPhysicsLink(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, btScalar mass, const btVector3& localInertiaDiagonal, const btTransform& initialWorldTrans, std::shared_ptr<physics::BodyShape> pShape, bool isBodyRoot) throw (dab::Exception)
{
	//// debug
	//{
	//	std::cout << "create physics link " << pUrdfLink.m_name << " with shape " << pShape->name() << "\n";

	//	// create Link
	//	std::cout << "create link " << pUrdfLink.m_name << "\n";
	//	std::cout << "mass " << mass << "\n";
	//	std::cout << "localInertiaDiagonal " << localInertiaDiagonal.x() << " " << localInertiaDiagonal.y() << " " << localInertiaDiagonal.z() << "\n";

	//	//inertialFrameInWorldSpace
	//	btVector3 pos = initialWorldTrans.getOrigin();
	//	btQuaternion rot = initialWorldTrans.getRotation();
	//	btMatrix3x3 basis = initialWorldTrans.getBasis();

	//	std::cout << "pos " << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
	//	std::cout << "rot " << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w() << "\n";
	//	std::cout << "basis 0 " << basis[0].x() << " " << basis[0].y() << " " << basis[0].z() << "\n";
	//	std::cout << "basis 1 " << basis[1].x() << " " << basis[1].y() << " " << basis[1].z() << "\n";
	//	std::cout << "basis 2 " << basis[2].x() << " " << basis[2].y() << " " << basis[2].z() << "\n";
	//}

	physics::Simulation& physics = physics::Simulation::get();

	std::shared_ptr<physics::BodyPart> _part = nullptr;

	//std::cout << "create physics link " << pUrdfLink.m_name << " with shape " << pShape->name() << "\n";

	try
	{
		// code from MyMultibodyCreator.cpp
		/*
		btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
		rbci.m_startWorldTransform = initialWorldTrans;
		btScalar sleep_threshold = btScalar(0.22360679775);
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


		const btVector3& initialPos = initialWorldTrans.getOrigin();
		const btQuaternion& initialOrient = initialWorldTrans.getRotation();
		_part = physics.addBodyPart(pUrdfModel.m_name, pUrdfLink.m_name, pShape->name(), isBodyRoot, mass, glm::vec3(initialPos.x(), initialPos.y(), initialPos.z()), glm::quat(initialOrient.w(), initialOrient.x(), initialOrient.y(), initialOrient.z()));
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create link "+ pUrdfLink.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw;
	}

	return _part;
}

std::shared_ptr<vis::BodyPart> 
UrdfImporter::createVisualLink(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, std::shared_ptr<vis::BodyShape> pShape) throw (dab::Exception)
{
	//std::cout << "create visual link " << pUrdfLink.m_name << " with shape " << pShape->name() << "\n";

	vis::BodyVisualization& visuals = vis::BodyVisualization::get();

	std::shared_ptr<dab::vis::BodyPart> _part = visuals.addBodyPart(pUrdfModel.m_name, pUrdfLink.m_name, pShape->name());

	return _part;
}

std::shared_ptr<physics::BodyJoint> 
UrdfImporter::createJoint(UrdfModel& pUrdfModel, UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild)
{
	//std::cout << "UrdfImporter::createJoint\n";

	//std::cout << "joint name " << pUrdfJoint.m_name << "\n";
	//std::cout << "child link name " << pUrdfJoint.m_childLinkName << "\n";
	//std::cout << "parent link name " << pUrdfJoint.m_parentLinkName << "\n";

	//btVector3 prevPos = pOffsetInParent.getOrigin();
	//btVector3 nextPos = pOffsetInChild.getOrigin();
	//btQuaternion prevRot = pOffsetInParent.getRotation();
	//btQuaternion nextRot = pOffsetInChild.getRotation();
	//btMatrix3x3 prevBasis = pOffsetInParent.getBasis();
	//btMatrix3x3 nextBasis = pOffsetInChild.getBasis();

	//std::cout << "prevPos " << prevPos.x() << " " << prevPos.y() << " " << prevPos.z() << "\n";
	//std::cout << "prevRot " << prevRot.x() << " " << prevRot.y() << " " << prevRot.z() << " " << prevRot.w() << "\n";
	//std::cout << "prevBasis 0 " << prevBasis[0].x() << " " << prevBasis[0].y() << " " << prevBasis[0].z() << "\n";
	//std::cout << "prevBasis 1 " << prevBasis[1].x() << " " << prevBasis[1].y() << " " << prevBasis[1].z() << "\n";
	//std::cout << "prevBasis 2 " << prevBasis[2].x() << " " << prevBasis[2].y() << " " << prevBasis[2].z() << "\n";

	//std::cout << "nextPos " << nextPos.x() << " " << nextPos.y() << " " << nextPos.z() << "\n";
	//std::cout << "nextRot " << nextRot.x() << " " << nextRot.y() << " " << nextRot.z() << " " << nextRot.w() << "\n";
	//std::cout << "nextBasis 0 " << nextBasis[0].x() << " " << nextBasis[0].y() << " " << nextBasis[0].z() << "\n";
	//std::cout << "nextBasis 1 " << nextBasis[1].x() << " " << nextBasis[1].y() << " " << nextBasis[1].z() << "\n";
	//std::cout << "nextBasis 2 " << nextBasis[2].x() << " " << nextBasis[2].y() << " " << nextBasis[2].z() << "\n";

	std::shared_ptr<physics::BodyJoint> _joint = nullptr;

	try
	{
		switch (pUrdfJoint.m_type)
		{
		case URDFFixedJoint:
		{
			// TODO : 476 (in URDF2Bullet.cpp)
			throw dab::Exception("Urdf Error: URDFFixedJoint not yet implemented ", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		case URDFSphericalJoint:
		{
			// TODO : 417 (in URDF2Bullet.cpp)
			throw dab::Exception("Urdf Error: URDFSphericalJoint not yet implemented ", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		case URDFPlanarJoint:
		{
			// TODO : 432 (in URDF2Bullet.cpp)
			throw dab::Exception("Urdf Error: URDFPlanarJoint not yet implemented ", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		case URDFFloatingJoint:
		{
			// TODO : 474 (in URDF2Bullet.cpp)
			throw dab::Exception("Urdf Error: URDFFloatingJoint not yet implemented ", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		case URDFContinuousJoint:
		case URDFRevoluteJoint:
		{
			// TODO : 511 (in URDF2Bullet.cpp)
			//std::cout << "create revolute joint\n";

			btGeneric6DofSpring2Constraint* dof6 = 0;

			_joint = createRevoluteJoint(pUrdfModel, pUrdfJoint, pOffsetInParent, pOffsetInChild);

			break;
		}
		case URDFPrismaticJoint:
		{
			// TODO : 568 (in URDF2Bullet.cpp)
			throw dab::Exception("Urdf Error: URDFPrismaticJoint not yet implemented ", __FILE__, __FUNCTION__, __LINE__);

			break;
		}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: failed to create joint " + pUrdfJoint.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _joint;
}

//std::shared_ptr<physics::BodyJoint> 
//UrdfImporter::createRevoluteJoint(UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, const btVector3& pJointAxisInJointSpace, btScalar jointLowerLimit, btScalar jointUpperLimit) throw (dab::Exception)
//{
//	//std::cout << "UrdfImporter::createRevoluteJoint parent " << pUrdfParentLink.m_name << " child " << pUrdfChildLink.m_name << "\n";
//
//	//dof6 = creation.createRevoluteJoint(urdfLinkIndex, *childRigidBody, *parentRigidBody, offsetInCild, offsetInParent, jointAxisInJointSpace, jointLowerLimit, jointUpperLimit);
//
//	physics::Simulation& physics = physics::Simulation::get();
//
//	std::shared_ptr<physics::Universal2Joint> _joint = nullptr;
//
//	try
//	{ 
//		//only handle principle axis at the moment,
//		//@todo(erwincoumans) orient the constraint for non-principal axis
//		int principleAxis = pJointAxisInJointSpace.closestAxis();
//
//		switch (principleAxis)
//		{
//			case 0:
//			{
//				_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_ZYX);
//
//				_joint->nativeJoint()->setLinearLowerLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setLinearUpperLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setAngularLowerLimit(btVector3(jointLowerLimit, 0, 0));
//				_joint->nativeJoint()->setAngularUpperLimit(btVector3(jointUpperLimit, 0, 0));
//
//				break;
//			}
//			case 1:
//			{
//				_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_XZY);
//				
//				_joint->nativeJoint()->setLinearLowerLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setLinearUpperLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setAngularLowerLimit(btVector3(0, jointLowerLimit, 0));
//				_joint->nativeJoint()->setAngularUpperLimit(btVector3(0, jointUpperLimit, 0));
//				break;
//			}
//			case 2:
//			default:
//			{
//				_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_XYZ);
//
//				_joint->nativeJoint()->setLinearLowerLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setLinearUpperLimit(btVector3(0, 0, 0));
//				_joint->nativeJoint()->setAngularLowerLimit(btVector3(0, 0, jointLowerLimit));
//				_joint->nativeJoint()->setAngularUpperLimit(btVector3(0, 0, jointUpperLimit));
//			}
//		}
//	}
//	catch (dab::Exception& e)
//	{
//		e += dab::Exception("Urdf Error: faild to create revolute joint " + pUrdfJoint.m_name, __FILE__, __FUNCTION__, __LINE__);
//		throw e;
//	}
//
//	return _joint;
//}

std::shared_ptr<physics::BodyJoint> 
UrdfImporter::createRevoluteJoint(UrdfModel& pUrdfModel, UrdfJoint& pUrdfJoint, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild) throw (dab::Exception)
{
	//std::cout << "UrdfImporter::createRevoluteJoint " << pUrdfJoint.m_name << "\n";

	//dof6 = creation.createRevoluteJoint(urdfLinkIndex, *childRigidBody, *parentRigidBody, offsetInCild, offsetInParent, jointAxisInJointSpace, jointLowerLimit, jointUpperLimit);

	physics::Simulation& physics = physics::Simulation::get();

	std::shared_ptr<physics::UniversalJoint> _joint = nullptr;
	std::shared_ptr<physics::UniversalMotor> _motor = nullptr;

	const btVector3& jointAxis = pUrdfJoint.m_localJointAxis;
	double jointLowerLimit = pUrdfJoint.m_lowerLimit;
	double jointUpperLimit = pUrdfJoint.m_upperLimit;

	if (jointLowerLimit > jointUpperLimit) //disable joint limits
	{
		jointLowerLimit = 1.0;
		jointUpperLimit = -1.0;
	}

	try
	{
		//only handle principle axis at the moment,
		//@todo(erwincoumans) orient the constraint for non-principal axis
		int principleAxis = jointAxis.closestAxis();
		 
		//std::cout << "joint " << pUrdfJoint.m_name << " axis " << jointAxis.x() << " " << jointAxis.y() << " " << jointAxis.z() << "\n";

		switch (principleAxis)
		{
			case 0:
			{
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_ZYX);
				
				btVector3 parentPos = pOffsetInParent.getOrigin();
				btVector3 childPos = pOffsetInChild.getOrigin();
				btQuaternion parentRot = pOffsetInParent.getRotation();
				btQuaternion childRot = pOffsetInChild.getRotation();
				glm::vec3 parentPos2(parentPos.x(), parentPos.y(), parentPos.z());
				glm::vec3 childPos2(childPos.x(), childPos.y(), childPos.z());
				glm::quat parentRot2(parentRot.w(), parentRot.x(), parentRot.y(), parentRot.z());
				glm::quat childRot2(childRot.w(), childRot.x(), childRot.y(), childRot.z());

				_joint = physics.addUniversalJoint(pUrdfModel.m_name, pUrdfJoint.m_name, pUrdfJoint.m_parentLinkName, pUrdfJoint.m_childLinkName, parentPos2, parentRot2, childPos2, childRot2, RO_ZYX);

				//std::cout << "create joint " << pUrdfJoint.m_name << " RO_ZYX " << "\n";

				//btVector3 parentPos = pOffsetInParent.getOrigin();
				//btVector3 childPos = pOffsetInChild.getOrigin();
				//btQuaternion parentRot = pOffsetInParent.getRotation();
				//btQuaternion childRot = pOffsetInChild.getRotation();

				//glm::vec3 parentPos2(parentPos.x(), parentPos.y(), parentPos.z());
				//glm::vec3 childPos2(childPos.x(), childPos.y(), childPos.z());
				//glm::quat parentRot2(parentRot.w(), parentRot.x(), parentRot.y(), parentRot.z());
				//glm::quat childRot2(childRot.w(), childRot.x(), childRot.y(), childRot.z());
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_parentLinkName, pUrdfJoint.m_childLinkName, parentPos2, parentRot2, childPos2, childRot2);

				_joint->setLinearLowerLimit({ 0, 0, 0 });
				_joint->setLinearUpperLimit({ 0, 0, 0 });
				_joint->setAngularLowerLimit({ static_cast<float>(jointLowerLimit), 0, 0 });
				_joint->setAngularUpperLimit({ static_cast<float>(jointUpperLimit), 0, 0 });

				// create motor
				if (pUrdfJoint.m_effortLimit > 0.0)
				{
					_motor = physics.addUniversalMotor(pUrdfModel.m_name, pUrdfJoint.m_name);
					_motor->setLinearActive({ false, false,  false });
					_motor->setAngularActive({ true, false, false });
					_motor->setLinearServoActive({ false, false,  false });
					_motor->setAngularServoActive({ false, false,  false });
					_motor->setLinearSpringActive({ false, false,  false });
					_motor->setAngularSpringActive({ false, false,  false });
					//_motor->setMaxLinearMotorForce({ 0, 0, 0 });
					//_motor->setMaxAngularMotorForce({ static_cast<float>(pUrdfJoint.m_effortLimit), static_cast<float>(pUrdfJoint.m_effortLimit), static_cast<float>(pUrdfJoint.m_effortLimit) });
					//_motor->setAngularVelocity({10.0, 10.0, 10.0 });
					//_motor->setMaxAngularMotorForce({ static_cast<float>(pUrdfJoint.m_effortLimit), 0, 0 });
				}

				break;
			}
			case 1:
			{
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_XZY);

				btVector3 parentPos = pOffsetInParent.getOrigin();
				btVector3 childPos = pOffsetInChild.getOrigin();
				btQuaternion parentRot = pOffsetInParent.getRotation();
				btQuaternion childRot = pOffsetInChild.getRotation();
				glm::vec3 parentPos2(parentPos.x(), parentPos.y(), parentPos.z());
				glm::vec3 childPos2(childPos.x(), childPos.y(), childPos.z());
				glm::quat parentRot2(parentRot.w(), parentRot.x(), parentRot.y(), parentRot.z());
				glm::quat childRot2(childRot.w(), childRot.x(), childRot.y(), childRot.z());

				_joint = physics.addUniversalJoint(pUrdfModel.m_name, pUrdfJoint.m_name, pUrdfJoint.m_parentLinkName, pUrdfJoint.m_childLinkName, parentPos2, parentRot2, childPos2, childRot2, RO_XZY);
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, childPos2, childRot2, parentPos2, parentRot2, RO_XZY);

				//std::cout << "create joint " << pUrdfJoint.m_name << " RO_XZY " << "\n";

				_joint->setLinearLowerLimit({ 0, 0, 0 });
				_joint->setLinearUpperLimit({ 0, 0, 0 });
				_joint->setAngularLowerLimit({ 0, static_cast<float>(jointLowerLimit), 0 });
				_joint->setAngularUpperLimit({ 0, static_cast<float>(jointUpperLimit), 0 });

				//btVector3 parentPos = pOffsetInParent.getOrigin();
				//btVector3 childPos = pOffsetInChild.getOrigin();
				//btQuaternion parentRot = pOffsetInParent.getRotation();
				//btQuaternion childRot = pOffsetInChild.getRotation();
				//glm::vec3 parentPos2(parentPos.x(), parentPos.y(), parentPos.z());
				//glm::vec3 childPos2(childPos.x(), childPos.y(), childPos.z());
				//glm::quat parentRot2(parentRot.w(), parentRot.x(), parentRot.y(), parentRot.z());
				//glm::quat childRot2(childRot.w(), childRot.x(), childRot.y(), childRot.z());
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_parentLinkName, pUrdfJoint.m_childLinkName, parentPos2, parentRot2, childPos2, childRot2);

				// create motor
				if (pUrdfJoint.m_effortLimit > 0.0)
				{
					_motor = physics.addUniversalMotor(pUrdfModel.m_name, pUrdfJoint.m_name);
					_motor->setLinearActive({ false, false,  false });
					_motor->setAngularActive({ false, true, false });
					_motor->setLinearServoActive({ false, false,  false });
					_motor->setAngularServoActive({ false, false,  false });
					_motor->setLinearSpringActive({ false, false,  false });
					_motor->setAngularSpringActive({ false, false,  false });
					//_motor->setMaxLinearMotorForce({ 0, 0, 0 });
					//_motor->setMaxAngularMotorForce({ static_cast<float>(pUrdfJoint.m_effortLimit), static_cast<float>(pUrdfJoint.m_effortLimit), static_cast<float>(pUrdfJoint.m_effortLimit) });
					//_motor->setAngularVelocity({ 10.0, 10.0, 10.0 });
					//_motor->setMaxAngularMotorForce({ 0, static_cast<float>(pUrdfJoint.m_effortLimit), 0 });
				}

				break;
			}
			case 2:
			default:
			{
				//std::cout << "case 2 create joint " << pUrdfJoint.m_name << "\n";

				// orig version
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, pOffsetInChild, pOffsetInParent, RO_XYZ);

				btVector3 parentPos = pOffsetInParent.getOrigin();
				btVector3 childPos = pOffsetInChild.getOrigin();
				btQuaternion parentRot = pOffsetInParent.getRotation();
				btQuaternion childRot = pOffsetInChild.getRotation();
				glm::vec3 parentPos2(parentPos.x(), parentPos.y(), parentPos.z());
				glm::vec3 childPos2(childPos.x(), childPos.y(), childPos.z());
				glm::quat parentRot2(parentRot.w(), parentRot.x(), parentRot.y(), parentRot.z());
				glm::quat childRot2(childRot.w(), childRot.x(), childRot.y(), childRot.z());

				_joint = physics.addUniversalJoint(pUrdfModel.m_name, pUrdfJoint.m_name, pUrdfJoint.m_parentLinkName, pUrdfJoint.m_childLinkName, parentPos2, parentRot2, childPos2, childRot2, RO_XYZ);
				//_joint = physics.addUniversalJoint(pUrdfJoint.m_name, pUrdfJoint.m_childLinkName, pUrdfJoint.m_parentLinkName, childPos2, childRot2, parentPos2, parentRot2, RO_XYZ);

				//std::cout << "create joint " << pUrdfJoint.m_name << " RO_XYZ " << "\n";

				_joint->setLinearLowerLimit({ 0, 0, 0 });
				_joint->setLinearUpperLimit({ 0, 0, 0 });
				_joint->setAngularLowerLimit({ 0, 0, static_cast<float>(jointLowerLimit) });
				_joint->setAngularUpperLimit({ 0, 0, static_cast<float>(jointUpperLimit) });

				// create motor
				if (pUrdfJoint.m_effortLimit > 0.0)
				{
					//std::cout << "create motor\n";

					_motor = physics.addUniversalMotor(pUrdfModel.m_name, pUrdfJoint.m_name);
					_motor->setLinearActive({ false, false,  false });
					_motor->setAngularActive({ false, false, true });
					_motor->setLinearServoActive({ false, false, false });
					_motor->setAngularServoActive({ false, false, false });
					_motor->setLinearSpringActive({ false, false, false });
					_motor->setAngularSpringActive({ false, false, false });
					_motor->setMaxLinearMotorForce({ 0, 0, 0 });
					//_motor->setMaxAngularMotorForce({ 0, 0, static_cast<float>(pUrdfJoint.m_effortLimit) });
					_motor->setMaxAngularMotorForce({ 0, 0, 10000 });

					//_motor->setAngularVelocity({ 10.0, 10.0, 10.0 });
					//_motor->setMaxLinearMotorForce({ 1000, 1000, 1000 });
					//_motor->setMaxAngularMotorForce({ 0, 0, static_cast<float>(pUrdfJoint.m_effortLimit) });
				}
			}
		}
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Urdf Error: faild to create revolute joint " + pUrdfJoint.m_name, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}

	return _joint;
}


std::string 
UrdfImporter::getPathPrefix(const std::string& pFilePath)
{
	std::string pathPrefix = pFilePath;

	#ifdef _WIN32
	std::replace(pathPrefix.begin(), pathPrefix.end(), '/', '\\');
	const int idx = pathPrefix.find_last_of("\\");
	if (std::string::npos != idx) pathPrefix = pathPrefix.substr(0, idx + 1);
	#else
	std::replace(pathPrefix.begin(), pathPrefix.end(), '\\', '/');
	const int idx = pathPrefix.find_last_of("/");
	if (std::string::npos != idx) pathPrefix = pathPrefix.substr(0, idx + 1);
	#endif

	return pathPrefix;
}

std::string 
UrdfImporter::getMeshFileName(const std::string& pMeshFileString)
{
	std::string packagePrefix = "package:\//";
	if (pMeshFileString.find(packagePrefix) != std::string::npos)
	{
		return pMeshFileString.substr(packagePrefix.length(), pMeshFileString.length() - packagePrefix.length());
	}

	return pMeshFileString;
}

std::string 
UrdfImporter::getMeshName(const std::string& pMeshFileString)
{
	std::string meshName = getMeshFileName(pMeshFileString);

	size_t suffixPos = meshName.find_last_of(".");
	if (suffixPos != std::string::npos)
	{
		return meshName.substr(0, suffixPos);
	}

	return meshName;
}

std::string 
UrdfImporter::getTextureName(const std::string& pTextureFileString)
{
	std::string textureName = getMeshFileName(pTextureFileString);

	size_t suffixPos = textureName.find_last_of(".");
	if (suffixPos != std::string::npos)
	{
		return textureName.substr(0, suffixPos);
	}

	return textureName;
}

void
UrdfImporter::getMassAndInertia(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame)
{
	//std::cout << "getMassAndInertia2 for link " << pUrdfLink.m_name << "\n";

	mass = pUrdfLink.m_inertia.m_mass * sMassScale;
	localInertiaDiagonal.setValue(0, 0, 0);
	inertialFrame.setOrigin(pUrdfLink.m_inertia.m_linkLocalFrame.getOrigin());
	inertialFrame.setBasis(pUrdfLink.m_inertia.m_linkLocalFrame.getBasis());
}

//void 
//UrdfImporter::getMassAndInertia(UrdfModel& pUrdfModel, UrdfLink& pUrdfLink, btScalar& mass, btVector3& localInertiaDiagonal, btTransform& inertialFrame)
//{
//	std::cout << "get mass and inertia for link " << pUrdfLink.m_name << "\n";
//
//	btMatrix3x3 linkInertiaBasis;
//
//	btScalar linkMass, principalInertiaX, principalInertiaY, principalInertiaZ;
//	if (pUrdfLink.m_parentJoint == 0 && pUrdfModel.m_overrideFixedBase)
//	{
//		linkMass = 0.f;
//		principalInertiaX = 0.f;
//		principalInertiaY = 0.f;
//		principalInertiaZ = 0.f;
//		linkInertiaBasis.setIdentity();
//	}
//	else
//	{
//		linkMass = pUrdfLink.m_inertia.m_mass;
//		if (pUrdfLink.m_inertia.m_ixy == 0.0 &&
//			pUrdfLink.m_inertia.m_ixz == 0.0 &&
//			pUrdfLink.m_inertia.m_iyz == 0.0)
//		{
//			principalInertiaX = pUrdfLink.m_inertia.m_ixx;
//			principalInertiaY = pUrdfLink.m_inertia.m_iyy;
//			principalInertiaZ = pUrdfLink.m_inertia.m_izz;
//			linkInertiaBasis.setIdentity();
//		}
//		else
//		{
//			principalInertiaX = pUrdfLink.m_inertia.m_ixx;
//			btMatrix3x3 inertiaTensor(pUrdfLink.m_inertia.m_ixx, pUrdfLink.m_inertia.m_ixy, pUrdfLink.m_inertia.m_ixz,
//				pUrdfLink.m_inertia.m_ixy, pUrdfLink.m_inertia.m_iyy, pUrdfLink.m_inertia.m_iyz,
//				pUrdfLink.m_inertia.m_ixz, pUrdfLink.m_inertia.m_iyz, pUrdfLink.m_inertia.m_izz);
//			btScalar threshold = 1.0e-6;
//			int numIterations = 30;
//			inertiaTensor.diagonalize(linkInertiaBasis, threshold, numIterations);
//			principalInertiaX = inertiaTensor[0][0];
//			principalInertiaY = inertiaTensor[1][1];
//			principalInertiaZ = inertiaTensor[2][2];
//		}
//	}
//	mass = linkMass;
//	if (principalInertiaX < 0 ||
//		principalInertiaX >(principalInertiaY + principalInertiaZ) ||
//		principalInertiaY < 0 ||
//		principalInertiaY >(principalInertiaX + principalInertiaZ) ||
//		principalInertiaZ < 0 ||
//		principalInertiaZ >(principalInertiaX + principalInertiaY))
//	{
//		std::cout << "Warning: Bad inertia tensor properties, setting inertia to zero for link: " << pUrdfLink.m_name << "\n";
//		principalInertiaX = 0.f;
//		principalInertiaY = 0.f;
//		principalInertiaZ = 0.f;
//		linkInertiaBasis.setIdentity();
//	}
//	localInertiaDiagonal.setValue(principalInertiaX, principalInertiaY, principalInertiaZ);
//	inertialFrame.setOrigin(pUrdfLink.m_inertia.m_linkLocalFrame.getOrigin());
//	inertialFrame.setBasis(pUrdfLink.m_inertia.m_linkLocalFrame.getBasis() * linkInertiaBasis);
//}

bool 
UrdfImporter::getJointInfo(UrdfLink& pUrdfLink, btTransform& parent2joint, btTransform& linkTransformInWorld, btVector3& jointAxisInJointSpace, int& jointType, btScalar& jointLowerLimit, btScalar& jointUpperLimit, btScalar& jointDamping, btScalar& jointFriction, btScalar& jointMaxForce, btScalar& jointMaxVelocity)
{
	jointLowerLimit = 0.f;
	jointUpperLimit = 0.f;
	jointDamping = 0.f;
	jointFriction = 0.f;
	jointMaxForce = 0.f;
	jointMaxVelocity = 0.f;

	linkTransformInWorld = pUrdfLink.m_linkTransformInWorld;

	if (pUrdfLink.m_parentJoint)
	{
		UrdfJoint* pj = pUrdfLink.m_parentJoint;
		parent2joint = pj->m_parentLinkToJointTransform;
		jointType = pj->m_type;
		jointAxisInJointSpace = pj->m_localJointAxis;
		jointLowerLimit = pj->m_lowerLimit;
		jointUpperLimit = pj->m_upperLimit;
		jointDamping = pj->m_jointDamping;
		jointFriction = pj->m_jointFriction;
		jointMaxForce = pj->m_effortLimit;
		jointMaxVelocity = pj->m_velocityLimit;
		return true;
	}
	else
	{
		parent2joint.setIdentity();
		return false;
	}
}

void 
UrdfImporter::processContactParameters(const URDFLinkContactInfo& contactInfo, btCollisionObject* col)
{
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_LATERAL_FRICTION) != 0)
	{
		col->setFriction(contactInfo.m_lateralFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_RESTITUTION) != 0)
	{
		col->setRestitution(contactInfo.m_restitution);
	}

	if ((contactInfo.m_flags & URDF_CONTACT_HAS_ROLLING_FRICTION) != 0)
	{
		col->setRollingFriction(contactInfo.m_rollingFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_SPINNING_FRICTION) != 0)
	{
		col->setSpinningFriction(contactInfo.m_spinningFriction);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_STIFFNESS_DAMPING) != 0)
	{
		col->setContactStiffnessAndDamping(contactInfo.m_contactStiffness, contactInfo.m_contactDamping);
	}
	if ((contactInfo.m_flags & URDF_CONTACT_HAS_FRICTION_ANCHOR) != 0)
	{
		col->setCollisionFlags(col->getCollisionFlags() | btCollisionObject::CF_HAS_FRICTION_ANCHOR);
	}
}