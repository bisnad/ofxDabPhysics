/** \file dab_phyiscs_simulation.cpp
*/

#include "dab_physics_simulation.h"
#include "dab_physics_body.h"
#include "dab_physics_body_shape.h"
#include "dab_physics_body_part.h"
#include "dab_physics_body_joint.h"
#include "dab_physics_universal_joint.h"
#include "dab_physics_body_motor.h"
#include "dab_physics_universal_motor.h"
#include "dab_physics_behavior.h"
#include "dab_physics_behavior_force.h"
#include "dab_physics_behavior_random_force.h"
#include "dab_physics_behavior_torque.h"
#include "dab_physics_behavior_random_torque.h"
#include "dab_physics_behavior_rotation_target.h"
#include "dab_physics_behavior_random_rotation_target.h"
#include "dab_physics_behavior_target_attraction.h"
#include "dab_physics_behavior_volume.h"
#include "dab_physics_behavior_speed.h"
#include "dab_geom_mesh_manager.h"
#include <BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btLemkeSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <BulletDynamics/MLCPSolvers/btPATHSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>

using namespace dab;
using namespace dab::physics;

//glm::vec3 Simulation::sGravity = glm::vec3(0.0, 0.0, -9.81);
glm::vec3 Simulation::sGravity = glm::vec3(0.0, 0.0, -9.81);
//glm::vec3 Simulation::sGravity = glm::vec3(0.0, 0.0, 0.0);
float Simulation::sTimeStep = 1.0f / 60.f;
int Simulation::sSubSteps = 200;

Simulation::Simulation()
	: btCollisionDispatcher(new btDefaultCollisionConfiguration())
	, mGravity(sGravity)
	, mTimeStep(sTimeStep)
	, mSubSteps(sSubSteps)
	, mUpdateInterval(0.01)
	, mPaused(true)
{
	initPhysics();
}

Simulation::~Simulation()
{}

void 
Simulation::initPhysics()
{
	mCollisionConfiguration = new btDefaultCollisionConfiguration();
	mCollisionDispatcher = new	btCollisionDispatcher(mCollisionConfiguration);
	mBroadPhase = new btDbvtBroadphase();



	/////// uncomment the corresponding line to test a solver.
	//mConstraintSolver = new btSequentialImpulseConstraintSolver();
	//mConstraintSolver = new btSequentialImpulseConstraintSolverMt();
	mConstraintSolver = new btMLCPSolver(new btSolveProjectedGaussSeidel());
	//mConstraintSolver = new btMLCPSolver(new btDantzigSolver());
	//mConstraintSolver = new btMLCPSolver(new btLemkeSolver());

	mDynamicsWorld = new btDiscreteDynamicsWorld(mCollisionDispatcher, mBroadPhase, mConstraintSolver, mCollisionConfiguration);
	//mDynamicsWorld = new btDiscreteDynamicsWorld(this, mBroadPhase, mConstraintSolver, mCollisionConfiguration);
	mDynamicsWorld->setGravity(btVector3(mGravity.x, mGravity.y, mGravity.z));

	mDynamicsWorld->getPairCache()->setOverlapFilterCallback(this);

 
	//mCollisionDispatcher->setNearCallback(MyNearCallback);

	//mCollisionDispatcher->setNearCallback(Simulation::customNearCallback);
}

const glm::vec3& 
Simulation::gravity() const
{
	return mGravity;
}

float 
Simulation::timeStep() const
{
	return mTimeStep;
}

int 
Simulation::subSteps() const
{
	return mSubSteps;
}

void 
Simulation::setGravity(const glm::vec3& pGravity)
{
	mGravity = pGravity;
	mDynamicsWorld->setGravity(btVector3(mGravity.x, mGravity.y, mGravity.z));
}

void 
Simulation::setTimeStep(float pTimeStep)
{
	mTimeStep = pTimeStep;

	//std::cout << "Simulation setTimeStep " << mTimeStep << "\n";
}

void 
Simulation::setSupSteps(int pSubSteps)
{
	mSubSteps = pSubSteps;

	//std::cout << "Simulation setSupSteps " << mSubSteps << "\n";
}

const std::vector<std::shared_ptr<BodyShape>>&
Simulation::shapes() const
{
	return mShapes.values();
}

const std::vector<std::shared_ptr<Body>>& 
Simulation::bodies() const
{
	return mBodies.values();
}

bool 
Simulation::shapeExists(const std::string& pShapeName) const
{
	return mShapes.contains(pShapeName);
}

bool 
Simulation::bodyExists(const std::string& pBodyName) const
{
	return mBodies.contains(pBodyName);
}

bool 
Simulation::partExists(const std::string& pBodyName, const std::string& pPartName) const
{
	if (bodyExists(pBodyName) == false) return false;
	return mBodies[pBodyName]->partExists(pPartName);
}

bool 
Simulation::jointExists(const std::string& pBodyName, const std::string& pPartName) const
{
	if (bodyExists(pBodyName) == false) return false;
	return mBodies[pBodyName]->jointExists(pPartName);
}

bool 
Simulation::motorExists(const std::string& pBodyName, const std::string& pPartName) const
{
	if (bodyExists(pBodyName) == false) return false;
	return mBodies[pBodyName]->motorExists(pPartName);
}

bool 
Simulation::behaviorExists(const std::string& pBodyName, const std::string& pBehaviorName) const
{
	if (bodyExists(pBodyName) == false) return false;
	return mBodies[pBodyName]->behaviorExists(pBehaviorName);
}

std::shared_ptr<BodyShape>
Simulation::shape(const std::string& pShapeName) throw (dab::Exception)
{
	try
	{
		return mShapes[pShapeName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve shape, no shape with name " + pShapeName + " exists ", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyShape>
Simulation::shape(const std::string& pShapeName) const throw (dab::Exception)
{
	try
	{
		return mShapes[pShapeName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve shape, no shape with name " + pShapeName + " exists ", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<Body>
Simulation::body(const std::string& pBodyName) throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve body, no body with name " + pBodyName + " exists ", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<Body>
Simulation::body(const std::string& pBodyName) const throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName];
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve body, no body with name " + pBodyName + " exists ", __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<BodyPart> 
Simulation::part(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->part(pPartName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve part " + pPartName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyPart> 
Simulation::part(const std::string& pBodyName, const std::string& pPartName) const throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->part(pPartName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve part " + pPartName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<BodyJoint> 
Simulation::joint(const std::string& pBodyName, const std::string& pJointName) throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->joint(pJointName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve joint " + pJointName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyJoint> 
Simulation::joint(const std::string& pBodyName, const std::string& pJointName) const throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->joint(pJointName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve joint " + pJointName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<BodyMotor> 
Simulation::motor(const std::string& pBodyName, const std::string& pMotorName) throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->motor(pMotorName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve motor " + pMotorName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<BodyMotor> 
Simulation::motor(const std::string& pBodyName, const std::string& pMotorName) const throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->motor(pMotorName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve motor " + pMotorName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

std::shared_ptr<Behavior> 
Simulation::behavior(const std::string& pBodyName, const std::string& pBehaviorName) throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->behavior(pBehaviorName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve behavior " + pBehaviorName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::shared_ptr<Behavior> 
Simulation::behavior(const std::string& pBodyName, const std::string& pBehaviorName) const throw (dab::Exception)
{
	try
	{
		return mBodies[pBodyName]->behavior(pBehaviorName);
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to retrieve behavior " + pBehaviorName + " from body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

const std::vector<std::shared_ptr<BodyPart>>&
Simulation::parts() const
{
	return mParts;
}

const std::vector<std::shared_ptr<BodyJoint>>&
Simulation::joints() const
{
	return mJoints;
}

const std::vector<std::shared_ptr<BodyMotor>>&
Simulation::motors() const
{
	return mMotors;
}

const std::vector<std::shared_ptr<Behavior>>&
Simulation::behaviors() const
{
	return mBehaviors;
}

const std::vector<std::shared_ptr<BodyPart>>
Simulation::parts(const std::string& pBodyName) const throw (dab::Exception)
{
	try
	{
		return body(pBodyName)->parts();
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

const std::vector<std::shared_ptr<BodyJoint>>
Simulation::joints(const std::string& pBodyName) const throw (dab::Exception)
{
	try
	{
		return body(pBodyName)->joints();
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

const std::vector<std::shared_ptr<BodyMotor>>
Simulation::motors(const std::string& pBodyName) const throw (dab::Exception)
{
	try
	{
		return body(pBodyName)->motors();
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

const std::vector<std::shared_ptr<Behavior>> 
Simulation::behaviors(const std::string& pBodyName) const throw (dab::Exception)
{
	try
	{
		return body(pBodyName)->behaviors();
	}
	catch (dab::Exception& e)
	{
		throw;
	}
}

std::shared_ptr<BodyPlaneShape>
Simulation::addPlaneShape(const std::string& pShapeName, const glm::vec3& pNormal) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create plane shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyPlaneShape> _bodyShape(new BodyPlaneShape(pShapeName, pNormal));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodySphereShape>
Simulation::addSphereShape(const std::string& pShapeName, float pRadius) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create sphere shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodySphereShape> _bodyShape(new BodySphereShape(pShapeName, pRadius));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodyBoxShape>
Simulation::addBoxShape(const std::string& pShapeName, const glm::vec3& pSize) throw (dab::Exception)
{
	//std::cout << "BodySimulation::addBoxShape " << pShapeName << "\n";

	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create box shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyBoxShape> _bodyShape(new BodyBoxShape(pShapeName, pSize));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodyCylinderShape>
Simulation::addCylinderShape(const std::string& pShapeName, const glm::vec3& pSize) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create cylinder shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCylinderShape> _bodyShape(new BodyCylinderShape(pShapeName, pSize));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodyCapsuleShape>
Simulation::addCapsuleShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create capsule shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCapsuleShape> _bodyShape(new BodyCapsuleShape(pShapeName, pRadius, pHeight));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodyConeShape>
Simulation::addConeShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: failed to create cone shape, shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyConeShape> _bodyShape(new BodyConeShape(pShapeName, pRadius, pHeight));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<BodyConvexMeshShape>
Simulation::addConvexMeshShape(const std::string& pShapeName, const std::string& pMeshName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	try
	{
		const std::vector<glm::vec3>& vertices = dab::geom::MeshManager::get().vertices(pMeshName);
		std::shared_ptr<BodyConvexMeshShape> _bodyShape(new BodyConvexMeshShape(pShapeName, pMeshName, vertices));

		mShapes.add(_bodyShape->name(), _bodyShape);

		return _bodyShape;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to obtain vertices for mesh " + pMeshName, __FILE__, __FUNCTION__, __LINE__);
	}
}

std::shared_ptr<BodyCompoundShape>
Simulation::addCompoundShape(const std::string& pShapeName) throw (dab::Exception)
{
	if (mShapes.contains(pShapeName) == true) throw dab::Exception("Sim Error: shape with name " + pShapeName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyCompoundShape> _bodyShape(new BodyCompoundShape(pShapeName));

	mShapes.add(_bodyShape->name(), _bodyShape);

	return _bodyShape;
}

std::shared_ptr<Body>
Simulation::addBody(const std::string& pBodyName) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == true) throw dab::Exception("Sim Error: failed to body, body with name " + pBodyName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body(new Body(pBodyName));

	mBodies.add(pBodyName, _body);

	return _body;
}

std::shared_ptr<BodyPart>
Simulation::addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass) throw (dab::Exception)
{
	if(mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (mShapes.contains(pShapeName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + ", shape with name " + pShapeName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mParts.contains(pPartName)) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because part already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyShape> _bodyShape = mShapes[pShapeName];
	std::shared_ptr<BodyPart> _part(new BodyPart(pPartName, _bodyShape, pMass));

	if (isBodyRoot == true) _body->mRootPart = _part;
	_body->mParts.add(_part->name(), _part);
	_part->mBody = _body;
	mParts.push_back(_part);

	mDynamicsWorld->addRigidBody(_part->mNativeBody);

	return _part;
}

std::shared_ptr<BodyPart> 
Simulation::addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass, const glm::vec3& pPosition, const glm::quat& pOrientation) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (mShapes.contains(pShapeName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + ", shape with name " + pShapeName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];
	if (_body->mParts.contains(pPartName)) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because part already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyShape> _bodyShape = mShapes[pShapeName];
	std::shared_ptr<BodyPart> _part(new BodyPart(pPartName, _bodyShape, pMass, pPosition, pOrientation));

	if (isBodyRoot == true) _body->mRootPart = _part;
	_body->mParts.add(_part->name(), _part);
	_part->mBody = _body;
	mParts.push_back(_part);

	mDynamicsWorld->addRigidBody(_part->mNativeBody);

	return _part;
}


std::shared_ptr<BodyPart> 
Simulation::addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass, const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (mShapes.contains(pShapeName) == false) throw dab::Exception("Sim Error: failed to create body part " + pPartName + ", shape with name " + pShapeName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	
	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mParts.contains(pPartName)) throw dab::Exception("Sim Error: failed to create body part " + pPartName + " for body " + pBodyName + " because part already exists", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyShape> _bodyShape = mShapes[pShapeName];
	std::shared_ptr<BodyPart> _part(new BodyPart(pPartName, _bodyShape, pMass, pLocalInertiaDiagonal, pInitialWorldTrans));

	if (isBodyRoot == true) _body->mRootPart = _part;
	_body->mParts.add(_part->name(), _part);
	_part->mBody = _body;
	mParts.push_back(_part);

	mDynamicsWorld->addRigidBody(_part->mNativeBody);

	return _part;
}


std::shared_ptr<UniversalJoint>
Simulation::addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pPrevPartName, const std::string& pNextPartName, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mJoints.contains(pJointName) == true) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since joint already exists", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pPrevPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pPrevPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pNextPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pNextPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyPart> prevPart = _body->mParts[pPrevPartName];
	std::shared_ptr<BodyPart> nextPart = _body->mParts[pNextPartName];
	std::shared_ptr<UniversalJoint> _joint(new UniversalJoint(pJointName, prevPart, nextPart, pPrevJointPos, pNextJointPos));

	prevPart->mNextJoints.push_back(_joint);
	nextPart->mPrevJoints.push_back(_joint);

	_body->mJoints.add(pJointName, _joint);
	_joint->mBody = _body;
	mJoints.push_back(_joint);

	mDynamicsWorld->addConstraint(_joint->mNativeJoint);

	addChildParentParts(pNextPartName, pPrevPartName);

	return _joint;
}

std::shared_ptr<UniversalJoint> 
Simulation::addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pParentPartName, const std::string& pChildPartName, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot, RotateOrder pRotateOrder) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mJoints.contains(pJointName) == true) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since joint already exists", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pParentPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pParentPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pChildPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pChildPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyPart> parentPart = _body->mParts[pParentPartName];
	std::shared_ptr<BodyPart> childPart = _body->mParts[pChildPartName];
	std::shared_ptr<UniversalJoint> _joint(new UniversalJoint(pJointName, parentPart, childPart, pPrevJointPos, pPrevJointRot, pNextJointPos, pNextJointRot, pRotateOrder));

	parentPart->mNextJoints.push_back(_joint);
	childPart->mPrevJoints.push_back(_joint);

	_body->mJoints.add(pJointName, _joint);
	_joint->mBody = _body;
	mJoints.push_back(_joint);

	mDynamicsWorld->addConstraint(_joint->mNativeJoint);

	addChildParentParts(pChildPartName, pParentPartName);

	return _joint;
}

std::shared_ptr<UniversalJoint> 
Simulation::addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pParentPartName, const std::string& pChildPartName, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, RotateOrder pRotateOrder) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mJoints.contains(pJointName) == true) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since joint already exists", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pParentPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pParentPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mParts.contains(pChildPartName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " since part " + pChildPartName + " does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyPart> parentPart = _body->mParts[pParentPartName];
	std::shared_ptr<BodyPart> childPart = _body->mParts[pChildPartName];
	std::shared_ptr<UniversalJoint> _joint(new UniversalJoint(pJointName, parentPart, childPart, pOffsetInParent, pOffsetInChild, pRotateOrder));

	parentPart->mNextJoints.push_back(_joint);
	childPart->mPrevJoints.push_back(_joint);

	_body->mJoints.add(pJointName, _joint);
	_joint->mBody = _body;
	mJoints.push_back(_joint);

	mDynamicsWorld->addConstraint(_joint->mNativeJoint);

	addChildParentParts(pChildPartName, pParentPartName);

	return _joint;
}


std::shared_ptr<UniversalMotor>
Simulation::addUniversalMotor(const std::string& pBodyName, const std::string& pJointName) throw (dab::Exception)
{
	if (mBodies.contains(pBodyName) == false) throw dab::Exception("Sim Error: failed to create joint " + pJointName + " for body " + pBodyName + " because body does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<Body> _body = mBodies[pBodyName];

	if (_body->mMotors.contains(pJointName) == true) throw dab::Exception("Sim Error: failed to create motor " + pJointName + " for body " + pBodyName + " since motor already exists", __FILE__, __FUNCTION__, __LINE__);
	if (_body->mJoints.contains(pJointName) == false) throw dab::Exception("Sim Error: failed to create motor " + pJointName + " for body " + pBodyName + " since joint does not exist", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<BodyJoint> _joint = _body->mJoints[pJointName];
	std::shared_ptr<UniversalJoint> _universalJoint = std::dynamic_pointer_cast<UniversalJoint>(_joint);

	if (_universalJoint == nullptr) throw dab::Exception("Sim Error: failed to create motor " + pJointName + " since joint is not a universal joint", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<UniversalMotor> _universalMotor(new UniversalMotor(_universalJoint));
	_universalJoint->setMotor(_universalMotor);

	_body->mMotors.add(pJointName, _universalMotor);
	_universalMotor->mBody = _body;
	mMotors.push_back(_universalMotor);

	return _universalMotor;
}

std::shared_ptr<Body> 
Simulation::copyBody(const std::string& pSourceBodyName, const std::string& pTargetBodyName) throw (dab::Exception)
{
	if (mBodies.contains(pSourceBodyName) == false) throw dab::Exception("Sim Error: failed to copy body, source body " + pSourceBodyName + " does not exist", __FILE__, __FUNCTION__, __LINE__);
	if (mBodies.contains(pTargetBodyName) == true) throw dab::Exception("Sim Error: failed to copy body, target body " + pTargetBodyName + " already exists", __FILE__, __FUNCTION__, __LINE__);

	try
	{
		std::shared_ptr<Body> _sourceBody = mBodies[pSourceBodyName];
		std::shared_ptr<Body> _targetBody = addBody(pTargetBodyName);

		// copy parts
		int partCount = _sourceBody->mParts.size();
		for (int pI = 0; pI < partCount; ++pI)
		{
			copyBodyPart(_sourceBody, _sourceBody->mParts[pI], _targetBody);
		}

		// copy joints
		int jointCount = _sourceBody->mJoints.size();
		for (int jI = 0; jI < jointCount; ++jI)
		{
			copyBodyJoint(_sourceBody, _sourceBody->mJoints[jI], _targetBody);
		}

		// copy motors
		int motorCount = _sourceBody->mMotors.size();
		for (int mI = 0; mI < motorCount; ++mI)
		{
			copyBodyMotor(_sourceBody, _sourceBody->mMotors[mI], _targetBody);
		}

		// copy behaviors
		int behaviorCount = _sourceBody->mBehaviors.size();
		for (int bI = 0; bI < behaviorCount; ++bI)
		{
			copyBodyBehavior(_sourceBody, _sourceBody->mBehaviors[bI], _targetBody);
		}

		return _targetBody;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to create body " + pTargetBodyName  + " by copying source body " + pSourceBodyName, __FILE__, __FUNCTION__, __LINE__ );
		throw e;
	}
}

void 
Simulation::update()
{
	if (mPaused == true) return;

	try
	{
		for (auto _behavior : mBehaviors)
		{
			//std::cout << "body " << _behavior->body()->name() << " beh " << _behavior->name() << " update\n";

			_behavior->update();
		}
	}
	catch (dab::Exception& e)
	{
		std::cout << e << "\n";
	}

	mDynamicsWorld->stepSimulation(mTimeStep, mSubSteps);
}

void
Simulation::start()
{
	if (isThreadRunning() == false)
	{
		mTime = ofGetElapsedTimef();
		mPaused = false;
		startThread();
	}
}

void
Simulation::stop()
{
	if (isThreadRunning() == true) stopThread();
}

bool
Simulation::paused() const
{
	return mPaused;
}

void
Simulation::setPaused(bool pPaused)
{
	mPaused = pPaused;
}

void
Simulation::threadedFunction()
{
	while (isThreadRunning())
	{
		if (mPaused == false)
		{
			double currentTime = ofGetElapsedTimef();
			if (currentTime - mTime < mUpdateInterval) continue;
			else mTime = currentTime;

			update();
		}

		std::this_thread::sleep_for(std::chrono::microseconds(10));
	}
}

bool 
Simulation::needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const
{
	if (mPaused == true) return false;

	bool collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
	collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

	if (proxy0 != nullptr && proxy1 != nullptr)
	{
		btRigidBody* nativeBody0 = reinterpret_cast<btRigidBody*>(proxy0->m_clientObject);
		btRigidBody* nativeBody1 = reinterpret_cast<btRigidBody*>(proxy1->m_clientObject);
		BodyPart* body0 = reinterpret_cast<BodyPart*>(nativeBody0->getUserPointer());
		BodyPart* body1 = reinterpret_cast<BodyPart*>(nativeBody1->getUserPointer());
		const std::vector<std::shared_ptr<BodyJoint>>& body0_nextjoints = body0->nextJoints();
		const std::vector<std::shared_ptr<BodyJoint>>& body1_nextjoints = body1->nextJoints();
		const std::vector<std::shared_ptr<BodyJoint>>& body0_prevjoints = body0->prevJoints();
		const std::vector<std::shared_ptr<BodyJoint>>& body1_prevjoints = body1->prevJoints();
		
		//std::cout << "body0 " << body0->name() << " as child: prev links: \n";
		for (int lI = 0; lI < body0_prevjoints.size(); ++lI)
		{
			//std::cout << "lI " << lI << " prev " << body0_prevjoints[lI]->prevPart()->name() << " next " << body0_prevjoints[lI]->nextPart()->name() << "\n";

			std::string parentName = body0_prevjoints[lI]->prevPart()->name();

			if (body1->name() == parentName)
			{
				//std::cout << "link 1: body0 " << body0->name() << " body1 " << body1->name() << " parentchild\n";

				collides = false;
			}
		}

		//std::cout << "body0 " << body0->name() << " as parent: next links: \n";
		for (int lI = 0; lI < body0_nextjoints.size(); ++lI)
		{
			std::string childName = body0_nextjoints[lI]->nextPart()->name();

			//std::cout << "lI " << lI << " prev " << body0_nextjoints[lI]->prevPart()->name() << " next " << body0_nextjoints[lI]->nextPart()->name() << "\n";

			if (body1->name() == childName)
			{
				//std::cout << "link 1: body0 " << body0->name() << " body1 " << body1->name() << " parentchild\n";

				collides = false;
			}
		}
	}


	// TODO: remove collision flag based on custom criteria
	//std::cout << "collides " << collides << "\n";

	return collides;
}

bool 
Simulation::needsCollision(btCollisionObject* body0, btCollisionObject* body1)
{
	bool collides = btCollisionDispatcher::needsCollision(body0, body1);

	std::cout << "needs collision\n";

	return collides;
}

bool 
Simulation::needsResponse(btCollisionObject* body0, btCollisionObject* body1)
{
	std::cout << "needs response\n";

	return false;
}

void 
Simulation::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher)
{
	btBroadphasePairArray array = pairCache->getOverlappingPairArray();

	std::vector<btBroadphasePair> pairsForRemoval;

	int arraySize = array.size();
	for (int aI = 0; aI < arraySize; ++aI)
	{
		btBroadphasePair pair = array[aI];

		btBroadphaseProxy* proxy0 = pair.m_pProxy0;
		btBroadphaseProxy* proxy1 = pair.m_pProxy1;

		if (proxy0 != nullptr && proxy1 != nullptr)
		{
			btRigidBody* nativeBody0 = reinterpret_cast<btRigidBody*>(proxy0->m_clientObject);
			btRigidBody* nativeBody1 = reinterpret_cast<btRigidBody*>(proxy1->m_clientObject);
		
			BodyPart* body0 = reinterpret_cast<BodyPart*>(nativeBody0->getUserPointer());
			BodyPart* body1 = reinterpret_cast<BodyPart*>(nativeBody1->getUserPointer());

			if (checkChildParentCollision(body0, body1) == true)
			{
				//std::cout << "collision body0 " << body0->name() << " body1 " << body1->name() << " parentChildColl \n";

				//pairsForRemoval.push_back(pair);
			}
		}
	}

	int pairsForRemovalCount = pairsForRemoval.size();
	for (int i = 0; i < pairsForRemovalCount; ++i)
	{
		btBroadphasePair pair = pairsForRemoval[i];
		btBroadphaseProxy* proxy0 = pair.m_pProxy0;
		btBroadphaseProxy* proxy1 = pair.m_pProxy1;

		pairCache->removeOverlappingPair(proxy0, proxy1, dispatcher);
	}

	btCollisionDispatcher::dispatchAllCollisionPairs(pairCache, dispatchInfo, dispatcher);
}

//void 
//Simulation::nearCollision(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo)
//{
//	std::cout << "beep\n";
//}

bool 
Simulation::checkChildParentCollision(BodyPart* pPart1, BodyPart* pPart2) const
{
	//const std::vector< std::shared_ptr<BodyJoint > >& prevJoints = pPart1->prevJoints();
	//const std::vector< std::shared_ptr<BodyJoint > >& nextJoints = pPart1->nextJoints();

	////std::cout << pPart1->name() << " prev joints " << prevJoints.size() << "\n";
	////std::cout << pPart1->name() << " next joints " << nextJoints.size() << "\n";

	//int prevJointCount = prevJoints.size();
	//for (int jI = 0; jI < prevJointCount; ++jI)
	//{
	//	if (prevJoints[jI]->prevPart().get() == pPart2) return true;
	//}

	//int nextJointCount = nextJoints.size();
	//for (int jI = 0; jI < nextJointCount; ++jI)
	//{
	//	if (nextJoints[jI]->nextPart().get() == pPart2) return true;
	//}

	//return false;

	const std::vector< std::shared_ptr<BodyJoint > >& part1PrevJoints = pPart1->prevJoints();
	const std::vector< std::shared_ptr<BodyJoint > >& part2PrevJoints = pPart2->prevJoints();

	//std::cout << pPart1->name() << " prev joints " << prevJoints.size() << "\n";
	//std::cout << pPart1->name() << " next joints " << nextJoints.size() << "\n";

	int part1PrevJointCount = part1PrevJoints.size();
	for (int jI = 0; jI < part1PrevJointCount; ++jI)
	{
		if (part1PrevJoints[jI]->prevPart().get() == pPart2) return true;
	}

	int part2PrevJointCount = part2PrevJoints.size();
	for (int jI = 0; jI < part2PrevJointCount; ++jI)
	{
		if (part2PrevJoints[jI]->prevPart().get() == pPart1) return true;
	}

	return false;
}

btQuaternion 
Simulation::euler2quat(const std::array<float, 3>& pEuler) const
{
	double phi, the, psi;
	double roll = pEuler[0];
	double pitch = pEuler[1];
	double yaw = pEuler[2];

	phi = roll / 2.0;
	the = pitch / 2.0;
	psi = yaw / 2.0;

	btQuaternion _quat(
		sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi),
		cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi),
		cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi),
		cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi));

	_quat.normalize();

	return _quat;
}

void 
Simulation::addChildParentParts(const std::string& pChildPartName, const std::string& pParentPartName)
{
	mChildParentPartNames[pChildPartName] = pParentPartName;
}

bool 
Simulation::checkChildParentParts(const std::string& pPart1Name, const std::string& pPart2Name) const
{
	if (mChildParentPartNames.find(pPart1Name) == mChildParentPartNames.end()) return false;
	return (mChildParentPartNames.at(pPart1Name) == pPart2Name);
}


std::shared_ptr<BodyPart> 
Simulation::copyBodyPart(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyPart> pSourcePart, std::shared_ptr<Body> pTargetBody) throw (dab::Exception)
{
	std::shared_ptr<BodyPart> _targetPart = addBodyPart(pTargetBody->name(), pSourcePart->name(), pSourcePart->shape()->name(), (pSourcePart == pSourceBody->mRootPart), pSourcePart->mass());

	_targetPart->setLinearDamping(pSourcePart->linearDamping());
	_targetPart->setAngularDamping(pSourcePart->angularDamping());
	_targetPart->setFriction(pSourcePart->friction());
	_targetPart->setRollingFriction(pSourcePart->rollingFriction());
	_targetPart->setRestitution(pSourcePart->restitution());
	_targetPart->setPosition(pSourcePart->position());
	_targetPart->setRotation(pSourcePart->rotation());
	_targetPart->mShapeTransform = pSourcePart->mShapeTransform;
	_targetPart->mShapeMatrix = pSourcePart->mShapeMatrix;

	return _targetPart;
}

std::shared_ptr<BodyJoint>
Simulation::copyBodyJoint(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyJoint> pSourceJoint, std::shared_ptr<Body> pTargetBody) throw (dab::Exception)
{
	// deal with universal joints only
	if (std::dynamic_pointer_cast<UniversalJoint>(pSourceJoint) == nullptr) throw dab::Exception("Sim Error: failed to copy joint " + pSourceJoint->name() + " only joints of type UniversalJoint supported", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<UniversalJoint> _sourceJoint = std::static_pointer_cast<UniversalJoint>(pSourceJoint);
	std::shared_ptr<UniversalJoint> _targetJoint = addUniversalJoint(pTargetBody->name(), _sourceJoint->name(), _sourceJoint->prevPart()->name(), _sourceJoint->nextPart()->name(), _sourceJoint->prevJointPos(), _sourceJoint->prevJointRot(), _sourceJoint->nextJointPos(), _sourceJoint->nextJointRot(), _sourceJoint->rotateOrder());

	_targetJoint->setLinearLowerLimit(_sourceJoint->linearLowerLimit());
	_targetJoint->setLinearUpperLimit(_sourceJoint->linearUpperLimit());
	_targetJoint->setAngularLowerLimit(_sourceJoint->angularLowerLimit());
	_targetJoint->setAngularUpperLimit(_sourceJoint->angularUpperLimit());
	_targetJoint->setLinearStopERP(_sourceJoint->linearStopERP());
	_targetJoint->setAngularStopERP(_sourceJoint->angularStopERP());
	_targetJoint->setLinearStopCFM(_sourceJoint->linearStopCFM());
	_targetJoint->setAngularStopCFM(_sourceJoint->angularStopCFM());

	return _sourceJoint;

}

std::shared_ptr<BodyMotor>
Simulation::copyBodyMotor(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyMotor> pSourceMotor, std::shared_ptr<Body> pTargetBody) throw (dab::Exception)
{
	// deal with universal motors only
	if (std::dynamic_pointer_cast<UniversalMotor>(pSourceMotor) == nullptr) throw dab::Exception("Sim Error: failed to copy motor " + pSourceMotor->name() + " only motors of type UniversalMotor supported", __FILE__, __FUNCTION__, __LINE__);

	std::shared_ptr<UniversalMotor> _sourceMotor = std::static_pointer_cast<UniversalMotor>(pSourceMotor);
	std::shared_ptr<UniversalMotor> _targetMotor = addUniversalMotor(pTargetBody->name(), _sourceMotor->name());

	_targetMotor->setBounce(_sourceMotor->bounce());
	_targetMotor->setDamping(_sourceMotor->damping());
	_targetMotor->setLinearActive(_sourceMotor->linearActive());
	_targetMotor->setAngularActive(_sourceMotor->angularActive());
	_targetMotor->setLinearLowerLimit(_sourceMotor->linearLowerLimit());
	_targetMotor->setLinearUpperLimit(_sourceMotor->linearUpperLimit());
	_targetMotor->setAngularLowerLimit(_sourceMotor->angularLowerLimit());
	_targetMotor->setAngularUpperLimit(_sourceMotor->angularUpperLimit());
	_targetMotor->setLinearStopERP(_sourceMotor->linearStopERP());
	_targetMotor->setAngularStopERP(_sourceMotor->angularStopERP());
	_targetMotor->setLinearStopCFM(_sourceMotor->linearStopCFM());
	_targetMotor->setAngularStopCFM(_sourceMotor->angularStopCFM());
	_targetMotor->setMaxLinearMotorForce(_sourceMotor->maxLinearMotorForce());
	_targetMotor->setMaxAngularMotorForce(_sourceMotor->maxAngularMotorForce());
	_targetMotor->setLinearVelocity(_sourceMotor->linearVelocity());
	_targetMotor->setAngularVelocity(_sourceMotor->angularVelocity());
	_targetMotor->setLinearServoActive(_sourceMotor->linearServoActive());
	_targetMotor->setAngularServoActive(_sourceMotor->angularServoActive());
	_targetMotor->setLinearServoTarget(_sourceMotor->linearServoTarget());
	_targetMotor->setAngularServoTarget(_sourceMotor->angularServoTarget());
	_targetMotor->setLinearSpringActive(_sourceMotor->linearSpringActive());
	_targetMotor->setAngularSpringActive(_sourceMotor->angularSpringActive());
	_targetMotor->setLinearSpringStiffness(_sourceMotor->linearSpringStiffness());
	_targetMotor->setAngularSpringStiffness(_sourceMotor->angularSpringStiffness());
	_targetMotor->setLinearSpringDamping(_sourceMotor->linearSpringDamping());
	_targetMotor->setAngularSpringDamping(_sourceMotor->angularSpringDamping());
	_targetMotor->setLinearSpringRestLength(_sourceMotor->linearSpringRestLength());
	_targetMotor->setAngularSpringRestLength(_sourceMotor->angularSpringRestLength());

	return _targetMotor;

}

std::shared_ptr<Behavior>
Simulation::copyBodyBehavior(std::shared_ptr<Body> pSourceBody, std::shared_ptr<Behavior> pSourceBehavior, std::shared_ptr<Body> pTargetBody) throw (dab::Exception)
{
	const std::vector<std::shared_ptr<BodyPart>> _sourceParts = pSourceBehavior->parts();
	const std::vector<std::shared_ptr<BodyJoint>> _sourceJoints = pSourceBehavior->joints();
	const std::vector<std::shared_ptr<BodyMotor>> _sourceMotors = pSourceBehavior->motors();
	const std::map<std::string, AbstractValue* > _sourceParameters = pSourceBehavior->parameters();

	std::vector<std::string> partNames;
	std::vector<std::string> jointNames;
	std::vector<std::string> motorNames;

	for (auto part : _sourceParts) partNames.push_back(part->name());
	for (auto joint : _sourceJoints) jointNames.push_back(joint->name());
	for (auto motor : _sourceMotors) motorNames.push_back(motor->name());

	std::shared_ptr<Behavior> _targetBehavior = nullptr;

	if (std::dynamic_pointer_cast<ForceBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::ForceBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<RandomForceBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::RandomForceBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<TorqueBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::TorqueBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<RandomTorqueBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::RandomTorqueBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<RotationTargetBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::RotationTargetBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<RandomRotationTargetBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::RandomRotationTargetBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<TargetAttractionBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::TargetAttractionBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<VolumeBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::VolumeBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else if (std::dynamic_pointer_cast<SpeedBehavior>(pSourceBehavior) != nullptr)
	{
		_targetBehavior = addBehavior<dab::physics::SpeedBehavior>(pTargetBody->name(), pSourceBehavior->name(), partNames, jointNames, motorNames);
	}
	else
	{
		throw Exception("Sim Error: unknown behaviour type", __FILE__, __FUNCTION__, __LINE__);
	}

	const std::map<std::string, AbstractValue* > _targetParameters = _targetBehavior->parameters();
	for (auto iter : _targetParameters)
	{
		*(_targetParameters.at(iter.first)) = *(_sourceParameters.at(iter.first));
	}

	return _targetBehavior;
}
