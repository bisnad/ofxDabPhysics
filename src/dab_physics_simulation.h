/** \file dab_phyiscs_simulation.h
*/

#pragma once

#include <btBulletDynamicsCommon.h>
#include "ofVectorMath.h"
#include "dab_singleton.h"
#include "dab_index_map.h"
#include "dab_exception.h"
#include "ofThread.h"
//#include "bullet_extras/BulletWorldImporter/btBulletWorldImporter.h"

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
class BodyCompoundShape;
class BodyConvexMeshShape;
class BodyPart;
class BodyJoint;
class HingeJoint;
class UniversalJoint;
class Universal2Joint;
class BodyMotor;
class HingeMotor;
class UniversalMotor;
class Universal2Motor;
class Behavior;


class Simulation : public dab::Singleton<Simulation>, public btOverlapFilterCallback, public btCollisionDispatcher, public ofThread
{
public:
	Simulation();
	~Simulation();

	const glm::vec3& gravity() const;
	float timeStep() const;
	int subSteps() const;

	void setGravity(const glm::vec3& pGravity);
	void setTimeStep(float pTimeStep);
	void setSupSteps(int pSubSteps);

	bool shapeExists(const std::string& pShapeName) const;
	bool bodyExists(const std::string& pBodyName) const;
	bool partExists(const std::string& pBodyName, const std::string& pPartName) const;
	bool jointExists(const std::string& pBodyName, const std::string& pPartName) const;
	bool motorExists(const std::string& pBodyName, const std::string& pPartName) const;
	bool behaviorExists(const std::string& pBodyName, const std::string& pBehaviorName) const;

	const std::vector<std::shared_ptr<BodyShape>>& shapes() const;
	const std::vector<std::shared_ptr<Body>>& bodies() const;
	const std::vector<std::shared_ptr<BodyPart>>& parts() const;
	const std::vector<std::shared_ptr<BodyJoint>>& joints() const;
	const std::vector<std::shared_ptr<BodyMotor>>& motors() const;
	const std::vector<std::shared_ptr<Behavior>>& behaviors() const;

	const std::vector<std::shared_ptr<BodyPart>> parts(const std::string& pBodyName) const throw (dab::Exception);
	const std::vector<std::shared_ptr<BodyJoint>> joints(const std::string& pBodyName) const throw (dab::Exception);
	const std::vector<std::shared_ptr<BodyMotor>> motors(const std::string& pBodyName) const throw (dab::Exception);
	const std::vector<std::shared_ptr<Behavior>> behaviors(const std::string& pBodyName) const throw (dab::Exception);

	std::shared_ptr<Body> body(const std::string& pBodyName) throw (dab::Exception);
	const std::shared_ptr<Body> body(const std::string& pBodyName) const throw (dab::Exception);
	std::shared_ptr<BodyShape> shape(const std::string& pShapeName) throw (dab::Exception);
	const std::shared_ptr<BodyShape> shape(const std::string& pShapeName) const throw (dab::Exception);
	std::shared_ptr<BodyPart> part(const std::string& pBodyName, const std::string& pPartName) throw (dab::Exception);
	const std::shared_ptr<BodyPart> part(const std::string& pBodyName, const std::string& pPartName) const throw (dab::Exception);
	std::shared_ptr<BodyJoint> joint(const std::string& pBodyName, const std::string& pJointName) throw (dab::Exception);
	const std::shared_ptr<BodyJoint> joint(const std::string& pBodyName, const std::string& pJointName) const throw (dab::Exception);
	std::shared_ptr<BodyMotor> motor(const std::string& pBodyName, const std::string& pMotorName) throw (dab::Exception);
	const std::shared_ptr<BodyMotor> motor(const std::string& pBodyName, const std::string& pMotorName) const throw (dab::Exception);
	std::shared_ptr<Behavior> behavior(const std::string& pBodyName, const std::string& pBehaviorName) throw (dab::Exception);
	const std::shared_ptr<Behavior> behavior(const std::string& pBodyName, const std::string& pBehaviorName) const throw (dab::Exception);

	std::shared_ptr<BodyPlaneShape> addPlaneShape(const std::string& pShapeName, const glm::vec3& pNormal) throw (dab::Exception);
	std::shared_ptr<BodySphereShape> addSphereShape(const std::string& pShapeName, float pRadius) throw (dab::Exception);
	std::shared_ptr<BodyBoxShape> addBoxShape(const std::string& pShapeName, const glm::vec3& pSize) throw (dab::Exception);
	std::shared_ptr<BodyCylinderShape> addCylinderShape(const std::string& pShapeName, const glm::vec3& pSize) throw (dab::Exception);
	std::shared_ptr<BodyCapsuleShape> addCapsuleShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception);
	std::shared_ptr<BodyConeShape> addConeShape(const std::string& pShapeName, float pRadius, float pHeight) throw (dab::Exception);
	std::shared_ptr<BodyConvexMeshShape> addConvexMeshShape(const std::string& pShapeName, const std::string& pMeshName) throw (dab::Exception);
	std::shared_ptr<BodyCompoundShape> addCompoundShape(const std::string& pShapeName) throw (dab::Exception);

	std::shared_ptr<Body> addBody(const std::string& pBodyName) throw (dab::Exception);
	std::shared_ptr<Body> copyBody(const std::string& pSourceBodyName, const std::string& pCopyBodyName) throw (dab::Exception);

	std::shared_ptr<BodyPart> addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass) throw (dab::Exception);
	std::shared_ptr<BodyPart> addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass, const glm::vec3& pPosition, const glm::quat& pOrientation) throw (dab::Exception);
	std::shared_ptr<BodyPart> addBodyPart(const std::string& pBodyName, const std::string& pPartName, const std::string& pShapeName, bool isBodyRoot, float pMass, const btVector3& pLocalInertiaDiagonal, const btTransform& pInitialWorldTrans) throw (dab::Exception);

	std::shared_ptr<UniversalJoint> addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pPrevPartName, const std::string& pNextPartName, const glm::vec3& pPrevJointPos, const glm::vec3& pNextJointPos) throw (dab::Exception);
	std::shared_ptr<UniversalJoint> addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pParentPartName, const std::string& pChildPartName, const glm::vec3& pPrevJointPos, const glm::quat& pPrevJointRot, const glm::vec3& pNextJointPos, const glm::quat& pNextJointRot, RotateOrder pRotateOrder) throw (dab::Exception);
	std::shared_ptr<UniversalJoint> addUniversalJoint(const std::string& pBodyName, const std::string& pJointName, const std::string& pParentPartName, const std::string& pChildPartName, const btTransform& pOffsetInParent, const btTransform& pOffsetInChild, RotateOrder pRotateOrder) throw (dab::Exception);

	std::shared_ptr<UniversalMotor> addUniversalMotor(const std::string& pBodyName, const std::string& pJointName) throw (dab::Exception);

	template<class BehaviorType> std::shared_ptr<BehaviorType> addBehavior(const std::string& pBodyName, const std::string& pBehaviorName, const std::vector<std::string>& pPartNames, const std::vector<std::string>& pJointNames, const std::vector<std::string>& pMotorNames) throw (dab::Exception);
	template<class BehaviorType> std::vector<std::shared_ptr<BehaviorType>> addNeighborBehavior(const std::vector<std::string>& pBodyNames, const std::string& pBehaviorName, const std::vector<std::string>& pPartNames, const std::vector<std::string>& pNeighborPartNames, const std::vector<std::string>& pJointNames, const std::vector<std::string>& pNeighborJointNames, const std::vector<std::string>& pMotorNames, const std::vector<std::string>& pNeighborMotorNames) throw (dab::Exception);

	void start();
	void stop();
	bool paused() const;
	void setPaused(bool pPaused);
	void update();

	//collision callback
	bool needBroadphaseCollision(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1) const;
	bool needsCollision(btCollisionObject* body0, btCollisionObject* body1);
	bool needsResponse(btCollisionObject* body0, btCollisionObject* body1);
	void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& dispatchInfo, btDispatcher* dispatcher);

	// util functions
	btQuaternion euler2quat(const std::array<float, 3>& pEuler) const;


	static void customNearCallback(btBroadphasePair& collisionPair, btCollisionDispatcher& dispatcher, btDispatcherInfo& dispatchInfo)
	{}

protected:
	static glm::vec3 sGravity;
	static float sTimeStep;
	static int sSubSteps;

	glm::vec3 mGravity;
	float mTimeStep;
	int mSubSteps;

	bool mPaused;
	double mUpdateInterval;
	double mTime;

	dab::IndexMap<std::string, std::shared_ptr<BodyShape> > mShapes;
	dab::IndexMap<std::string, std::shared_ptr<Body>> mBodies;

	std::vector<std::shared_ptr<BodyPart> > mParts;
	std::vector<std::shared_ptr<BodyJoint> > mJoints;
	std::vector<std::shared_ptr<BodyMotor> > mMotors;
	std::vector<std::shared_ptr<Behavior> > mBehaviors;

	btDefaultCollisionConfiguration* mCollisionConfiguration = nullptr;
	btCollisionDispatcher* mCollisionDispatcher = nullptr;
	btBroadphaseInterface* mBroadPhase = nullptr;
	btConstraintSolver* mConstraintSolver = nullptr;
	btDiscreteDynamicsWorld* mDynamicsWorld = nullptr;

	std::map<std::string, std::string> mChildParentPartNames;

	void initPhysics();
	void threadedFunction();
	bool checkChildParentCollision(BodyPart* pPart1, BodyPart* pPart2) const;

	void addChildParentParts(const std::string& pChildPartName, const std::string& pParentPartName);
	bool checkChildParentParts(const std::string& pPart1Name, const std::string& pPart2Name) const;
	std::shared_ptr<BodyPart> copyBodyPart(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyPart> pSourcePart, std::shared_ptr<Body> pTargetBody) throw (dab::Exception);
	std::shared_ptr<BodyJoint> copyBodyJoint(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyJoint> pSourceJoint, std::shared_ptr<Body> pTargetBody) throw (dab::Exception);
	std::shared_ptr<BodyMotor> copyBodyMotor(std::shared_ptr<Body> pSourceBody, std::shared_ptr<BodyMotor> pSourceMotor, std::shared_ptr<Body> pTargetBody) throw (dab::Exception);
	std::shared_ptr<Behavior> copyBodyBehavior(std::shared_ptr<Body> pSourceBody, std::shared_ptr<Behavior> pSourceBehavior, std::shared_ptr<Body> pTargetBody) throw (dab::Exception);

};

template<class BehaviorType> 
std::shared_ptr<BehaviorType> 
Simulation::addBehavior(const std::string& pBodyName, const std::string& pBehaviorName, const std::vector<std::string>& pPartNames, const std::vector<std::string>& pJointNames, const std::vector<std::string>& pMotorNames) throw (dab::Exception)
{
	try
	{
		std::shared_ptr<Body> _body = body(pBodyName);

		std::vector<std::shared_ptr<BodyPart>> _parts;
		for (auto partName : pPartNames) _parts.push_back(_body->part(partName));

		std::vector<std::shared_ptr<BodyJoint>> _joints;
		for (auto jointName : pJointNames) _joints.push_back(_body->joint(jointName));

		std::vector<std::shared_ptr<BodyMotor>> _motors;
		for (auto motorName : pMotorNames) _motors.push_back(_body->motor(motorName));

		std::shared_ptr<BehaviorType> _behavior(new BehaviorType(pBehaviorName, _parts, _joints, _motors));

		_body->addBehavior(_behavior);
		_behavior->mBody = _body;
		mBehaviors.push_back(_behavior);

		return _behavior;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to add behaviour " + pBehaviorName + " to body " + pBodyName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}

template<class BehaviorType> 
std::vector<std::shared_ptr<BehaviorType>> 
Simulation::addNeighborBehavior(const std::vector<std::string>& pBodyNames, const std::string& pBehaviorName, const std::vector<std::string>& pPartNames, const std::vector<std::string>& pNeighborPartNames, const std::vector<std::string>& pJointNames, const std::vector<std::string>& pNeighborJointNames, const std::vector<std::string>& pMotorNames, const std::vector<std::string>& pNeighborMotorNames) throw (dab::Exception)
{
	std::vector<std::shared_ptr<BehaviorType>> _neighborBehaviors;

	try
	{
		for (auto bodyName : pBodyNames)
		{
			std::shared_ptr<Body> _body = body(bodyName);

			std::vector<std::shared_ptr<BodyPart>> _parts;
			std::vector<std::shared_ptr<BodyJoint>> _joints;
			std::vector<std::shared_ptr<BodyMotor>> _motors;

			std::vector<std::shared_ptr<BodyPart>> _neighborParts;
			std::vector<std::shared_ptr<BodyJoint>> _neighborJoints;
			std::vector<std::shared_ptr<BodyMotor>> _neighborMotors;

			for (auto partName : pPartNames) _parts.push_back(_body->part(partName));
			for (auto jointName : pJointNames) _joints.push_back(_body->joint(jointName));
			for (auto motorName : pMotorNames) _motors.push_back(_body->motor(motorName));

			for (auto neighborBodyName : pBodyNames)
			{
				if (neighborBodyName != bodyName)
				{
					std::shared_ptr<Body> _neighborBody = body(neighborBodyName);

					for (auto partName : pNeighborPartNames) _neighborParts.push_back(_neighborBody->part(partName));
					for (auto jointName : pNeighborJointNames) _neighborJoints.push_back(_neighborBody->joint(jointName));
					for (auto motorName : pNeighborMotorNames) _neighborMotors.push_back(_neighborBody->motor(motorName));
				}
			}

			std::shared_ptr<BehaviorType> _neighborBehavior(new BehaviorType(pBehaviorName, _parts, _neighborParts, _neighborJoints, _joints, _motors, _neighborMotors));

			_body->addBehavior(_neighborBehavior);
			_neighborBehavior->mBody = _body;
			_neighborBehaviors.push_back(_neighborBehavior);
			mBehaviors.push_back(_neighborBehavior);
		}

		return _neighborBehaviors;
	}
	catch (dab::Exception& e)
	{
		e += dab::Exception("Sim Error: failed to add neigbor behaviour " + pBehaviorName, __FILE__, __FUNCTION__, __LINE__);
		throw e;
	}
}


};

};

