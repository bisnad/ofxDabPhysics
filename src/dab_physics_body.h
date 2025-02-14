/** \file dab_physics_body.h
*/

#pragma once

#include "dab_index_map.h"
#include "dab_exception.h"
#include "dab_physics_simulation.h"

namespace dab
{

namespace physics
{

class BodyPart;
class BodyJoint;
class BodyMotor;
class Behavior;

class Body
{
public:
	friend class Simulation;

	Body(const std::string& pName);
	~Body();

	const std::string& name() const;
	std::shared_ptr<BodyPart> rootPart() const;
	const std::vector<std::shared_ptr<BodyPart>>& parts() const;
	const std::vector<std::shared_ptr<BodyJoint>>& joints() const;
	const std::vector<std::shared_ptr<BodyMotor>>& motors() const;
	const std::vector<std::shared_ptr<Behavior>>& behaviors() const;

	bool partExists(const std::string& pPartName) const;
	bool jointExists(const std::string& pJointName) const;
	bool motorExists(const std::string& pMotorName) const;
	bool behaviorExists(const std::string& pBehaviorName) const;

	std::shared_ptr<BodyPart> part(const std::string& pPartName) throw (dab::Exception);
	const std::shared_ptr<BodyPart> part(const std::string& pPartName) const throw (dab::Exception);
	std::shared_ptr<BodyJoint> joint(const std::string& pJointName) throw (dab::Exception);
	const std::shared_ptr<BodyJoint> joint(const std::string& pJointName) const throw (dab::Exception);
	std::shared_ptr<BodyMotor> motor(const std::string& pMotorName) throw (dab::Exception);
	const std::shared_ptr<BodyMotor> motor(const std::string& pMotorName) const throw (dab::Exception);
	std::shared_ptr<Behavior> behavior(const std::string& pBehaviorName) throw (dab::Exception);
	const std::shared_ptr<Behavior> behavior(const std::string& pBehaviorName) const throw (dab::Exception);

protected:

	//void initBody();
	void setRootPart(std::shared_ptr<BodyPart> pPart) throw (dab::Exception);
	void addPart(std::shared_ptr<BodyPart> pPart) throw (dab::Exception);
	void addJoint(std::shared_ptr<BodyJoint> pJoint) throw (dab::Exception);
	void addMotor(std::shared_ptr<BodyMotor> pMotor) throw (dab::Exception);
	void addBehavior(std::shared_ptr<Behavior> pBehavior) throw (dab::Exception);

	std::string mName;
	std::shared_ptr<BodyPart> mRootPart;
	IndexMap< std::string, std::shared_ptr< BodyPart > > mParts;
	IndexMap< std::string, std::shared_ptr< BodyJoint > > mJoints;
	IndexMap< std::string, std::shared_ptr< BodyMotor > > mMotors;
	IndexMap< std::string, std::shared_ptr< Behavior > > mBehaviors;
};

};

};